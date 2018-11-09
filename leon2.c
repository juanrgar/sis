/*
 * This file is part of SIS.
 *
 * SIS, SPARC instruction simulator V2.8 Copyright (C) 2015 Jiri Gaisler
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * Leon2 emulation, based on leon3.c and erc32.c/
 */

/* The control space devices.  */

#include "config.h"
#include <errno.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#ifdef HAVE_TERMIOS_H
#include <termios.h>
#endif
#include <sys/file.h>
#include <unistd.h>
#include "sis.h"
#include "grlib.h"
/* #include "sim-config.h" */

/* APB registers */
#define APBSTART	0x80000000
#define APBEND		0x80000100

/* Memory exception waitstates */
#define MEM_EX_WS	1

#define MOK		0

/* LEON2 APB register addresses */

#define IRQCTRL_IPR	0x094
#define IRQCTRL_IMR	0x090
#define IRQCTRL_ICR	0x09C
#define IRQCTRL_IFR	0x098
#define TIMER_SCALER	0x060
#define TIMER_SCLOAD	0x064
#define LEON2_CONFIG	0x024
#define TIMER_TIMER1	0x040
#define TIMER_RELOAD1	0x044
#define TIMER_CTRL1	0x048
#define TIMER_TIMER2	0x050
#define TIMER_RELOAD2	0x054
#define TIMER_CTRL2	0x058
#define CACHE_CTRL	0x014
#define POWER_DOWN	0x018

#define APBUART_RXTX	0x070
#define APBUART_STATUS	0x074

/* Size of UART buffers (bytes).  */
#define UARTBUF	1024

/* Number of simulator ticks between flushing the UARTS.  */
/* For good performance, keep above 1000.  */
#define UART_FLUSH_TIME	  3000

/* New uart defines.  */
#define UART_TX_TIME	1000
#define UART_RX_TIME	1000
#define UARTA_DR	0x1
#define UARTA_SRE	0x2
#define UARTA_HRE	0x4
#define UARTA_OR	0x10

/* IRQCTRL registers.  */

static uint32 irqctrl_ipr;
static uint32 irqctrl_imr;
static uint32 irqctrl_ifr;

/* TIMER registers.  */

#define NTIMERS		2
#define TIMER_IRQ	8

static uint32 gpt_scaler;
static uint32 gpt_scaler_start;
static uint32 gpt_counter[NTIMERS];
static uint32 gpt_reload[NTIMERS];
static uint32 gpt_ctrl[NTIMERS];

/* ROM size 16 Mbyte.  */
#define ROM_START	0x00000000
#define ROM_MASK	0x00ffffff
#define ROM_END		(ROM_START + ROM_MASK + 1)

/* RAM size 16 Mbyte.  */
#define RAM_START	0x40000000
#define RAM_MASK	0x00ffffff
#define RAM_END		(RAM_START + RAM_MASK + 1)

/* Memory.  */
static unsigned char romb[ROM_END - ROM_START];
static unsigned char ramb[RAM_END - RAM_START];
static uint32 cache_ctrl;


/* UART support variables.  */

/* File descriptor for input file.  */
static int32 fd1, fd2;

/* UART status register */
static int32 Ucontrol;

static unsigned char aq[UARTBUF], bq[UARTBUF];
static int32 anum, aind = 0;
static int32 bnum, bind = 0;
static char wbufa[UARTBUF], wbufb[UARTBUF];
static unsigned wnuma;
static unsigned wnumb;
static FILE *f1in, *f1out;
#ifdef HAVE_TERMIOS_H
static struct termios ioc1, ioc2, iocold1, iocold2;
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK 0
#endif

static int f1open = 0;

static char uarta_sreg, uarta_hreg;
static uint32 uart_stat_reg;
static uint32 uarta_data;

/* Forward declarations. */

static void mem_init (void);
static void close_port (void);
static void leon2_reset (void);
static void irqctrl_intack (int32 level);
static void chk_irq (void);
static void set_irq (int32 level);
static int32 apb_read (uint32 addr, uint32 * data);
static int apb_write (uint32 addr, uint32 data);
static void port_init (void);
static uint32 grlib_read_uart (uint32 addr);
static void grlib_write_uart (uint32 addr, uint32 data);
static void flush_uart (void);
static void uarta_tx (void);
static void uart_rx (int32 arg);
static void uart_intr (int32 arg);
static void uart_irq_start (void);
static void gpt_intr (int32 arg);
static void gpt_init (void);
static void gpt_reset (void);
static void gpt_scaler_set (uint32 val);
static void timer_ctrl (uint32 val, int i);
static unsigned char *get_mem_ptr (uint32 addr, uint32 size);
static void store_bytes (unsigned char *mem, uint32 waddr,
			 uint32 * data, int sz, int32 * ws);

/* static host_callback *callback; */


/* One-time init. */

static void
init_sim (void)
{
  /* callback = sim_callback; */
  grlib_init ();
  mem_init ();
  port_init ();
  gpt_init ();
}

/* Power-on reset init. */

static void
reset (void)
{
  leon2_reset ();
  uart_irq_start ();
  gpt_reset ();
}

/* IU error mode manager. */

static void
error_mode (uint32 pc)
{

}

/* Memory init. */

static void
mem_init (void)
{

  if (sis_verbose)
    printf ("RAM start: 0x%x, RAM size: %d K, ROM size: %d K\n",
	    RAM_START, (RAM_MASK + 1) / 1024, (ROM_MASK + 1) / 1024);
}

/* Flush ports when simulator stops. */

static void
sim_halt (void)
{
#ifdef FAST_UART
  flush_uart ();
#endif
}

static void
close_port (void)
{
  if (f1open && f1in != stdin)
    fclose (f1in);
}

static void
exit_sim (void)
{
  close_port ();
}

static void
leon2_reset (void)
{
  int i;

  irqctrl_ipr = 0;
  irqctrl_imr = 0;
  irqctrl_ifr = 0;

  wnuma = wnumb = 0;
  anum = aind = bnum = bind = 0;

  uart_stat_reg = UARTA_SRE | UARTA_HRE;

  gpt_counter[0] = 0xffffffff;
  gpt_reload[0] = 0xffffffff;
  gpt_scaler = 0xffff;
  gpt_ctrl[0] = 0;
  gpt_ctrl[1] = 0;
}

static void
irqctrl_intack (int32 level)
{
  int irq_test;

  if (sis_verbose > 2)
    printf ("interrupt %d acknowledged\n", level);
  if (irqctrl_ifr & (1 << level))
    irqctrl_ifr &= ~(1 << level);
  else
    irqctrl_ipr &= ~(1 << level);
  chk_irq ();
}

static void
chk_irq (void)
{
  int32 i;
  uint32 itmp;
  int old_irl;

  old_irl = ext_irl;
  itmp = ((irqctrl_ipr | irqctrl_ifr) & irqctrl_imr) & 0x0fffe;
  ext_irl = 0;
  if (itmp != 0)
    {
      for (i = 15; i > 0; i--)
	{
	  if (((itmp >> i) & 1) != 0)
	    {
	      if ((sis_verbose > 2) && (i > old_irl))
		printf ("IU irl: %d\n", i);
	      ext_irl = i;
	      set_int (i, irqctrl_intack, i);
	      break;
	    }
	}
    }
}

static void
set_irq (int32 level)
{
  irqctrl_ipr |= (1 << level);
  chk_irq ();
}

static int32
apb_read (uint32 addr, uint32 * data)
{

  switch (addr & 0xfff)
    {

    case APBUART_RXTX:		/* 0x100 */
    case APBUART_STATUS:	/* 0x104 */
      *data = grlib_read_uart (addr);
      break;

    case IRQCTRL_IPR:		/* 0x204 */
      *data = irqctrl_ipr;
      break;

    case IRQCTRL_IFR:		/* 0x208 */
      *data = irqctrl_ifr;
      break;

    case IRQCTRL_IMR:		/* 0x240 */
      *data = irqctrl_imr;
      break;

    case TIMER_SCALER:		/* 0x300 */
      *data = gpt_scaler - (now () - gpt_scaler_start);
      break;

    case TIMER_SCLOAD:		/* 0x304 */
      *data = gpt_scaler;
      break;

    case LEON2_CONFIG:		/* 0x308 */
      *data = 0x700310;
      break;

    case TIMER_TIMER1:		/* 0x310 */
      *data = gpt_counter[0];
      break;

    case TIMER_RELOAD1:	/* 0x314 */
      *data = gpt_reload[0];
      break;

    case TIMER_CTRL1:		/* 0x318 */
      *data = gpt_ctrl[0];
      break;

    case TIMER_TIMER2:		/* 0x320 */
      *data = gpt_counter[1];
      break;

    case TIMER_RELOAD2:	/* 0x324 */
      *data = gpt_reload[1];
      break;

    case TIMER_CTRL2:		/* 0x328 */
      *data = gpt_ctrl[1];
      break;

    case CACHE_CTRL:		/* 0x328 */
      *data = cache_ctrl;
      break;

    default:
      *data = 0;
      break;
    }

  if (sis_verbose > 1)
    printf ("APB read  a: %08x, d: %08x\n", addr, *data);

  return MOK;
}

static int
apb_write (uint32 addr, uint32 data)
{
  if (sis_verbose > 1)
    printf ("APB write a: %08x, d: %08x\n", addr, data);
  switch (addr & 0xff)
    {

    case APBUART_RXTX:		/* 0x100 */
    case APBUART_STATUS:	/* 0x104 */
      grlib_write_uart (addr, data);
      break;

    case IRQCTRL_IFR:		/* 0x208 */
      irqctrl_ifr = data & 0xfffe;
      chk_irq ();
      break;

    case IRQCTRL_ICR:		/* 0x20C */
      irqctrl_ipr &= ~data & 0x0fffe;
      chk_irq ();
      break;

    case IRQCTRL_IMR:		/* 0x240 */
      irqctrl_imr = data & 0x7ffe;
      chk_irq ();
      break;

    case TIMER_SCLOAD:		/* 0x304 */
      gpt_scaler_set (data);
      break;

    case TIMER_TIMER1:		/* 0x310 */
      gpt_counter[0] = data;
      break;

    case TIMER_RELOAD1:	/* 0x314 */
      gpt_reload[0] = data;
      break;

    case TIMER_CTRL1:		/* 0x318 */
      timer_ctrl (data, 0);
      break;

    case TIMER_TIMER2:		/* 0x320 */
      gpt_counter[1] = data;
      break;

    case TIMER_RELOAD2:	/* 0x324 */
      gpt_reload[1] = data;
      break;

    case TIMER_CTRL2:		/* 0x328 */
      timer_ctrl (data, 1);
      break;

    case POWER_DOWN:		/* 0x328 */
      wait_for_irq ();
      break;

    case CACHE_CTRL:		/* 0x328 */
      cache_ctrl = data & 0x1000f;
      break;

    default:
      break;
    }
  return MOK;
}


/* APBUART. */

static int ifd1 = -1, ofd1 = -1;

static void
init_stdio (void)
{
  if (dumbio)
    return;
#ifdef HAVE_TERMIOS_H
  if (ifd1 == 0 && f1open)
    {
      tcsetattr (0, TCSANOW, &ioc1);
      tcflush (ifd1, TCIFLUSH);
    }
#endif
}

static void
restore_stdio (void)
{
  if (dumbio)
    return;
#ifdef HAVE_TERMIOS_H
  if (ifd1 == 0 && f1open && tty_setup)
    tcsetattr (0, TCSANOW, &iocold1);
#endif
}

#define DO_STDIO_READ( _fd_, _buf_, _len_ )          \
		( dumbio || nouartrx \
		? (0) /* no bytes read, no delay */   \
          : read( _fd_, _buf_, _len_ ) )

static void
port_init (void)
{
  f1in = stdin;
  f1out = stdout;
  if (uart_dev1[0] != 0)
    if ((fd1 = open (uart_dev1, O_RDWR | O_NONBLOCK)) < 0)
      printf ("Warning, couldn't open output device %s\n", uart_dev1);
    else
      {
	if (sis_verbose)
	  printf ("serial port A on %s\n", uart_dev1);
	f1in = f1out = fdopen (fd1, "r+");
	setbuf (f1out, NULL);
	f1open = 1;
      }
  if (f1in)
    ifd1 = fileno (f1in);
  if (ifd1 == 0)
    {
    /*   if (callback && !callback->isatty (callback, ifd1)) */
	/* tty_setup = 0; */
      if (sis_verbose)
	printf ("serial port A on stdin/stdout\n");
      if (!dumbio)
	{
#ifdef HAVE_TERMIOS_H
	  tcgetattr (ifd1, &ioc1);
	  if (tty_setup)
	    {
	      iocold1 = ioc1;
	      ioc1.c_lflag &= ~(ICANON | ECHO);
	      ioc1.c_cc[VMIN] = 0;
	      ioc1.c_cc[VTIME] = 0;
	    }
#endif
	}
      f1open = 1;
    }

  if (f1out)
    {
      ofd1 = fileno (f1out);
      if (!dumbio && tty_setup && ofd1 == 1)
	setbuf (f1out, NULL);
    }

  wnuma = 0;
}

static uint32
grlib_read_uart (uint32 addr)
{

  unsigned tmp = 0;

  switch (addr & 0xfff)
    {

    case 0x070:		/* UART 1 RX/TX */
#ifndef _WIN32
#ifdef FAST_UART
      if (aind < anum)
	{
	  if ((aind + 1) < anum)
	    set_irq (3);
	  return (uint32) aq[aind++];
	}
      else
	{
	  if (f1open)
	    anum = DO_STDIO_READ (ifd1, aq, UARTBUF);
	  else
	    anum = 0;
	  if (anum > 0)
	    {
	      aind = 0;
	      if ((aind + 1) < anum)
		set_irq (3);
	      return (uint32) aq[aind++];
	    }
	  else
	    return (uint32) aq[aind];
	}
#else
      tmp = uarta_data;
      /* uarta_data &= ~UART_DR; */
      /* uart_stat_reg &= ~UARTA_DR; */
      return tmp;
#endif
#else
      return 0;
#endif
      break;

    case 0x074:		/* UART status register  */
#ifndef _WIN32
#ifdef FAST_UART

      Ucontrol = 0;
      if (aind < anum)
	Ucontrol |= 0x00000001;
      else
	{
	  if (f1open)
	    anum = DO_STDIO_READ (ifd1, aq, UARTBUF);
	  else
	    anum = 0;
	  if (anum > 0)
	    {
	      Ucontrol |= 0x00000001;
	      aind = 0;
	      set_irq (3);
	    }
	}
      Ucontrol |= 0x00000006;
      return Ucontrol;
#else
      return uart_stat_reg;
#endif
#else
      return 0x00060006;
#endif
      break;
    default:
      if (sis_verbose)
	printf ("Read from unimplemented LEON2 register (%x)\n", addr);

    }
  return 0;
}

static void
grlib_write_uart (uint32 addr, uint32 data)
{
  unsigned char c;

  c = (unsigned char) data;
  switch (addr & 0xfff)
    {

    case 0x070:		/* UART A */
#ifdef FAST_UART
      if (f1open)
	{
	  if (wnuma < UARTBUF)
	    wbufa[wnuma++] = c;
	  else
	    {
	      while (wnuma)
		{
		  /* if (ofd1 == 1 && callback) */
		  /*   wnuma -= callback->write_stdout (callback, wbufa, wnuma); */
		  /* else */
		    wnuma -= fwrite (wbufa, 1, wnuma, f1out);
		}
	      wbufa[wnuma++] = c;
	    }
	}
      set_irq (3);
#else
      if (uart_stat_reg & UARTA_SRE)
	{
	  uarta_sreg = c;
	  uart_stat_reg &= ~UARTA_SRE;
	  event (uarta_tx, 0, UART_TX_TIME);
	}
      else
	{
	  uarta_hreg = c;
	  uart_stat_reg &= ~UARTA_HRE;
	}
#endif
      break;

    case 0x074:		/* UART status register */
#ifndef FAST_UART
      uart_stat_reg &= 1;
#endif
      break;
    default:
      if (sis_verbose)
	printf ("Write to unimplemented APB register (%x)\n", addr);

    }
}

static void
flush_uart (void)
{
  while (wnuma && f1open)
    {
    /*   if (ofd1 == 1 && callback) */
	/* { */
	/*   wnuma -= callback->write_stdout (callback, wbufa, wnuma); */
	/*   callback->flush_stdout (callback); */
	/* } */
    /*   else */
	wnuma -= fwrite (wbufa, 1, wnuma, f1out);
    }
}

static void
uarta_tx (void)
{
  while (f1open)
    {
    /*   if (ofd1 == 1 && callback) */
	/* while (callback->write_stdout (callback, &uarta_sreg, 1) != 1) */
	/*   continue; */
    /*   else */
	while (fwrite (&uarta_sreg, 1, 1, f1out) != 1)
	  continue;
    }
  if (uart_stat_reg & UARTA_HRE)
    {
      uart_stat_reg |= UARTA_SRE;
    }
  else
    {
      uarta_sreg = uarta_hreg;
      uart_stat_reg |= UARTA_HRE;
      event (uarta_tx, 0, UART_TX_TIME);
    }
  set_irq (3);
}

static void
uart_rx (int32 arg)
{
  char rxd;
  int32 rsize = 0;

  if (f1open)
    rsize = DO_STDIO_READ (ifd1, &rxd, 1);
  else
    rsize = 0;
  if (rsize > 0)
    {
      uarta_data = rxd;
      if (uart_stat_reg & UARTA_DR)
	{
	  uart_stat_reg |= UARTA_OR;
	}
      uart_stat_reg |= UARTA_DR;
      set_irq (3);
    }
  event (uart_rx, 0, UART_RX_TIME);
}

static void
uart_intr (int32 arg)
{
  /* Check for UART interrupts every 1000 clk.  */
  grlib_read_uart (APBUART_STATUS);
  flush_uart ();
  event (uart_intr, 0, UART_FLUSH_TIME);
}


static void
uart_irq_start (void)
{
#ifdef FAST_UART
  event (uart_intr, 0, UART_FLUSH_TIME);
#else
#ifndef _WIN32
  event (uart_rx, 0, UART_RX_TIME);
#endif
#endif
}

/* TIMER */

static void
gpt_intr (int32 arg)
{
  int i;

  for (i = 0; i < NTIMERS; i++)
    {
      if (gpt_ctrl[i] & 1)
	{
	  gpt_counter[i] -= 1;
	  if (gpt_counter[i] == -1)
	    {
	      set_irq (TIMER_IRQ + i);
	      if (gpt_ctrl[i] & 2)
		gpt_counter[i] = gpt_reload[i];
	    }
	}
    }
  event (gpt_intr, 0, gpt_scaler + 1);
  gpt_scaler_start = now ();
}

static void
gpt_init (void)
{
  if (sis_verbose)
    printf ("GPT started (period %d)\n\r", gpt_scaler + 1);
}

static void
gpt_reset (void)
{
  event (gpt_intr, 0, gpt_scaler + 1);
  gpt_scaler_start = now ();
}

static void
gpt_scaler_set (uint32 val)
{
  /* Mask for 16-bit scaler. */
  gpt_scaler = val & 0x0ffff;
}

static void
timer_ctrl (uint32 val, int i)
{
  if (val & 4)
    {
      /* Reload.  */
      gpt_counter[i] = gpt_reload[i];
    }
  gpt_ctrl[i] = val & 0xb;
}

/* Store data in host byte order.  MEM points to the beginning of the
   emulated memory; WADDR contains the index the emulated memory,
   DATA points to words in host byte order to be stored.  SZ contains log(2)
   of the number of bytes to retrieve, and can be 0 (1 byte), 1 (one half-word),
   2 (one word), or 3 (two words); WS should return the number of wait-states. */

static void
store_bytes (unsigned char *mem, uint32 waddr, uint32 * data, int32 sz,
	     int32 * ws)
{
  switch (sz)
    {
    case 0:
#ifdef HOST_LITTLE_ENDIAN
      waddr ^= EBT;
#endif
      mem[waddr] = *data & 0x0ff;
      *ws = 0;
      break;
    case 1:
#ifdef HOST_LITTLE_ENDIAN
      waddr ^= 2;
#endif
      memcpy (&mem[waddr], data, 2);
      *ws = 0;
      break;
    case 2:
      memcpy (&mem[waddr], data, 4);
      *ws = 0;
      break;
    case 3:
      memcpy (&mem[waddr], data, 8);
      *ws = 0;
      break;
    }
}


/* Memory emulation.  */

static int
memory_iread (uint32 addr, uint32 * data, int32 * ws)
{
  if ((addr >= RAM_START) && (addr < RAM_END))
    {
      memcpy (data, &ramb[addr & RAM_MASK], 4);
      *ws = 0;
      return 0;
    }
  else if (addr < ROM_END)
    {
      memcpy (data, &romb[addr], 4);
      *ws = 0;
      return 0;
    }

  if (sis_verbose)
    printf ("Memory exception at %x (illegal address)\n", addr);
  *ws = MEM_EX_WS;
  return 1;
}

static int
memory_read (uint32 addr, uint32 * data, int32 sz, int32 * ws)
{
  int32 mexc;

  if ((addr >= RAM_START) && (addr < RAM_END))
    {
      memcpy (data, &ramb[addr & RAM_MASK & ~3], 4);
      *ws = 0;
      return 0;
    }
  else if ((addr >= APBSTART) && (addr < APBEND))
    {
      mexc = apb_read (addr, data);
      if (mexc)
	*ws = MEM_EX_WS;
      else
	*ws = 0;
      return mexc;
    }
  else if (addr < ROM_END)
    {
      memcpy (data, &romb[addr & ~3], 4);
      *ws = 0;
      return 0;
    }

  if (sis_verbose)
    printf ("Memory exception at %x (illegal address)\n", addr);
  *ws = MEM_EX_WS;
  return 1;
}

static int
memory_read_asi (int32 asi, uint32 addr, uint32 * data, int32 sz, int32 * ws)
{
  return memory_read (addr, data, sz, ws);
}

static int
memory_write (uint32 addr, uint32 * data, int32 sz, int32 * ws)
{
  uint32 byte_addr;
  uint32 byte_mask;
  uint32 waddr;
  uint32 *ram;
  int32 mexc;
  int i;
  int wphit[2];

  if ((addr >= RAM_START) && (addr < RAM_END))
    {
      waddr = addr & RAM_MASK;
      store_bytes (ramb, waddr, data, sz, ws);
      return 0;
    }
  else if ((addr >= APBSTART) && (addr < APBEND))
    {
      if (sz != 2)
	{
	  *ws = MEM_EX_WS;
	  return 1;
	}
      apb_write (addr, *data);
      *ws = 0;
      return 0;
    }
  else if (addr < ROM_END)
    {
      *ws = 0;
      store_bytes (romb, addr, data, sz, ws);
      return 0;
    }

  *ws = MEM_EX_WS;
  return 1;
}

static int
memory_write_asi (int32 asi, uint32 addr, uint32 * data, int32 sz, int32 * ws)
{
  return memory_write (addr, data, sz, ws);
}

static unsigned char *
get_mem_ptr (uint32 addr, uint32 size)
{
  if ((addr + size) < ROM_END)
    {
      return &romb[addr];
    }
  else if ((addr >= RAM_START) && ((addr + size) < RAM_END))
    {
      return &ramb[addr & RAM_MASK];
    }

  return (char *) -1;
}

static int
sis_memory_write (uint32 addr, const unsigned char *data, uint32 length)
{
  char *mem;

  if ((mem = get_mem_ptr (addr, length)) == ((char *) -1))
    return 0;

  memcpy (mem, data, length);
  return length;
}

static int
sis_memory_read (uint32 addr, char *data, uint32 length)
{
  char *mem;
  int ws;
  int w4;

  if (length == 4)
    {
      memory_read (addr, &w4, length, &ws);
      memcpy (data, &w4, length);
      return 4;
    }

  if ((mem = get_mem_ptr (addr, length)) == ((char *) -1))
    return 0;

  memcpy (data, mem, length);
  return length;
}

static void
boot_init (void)
{
  /* Generate 1 MHz RTC tick.  */
  apb_write (TIMER_SCALER, sregs.freq - 1);
  apb_write (TIMER_SCLOAD, sregs.freq - 1);
  apb_write (TIMER_TIMER1, -1);
  apb_write (TIMER_RELOAD1, -1);
  apb_write (TIMER_CTRL1, 0x7);

  sregs.wim = 2;
  sregs.psr = 0x000010e0;
  sregs.r[30] = RAM_END;
  sregs.r[14] = sregs.r[30] - 96 * 4;
  cache_ctrl = 0x01000f;
}

const struct memsys leon2 = {
  init_sim,
  reset,
  error_mode,
  sim_halt,
  exit_sim,
  init_stdio,
  restore_stdio,
  memory_iread,
  memory_read,
  memory_read_asi,
  memory_write,
  memory_write_asi,
  sis_memory_write,
  sis_memory_read,
  boot_init
};
