/* This file is part of SIS (SPARC instruction simulator)

   Copyright (C) 1995-2017 Free Software Foundation, Inc.
   Contributed by Jiri Gaisler, European Space Agency

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

#include "config.h"
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef HAVE_FCNTL_H
#include <fcntl.h>
#endif
#include "sis.h"
#include "libiberty.h"
#include "bfd.h"
#include <dis-asm.h>
#include "sim-config.h"

#include "gdb/remote-sim.h"
#include "gdb/signals.h"
#include "gdb/common/break-common.h"

#define PSR_CWP 0x7

int
run_sim(sregs, icount, dis)
    struct pstate  *sregs;
    uint64          icount;
    int             dis;
{
    int             mexc, irq;

    if (sis_verbose)
	(*sim_callback->printf_filtered) (sim_callback, "resuming at %x\n",
					  sregs->pc);
   ms->init_stdio ();
   sregs->starttime = get_time();
   irq = 0;
   if (sregs->err_mode) icount = 0;
   if ((sregs->pc != 0) && (ebase.simtime == 0))
	ms->boot_init ();
   while (icount > 0) {
	sregs->fhold = 0;
	sregs->icnt = 1;
        mexc = ms->memory_iread (sregs->pc, &sregs->inst, &sregs->hold);
        if (sregs->annul) {
            sregs->annul = 0;
            sregs->pc = sregs->npc;
            sregs->npc = sregs->npc + 4;
        } else {
	    if (ext_irl) irq = check_interrupts(sregs);
	    if (!irq) {
		if (mexc) {
		    sregs->trap = I_ACC_EXC;
		} else {
		    dispatch_instruction(sregs);
		    icount--;
		}
	    }
	    if (sregs->trap) {
                irq = 0;
		if ((sregs->err_mode = execute_trap(sregs)) == WPT_HIT) {
		    sregs->err_mode = 0;
		    sregs->trap = 0;
		    icount = 0;
		}
		if (sregs->err_mode) icount = 0;
	    }
	}
	advance_time(sregs);
	if (ctrl_c)
	    icount = 0;
    }
    ms->sim_halt ();
    sregs->tottime += get_time() - sregs->starttime;
    ms->restore_stdio ();
    clearerr(stdin);
    if (sregs->err_mode) {
	ms->error_mode (sregs->pc);
	return ERROR;
    }
    if (ctrl_c) {
	ctrl_c = 0;
	sregs->wphit = sregs->bphit = 0;
	return CTRL_C;
    }
    if ((sregs->bphit) || (sregs->wphit))
	return (BPT_HIT);
    return TIME_OUT;
}

SIM_DESC
sim_open (kind, callback, abfd, argv)
     SIM_OPEN_KIND kind;
     struct host_callback_struct *callback;
     struct bfd *abfd;
     char * const *argv;
{

    int             argc = 0;
    int             stat = 1;
    int             freq = 0;

    sim_callback = callback;
    sis_gdb_break = 1;

    sis_gdb_break = 1;
    argc = countargv (argv);
    while (stat < argc) {
	if (argv[stat][0] == '-') {
	    if (strcmp(argv[stat], "-v") == 0) {
		sis_verbose++;
	    } else
	    if (strcmp(argv[stat], "-nfp") == 0) {
		nfp = 1;
	    } else
            if (strcmp(argv[stat], "-ift") == 0) {
                ift = 1;
	    } else
	    if (strcmp(argv[stat], "-sparclite") == 0) {
		sparclite = 1;
	    } else
	    if (strcmp(argv[stat], "-sparclite-board") == 0) {
		sparclite_board = 1;
            } else 
            if (strcmp(argv[stat], "-dumbio") == 0) {
		dumbio = 1;
	    } else
            if (strcmp(argv[stat], "-nouartrx") == 0) {
		nouartrx = 1;
	    } else
	    if (strcmp (argv[stat], "-leon2") == 0) {
		ms = &leon2;
		cputype = CPU_LEON2;
	    } else
            if (strcmp (argv[stat], "-leon3") == 0) {
		ms = &leon3;
                cputype = CPU_LEON3;
            } else
            if (strcmp(argv[stat], "-wrp") == 0) {
                wrp = 1;
	    } else
            if (strcmp(argv[stat], "-rom8") == 0) {
                rom8 = 1;
	    } else 
            if (strcmp(argv[stat], "-uben") == 0) {
                uben = 1;
	    } else 
	    if (strcmp(argv[stat], "-uart1") == 0) {
		if ((stat + 1) < argc)
		    strcpy(uart_dev1, argv[++stat]);
	    } else
	    if (strcmp(argv[stat], "-uart2") == 0) {
		if ((stat + 1) < argc)
		    strcpy(uart_dev2, argv[++stat]);
	    } else
	    if (strcmp(argv[stat], "-nogdb") == 0) {
		sis_gdb_break = 0;
	    } else
	    if (strcmp(argv[stat], "-freq") == 0) {
		if ((stat + 1) < argc) {
		    freq = strtol(argv[++stat], (char **)NULL, 0);
		}
	    } else
	    if (strncmp(argv[stat], "--sysroot=", sizeof("--sysroot=") - 1) == 0) {
		/* Ignore until we start to support this.  */
	    } else {
		(*sim_callback->printf_filtered) (sim_callback,
						  "unknown option %s\n",
						  argv[stat]);
	    }
	} else
	    bfd_load(argv[stat]);
	stat++;
    }

    if ((cputype == CPU_LEON3) || (cputype == CPU_LEON2))
        sregs.freq = freq ? freq : 50;
    else
        sregs.freq = freq ? freq : 14;

    if (sis_verbose) {
	(*sim_callback->printf_filtered) (sim_callback, "\n SIS - SPARC instruction simulator %s\n", sis_version);
	(*sim_callback->printf_filtered) (sim_callback, " Bug-reports to Jiri Gaisler (jiri@gaisler.se)\n\n");
        switch (cputype) {
        case CPU_LEON2:
            (*sim_callback->printf_filtered) (sim_callback, "LEON2 emulation enabled\n");
            break;
        case CPU_LEON3:
            (*sim_callback->printf_filtered) (sim_callback, "LEON3 emulation enabled\n");
            break;
        default:
            (*sim_callback->printf_filtered) (sim_callback, "ERC32 emulation enabled\n");
        }
	if (nfp)
	  (*sim_callback->printf_filtered) (sim_callback, "no FPU\n");
	if (sparclite)
	  (*sim_callback->printf_filtered) (sim_callback, "simulating Sparclite\n");
	if (dumbio)
	  (*sim_callback->printf_filtered) (sim_callback, "dumb IO (no input, dumb output)\n");
	if (sis_gdb_break == 0)
	  (*sim_callback->printf_filtered) (sim_callback, "disabling GDB trap handling for breakpoints\n");
	  (*sim_callback->printf_filtered) (sim_callback, "CPU freq %3.1f MHz\n", sregs.freq);
    }

#ifdef F_GETFL
    termsave = fcntl(0, F_GETFL, 0);
#endif
    INIT_DISASSEMBLE_INFO(dinfo, stdout,(fprintf_ftype)fprintf);
#ifdef HOST_LITTLE_ENDIAN
    dinfo.endian = BFD_ENDIAN_LITTLE;
#else
    dinfo.endian = BFD_ENDIAN_BIG;
#endif
    reset_all();
    ebase.simtime = 0;
    ms->init_sim ();
    init_bpt(&sregs);
    reset_stat(&sregs);

    /* Fudge our descriptor for now.  */
    return (SIM_DESC) 1;
}

void
sim_close(sd, quitting)
     SIM_DESC sd;
     int quitting;
{

    ms->exit_sim ();
#if defined(F_GETFL) && defined(F_SETFL)
    fcntl(0, F_SETFL, termsave);
#endif

};

SIM_RC
sim_load(sd, prog, abfd, from_tty)
     SIM_DESC sd;
     const char *prog;
     bfd *abfd;
     int from_tty;
{
    bfd_load (prog);
    return SIM_RC_OK;
}

SIM_RC
sim_create_inferior(sd, abfd, argv, env)
     SIM_DESC sd;
     struct bfd *abfd;
     char * const *argv;
     char * const *env;
{
    bfd_vma start_address = 0;
    if (abfd != NULL)
      start_address = bfd_get_start_address (abfd);

    ebase.simtime = 0;
    reset_all();
    reset_stat(&sregs);
    sregs.pc = start_address & ~3;
    sregs.npc = sregs.pc + 4;
    return SIM_RC_OK;
}

int
sim_store_register(sd, regno, value, length)
    SIM_DESC sd;
    int             regno;
    unsigned char  *value;
    int length;
{
    int regval;

    regval = (value[0] << 24) | (value[1] << 16)
		 | (value[2] << 8) | value[3];
    set_regi(&sregs, regno, regval);
    return length;
}


int
sim_fetch_register(sd, regno, buf, length)
     SIM_DESC sd;
    int             regno;
    unsigned char  *buf;
     int length;
{
    get_regi(&sregs, regno, buf);
    return -1;
}

int
sim_write (SIM_DESC sd, SIM_ADDR mem, const unsigned char *buf, int length)
{
    int i, len;

    for (i = 0; i < length; i++) {
	ms->sis_memory_write ((mem + i) ^ EBT, &buf[i], 1);
    }
    return length;
}

int
sim_read (SIM_DESC sd, SIM_ADDR mem, unsigned char *buf, int length)
{
    int i, len;

    for (i = 0; i < length; i++) {
	ms->sis_memory_read ((mem + i) ^ EBT, &buf[i], 1);
    }
    return length;
}

void
sim_info(sd, verbose)
     SIM_DESC sd;
     int verbose;
{
    show_stat(&sregs);
}

int             simstat = OK;

void
sim_stop_reason(sd, reason, sigrc)
     SIM_DESC sd;
     enum sim_stop * reason;
     int *sigrc;
{

    switch (simstat) {
    case CTRL_C:
	*reason = sim_stopped;
	*sigrc = GDB_SIGNAL_INT;
	break;
    case OK:
    case TIME_OUT:
	*reason = sim_stopped;
	*sigrc = 0;
	break;
    case BPT_HIT:
	*reason = sim_stopped;
	*sigrc = GDB_SIGNAL_TRAP;
	break;
    case ERROR:
	*sigrc = 0;
	*reason = sim_exited;
    }

    if (sis_verbose)
	(*sim_callback->printf_filtered) (sim_callback,
	    "sim_stop_reason %x : %x\n", *reason, *sigrc);
}

/* Flush all register windows out to the stack.  Starting after the invalid
   window, flush all windows up to, and including the current window.  This
   allows GDB to do backtraces and look at local variables for frames that
   are still in the register windows.  Note that strictly speaking, this
   behavior is *wrong* for several reasons.  First, it doesn't use the window
   overflow handlers.  It therefore assumes standard frame layouts and window
   handling policies.  Second, it changes system state behind the back of the
   target program.  I expect this to mainly pose problems when debugging trap
   handlers.
*/

static void
flush_windows ()
{
  int invwin;
  int cwp;
  int win;
  int ws;

  /* Keep current window handy */

  cwp = sregs.psr & PSR_CWP;

  /* Calculate the invalid window from the wim. */

  for (invwin = 0; invwin <= PSR_CWP; invwin++)
    if ((sregs.wim >> invwin) & 1)
      break;

  /* Start saving with the window after the invalid window. */

  invwin = (invwin - 1) & PSR_CWP;

  for (win = invwin; ; win = (win - 1) & PSR_CWP)
    {
      uint32 sp;
      int i;

      sp = sregs.r[(win * 16 + 14) & 0x7f];
#if 1
      if (sis_verbose > 2) {
	uint32 fp = sregs.r[(win * 16 + 30) & 0x7f];
	printf("flush_window: win %d, sp %x, fp %x\n", win, sp, fp);
      }
#endif

      for (i = 0; i < 16; i++)
	ms->memory_write (sp + 4 * i, &sregs.r[(win * 16 + 16 + i) & 0x7f], 2,
		      &ws);

      if (win == cwp)
	break;
    }
}

void
sim_resume(SIM_DESC sd, int step, int siggnal)
{
    if (sis_verbose)
	(*sim_callback->printf_filtered) (sim_callback,
	    "sim_resume %x : %x : %x : %x : 0x%08x\n", step, siggnal, sregs.bphit, sregs.wphit, sregs.pc);
    if (step) {
	sregs.bphit = 0;
	sregs.wphit = 1;
        simstat = run_sim(&sregs, 1, 0);
	sregs.bphit = 0;
	sregs.wphit = 0;
    } else if (sregs.bphit || sregs.wphit) {
	sregs.bphit = 0;
	sregs.wphit = 1;
        simstat = run_sim(&sregs, 1, 0);
	sregs.bphit = sregs.wphit = 0;
        simstat = run_sim(&sregs, UINT64_MAX, 0);
	sregs.bphit = 0;
    }
    else
        simstat = run_sim(&sregs, UINT64_MAX, 0);

    if (sis_gdb_break) flush_windows ();
}

void
sim_do_command(sd, cmd)
     SIM_DESC sd;
     const char *cmd;
{
    exec_cmd(&sregs, cmd);
}

char **
sim_complete_command (SIM_DESC sd, const char *text, const char *word)
{
  return NULL;
}

int
sim_stop (SIM_DESC sd)
{
  ctrl_c = 1;
  return 1;
}

static int
sis_insert_watchpoint_read(int addr, unsigned char mask)
{
    if (sregs.wprnum < WPR_MAX) {
	sregs.wprs[sregs.wprnum] = addr;
	sregs.wprm[sregs.wprnum] = mask;
	sregs.wprnum++;
	if (sis_verbose)
	    (*sim_callback->printf_filtered) (sim_callback, "inserted read watchpoint at %x\n", addr);
	return SIM_RC_OK;
    } else
	return SIM_RC_FAIL;
}

static int
sis_remove_watchpoint_read(int addr)
{
    int             i = 0;

    while ((i < sregs.wprnum) && (sregs.wprs[i] != addr))
	i++;
    if (addr == sregs.wprs[i]) {
	for (; i < sregs.wprnum - 1; i++)
	    sregs.wprs[i] = sregs.wprs[i + 1];
	sregs.wprnum -= 1;
	if (sis_verbose)
	    (*sim_callback->printf_filtered) (sim_callback, "removed read watchpoint at %x\n", addr);
	return 0;
    }
    return 1;
}

static int
sis_insert_watchpoint_write(int32 addr, unsigned char mask)
{
    if (sregs.wpwnum < WPR_MAX) {
	sregs.wpws[sregs.wpwnum] = addr;
	sregs.wpwm[sregs.wpwnum] = mask;
	sregs.wpwnum++;
	if (sis_verbose)
	    (*sim_callback->printf_filtered) (sim_callback, "sim_insert_watchpoint_write: 0x%08x : %x\n", addr, mask);
	return SIM_RC_OK;
    } else
	return SIM_RC_FAIL;
}

static int
sis_remove_watchpoint_write(int addr)
{
    int             i = 0;

    while ((i < sregs.wpwnum) && (sregs.wpws[i] != addr))
	i++;
    if (addr == sregs.wpws[i]) {
	for (; i < sregs.wpwnum - 1; i++)
	    sregs.wpws[i] = sregs.wpws[i + 1];
	sregs.wpwnum -= 1;
	if (sis_verbose)
	    (*sim_callback->printf_filtered) (sim_callback, "removed write watchpoint at %x\n", addr);
	return SIM_RC_OK;
    }
    return SIM_RC_FAIL;
}

int sim_can_use_hw_breakpoint (SIM_DESC sd, int type, int cnt, int othertype)
{
    if (type == 2)	/* bp_hardware_breakpoint not supported */
        return 0;
     else
	return 1;
}


int sim_set_watchpoint (SIM_DESC sd, SIM_ADDR mem, int length, int type)
{
  int res;
  unsigned char mask;

  switch (length) {
  case 1:  mask = 0; break;
  case 2:  mask = 1; break;
  case 4:  mask = 3; break;
  default: mask = 7; break;
  }

  switch (type) {
    case 0:
      res = sis_insert_watchpoint_write (mem, mask);
      break;
    case 1:
      res = sis_insert_watchpoint_read (mem, mask);
      break;
    case 2:
      if ((res = sis_insert_watchpoint_write (mem, mask)) == SIM_RC_OK)
          res = sis_insert_watchpoint_read (mem, mask);
	  if (res == SIM_RC_FAIL)
	      sis_remove_watchpoint_read (mem);
      break;
    default:
      res = -1;
  }
  return (res);
}


int sim_clear_watchpoint (SIM_DESC sd, SIM_ADDR mem, int length, int type)
{
  int res;
  switch (type) {
    case 0:
      res = sis_remove_watchpoint_write (mem);
      break;
    case 1:
      res = sis_remove_watchpoint_read (mem);
      break;
    case 2:
      if ((res = sis_remove_watchpoint_write (mem)) == SIM_RC_OK)
          res = sis_remove_watchpoint_read (mem);
      else
          sis_remove_watchpoint_read (mem);
      break;
    default:
      res = -1;
  }
  return (res);
}

int sim_stopped_by_watchpoint (SIM_DESC sd)
{
    if (sis_verbose)
       (*sim_callback->printf_filtered) (sim_callback, "sim_stopped_by_watchpoint %x\n", sregs.wphit);
    return((sregs.wphit != 0));
}

int sim_watchpoint_address (SIM_DESC sd)
{
    return(sregs.wpaddress);
}

#if 0 /* FIXME: These shouldn't exist.  */

int
sim_insert_breakpoint(int addr)
{
    if (sregs.bptnum < BPT_MAX) {
	sregs.bpts[sregs.bptnum] = addr & ~0x3;
	sregs.bptnum++;
	if (sis_verbose)
	    (*sim_callback->printf_filtered) (sim_callback, "inserted HW BP at %x\n", addr);
	return 0;
    } else
	return 1;
}

int
sim_remove_breakpoint(int addr)
{
    int             i = 0;

    while ((i < sregs.bptnum) && (sregs.bpts[i] != addr))
	i++;
    if (addr == sregs.bpts[i]) {
	for (; i < sregs.bptnum - 1; i++)
	    sregs.bpts[i] = sregs.bpts[i + 1];
	sregs.bptnum -= 1;
	if (sis_verbose)
	    (*sim_callback->printf_filtered) (sim_callback, "removed HW BP at %x\n", addr);
	return 0;
    }
    return 1;
}

#endif
