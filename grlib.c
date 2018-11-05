/*
 * This file is part of SIS.
 *
 * SIS, SPARC instruction simulator. Copyright (C) 2014 Jiri Gaisler
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
 */


#include "sis.h"
#include "grlib.h"


/* APB PNP */

static uint32 apbppmem[32 * 2];	/* 32-entry APB PP AREA */
static int apbppindex;

int
grlib_apbpp_add (uint32 id, uint32 addr)
{
  apbppmem[apbppindex++] = id;
  apbppmem[apbppindex++] = addr;
  if (apbppindex >= (32 * 2))
    apbppindex = 0;		/* prevent overflow of area */
  return apbppindex;
}

uint32
grlib_apbpnp_read (uint32 addr)
{
  uint32 read_data;
  addr &= 0xff;
  read_data = apbppmem[addr >> 2];

  return read_data;
}

/* AHB PNP */

static uint32 ahbppmem[128 * 8];	/* 128-entry AHB PP AREA */
static int ahbmppindex;
static int ahbsppindex = 64 * 8;

int
grlib_ahbmpp_add (uint32 id)
{
  ahbppmem[ahbmppindex] = id;
  ahbmppindex += 8;
  if (ahbmppindex >= (64 * 8))
    ahbmppindex = 0;		/* prevent overflow of area */
  return ahbmppindex;
}

int
grlib_ahbspp_add (uint32 id, uint32 addr1, uint32 addr2,
		  uint32 addr3, uint32 addr4)
{
  ahbppmem[ahbsppindex] = id;
  ahbsppindex += 4;
  ahbppmem[ahbsppindex++] = addr1;
  ahbppmem[ahbsppindex++] = addr2;
  ahbppmem[ahbsppindex++] = addr3;
  ahbppmem[ahbsppindex++] = addr4;
  if (ahbsppindex >= (128 * 8))
    ahbsppindex = 64 * 8;	/* prevent overflow of area */
  return ahbsppindex;
}

uint32
grlib_ahbpnp_read (uint32 addr)
{
  uint32 read_data;

  addr &= 0xfff;
  read_data = ahbppmem[addr >> 2];
  return read_data;

}

void
grlib_init ()
{
  /* Add PP records for Leon3, APB bridge and interrupt controller
     as this is not done elsewhere */

  grlib_ahbmpp_add (GRLIB_PP_ID (VENDOR_GAISLER, GAISLER_LEON3, 0, 0));
  grlib_ahbspp_add (GRLIB_PP_ID (VENDOR_GAISLER, GAISLER_APBMST, 0, 0),
		    GRLIB_PP_AHBADDR (0x80000000, 0xFFF, 0, 0, 2), 0, 0, 0);

  grlib_apbpp_add (GRLIB_PP_ID (VENDOR_GAISLER, GAISLER_IRQMP, 2, 0),
		   GRLIB_PP_APBADDR (0x80000200, 0xFFF));

}
