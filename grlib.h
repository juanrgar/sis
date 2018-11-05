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


/* Definitions for AMBA PNP in Gaisler Research GRLIB SOC */

/* Vendors */

#define VENDOR_GAISLER	1
#define VENDOR_PENDER	2
#define VENDOR_ESA	4
#define VENDOR_DLR	10

/* Devices */

#define GAISLER_LEON3	0x003
#define GAISLER_APBMST	0x006
#define GAISLER_SRCTRL	0x008
#define GAISLER_APBUART	0x00C
#define GAISLER_IRQMP	0x00D
#define GAISLER_GPTIMER	0x011
#define ESA_MCTRL	0x00F

/* How to build entries in the plug&play area */

#define GRLIB_PP_ID(v, d, x, i) ((v & 0xff) << 24) | ((d & 0x3ff) << 12) |\
			((x & 0x1f) << 5) | (i & 0x1f)
#define GRLIB_PP_AHBADDR(a, m, p, c, t) (a & 0xfff00000) | ((m & 0xfff) << 4) |\
			((p & 1) << 17) | ((c & 1) << 16) | (t & 0x3)
#define GRLIB_PP_APBADDR(a, m) ((a & 0xfff00)<< 12) | ((m & 0xfff) << 4) | 1

#define AHBPP_START	0xFFFFF000
#define AHBPP_END	0xFFFFFFFF
#define APBPP_START	0x800FF000
#define APBPP_END	0x800FFFFF

extern int grlib_apbpp_add (uint32 id, uint32 addr);
extern int grlib_ahbmpp_add (uint32 id);
extern int grlib_ahbspp_add (uint32 id, uint32 addr1, uint32 addr2,
			     uint32 addr3, uint32 addr4);
extern uint32 grlib_ahbpnp_read (uint32 addr);
extern uint32 grlib_apbpnp_read (uint32 addr);
extern void grlib_init ();
