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
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#include <stdio.h>
#include <fcntl.h>
#include "sis.h"
#include <dis-asm.h>
/* #include "sim-config.h" */
#include <inttypes.h>

#define	VAL(x)	strtol(x,(char **)NULL,0)

/* Structures and functions from readline library */

#include "readline/readline.h"
#include "readline/history.h"

/* Command history buffer length - MUST be binary */
#define HIST_LEN	64

#ifdef ERA
extern int era;
#endif

int
run_sim(sregs, icount, dis)
    struct pstate  *sregs;
    uint64          icount;
    int             dis;
{
    int             irq, mexc, deb;

    sregs->starttime = get_time();
    ms->init_stdio ();
    if (sregs->err_mode) icount = 0;
    deb = dis || sregs->histlen || sregs->bptnum;
    irq = 0;
    while (icount > 0) {

	mexc = ms->memory_iread (sregs->pc, &sregs->inst, &sregs->hold);
	sregs->icnt = 1;
	if (sregs->annul) {
	    sregs->annul = 0;
	    sregs->pc = sregs->npc;
	    sregs->npc = sregs->npc + 4;
	} else {
	    sregs->fhold = 0;
	    if (ext_irl) irq = check_interrupts(sregs);
	    if (!irq) {
		if (mexc) {
		    sregs->trap = I_ACC_EXC;
		} else {
		    if (deb) {
			if (sregs->histlen) {
			    sregs->histbuf[sregs->histind].addr = sregs->pc;
			    sregs->histbuf[sregs->histind].time = ebase.simtime;
			    sregs->histind++;
			    if (sregs->histind >= sregs->histlen)
				sregs->histind = 0;
			}
			if (dis) {
			    printf(" %8" PRIu64 " ", ebase.simtime);
			    dis_mem(sregs->pc, 1, &dinfo);
			}
			if ((sregs->bptnum) && (sregs->bphit = check_bpt(sregs)))
			    icount = 0;
			else {
			    dispatch_instruction(sregs);
			    icount--;
			}
		    } else {
			dispatch_instruction(sregs);
			icount--;
		    }
		}
	    }
	    if (sregs->trap) {
		irq = 0;
		if ((sregs->err_mode = execute_trap(sregs)) == WPT_HIT) {
		    sregs->err_mode = 0;
		    sregs->trap = 0;
		    icount = 0;
		}
		if (sregs->err_mode) {
		    ms->error_mode (sregs->pc);
		    icount = 0;
		}
	    }
	}
	advance_time(sregs);
	if (ctrl_c || (sregs->tlimit <= ebase.simtime)) {
	    icount = 0;
	    if (sregs->tlimit <= ebase.simtime) sregs->tlimit = -1;
	}
    }
    sregs->tottime += get_time() - sregs->starttime;
    ms->restore_stdio ();
    if (sregs->err_mode)
	return ERROR;
    if (ctrl_c) {
	ctrl_c = 0;
	return CTRL_C;
    }
    if (sregs->bphit)
	return (BPT_HIT);
    if (sregs->wphit)
	return (WPT_HIT);
    return TIME_OUT;
}

int
main(argc, argv)
    int             argc;
    char          **argv;
{

    int             cont = 1;
    int             stat = 1;
    int             freq = 14;
    int             copt = 0;

    char           *cfile, *bacmd;
    char           *cmdq[HIST_LEN];
    int             cmdi = 0;
    int             i;
    int             lfile = 0;
    char	    tlim[64] = "";
    int		    run = 0;

    cfile = 0;
    for (i = 0; i < 64; i++)
	cmdq[i] = 0;
    printf("\n SIS - SPARC instruction simulator %s,  copyright Jiri Gaisler 1995\n", sis_version);
    while (stat < argc) {
	if (argv[stat][0] == '-') {
	    if (strcmp(argv[stat], "-v") == 0) {
		sis_verbose += 1;
	    } else if (strcmp(argv[stat], "-c") == 0) {
		if ((stat + 1) < argc) {
		    copt = 1;
		    cfile = argv[++stat];
		}
	    } else if (strcmp(argv[stat], "-nfp") == 0)
		nfp = 1;
	    else if (strcmp(argv[stat], "-ift") == 0)
		ift = 1;
	    else if (strcmp(argv[stat], "-wrp") == 0)
		wrp = 1;
	    else if (strcmp(argv[stat], "-rom8") == 0)
		rom8 = 1;
	    else if (strcmp(argv[stat], "-uben") == 0)
		uben = 1;
	    else if (strcmp(argv[stat], "-uart1") == 0) {
		if ((stat + 1) < argc)
		    strcpy(uart_dev1, argv[++stat]);
	    } else if (strcmp(argv[stat], "-uart2") == 0) {
		if ((stat + 1) < argc)
		    strcpy(uart_dev2, argv[++stat]);
	    } else if (strcmp(argv[stat], "-freq") == 0) {
		if ((stat + 1) < argc)
		    freq = VAL(argv[++stat]);
	    } else if (strcmp(argv[stat], "-sparclite") == 0) {
		sparclite = 1;
#ifdef ERA
	    } else if (strcmp(argv[stat], "-era") == 0) {
		era = 1;
#endif
            } else if (strcmp(argv[stat], "-dumbio") == 0) {
		dumbio = 1;
	    } else if (strcmp(argv[stat], "-nouartrx") == 0) {
		nouartrx = 1;
	    } else if (strcmp(argv[stat], "-r") == 0) {
		run = 1;
	    } else if (strcmp (argv[stat], "-leon2") == 0) {
		ms = &leon2;
		if (freq == 14) freq = 50;
		cputype = CPU_LEON2;
	    } else if (strcmp (argv[stat], "-leon3") == 0) {
		ms = &leon3;
		if (freq == 14) freq = 50;
		cputype = CPU_LEON3;
	    } else if (strcmp(argv[stat], "-tlim") == 0) {
		if ((stat + 2) < argc) {
		    strcpy(tlim, "tlim ");
		    strcat(tlim, argv[++stat]);
		    strcat(tlim, " ");
		    strcat(tlim, argv[++stat]);
		}
	    } else {
		printf("unknown option %s\n", argv[stat]);
		usage();
		exit(1);
	    }
	} else {
	    lfile = stat;
	}
	stat++;
    }

    switch (cputype) {
    case CPU_LEON2:
	printf (" LEON2 emulation enabled\n");
	break;
    case CPU_LEON3:
	printf (" LEON3 emulation enabled\n");
	break;
    default:
	printf (" ERC32 emulation enabled\n");
    }

    if (nfp)
	printf(" FPU disabled\n");
    sregs.freq = freq;
    printf("\n");

    INIT_DISASSEMBLE_INFO(dinfo, stdout, (fprintf_ftype) fprintf);
/* #ifdef HOST_LITTLE_ENDIAN */
/*     dinfo.endian = BFD_ENDIAN_LITTLE; */
/* #else */
/*     dinfo.endian = BFD_ENDIAN_BIG; */
/* #endif */

#ifdef F_GETFL
    termsave = fcntl(0, F_GETFL, 0);
#endif
    using_history();
    init_signals();
    ebase.simtime = 0;
    reset_all();
    init_bpt(&sregs);
    ms->init_sim ();
    if (lfile)
        last_load_addr = bfd_load(argv[lfile]);
#ifdef STAT
    reset_stat(&sregs);
#endif

    if (copt) {
	bacmd = (char *) malloc(256);
	strcpy(bacmd, "batch ");
	strcat(bacmd, cfile);
	exec_cmd(&sregs, bacmd);
    }
    if (tlim[0]) {
	exec_cmd(&sregs, tlim);
    }

    while (cont) {

	if (cmdq[cmdi] != 0) {
#if 0
	    remove_history(cmdq[cmdi]);
#else
	    remove_history(cmdi);
#endif
	    free(cmdq[cmdi]);
	    cmdq[cmdi] = 0;
	}
	if (run) {
	    stat = exec_cmd(&sregs, "run");
	    cont = 0;
	}
	else {
	    cmdq[cmdi] = readline("sis> ");
	    if (cmdq[cmdi] && *cmdq[cmdi])
		add_history(cmdq[cmdi]);
	    if (cmdq[cmdi])
		stat = exec_cmd(&sregs, cmdq[cmdi]);
	    else {
		puts("\n");
		exit(0);
	    }
	}
	switch (stat) {
	case OK:
	    break;
	case CTRL_C:
	    printf("\b\bInterrupt!\n");
	case TIME_OUT:
	    printf(" Stopped at time %" PRIu64 " (%.3f ms)\n", ebase.simtime,
	      ((double) ebase.simtime / (double) sregs.freq) / 1000.0);
	    break;
	case BPT_HIT:
	    printf("breakpoint at 0x%08x reached\n", sregs.pc);
	    sregs.bphit = 1;
	    break;
	case ERROR:
	    printf("IU in error mode (%d)\n", sregs.trap);
	    stat = 0;
	    printf(" %8" PRIu64 " ", ebase.simtime);
	    dis_mem(sregs.pc, 1, &dinfo);
	    break;
	case WPT_HIT:
	    printf("watchpoint at 0x%08x reached, pc = 0x%08x\n",
	        sregs.wpaddress, sregs.pc);
	    sregs.wphit = 1;
	    break;
	default:
	    break;
	}
	ctrl_c = 0;
	stat = OK;

	cmdi = (cmdi + 1) & (HIST_LEN - 1);

    }
    return 0;
}
