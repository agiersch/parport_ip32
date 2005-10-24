/* Low-level parallel port routines for built-in port on SGI IP32
 *
 * Author: Arnaud Giersch <arnaud.giersch@free.fr>
 *
 * $Id: parport_ip32.c,v 1.38 2005-10-24 21:03:40 arnaud Exp $
 *
 * based on parport_pc.c by
 *	Phil Blundell, Tim Waugh, Jose Renau, David Campbell,
 *	Andrea Arcangeli, et al.
 *
 * Copyright (C) 2005 Arnaud Giersch.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/* Current status:
 *
 *	Basic modes are supported: PCSPP, PS2.
 *	Compatibility mode with FIFO support is present.
 *	FIFO can be driven with or without interrupts.
 *
 *	DMA support is not implemented (lack of documentation).
 *	EPP and ECP modes are not implemented (lack of interest).
 */

/* The built-in parallel port on the SGI 02 workstation (a.k.a. IP32) is an
 * IEEE 1284 parallel port driven by a Texas Instrument TL16PIR552PH chip[1].
 * This chip supports SPP, bidirectional, EPP and ECP modes.  It has a 16 byte
 * FIFO buffer and supports DMA transfers.
 *
 * [1] http://focus.ti.com/docs/prod/folders/print/tl16pir552.html
 *
 * Theoretically, we could simply use the parport_pc module.  It is however
 * not so simple.  The parport_pc code assumes that the parallel port
 * registers are port-mapped.  On the O2, they are memory-mapped.
 * Furthermore, each register is replicated on 256 consecutive addresses (as
 * it is for the built-in serial ports on the same chip).
 *
 * Parts of the code were directly adapted from parport_pc. A better approach
 * would certainly be to make the corresponding code arch-independent, with
 * some generic functions for register access.
 */

/*----------------------------------------------------------------------*/

#define DRV_NAME	"parport_ip32"
#define DRV_DESCRIPTION	"SGI IP32 built-in parallel port driver"
#define DRV_AUTHOR	"Arnaud Giersch <arnaud.giersch@free.fr>"
#define DRV_LICENSE	"GPL"
#define DRV_VERSION	"0.2pre"

/*--- Some configuration defines ---------------------------------------*/

/* DEBUG_PARPORT_IP32
 *	0	disable debug
 *	1	standard level: pr_debug1 is enabled
 *	2	parport_ip32_dump_state is enabled
 *	>2	verbose level: pr_debug is enabled
 */
#define DEBUG_PARPORT_IP32  1	/* disable for production */

/* Undefine to disable support for a particular mode. */
#define PARPORT_IP32_PS2
#define PARPORT_IP32_EPP	/* not implemented */
#define PARPORT_IP32_ECP	/* not implemented */

/*----------------------------------------------------------------------*/

/* Setup DEBUG macros.  This is done before any includes, just in case we
 * activate pr_debug() with DEBUG_PARPORT_IP32 >= 3.
 */
#if DEBUG_PARPORT_IP32 == 1
#	warning DEBUG_PARPORT_IP32 == 1
#elif DEBUG_PARPORT_IP32 == 2
#	warning DEBUG_PARPORT_IP32 == 2
#elif DEBUG_PARPORT_IP32 >= 3
#	warning DEBUG_PARPORT_IP32 >= 3
#	if ! defined(DEBUG)
#		define DEBUG /* enable pr_debug() in kernel.h */
#	endif
#endif

#include <linux/config.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/parport.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/types.h>

/*--- Parameters for the SGI O2 built-in parallel port -----------------*/

#include <asm/ip32/mace.h>
#include <asm/ip32/ip32_ints.h>

/* Physical I/O addresses */
#define PARPORT_IP32_IO		(MACE_BASE +				\
				 offsetof (struct sgi_mace, isa) +	\
				 offsetof (struct mace_isa, parallel))
#define PARPORT_IP32_IOHI	(MACE_BASE +				\
				 offsetof (struct sgi_mace, isa) +	\
				 offsetof (struct mace_isa, ecp1284))

#define PARPORT_IP32_IRQ	MACEISA_PARALLEL_IRQ
#define PARPORT_IP32_DMA	PARPORT_DMA_NONE

/* Memory mapped I/O addresses */
#define PARPORT_IP32_IO_ADDR	((void *)&mace->isa.parallel)
#define PARPORT_IP32_IOHI_ADDR	((void *)&mace->isa.ecp1284)

/* Registers are replicated 256 times */
#define PARPORT_IP32_REGSHIFT	8

/* Pointer mace is not exported in arch/mips/sgi-ip32/crime.c,
 * use our own pointer for modules.
 */
#if defined(MODULE)
static struct sgi_mace __iomem *mace = NULL;
#define parport_ip32_iomap_mace_address()	\
	(mace = ioremap (MACE_BASE, sizeof (struct sgi_mace)))
#define parport_ip32_iounmap_mace_address()	\
	iounmap (mace);
#else /* ! defined(MODULE) */
#define parport_ip32_iomap_mace_address()
#define parport_ip32_iounmap_mace_address()
#endif

/*--- Basic type definitions -------------------------------------------*/

/**
 * typedef parport_ip32_bool - our Boolean type
 *
 * This type is aliased to bool in the code.
 */
typedef unsigned int	parport_ip32_bool;
#define bool		parport_ip32_bool
#define true		1
#define false		0

/**
 * typedef parport_ip32_byte - a register's value
 *
 * Use unsigned int to avoid unnecessary conversions to/from int type in
 * computations.  This type is aliased to byte in the code.
 */
typedef unsigned int	parport_ip32_byte;
#define byte		parport_ip32_byte

/*--- Global variables -------------------------------------------------*/

/* Check dependencies across configuration options. */
#if ! defined(CONFIG_PARPORT_1284)
#	undef PARPORT_IP32_PS2
#	undef PARPORT_IP32_EPP
#	undef PARPORT_IP32_ECP
#endif

#if ! defined(CONFIG_PARPORT_IP32_FIFO)
#	undef PARPORT_IP32_ECP
#endif

/* Verbose probing on by default for debugging. */
#if DEBUG_PARPORT_IP32 >= 1
#	define DEFAULT_VERBOSE_PROBING	1
#else
#	define DEFAULT_VERBOSE_PROBING	0
#endif

/* Default prefix for printk */
#define PPIP32 DRV_NAME ": "

/*
 * These are the module parameters:
 * @param_verbose_probing:	log chit-chat during initialization
 * @param_irq:			IRQ line (-1 to disable)
 * @param_dma:			DMA channel (-1 to disable)
 * @use_fifo:			use hardware FIFO modes if available
 * @use_dma:			use DMA if available
 */
static int param_verbose_probing =	DEFAULT_VERBOSE_PROBING;
static int param_irq =			PARPORT_IP32_IRQ;
static int param_dma =			PARPORT_IP32_DMA;

#if defined(CONFIG_PARPORT_IP32_FIFO)
static int use_fifo =			1;
#if 0				/* DMA is not supported yet */
static int use_dma =			1;
#endif
#endif

/* We do not support more than one port. */
static struct parport *this_port = NULL;

/*--- I/O register definitions -----------------------------------------*/

/**
 * struct parport_ip32_regs - virtual addresses of parallel port registers
 * @data:	Data Register
 * @dsr:	Device Status Register
 * @dcr:	Device Control Register
 * @eppAddr:	EPP Address Register
 * @eppData0:	EPP Data Register 0
 * @eppData1:	EPP Data Register 1
 * @eppData2:	EPP Data Register 2
 * @eppData3:	EPP Data Register 3
 * @cFifo:	Parallel Port DATA FIFO
 * @ecpAFifo:	ECP FIFO (Address)
 * @ecpDFifo:	ECP FIFO (Data)
 * @tFifo:	Test FIFO
 * @cnfgA:	Configuration Register A
 * @cnfgB:	Configuration Register B
 * @ecr:	Extended Control Register
 */
struct parport_ip32_regs {
	void __iomem *data;
	void __iomem *dsr;
	void __iomem *dcr;
	void __iomem *eppAddr;
	void __iomem *eppData0;
	void __iomem *eppData1;
	void __iomem *eppData2;
	void __iomem *eppData3;
	void __iomem *cFifo;
	void __iomem *ecpAFifo;
	void __iomem *ecpDFifo;
	void __iomem *tFifo;
	void __iomem *cnfgA;
	void __iomem *cnfgB;
	void __iomem *ecr;
};

#undef BIT
#define BIT(n) (1U << (n))

/* Device Status Register */
#define DSR_nBUSY		BIT(7)	/* PARPORT_STATUS_BUSY */
#define DSR_nACK		BIT(6)	/* PARPORT_STATUS_ACK */
#define DSR_PERROR		BIT(5)	/* PARPORT_STATUS_PAPEROUT */
#define DSR_SELECT		BIT(4)	/* PARPORT_STATUS_SELECT */
#define DSR_nFAULT		BIT(3)	/* PARPORT_STATUS_ERROR */
#define DSR_nPRINT		BIT(2)	/* specific to the TL16PIR552 */
/* #define DSR_???		BIT(1) */
#define DSR_TIMEOUT		BIT(0)	/* EPP timeout */

/* Device Control Register */
/* #define DCR_???		BIT(7) */
/* #define DCR_???		BIT(6) */
#define DCR_DIR			BIT(5)	/* direction */
#define DCR_IRQ			BIT(4)	/* interrupt on nAck */
#define DCR_SELECT		BIT(3)	/* PARPORT_CONTROL_SELECT */
#define DCR_nINIT		BIT(2)	/* PARPORT_CONTROL_INIT */
#define DCR_AUTOFD		BIT(1)	/* PARPORT_CONTROL_AUTOFD */
#define DCR_STROBE		BIT(0)	/* PARPORT_CONTROL_STROBE */

/* ECP Configuration Register A */
#define CNFGA_IRQ		BIT(7)
#define CNFGA_ID_MASK		(BIT(6) | BIT(5) | BIT(4))
#define CNFGA_ID_SHIFT		4
#define CNFGA_ID_16		(00 << CNFGA_ID_SHIFT)
#define CNFGA_ID_8		(01 << CNFGA_ID_SHIFT)
#define CNFGA_ID_32		(02 << CNFGA_ID_SHIFT)
/* #define CNFGA_???		BIT(3) */
#define CNFGA_nBYTEINTRANS	BIT(2)
#define CNFGA_PWORDLEFT		(BIT(1) | BIT(0))

/* ECP Configuration Register B */
static const unsigned int cnfgb_irq_line[8] = {0, 7, 9, 10, 11, 14, 15, 5};
static const unsigned int cnfgb_dma_chan[8] = {0, 1, 2, 3, 0, 5, 6, 7};

#define CNFGB_COMPRESS		BIT(7)
#define CNFGB_INTRVAL		BIT(6)
#define CNFGB_IRQ_MASK		(BIT(5) | BIT(4) | BIT(3))
#define CNFGB_IRQ_SHIFT		3
#define CNFGB_IRQ(r)		\
	cnfgb_irq_line[((r) & CNFGB_IRQ_MASK) >> CNFGB_IRQ_SHIFT]
#define CNFGB_DMA_MASK		(BIT(2) | BIT(1) | BIT(0))
#define CNFGB_DMA_SHIFT		0
#define CNFGB_DMA(r)		\
	cnfgb_dma_chan[((r) & CNFGB_DMA_MASK) >> CNFGB_DMA_SHIFT]

/* Extended Control Register */
#define ECR_MODE_MASK		(BIT(7) | BIT(6) | BIT(5))
#define ECR_MODE_SHIFT		5
#define ECR_MODE_SPP		(00 << ECR_MODE_SHIFT)
#define ECR_MODE_PS2		(01 << ECR_MODE_SHIFT)
#define ECR_MODE_PPF		(02 << ECR_MODE_SHIFT)
#define ECR_MODE_ECP		(03 << ECR_MODE_SHIFT)
#define ECR_MODE_EPP		(04 << ECR_MODE_SHIFT)
/* #define ECR_MODE_???		(05 << ECR_MODE_SHIFT) */
#define ECR_MODE_TST		(06 << ECR_MODE_SHIFT)
#define ECR_MODE_CFG		(07 << ECR_MODE_SHIFT)
#define ECR_nERRINTR		BIT(4)
#define ECR_DMA			BIT(3)
#define ECR_SERVINTR		BIT(2)
#define ECR_F_FULL		BIT(1)
#define ECR_F_EMPTY		BIT(0)

/*--- Private data -----------------------------------------------------*/

/**
 * enum parport_ip32_irq_mode - operation mode of interrupt handler
 * @PARPORT_IP32_IRQ_FWD	forward interrupt to the upper parport layer
 * @PARPORT_IP32_IRQ_HERE	interrupt is handled locally
 */
enum parport_ip32_irq_mode { PARPORT_IP32_IRQ_FWD, PARPORT_IP32_IRQ_HERE };

/**
 * struct parport_ip32_private - private stuff for &struct parport
 * @regs:		register addresses
 * @dcr_init:		initial value for DCR
 * @ecr_init:		initial value for ECR
 * @dcr_cache:		cached contents of DCR
 * @dcr_writable:	bit mask of writable DCR bits
 * @ecr_present:	is an ECR register present?
 * @fifo_depth:		number of PWords that FIFO will hold
 * @pword:		number of bytes per PWord
 * @readIntrThreshold:	minimum number of PWords we can read
 *			if we get an interrupt
 * @writeIntrThreshold:	minimum number of PWords we can write
 *			if we get an interrupt
 * @irq_mode:		operation mode of interrupt handler for this port
 * @irq:		mutex used to wait for an interrupt
 */
struct parport_ip32_private {
	struct parport_ip32_regs regs;
	byte dcr_init;
	byte ecr_init;
	byte dcr_cache;
	byte dcr_writable;
	bool ecr_present;
	unsigned int fifo_depth;
	unsigned int pword;
	unsigned int readIntrThreshold;
	unsigned int writeIntrThreshold;
	enum parport_ip32_irq_mode irq_mode;
	struct semaphore irq;
};

/**
 * PRIV - fetch private stuff inside &struct parport
 * @p:	pointer to a &struct parport
 *
 * Returns the address of the &parport_ip32_private structure from the given
 * &parport structure.
 */
#define PRIV(p) ((struct parport_ip32_private *)(p)->physport->private_data)

/*--- Generic functions ------------------------------------------------*/

/**
 * parport_ip32_make_isa_registers - compute (ISA) register addresses
 * @regs:	pointer to &struct parport_ip32_regs to fill
 * @base:	base address of standard and EPP registers
 * @base_hi:	base address of ECP registers
 * @regshift:	how much to shift register offset by
 *
 * Compute register addresses, according to the ISA standard.  The addresses
 * of the standard and EPP registers are computed from address @base.  The
 * addresses of the ECP registers are computed from address @base_hi.
 */
static void
parport_ip32_make_isa_registers (struct parport_ip32_regs *regs,
				 void __iomem *base, void __iomem *base_hi,
				 unsigned int regshift)
{
#define r_base(offset)    ((u8 __iomem *)base    + ((offset) << regshift))
#define r_base_hi(offset) ((u8 __iomem *)base_hi + ((offset) << regshift))
	*regs = (struct parport_ip32_regs ){
		.data		= r_base (0),
		.dsr		= r_base (1),
		.dcr		= r_base (2),
		.eppAddr	= r_base (3),
		.eppData0	= r_base (4),
		.eppData1	= r_base (5),
		.eppData2	= r_base (6),
		.eppData3	= r_base (7),
		.cFifo		= r_base_hi (0),
		.ecpAFifo	= r_base (0),
		.ecpDFifo	= r_base_hi (0),
		.tFifo		= r_base_hi (0),
		.cnfgA		= r_base_hi (0),
		.cnfgB		= r_base_hi (1),
		.ecr		= r_base_hi (2)
	};
#undef r_base_hi
#undef r_base
}

/*--- I/O register access functions ------------------------------------*/

/* FIXME - Use io{read,write}8 (and _rep) when available on MIPS?  */
/* FIXME - Are the memory barriers really needed?  */

/**
 * parport_ip32_in - read a register
 * @addr:	address of register
 */
static inline byte parport_ip32_in (void __iomem *addr)
{
	byte val = readb (addr);
	rmb ();
	return val;
}

/**
 * parport_ip32_out - write some value to a register
 * @val:	value to write
 * @addr:	address of register
 */
static inline void parport_ip32_out (byte val, void __iomem *addr)
{
	writeb (val, addr);
	wmb ();
}

/**
 * parport_ip32_out_rep - write multiple values to a register
 * @addr:	address of register
 * @buf:	buffer of values to write
 * @count:	number of bytes to write
 */
static inline void parport_ip32_out_rep (void __iomem *addr,
					 const void *buf, unsigned long count)
{
	writesb (addr, buf, count);
	wmb ();
}

/*--- Debug code -------------------------------------------------------*/

/**
 * pr_debug1 - print debug messages
 *
 * This is like pr_debug(), but is defined for %DEBUG_PARPORT_IP32 >= 1
 */
#if DEBUG_PARPORT_IP32 >= 1
#	define pr_debug1(...)	printk (KERN_DEBUG __VA_ARGS__)
#else
#	define pr_debug1(...)
#endif

/**
 * parport_ip32_dump_state - print register status of parport
 * @p:		pointer to &struct parport
 * @str:	string to add in message
 * @show_ecp_config:	shall we dump ECP configuration registers too?
 *
 * This function is only here for debugging purpose, and should be used with
 * care.  Reading the parallel port registers may have undesired side effects.
 * Especially if @show_ecp_config is true, the parallel port is resetted.
 * This function is only defined if %DEBUG_PARPORT_IP32 >= 2.
 */
#if DEBUG_PARPORT_IP32 >= 2
static void parport_ip32_dump_state (struct parport *p, char *str,
				     bool show_ecp_config)
{
	/* here's hoping that reading these ports won't side-effect
	 * anything underneath */
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int i;

	printk (KERN_DEBUG PPIP32 "%s: state (%s):\n", p->name, str);
	if (priv->ecr_present) {
		static const char ecr_modes[8][4] = {"SPP", "PS2", "PPF",
						     "ECP", "EPP", "???",
						     "TST", "CFG"};
		byte ecr = parport_ip32_in (priv->regs.ecr);
		printk (KERN_DEBUG PPIP32 "    ecr=0x%02x", ecr);
		printk (" %s",
			ecr_modes[(ecr & ECR_MODE_MASK) >> ECR_MODE_SHIFT]);
		if (ecr & ECR_nERRINTR)	printk (",nErrIntrEn");
		if (ecr & ECR_DMA)	printk (",dmaEn");
		if (ecr & ECR_SERVINTR)	printk (",serviceIntr");
		if (ecr & ECR_F_FULL)	printk (",f_full");
		if (ecr & ECR_F_EMPTY)	printk (",f_empty");
		printk ("\n");
	}
	if (priv->ecr_present && show_ecp_config) {
		byte oecr, cnfgA, cnfgB;
		oecr = parport_ip32_in (priv->regs.ecr);
		parport_ip32_out (ECR_MODE_PS2, priv->regs.ecr);
		parport_ip32_out (ECR_MODE_CFG, priv->regs.ecr);
		cnfgA = parport_ip32_in (priv->regs.cnfgA);
		cnfgB = parport_ip32_in (priv->regs.cnfgB);
		parport_ip32_out (ECR_MODE_PS2, priv->regs.ecr);
		parport_ip32_out (oecr, priv->regs.ecr);
		printk (KERN_DEBUG PPIP32 "    cnfgA=0x%02x", cnfgA);
		printk (" ISA-%s", (cnfgA & CNFGA_IRQ)? "Level": "Pulses");
		switch (cnfgA & CNFGA_ID_MASK) {
		case CNFGA_ID_8:	printk (",8 bits"); break;
		case CNFGA_ID_16:	printk (",16 bits"); break;
		case CNFGA_ID_32:	printk (",32 bits"); break;
		default:		printk (",unknown ID"); break;
		}
		if (! (cnfgA & CNFGA_nBYTEINTRANS))  printk (",ByteInTrans");
		if ((cnfgA & CNFGA_ID_MASK) != CNFGA_ID_8) {
			printk (",%d byte%s left", cnfgA & CNFGA_PWORDLEFT,
				((cnfgA & CNFGA_PWORDLEFT) > 1)? "s": "");
		}
		printk ("\n");
		printk (KERN_DEBUG PPIP32 "    cnfgB=0x%02x", cnfgB);
		printk (" irq=%u,dma=%u", CNFGB_IRQ(cnfgB), CNFGB_DMA(cnfgB));
		printk (",intrValue=%d", !!(cnfgB & CNFGB_INTRVAL));
		if (cnfgB & CNFGB_COMPRESS)	printk (",compress");
		printk ("\n");
	}
	for (i=0; i<2; i++) {
		byte dcr = i? priv->dcr_cache: parport_ip32_in (priv->regs.dcr);
		printk (KERN_DEBUG PPIP32 "    dcr(%s)=0x%02x",
			i? "soft": "hard", dcr);
		printk (" %s", (dcr & DCR_DIR)? "rev": "fwd");
		if (dcr & DCR_IRQ)		printk (",ackIntEn");
		if (! (dcr & DCR_SELECT))	printk (",nSelectIn");
		if (dcr & DCR_nINIT)		printk (",nInit");
		if (! (dcr & DCR_AUTOFD))	printk (",nAutoFD");
		if (! (dcr & DCR_STROBE))	printk (",nStrobe");
		printk ("\n");
	}
#define sep (f++? ',': ' ')
	{
		unsigned int f = 0;
		byte dsr = parport_ip32_in (priv->regs.dsr);
		printk (KERN_DEBUG PPIP32 "    dsr=0x%02x", dsr);
		if (! (dsr & DSR_nBUSY))	printk ("%cBusy", sep);
		if (dsr & DSR_nACK)		printk ("%cnAck", sep);
		if (dsr & DSR_PERROR)		printk ("%cPError", sep);
		if (dsr & DSR_SELECT)		printk ("%cSelect", sep);
		if (dsr & DSR_nFAULT)		printk ("%cnFault", sep);
		if (! (dsr & DSR_nPRINT))	printk ("%c(Print)", sep);
		if (dsr & DSR_TIMEOUT)		printk ("%cTimeout", sep);
		printk ("\n");
	}
#undef sep
}
#else /* DEBUG_PARPORT_IP32 < 2 */
#define parport_ip32_dump_state(...)
#endif

/**
 * CHECK_EXTRA_BITS - track and log extra bits
 * @p:	pointer to &struct parport
 * @b:	byte to inspect
 * @m:	bit mask of authorized bits
 *
 * This is used to track and log extra bits that should not be there in
 * parport_ip32_write_control() and parport_ip32_frob_control().  It is only
 * defined if %DEBUG_PARPORT_IP32 >= 1.
 */
#if DEBUG_PARPORT_IP32 >= 1
#define CHECK_EXTRA_BITS(p, b, m)					\
	do {								\
		byte __b = (b), __m = (m);				\
		if (__b & ~__m)						\
			pr_debug1 (PPIP32 "%s: extra bits in %s(%s): "	\
				   "0x%02x/0x%02x\n",			\
				   (p)->name, __func__, #b, __b, __m);	\
	} while (0)
#else
#define CHECK_EXTRA_BITS(...)
#endif

/**
 * pr_trace, pr_trace1 - trace function calls
 * @p:		pointer to &struct parport
 * @fmt:	printk format string
 * @...:	parameters for format string
 *
 * Macros used to trace function calls.  The given string is formatted after
 * function name.  pr_trace() uses pr_debug(), and pr_trace1() uses
 * pr_debug1().  __pr_trace() is the low-level macro and is not to be used
 * directly.
 */
#define __pr_trace(pr, p, fmt, ...)					\
	pr ("%s: %s" fmt "\n", (p)->name, __func__ , ##__VA_ARGS__)
#define pr_trace(p, fmt, ...)	__pr_trace (pr_debug, p, fmt, __VA_ARGS__ )
#define pr_trace1(p, fmt, ...)	__pr_trace (pr_debug1, p, fmt, __VA_ARGS__ )

/**
 * __pr_probe, pr_probe - print message if @param_verbose_probing is true
 * @p:		pointer to &struct parport
 * @fmt:	printk format string
 * @...:	parameters for format string
 *
 * For new lines, use pr_probe().  Use __pr_probe() for continued lines.
 */
#define __pr_probe(...)							\
	do { if (param_verbose_probing) printk ( __VA_ARGS__ ); } while (0)
#define pr_probe(p, fmt, ...)						\
	__pr_probe (KERN_DEBUG PPIP32 "0x%lx: " fmt, (p)->base , ##__VA_ARGS__)

/*--- Some utility function to manipulate ECR register -----------------*/

/**
 * parport_ip32_read_econtrol - read contents of the ECR register
 * @p:	pointer to &struct parport
 */
static inline byte parport_ip32_read_econtrol (struct parport *p)
{
	byte c = parport_ip32_in (PRIV(p)->regs.ecr);
	pr_trace (p, "(): 0x%02x", c);
	return c;
}

/**
 * parport_ip32_write_econtrol - write new contents to the ECR register
 * @p:	pointer to &struct parport
 * @c:	new value to write
 */
static inline void parport_ip32_write_econtrol (struct parport *p, byte c)
{
	pr_trace (p, "(0x%02x)", c);
	parport_ip32_out (c, PRIV(p)->regs.ecr);
}

/**
 * parport_ip32_frob_econtrol - change bits from the ECR register
 * @p:		pointer to &struct parport
 * @mask:	bit mask of bits to change
 * @val:	new value for changed bits
 *
 * Read from the ECR, mask out the bits in @mask, exclusive-or with the bits
 * in @val, and write the result to the ECR.
 */
static inline void parport_ip32_frob_econtrol (struct parport *p,
					       byte mask, byte val)
{
	byte c;
	pr_trace (p, "(%02x, %02x)", mask, val);
	c = (parport_ip32_read_econtrol (p) & ~mask) ^ val;
	parport_ip32_write_econtrol (p, c);
}

/**
 * parport_ip32_set_mode - change mode of ECP port
 * @p:		pointer to &struct parport
 * @mode:	new ECP mode
 *
 * ECR is reset in a sane state (interrupts and DMA disabled), and placed in
 * mode @mode.  Go through PS2 mode if needed.
 */
static inline void parport_ip32_set_mode (struct parport *p, byte mode)
{
	byte omode;
	pr_trace (p, "(0x%02x)", mode);

	mode &= ECR_MODE_MASK;
	omode = parport_ip32_read_econtrol (p) & ECR_MODE_MASK;

	if (! (mode == ECR_MODE_SPP || mode == ECR_MODE_PS2 ||
	       omode == ECR_MODE_SPP || omode == ECR_MODE_PS2)) {
		/* We have to go through PS2 mode */
		byte ecr = ECR_MODE_PS2 | ECR_nERRINTR | ECR_SERVINTR;
		parport_ip32_write_econtrol (p, ecr);
	}
	parport_ip32_write_econtrol (p, mode | ECR_nERRINTR | ECR_SERVINTR);
}

/*--- Basic functions needed for parport -------------------------------*/

/**
 * parport_ip32_read_data - return current contents of the DATA register
 * @p:		pointer to &struct parport
 */
static inline byte parport_ip32_read_data (struct parport *p)
{
	byte d = parport_ip32_in (PRIV(p)->regs.data);
	pr_trace (p, "(): 0x%02x", d);
	return d;
}

/**
 * parport_ip32_write_data - set new contents for the DATA register
 * @p:		pointer to &struct parport
 * @d:		new value to write
 */
static inline void parport_ip32_write_data (struct parport *p, byte d)
{
	pr_trace (p, "(0x%02x)", d);
	parport_ip32_out (d, PRIV(p)->regs.data);
}

/**
 * parport_ip32_read_status - return current contents of the DSR register
 * @p:		pointer to &struct parport
 */
static inline byte parport_ip32_read_status (struct parport *p)
{
	byte s = parport_ip32_in (PRIV(p)->regs.dsr);
	pr_trace (p, "(): 0x%02x", s);
	return s;
}

/**
 * __parport_ip32_read_control - return cached contents of the DCR register
 * @p:		pointer to &struct parport
 */
static inline byte __parport_ip32_read_control (struct parport *p)
{
	byte c = PRIV(p)->dcr_cache; /* use soft copy */
	pr_trace (p, "(): 0x%02x", c);
	return c;
}

/**
 * parport_ip32_read_control - return cached contents of the DCR register
 * @p:		pointer to &struct parport
 *
 * The return value is masked so as to only return the value of %DCR_STROBE,
 * %DCR_AUTOFD, %DCR_nINIT, and %DCR_SELECT.
 */
static inline byte parport_ip32_read_control (struct parport *p)
{
	const byte rm = DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	byte c = __parport_ip32_read_control (p) & rm;
	pr_trace (p, "(): 0x%02x", c);
	return c;
}

/**
 * __parport_ip32_write_control - set new contents for the DCR register
 * @p:		pointer to &struct parport
 * @c:		new value to write
 */
static inline void __parport_ip32_write_control (struct parport *p, byte c)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_trace (p, "(0x%02x)", c);
	CHECK_EXTRA_BITS (p, c, priv->dcr_writable);
	c &= priv->dcr_writable; /* only writable bits */
	parport_ip32_out (c, priv->regs.dcr);
	priv->dcr_cache = c;		/* update soft copy */
}

/**
 * __parport_ip32_frob_control - change bits from the DCR register
 * @p:		pointer to &struct parport
 * @mask:	bit mask of bits to change
 * @val:	new value for changed bits
 *
 * This is equivalent to read from the DCR, mask out the bits in @mask,
 * exclusive-or with the bits in @val, and write the result to the DCR.
 * Actually, the cached contents of the DCR is used.
 */
static inline void __parport_ip32_frob_control (struct parport *p,
						byte mask, byte val)
{
	byte c;
	pr_trace (p, "(0x%02x, 0x%02x)", mask, val);
	c = (__parport_ip32_read_control (p) & ~mask) ^ val;
	__parport_ip32_write_control (p, c);
}

/**
 * parport_ip32_write_control - set new contents for the DCR register
 * @p:		pointer to &struct parport
 * @c:		new value to write
 *
 * The value is masked so as to only change the value of %DCR_STROBE,
 * %DCR_AUTOFD, %DCR_nINIT, and %DCR_SELECT.
 */
static inline void parport_ip32_write_control (struct parport *p, byte c)
{
	const byte wm = DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	pr_trace (p, "(0x%02x)", c);
	CHECK_EXTRA_BITS (p, c, wm);
	__parport_ip32_frob_control (p, wm, c & wm);
}

/**
 * parport_ip32_frob_control - change bits from the DCR register
 * @p:		pointer to &struct parport
 * @mask:	bit mask of bits to change
 * @val:	new value for changed bits
 *
 * This differs from __parport_ip32_frob_control() in that it only allows to
 * change the value of %DCR_STROBE, %DCR_AUTOFD, %DCR_nINIT, and %DCR_SELECT.
 */
static inline byte parport_ip32_frob_control (struct parport *p,
					      byte mask, byte val)
{
	const byte wm = DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	pr_trace (p, "(0x%02x, 0x%02x)\n", mask, val);
	CHECK_EXTRA_BITS (p, mask, wm);
	CHECK_EXTRA_BITS (p, val, wm);
	__parport_ip32_frob_control (p, mask & wm, val & wm);
	return parport_ip32_read_control (p);
}

/**
 * parport_ip32_disable_irq - disable interrupts on the rising edge of nACK
 * @p:		pointer to &struct parport
 */
static inline void parport_ip32_disable_irq (struct parport *p)
{
	pr_trace (p, "()");
	__parport_ip32_frob_control (p, DCR_IRQ, 0);
}

/**
 * parport_ip32_enable_irq - enable interrupts on the rising edge of nACK
 * @p:		pointer to &struct parport
 */
static inline void parport_ip32_enable_irq (struct parport *p)
{
	pr_trace (p, "()");
	__parport_ip32_frob_control (p, DCR_IRQ, DCR_IRQ);
}

/**
 * parport_ip32_data_forward - enable host-to-peripheral communications
 * @p:		pointer to &struct parport
 *
 * Enable the data line drivers, for 8-bit host-to-peripheral communications.
 */
static inline void parport_ip32_data_forward (struct parport *p)
{
	pr_trace (p, "()");
	__parport_ip32_frob_control (p, DCR_DIR, 0);
}

/**
 * parport_ip32_data_reverse - enable peripheral-to-host communications
 * @p:		pointer to &struct parport
 *
 * Place the data bus in a high impedance state, if @p->modes has the
 * PARPORT_MODE_TRISTATE bit set.
 */
static inline void parport_ip32_data_reverse (struct parport *p)
{
	pr_trace (p, "()");
	__parport_ip32_frob_control (p, DCR_DIR, DCR_DIR);
}

/**
 * parport_ip32_init_state - for core parport code
 */
static inline void parport_ip32_init_state (struct pardevice *dev,
					    struct parport_state *s)
{
	struct parport_ip32_private * const priv = PRIV(dev->port);
	pr_trace (dev->port, "(%s, %p)", dev->name, s);
	s->u.ip32.dcr = priv->dcr_init;
	s->u.ip32.ecr = priv->ecr_init;
}

/**
 * parport_ip32_save_state - for core parport code
 */
static inline void parport_ip32_save_state (struct parport *p,
					    struct parport_state *s)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_trace (p, "(%p)", s);
	s->u.ip32.dcr = __parport_ip32_read_control (p);
	if (priv->ecr_present) {
		s->u.ip32.ecr = parport_ip32_read_econtrol (p);
	}
}

/**
 * parport_ip32_restore_state - for core parport code
 */
static inline void parport_ip32_restore_state (struct parport *p,
					       struct parport_state *s)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_trace (p, "(%p)", s);
	if (priv->ecr_present) {
		parport_ip32_set_mode (p,  s->u.ip32.ecr & ECR_MODE_MASK);
		parport_ip32_write_econtrol (p, s->u.ip32.ecr);
	}
	__parport_ip32_write_control (p, s->u.ip32.dcr);
}

/*--- Interrupt handlers and associates --------------------------------*/

/**
 * parport_ip32_wakeup - wakes up code waiting for an interrupt
 * @port:	pointer to &struct parport
 */
static inline void parport_ip32_wakeup (struct parport *port)
{
	struct parport_ip32_private * const priv = PRIV(port);
	up (&priv->irq);
}

/**
 * parport_ip32_interrupt - interrupt handler
 *
 * Caught interrupts are forwarded to the upper parport layer if IRQ_mode is
 * %PARPORT_IP32_IRQ_FWD.
 */
static irqreturn_t parport_ip32_interrupt (int irq, void *dev_id,
					   struct pt_regs *regs)
{
	struct parport * const port = dev_id;
	struct parport_ip32_private * const priv = PRIV(port);
	enum parport_ip32_irq_mode irq_mode = priv->irq_mode;
	barrier ();		/* ensures that priv->irq_mode is read */
	pr_trace (port, "(%d)", irq);
	switch (irq_mode) {
	case PARPORT_IP32_IRQ_FWD:
		parport_generic_irq (irq, port, regs);
		break;
	case PARPORT_IP32_IRQ_HERE:
		parport_ip32_wakeup (port);
		break;
	}
	return IRQ_HANDLED;
}

/**
 * parport_ip32_timeout - timeout handler
 */
static void parport_ip32_timeout (unsigned long data)
{
	struct parport * const port = (struct parport *)data;
	parport_ip32_wakeup (port);
}

/*--- EPP mode functions -----------------------------------------------*/

/**
 * parport_ip32_clear_epp_timeout - clear Timeout bit in EPP mode
 * @p:	pointer to &struct parport
 *
 * Returns false if it failed to clear Timeout bit.  This is also used in SPP
 * and PS2 detection.
 */
static bool parport_ip32_clear_epp_timeout (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	bool cleared;
	byte r;

	if (! (parport_ip32_read_status (p) & DSR_TIMEOUT)) {
		cleared = true;
	} else {
		/* To clear timeout some chips require double read */
		parport_ip32_read_status (p);
		r = parport_ip32_read_status (p);
		/* Some reset by writing 1 */
		parport_ip32_out (r | DSR_TIMEOUT, priv->regs.dsr);
		/* Others by writing 0 */
		parport_ip32_out (r & ~DSR_TIMEOUT, priv->regs.dsr);

		r = parport_ip32_read_status (p);
		cleared = !(r & DSR_TIMEOUT);
	}

	pr_trace (p, "(): %s", cleared? "cleared": "failed");
	return cleared;
}

#if defined(PARPORT_IP32_EPP)

/*
 * FIXME - Insert here parport_ip32_epp_{read,write}_{data,address}().
 */

#endif /* defined(PARPORT_IP32_EPP) */

/*--- ECP mode functions (FIFO) ----------------------------------------*/

#if defined(CONFIG_PARPORT_IP32_FIFO)

/**
 * parport_ip32_fifo_addr - get address of FIFO register
 * @port:	pointer to &struct parport
 * @mode:	ECP operation mode
 *
 * Return address for FIFO register, according to mode @mode.
 */
static inline void __iomem *parport_ip32_fifo_addr (struct parport *port,
						    byte mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	void __iomem *fifo;
	switch (mode & ECR_MODE_MASK) {
	case ECR_MODE_PPF:	fifo = priv->regs.cFifo; break;
	case ECR_MODE_ECP:	fifo = priv->regs.ecpDFifo; break;
	default:		fifo = NULL; /* this should not happen! */
	}
	return fifo;
}

/**
 * parport_ip32_fwp_wait_break - check if the waiting function should return
 * @port:	pointer to &struct parport
 * @expire:	timeout expiring date, in jiffies
 *
 * parport_ip32_fwp_wait_break() checks if the waiting function should return
 * immediately or not.  The break conditions are:
 *	- expired timeout;
 *	- a pending signal;
 *	- nFault asserted low.
 * This function also calls cond_resched().
 */
static inline bool parport_ip32_fwp_wait_break (struct parport *port,
						unsigned long expire)
{
	/* Time to resched? */
	cond_resched ();
	/* Timed out? */
	if (time_after (jiffies, expire)) {
		printk (KERN_DEBUG PPIP32
			"%s: FIFO write timed out\n", port->name);
		return true;
	}
	/* Pending signal? */
	if (signal_pending (current)) {
		printk (KERN_DEBUG PPIP32
			"%s: Signal pending\n", port->name);
		return true;
	}
	/* nFault? */
	if (! (parport_ip32_read_status (port) & DSR_nFAULT)) {
		printk (KERN_DEBUG PPIP32
			"%s: nFault asserted low\n", port->name);
		return true;
	}
	return false;
}

/**
 * parport_ip32_fwp_wait_polling - wait for FIFO to empty (polling)
 * @port:	pointer to &struct parport
 *
 * Used by parport_ip32_fifo_write_pio_wait().
 */
static int parport_ip32_fwp_wait_polling (struct parport *port)
{
	static const unsigned int polling_interval = 50; /* usecs */
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	unsigned long expire;
	byte fifo_state;
	int count;

	expire = jiffies + physport->cad->timeout;
	count = 0;
	while (1) {
		if (parport_ip32_fwp_wait_break (port, expire))
			break;

		/* Check FIFO state */
		fifo_state = parport_ip32_read_econtrol (port);
		fifo_state &= (ECR_F_FULL | ECR_F_EMPTY);

		/* We do nothing when the FIFO is nor full, nor empty.  It
		 * appears that the FIFO full bit is not always reliable, the
		 * FIFO state is sometimes wrongly reported, and the chip gets
		 * confused if we give it another byte. */
		if (fifo_state == ECR_F_EMPTY) {
			/* FIFO is empty, fill it up */
			count = priv->fifo_depth;
			break;
		} else if (fifo_state == (ECR_F_FULL | ECR_F_EMPTY)) {
			/* Something wrong happened */
			count = -1;
			break;
		}

		/* Wait a moment... */
		udelay (polling_interval);
	} /* while (1) */

	return count;
}

/**
 * parport_ip32_fwp_wait_interrupt - wait for FIFO to empty (interrupt-driven)
 * @port:	pointer to &struct parport
 *
 * Used by parport_ip32_fifo_write_pio_wait().
 */
static int parport_ip32_fwp_wait_interrupt (struct parport *port)
{
	static const unsigned int nfault_check_interval = 100; /* msecs */
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	DEFINE_TIMER (timer, parport_ip32_timeout, 0, (unsigned long )port);
	unsigned long nfault_timeout;
	unsigned long expire;
	byte ecr, fifo_state;
	int count;

	/* nfault_timeout indicates that it is time to check for nFault (this
	 * check is done in parport_ip32_fwp_wait_break) */
	nfault_timeout = min ((unsigned long )physport->cad->timeout,
			      msecs_to_jiffies (nfault_check_interval));
	expire = jiffies + physport->cad->timeout;
	count = 0;
	while (1) {
		if (parport_ip32_fwp_wait_break (port, expire))
			break;

		/* Initialize mutex used to take interrupts into account */
		init_MUTEX_LOCKED (&priv->irq);

		/* Enable serviceIntr */
		parport_ip32_frob_econtrol (port, ECR_SERVINTR, 0);

		/* Enabling serviceIntr while the FIFO is empty does not
		 * always generate an interrupt, so check for emptiness
		 * now. */
		ecr = parport_ip32_read_econtrol (port);
		if (! (ecr & ECR_F_EMPTY)) {
			/* FIFO is not empty: wait for an interrupt or a
			 * timeout to occur */
			mod_timer (&timer, jiffies + nfault_timeout);
			down_interruptible (&priv->irq);
			del_timer (&timer);
			ecr = parport_ip32_read_econtrol (port);
		}

		/* Disable serviceIntr */
		parport_ip32_frob_econtrol (port, ECR_SERVINTR, ECR_SERVINTR);

		fifo_state = ecr & (ECR_F_FULL | ECR_F_EMPTY);
		if (fifo_state != ECR_F_EMPTY && ! (ecr & ECR_SERVINTR)) {
			/* FIFO is not empty, and we did not get any
			 * interrupt.  Either it's time to check for nFault,
			 * or a signal is pending.  This is verified in
			 * parport_ip32_fwp_wait_break, so we continue the
			 * loop. */
			continue;
		}
		if (fifo_state == ECR_F_EMPTY) {
			/* FIFO is empty, fill it up */
			count = priv->fifo_depth;
			break;
		} else if (fifo_state == 0) {
			/* FIFO is not empty, but we can safely push
			 * writeIntrThreshold bytes into it*/
			count = priv->writeIntrThreshold;
			break;
		} else if (fifo_state == (ECR_F_FULL | ECR_F_EMPTY)) {
			/* Something wrong happened */
			count = -1;
			break;
		}
	} /* while (1) */

	return count;
}

/**
 * parport_ip32_fifo_write_pio_wait - wait until FIFO empties a bit
 * @port:	pointer to &struct parport
 *
 * Returns the number of bytes that can safely be written in the FIFO.  A
 * return value of zero means that the calling function should terminate as
 * fast as possible.  If an error is returned (value less than zero), the FIFO
 * must be reset.
 *
 * Actually, parport_ip32_fifo_write_wait() uses one of the two lower level
 * function: parport_ip32_fwp_wait_polling() which waits by using an active
 * polling loop, and parport_ip32_fwp_wait_interrupt() which uses the help of
 * interrupts.
 */
static inline int parport_ip32_fifo_write_pio_wait (struct parport *port)
{
	int r;
	if (port->irq == PARPORT_IRQ_NONE) {
		r = parport_ip32_fwp_wait_polling (port);
	} else {
		r = parport_ip32_fwp_wait_interrupt (port);
	}
	if (r < 0) {
		printk (KERN_DEBUG PPIP32
			"%s: FIFO error in %s, ecr=0x%02x\n",
			port->name, __func__,
			parport_ip32_read_econtrol (port));
	}
	return r;
}

/**
 * parport_ip32_fifo_write_block_pio - write a block of data (PIO)
 * @port:	pointer to &struct parport
 * @buf:	buffer of data to write
 * @len:	length of buffer @buf
 * @mode:	operation mode (ECP_MODE_PPF or ECP_MODE_ECP)
 *
 * Uses PIO to write the contents of the buffer @buf into the parallel port
 * FIFO.  Returns the number of bytes that were actually written.  It can work
 * with or without the help of interrupts.  The parallel port must be
 * correctly initialized before calling parport_ip32_fifo_write_block_pio().
 */
static size_t parport_ip32_fifo_write_block_pio (struct parport *port,
						 const void *buf, size_t len,
						 byte mode)
{
	void __iomem * const fifo = parport_ip32_fifo_addr (port, mode);
	const u8 *bufp = buf;
	size_t left = len;

	parport_ip32_dump_state (port, "begin fifo_write_block_pio", false);

	while (left > 0) {
		int count;

		count = parport_ip32_fifo_write_pio_wait (port);
		if (count <= 0) {
			/* Transmission should be stopped */
			break;
		}
		if (count > left) {
			count = left;
		}

		pr_debug (PPIP32 "%s: .. push %lu byte%s\n", port->name,
			  (unsigned long)count, (count > 1)? "s": "");

		/* Write next byte(s) to FIFO */
		if (count == 1) {
			parport_ip32_out (*bufp, fifo);
			bufp++, left--;
		} else {
			parport_ip32_out_rep (fifo, bufp, count);
			bufp += count, left -= count;
		}
	}

	pr_debug (PPIP32 "%s: .. transfer %s (left=%lu)\n", port->name,
		  left? "aborted": "completed", (unsigned long)left);

	parport_ip32_dump_state (port, "end fifo_write_block_pio", false);

	return (len - left);
}

/*
 * FIXME - Insert here parport_ip32_fifo_write_block_dma().
 */

/**
 * parport_ip32_fifo_write_initialize - initialize a forward FIFO transfer
 * @port:	pointer to &struct parport
 * @mode:	operation mode (ECR_MODE_PPF or ECR_MODE_ECP)
 *
 * This function resets the FIFO and sets appropriate operation mode.  It
 * returns true if the peripheral is ready, and false otherwise.
 */
static bool parport_ip32_fifo_write_initialize (struct parport *port,
						byte mode)
{
	bool ready;

	pr_trace (port, "(0x%02x)", mode);

	/* Reset FIFO, go in forward mode, and disable ackIntEn */
	parport_ip32_set_mode (port, ECR_MODE_PS2);
	parport_ip32_write_control (port, DCR_SELECT | DCR_nINIT);
	parport_ip32_data_forward (port);
	parport_ip32_disable_irq (port);

	/* Go in desired mode */
	parport_ip32_set_mode (port, mode);

	/* Wait for peripheral to become ready */
	ready = !parport_wait_peripheral (port,
					  DSR_nBUSY | DSR_nFAULT,
					  DSR_nBUSY | DSR_nFAULT);

	return ready;
}

/**
 * parport_ip32_drain_fifo - wait for FIFO to empty
 * @port:	pointer to &struct parport
 * @timeout:	timeout, in jiffies
 *
 * This function waits for FIFO to empty.  It returns true when FIFO is empty,
 * or false if the timeout @timeout is reached before, or if a signal is
 * pending.
 */
static bool parport_ip32_drain_fifo (struct parport *port,
				     unsigned long timeout)
{
	unsigned long expire = jiffies + timeout;
	unsigned int polling_interval;
	unsigned int counter;

	pr_trace (port, "(%ums)", jiffies_to_msecs (timeout));

	/* Busy wait for approx. 200us */
	for (counter = 0; counter < 40; counter++) {
		if (parport_ip32_read_econtrol (port) & ECR_F_EMPTY)
			break;
		if (time_after (jiffies, expire))
			break;
		if (signal_pending (current))
			break;
		udelay (5);
	}
	/* Poll slowly.  Polling interval starts with 1 millisecond, and is
	 * increased exponentially until 128.  */
	polling_interval = 1; /* msecs */
	while (! (parport_ip32_read_econtrol (port) & ECR_F_EMPTY)) {
		if (time_after_eq (jiffies, expire))
			break;
		msleep_interruptible (polling_interval);
		if (signal_pending (current))
			break;
		if (polling_interval < 128) {
			polling_interval *= 2;
		}
	}

	return !!(parport_ip32_read_econtrol (port) & ECR_F_EMPTY);
}

/**
 * parport_ip32_get_fifo_residue - reset FIFO
 * @port:	pointer to &struct parport
 * @mode:	operation mode (ECR_MODE_PPF or ECR_MODE_ECP)
 *
 * This function resets FIFO, and returns the number of bytes remaining in it.
 */
static unsigned int parport_ip32_get_fifo_residue (struct parport *port,
						   byte mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	void __iomem * const fifo = parport_ip32_fifo_addr (port, mode);
	unsigned int residue;
	byte cnfga;

	/* FIXME - We are missing one byte if the printer is off-line.  I
	 * don't know how to detect this.  It looks that the full bit is not
	 * always reliable.  For the moment, the problem is avoided in most
	 * cases by testing for BUSY in parport_ip32_fifo_write_initialize().
	 */

	pr_trace (port, "(0x%02x)", mode);

	if (parport_ip32_read_econtrol (port) & ECR_F_EMPTY) {
		residue = 0;
	} else {
		printk (KERN_DEBUG PPIP32 "%s: FIFO is stuck\n", port->name);

		/* Stop all transfers.
		 *
		 * Microsoft's document instructs to drive DCR_STROBE to 0,
		 * but it doesn't work (at least in Compatibility mode, not
		 * tested in ECP mode).  Switching directly to Test mode (as
		 * in parport_pc) is not an option: it does confuse the port,
		 * ECP service interrupts are no more working after that.  A
		 * hard reset is then needed to revert to a sane state.
		 *
		 * Let's hope that the FIFO is really stuck and that the
		 * peripheral doesn't wake up now.
		 */
		parport_ip32_frob_control (port, DCR_STROBE, 0);

		/* Fill up FIFO */
		for (residue = priv->fifo_depth; residue > 0; residue--) {
			if (parport_ip32_read_econtrol (port) & ECR_F_FULL)
				break;
			parport_ip32_out (0x00, fifo);
		}
	}

	if (residue) {
		pr_debug1 (PPIP32 "%s: %d PWord%s left in FIFO\n",
			   port->name, residue,
			   (residue == 1)? " was": "s were");
	}

	/* Now reset the FIFO, change to Config mode, and clean up */
	parport_ip32_set_mode (port, ECR_MODE_CFG);
	cnfga = parport_ip32_in (priv->regs.cnfgA);

	if (! (cnfga & CNFGA_nBYTEINTRANS)) {
		pr_debug1 (PPIP32 "%s: cnfgA contains 0x%02x\n",
			   port->name, cnfga);
		pr_debug1 (PPIP32 "%s: Accounting for extra byte\n",
			   port->name);
		residue ++;
	}

	/* Don't care about partial PWords until support is added for
	 * PWord != 1 byte. */

	/* Back to PS2 mode. */
	parport_ip32_set_mode (port, ECR_MODE_PS2);

	return residue;
}

/**
 * parport_ip32_fifo_write_finalize - finalize a forward FIFO transfer
 * @port:	pointer to &struct parport
 * @mode:	operation mode (ECR_MODE_PPF or ECR_MODE_ECP)
 *
 * Finalize a forward FIFO transfer.  Returns 1 if FIFO is empty and the
 * status of BUSY is low.  Returns -count otherwise, where count is the number
 * of bytes remaining in the FIFO.  Note that zero can be returned in case
 * there is a BUSY timeout.
 */
static int parport_ip32_fifo_write_finalize (struct parport *port, int mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	unsigned int ready;
	unsigned int residue;

	pr_trace (port, "(0x%02x)", mode);

	/* Wait FIFO to empty.  Timeout is proportional to FIFO_depth.  */
	parport_ip32_drain_fifo (port,
				 physport->cad->timeout * priv->fifo_depth);

	/* Check for a potential residue */
	residue = parport_ip32_get_fifo_residue (port, mode);

	/* Then, wait for BUSY to get low. */
	ready = !parport_wait_peripheral (port, DSR_nBUSY, DSR_nBUSY);
	if (! ready) {
		printk (KERN_DEBUG PPIP32 "%s: BUSY timeout\n", port->name);
	}

	/* Reset FIFO */
	parport_ip32_set_mode (port, ECR_MODE_PS2);

	if (residue) {
		return -residue;
	} else {
		return ready;
	}
}

/**
 * parport_ip32_fifo_write_block - write a block of data
 * @port:	pointer to &struct parport
 * @buf:	buffer of data to write
 * @len:	length of buffer @buf
 * @mode:	operation mode (ECP_MODE_PPF or ECP_MODE_ECP)
 *
 * Uses PIO or DMA to write the contents of the buffer @buf into the parallel
 * port FIFO.  Returns the number of bytes that were actually written.
 * Parameter @mode defines the operation mode, compatibility (ECR_MODE_PPF) or
 * ECP (ECR_MODE_ECP).
 */
static size_t parport_ip32_fifo_write_block (struct parport *port,
					     const void *buf, size_t len,
					     byte mode)
{
	static bool ready_before = true;
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	size_t written = 0;
	int r;

	if (len == 0) {
		/* There is nothing to do */
		goto out;
	}
	if (! parport_ip32_fifo_write_initialize (port, mode)) {
		/* Avoid to flood the logs */
		if (ready_before) {
			printk (KERN_INFO PPIP32 "%s: not ready\n",
				port->name);
		}
		ready_before = false;
		goto out;
	}
	ready_before = true;

	pr_trace (port, " -> len=%lu", (unsigned long)len);

	physport->ieee1284.phase = IEEE1284_PH_FWD_DATA;
	priv->irq_mode = PARPORT_IP32_IRQ_HERE;

	/* FIXME - Use parport_ip32_fifo_write_block_dma when available, and
	 * use_dma is true, and port->dma != PARPORT_DMA_NONE.  Maybe some
	 * threshold value should be set for @len under which we revert to PIO
	 * mode?
	 */
	written = parport_ip32_fifo_write_block_pio (port, buf, len, mode);

	/* Finalize the transfer */
	r = parport_ip32_fifo_write_finalize (port, mode);
	if (r < 0) {
		written += r;
	}

	priv->irq_mode = PARPORT_IP32_IRQ_FWD;
	physport->ieee1284.phase = IEEE1284_PH_FWD_IDLE;

	pr_trace (port, " <- written=%lu", (unsigned long)written);

out:
	return written;
}

/**
 * parport_ip32_compat_write_data - write a block of data in compatibility mode
 * @port:	pointer to &struct parport
 * @buf:	buffer of data to write
 * @len:	length of buffer @buf
 * @flags:	ignored
 */
static size_t parport_ip32_compat_write_data (struct parport *port,
					      const void *buf, size_t len,
					      int flags)
{
	struct parport * const physport = port->physport;
	size_t written;

	pr_trace (port, "(...)");

	/* Special case: a timeout of zero means we cannot call schedule().
	 * Also if O_NONBLOCK is set then use the default implementation. */
	if (!use_fifo ||
	    physport->cad->timeout <= PARPORT_INACTIVITY_O_NONBLOCK) {
		written = parport_ieee1284_write_compat (port, buf, len,
							 flags);
	} else {
		written = parport_ip32_fifo_write_block (port, buf, len,
							 ECR_MODE_PPF);
	}
	return written;
}

#if defined(PARPORT_IP32_ECP)

/*
 * FIXME - Insert here parport_ip32_ecp_{read,write}_{data,address}().
 */

#endif /* defined(PARPORT_IP32_ECP) */

#endif /* defined(CONFIG_PARPORT_IP32_FIFO) */

/*--- Default operations -----------------------------------------------*/

/* To ensure the correct types, define wrappers around parport operations. */

#define parport_byte    unsigned char
#define parport_read_wrapper(f)				      \
	static parport_byte f ## _wrapper (struct parport *p) \
	{ return f (p); }
#define parport_write_wrapper(f)				      \
	static void f ## _wrapper (struct parport *p, parport_byte v) \
	{ f (p, v); }
#define parport_frob_wrapper(f)						\
	static parport_byte f ## _wrapper (struct parport *p,           \
					   parport_byte m, parport_byte v) \
	{ return f (p, m, v); }

parport_read_wrapper (parport_ip32_read_data)
parport_read_wrapper (parport_ip32_read_control)
parport_read_wrapper (parport_ip32_read_status)
parport_write_wrapper (parport_ip32_write_data)
parport_write_wrapper (parport_ip32_write_control)
parport_frob_wrapper (parport_ip32_frob_control)

#undef parport_frob_wrapper
#undef parport_write_wrapper
#undef parport_read_wrapper
#undef parport_byte

static __initdata struct parport_operations parport_ip32_ops = {
	.write_data		= parport_ip32_write_data_wrapper,
	.read_data		= parport_ip32_read_data_wrapper,

	.write_control		= parport_ip32_write_control_wrapper,
	.read_control		= parport_ip32_read_control_wrapper,
	.frob_control		= parport_ip32_frob_control_wrapper,

	.read_status		= parport_ip32_read_status_wrapper,

	.enable_irq		= parport_ip32_enable_irq,
	.disable_irq		= parport_ip32_disable_irq,

	.data_forward		= parport_ip32_data_forward,
	.data_reverse		= parport_ip32_data_reverse,

	.init_state		= parport_ip32_init_state,
	.save_state		= parport_ip32_save_state,
	.restore_state		= parport_ip32_restore_state,

	.epp_write_data		= parport_ieee1284_epp_write_data,
	.epp_read_data		= parport_ieee1284_epp_read_data,
	.epp_write_addr		= parport_ieee1284_epp_write_addr,
	.epp_read_addr		= parport_ieee1284_epp_read_addr,

	.ecp_write_data		= parport_ieee1284_ecp_write_data,
	.ecp_read_data		= parport_ieee1284_ecp_read_data,
	.ecp_write_addr		= parport_ieee1284_ecp_write_addr,

	.compat_write_data	= parport_ieee1284_write_compat,
	.nibble_read_data	= parport_ieee1284_read_nibble,
	.byte_read_data		= parport_ieee1284_read_byte,

	.owner			= THIS_MODULE,
};

/*--- Device detection -------------------------------------------------*/

/**
 * parport_ip32_ecr_supported - check if the ECR is functional
 * @p:	pointer to the &parport structure
 *
 * Check if the Extended Control Register is functional.  Returns false if it
 * is not.  Adjust parameters in the &parport structure.  On return, the port
 * is placed in SPP mode.
 *
 * Old style XT ports alias IO ports every 0x400, hence accessing ECR on these
 * cards actually accesses the DCR.  Modern cards don't do this but reading
 * from ECR will return 0xff regardless of what is written here if the card
 * does NOT support ECP.
 *
 * We first check to see if ECR is the same as DCR.  If not, the low two bits
 * of ECR aren't writable, so we check by writing ECR and reading it back to
 * see if it's what we expect.
 */
static __init bool parport_ip32_ecr_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	byte dcr, ecr, mask;

	parport_ip32_out (DCR_SELECT | DCR_nINIT, priv->regs.dcr);
	dcr = parport_ip32_in (priv->regs.dcr);
	mask = ECR_F_FULL | ECR_F_EMPTY;
	if ((parport_ip32_in (priv->regs.ecr) & mask) == (dcr & mask)) {
		/* Toggle bit ECR_F_FULL */
		mask = ECR_F_FULL;
		parport_ip32_out (dcr ^ mask, priv->regs.dcr);
		dcr = parport_ip32_in (priv->regs.dcr);
		if ((parport_ip32_in (priv->regs.ecr) & mask) == (dcr & mask))
			goto no_reg; /* Sure that no ECR register exists */
	}

	ecr = parport_ip32_in (priv->regs.ecr) & (ECR_F_FULL | ECR_F_EMPTY);
	if (ecr != ECR_F_EMPTY)
		goto no_reg;

	ecr = ECR_MODE_PS2 | ECR_nERRINTR | ECR_SERVINTR;
	parport_ip32_out (ecr, priv->regs.ecr);
	if (parport_ip32_in (priv->regs.ecr) != (ecr | ECR_F_EMPTY))
		goto no_reg;

	pr_probe (p, "Found working ECR register\n");
	ecr = ECR_MODE_SPP | ECR_nERRINTR | ECR_SERVINTR;
	priv->ecr_present = 1;
	priv->ecr_init = ecr;
	parport_ip32_write_control (p, DCR_SELECT | DCR_nINIT);
	parport_ip32_write_econtrol (p, ecr);
	return true;

no_reg:
	pr_probe (p, "ECR register not found\n");
	priv->ecr_present = 0;
	return false;
}

/**
 * parport_ip32_spp_supported - check for SPP mode
 * @p:	pointer to the &parport structure
 *
 * All ports support SPP mode. Check if the port behaves correctly.  Returns
 * false if there is something wrong.  Adjust parameters in the parport
 * structure.
 */
static __init bool parport_ip32_spp_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	byte r, w;

	if (priv->ecr_present) {
		parport_ip32_set_mode (p, ECR_MODE_SPP);
	}

	/* first clear an eventually pending EPP timeout
	 * I (sailer@ife.ee.ethz.ch) have an SMSC chipset
	 * that does not even respond to SPP cycles if an EPP
	 * timeout is pending
	 */
	parport_ip32_clear_epp_timeout (p);

	/* Do a simple read-write test to make sure the port exists. */
	w = 0x0c;		/* DCR_SELECT | DCR_nINIT */
	parport_ip32_out (w, priv->regs.dcr);

	/* Is there a control register that we can read from?  We do not care,
	 * as read_control just returns a software copy. */

	/* Try the data register.  The data lines aren't tri-stated at
	 * this stage, so we expect back what we wrote. */
	w = 0xaa;
	parport_ip32_write_data (p, w);
	r = parport_ip32_read_data (p);
	if (r != w)
		goto spp_nok;
	w = 0x55;
	parport_ip32_write_data (p, w);
	r = parport_ip32_read_data (p);
	if (r != w)
		goto spp_nok;

	pr_probe (p, "Found working SPP register\n");
	p->modes |= PARPORT_MODE_PCSPP;
	/* FIXME - What does SAFEININT exactly mean?  Is there some method to
	 * safely test for it?  Activate it unless someone complains.
	 */
	p->modes |= PARPORT_MODE_SAFEININT;
	priv->dcr_init = DCR_SELECT | DCR_nINIT;
	parport_ip32_write_control (p, priv->dcr_init);
	return true;

spp_nok:
	/* No parallel port found. */
	pr_probe (p, "SPP register not found\n");
	return false;
}

/**
 * parport_ip32_ps2_supported - check for bidirectional mode
 * @p:	pointer to the &parport structure
 *
 * Check if the port supports bidirectional (a.k.a. PS/2) mode.  Returns false
 * if mode is not supported.  Adjust parameters in the &parport structure.
 *
 * Bit DCR_DIR sets the PS/2 data direction; setting this high allows us to
 * read data from the data lines.  In theory we would get back 0xff but any
 * peripheral attached to the port may drag some or all of the lines down to
 * zero.  So if we get back anything that isn't the contents of the data
 * register we deem PS/2 support to be present.
 */
#if defined(PARPORT_IP32_PS2)
static __init bool parport_ip32_ps2_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	bool ok = false;

	if (priv->ecr_present) {
		parport_ip32_set_mode (p, ECR_MODE_PS2);
	}

	parport_ip32_clear_epp_timeout (p);

	/* Try to tri-state the buffer. */
	priv->dcr_writable |= DCR_DIR;
	parport_ip32_data_reverse (p);

	parport_ip32_write_data (p, 0x55);
	ok = ok || (parport_ip32_read_data (p) != 0x55);
	parport_ip32_write_data (p, 0xaa);
	ok = ok || (parport_ip32_read_data (p) != 0xaa);

	pr_probe (p, "PS2 mode%s supported\n", ok? "": " not");

	/* cancel input mode */
	parport_ip32_data_forward (p);

	if (ok) {
		p->modes |= PARPORT_MODE_TRISTATE;
		/* change default mode to PS2 */
		priv->ecr_init &= ~ECR_MODE_MASK;
		priv->ecr_init |= ECR_MODE_PS2;
	} else {
		priv->dcr_writable &= ~DCR_DIR;
	}

	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, priv->ecr_init);
	}

	return ok;
}
#else /* ! defined(PARPORT_IP32_PS2) */
#define parport_ip32_ps2_supported(...)		false
#endif

/**
 * parport_ip32_epp_supported - check for Enhanced Parallel Port
 * @p:	pointer to the &parport structure
 *
 * Check for Enhanced Parallel Port.  Returns false if mode is not supported.
 * Adjust parameters in the &parport structure.
 */
#if defined(PARPORT_IP32_EPP)
static __init bool parport_ip32_epp_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	bool ok;

	if (priv->ecr_present) {
		parport_ip32_set_mode (p, ECR_MODE_EPP);
	}

	/* If EPP timeout bit clear then EPP available */
	ok = parport_ip32_clear_epp_timeout (p);

	pr_probe (p, "EPP mode%s supported\n", ok? "": " not");

	if (ok) {
		/* Set up access functions to use EPP hardware. */
#if 0				/* not implemented */
		p->ops->epp_read_data = parport_ip32_epp_read_data;
		p->ops->epp_write_data = parport_ip32_epp_write_data;
		p->ops->epp_read_addr = parport_ip32_epp_read_addr;
		p->ops->epp_write_addr = parport_ip32_epp_write_addr;
		p->modes |= PARPORT_MODE_EPP;
		pr_probe (p, "Hardware support for EPP mode enabled\n");
#endif
	}

	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, priv->ecr_init);
	}

	return ok;
}
#else  /* ! defined(PARPORT_IP32_EPP) */
#define parport_ip32_epp_supported(...)		false
#endif

/**
 * parport_ip32_ecp_supported - check for Extended Capabilities Port
 * @p:	pointer to the &parport structure
 *
 * Check for Extended Capabilities Port.  Returns false if mode is not
 * supported.  Adjust parameters in the parport structure.  Actually the main
 * job of this function is to check if the FIFO is working correctly, and to
 * find its parameters.  It also enables hardware compatibility mode.
 */
#if defined(CONFIG_PARPORT_IP32_FIFO)
static __init bool parport_ip32_ecp_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	byte configa, configb;
	unsigned int pword;
	unsigned int i;

	/* If there is no ECR, we have no hope of supporting ECP. */
	if (! priv->ecr_present)
		return false;
	/* We must be able to tri-state the port. */
	if (! (p->modes & PARPORT_MODE_TRISTATE)) {
		pr_probe (p, "ECP modes not checked (can't TRISTATE)\n");
		return false;
	}

	/* Configuration mode */
	parport_ip32_set_mode (p, ECR_MODE_CFG);

	configa = parport_ip32_in (priv->regs.cnfgA);
	configb = parport_ip32_in (priv->regs.cnfgB);

	pr_probe (p, "ECP port cnfgA=0x%02x cnfgB=0x%02x\n", configa, configb);
	pr_probe (p, "Interrupts are ISA-%s\n",
		  (configa & CNFGA_IRQ)? "Level": "Pulses");
	pr_probe (p, "ECP settings irq=");
	if (CNFGB_IRQ (configb)) {
		__pr_probe ("%d", CNFGB_IRQ (configb));
	} else {
		__pr_probe ("<none or set by other means>");
	}
	__pr_probe (" dma=");
	if (CNFGB_DMA (configb)) {
		__pr_probe ("%d\n", CNFGB_DMA (configb));
	} else {
		__pr_probe ("<none or set by other means>\n");
	}

	/* Check for interrupt conflict */
	if (! (configb & CNFGB_INTRVAL) && p->irq != PARPORT_IRQ_NONE) {
		printk (KERN_NOTICE PPIP32
			"0x%lx: IRQ conflict detected, disabling IRQ\n",
			p->base);
		p->irq = PARPORT_IRQ_NONE;
	}

	/* Check for compression support */
	parport_ip32_out (configb | CNFGB_COMPRESS, priv->regs.cnfgB);
	if (parport_ip32_in (priv->regs.cnfgB) & CNFGB_COMPRESS) {
		pr_probe (p, "Hardware compression detected (unsupported)\n");
	}
	parport_ip32_out (configb & ~CNFGB_COMPRESS, priv->regs.cnfgB);

	/* Find out PWord size */
	switch (configa & CNFGA_ID_MASK) {
	case CNFGA_ID_8:	pword = 1; break;
	case CNFGA_ID_16:	pword = 2; break;
	case CNFGA_ID_32:	pword = 4; break;
	default:
		pr_probe (p, "Unknown implementation ID: 0x%0x\n",
			  (configa & CNFGA_ID_MASK) >> CNFGA_ID_SHIFT);
		goto ecp_error;
		break;
	}
	if (pword != 1) {
		pr_probe (p, "Unsupported PWord size: %u\n", pword);
		goto ecp_error;
	}
	priv->pword = pword;
	pr_probe (p, "PWord is %u bits\n", 8 * priv->pword);

	/* Reset FIFO and go in test mode (no interrupt, no DMA) */
	parport_ip32_set_mode (p, ECR_MODE_TST);

	/* FIFO must be empty now */
	if (! (parport_ip32_in (priv->regs.ecr) & ECR_F_EMPTY)) {
		pr_probe (p, "FIFO not reset\n");
		goto ecp_error;
	}

	/* Find out FIFO depth. */
	priv->fifo_depth = 0;
	for (i = 0; i < 1024; i++) {
		if (parport_ip32_in (priv->regs.ecr) & ECR_F_FULL) {
			/* FIFO full */
			priv->fifo_depth = i;
			break;
		}
		parport_ip32_out ((u8 )i, priv->regs.tFifo);
	}
	if (i >= 1024) {
		pr_probe (p, "Can't fill FIFO\n");
		goto ecp_error;
	}
	if (! priv->fifo_depth) {
		pr_probe (p, "Can't get FIFO depth\n");
		goto ecp_error;
	}
	pr_probe (p, "FIFO is %u PWords deep\n", priv->fifo_depth);

	/* Enable interrupts */
	parport_ip32_frob_econtrol (p, ECR_SERVINTR, 0);

	/* Find out writeIntrThreshold: number of PWords we know we can write
	 * if we get an interrupt. */
	priv->writeIntrThreshold = 0;
	for (i = 0; i < priv->fifo_depth; i++) {
		if (parport_ip32_in (priv->regs.tFifo) != (u8 )i) {
			pr_probe (p, "Invalid data in FIFO\n");
			goto ecp_error;
		}
		if (! priv->writeIntrThreshold
		    && parport_ip32_in (priv->regs.ecr) & ECR_SERVINTR) {
			/* writeIntrThreshold reached */
			priv->writeIntrThreshold = i + 1;
		}
		if (i + 1 < priv->fifo_depth
		    && parport_ip32_in (priv->regs.ecr) & ECR_F_EMPTY) {
			/* FIFO empty before the last byte? */
			pr_probe (p, "Data lost in FIFO\n");
			goto ecp_error;
		}
	}
	if (! priv->writeIntrThreshold) {
		pr_probe (p, "Can't get writeIntrThreshold\n");
		goto ecp_error;
	}
	pr_probe (p, "writeIntrThreshold is %u\n", priv->writeIntrThreshold);

	/* FIFO must be empty now */
	if (! (parport_ip32_in (priv->regs.ecr) & ECR_F_EMPTY)) {
		pr_probe (p, "Can't empty FIFO\n");
		goto ecp_error;
	}

	/* Reset FIFO */
	parport_ip32_set_mode (p, ECR_MODE_PS2);
	/* Set reverse direction (must be in PS2 mode) */
	parport_ip32_data_reverse (p);
	/* Test FIFO, no interrupt, no DMA */
	parport_ip32_set_mode (p, ECR_MODE_TST);
	/* Enable interrupts */
	parport_ip32_frob_econtrol (p, ECR_SERVINTR, 0);

	/* Find out readIntrThreshold: number of PWords we can read if we get
	 * an interrupt. */
	priv->readIntrThreshold = 0;
	for (i = 0; i < priv->fifo_depth; i++) {
		parport_ip32_out (0xaa, priv->regs.tFifo);
		if (! priv->readIntrThreshold
		    && parport_ip32_in (priv->regs.ecr) & ECR_SERVINTR) {
			/* readIntrThreshold reached */
			priv->readIntrThreshold = i + 1;
		}
	}
	if (! priv->readIntrThreshold) {
		pr_probe (p, "Can't get readIntrThreshold\n");
		goto ecp_error;
	}
	pr_probe (p, "readIntrThreshold is %u\n", priv->readIntrThreshold);

	/* Reset ECR */
	parport_ip32_set_mode (p, ECR_MODE_PS2);
	parport_ip32_data_forward (p);
	parport_ip32_write_econtrol (p, priv->ecr_init);

	/* Enable FIFO compatibility mode */
	p->ops->compat_write_data = parport_ip32_compat_write_data;
	p->modes |= PARPORT_MODE_COMPAT;
	pr_probe (p, "Hardware support for compatibility mode enabled\n");

#if defined(PARPORT_IP32_ECP)
#if 0				/* not implemented */
	p->ops->ecp_write_data = parport_ip32_ecp_write_data,
	p->ops->ecp_read_data  = parport_ip32_ecp_read_data,
	p->ops->ecp_write_addr = parport_ip32_ecp_write_addr,
	p->modes |= PARPORT_MODE_ECP;
	pr_probe (p, "Hardware support for ECP modes enabled\n");
#endif
#endif /* defined(PARPORT_IP32_ECP) */

	return true;

ecp_error:
	priv->fifo_depth = 0;
	parport_ip32_write_econtrol (p, priv->ecr_init);
	return false;
}
#else /* ! defined(CONFIG_PARPORT_IP32_FIFO) */
#define parport_ip32_ecp_supported(...)		false
#endif

/**
 * parport_ip32_probe_port - probe and register a new port
 * @base:	base I/O address for standard and EPP registers
 * @base_hi:	base I/O address for ECP registers
 * @irq:	IRQ line to use
 * @dma:	DMA channel to use
 * @regs:	pointer to the port register addresses structure
 *		NOTE: the structure is copied and can safely be freed
 *		      after call
 * @modes:	bit mask of extended modes to probe, combination of
 *		PARPORT_MODE_EPP and PARPORT_MODE_ECP
 *
 * Returns the new allocated &parport structure.  Parameters @base and
 * @base_hi are only used to fill in the &parport structure.
 */
static __init
struct parport *parport_ip32_probe_port (unsigned long base,
					 unsigned long base_hi,
					 int irq, int dma,
					 const struct parport_ip32_regs *regs,
					 unsigned int modes)
{
	struct parport_ip32_private *priv;
	struct parport_operations *ops;
	struct parport *p;

	switch (irq) {
	case PARPORT_IRQ_AUTO:
	case PARPORT_IRQ_PROBEONLY:
		printk (KERN_DEBUG PPIP32 "irq probing is not supported\n");
		irq = PARPORT_IRQ_NONE;
		break;
	}

	switch (dma) {
	case PARPORT_DMA_AUTO:
		printk (KERN_DEBUG PPIP32 "dma probing is not supported\n");
	case PARPORT_DMA_NOFIFO:
		dma = PARPORT_DMA_NONE;
		break;
	}

	ops = kmalloc (sizeof *ops, GFP_KERNEL);
	priv = kmalloc (sizeof *priv, GFP_KERNEL);
	/* A misnomer, actually it's allocate and reserve parport number. */
	p = parport_register_port(base, irq, dma, ops);
	if (! ops || ! priv || ! p) {
		goto error;
	}

	/* Initialize parport structure. Be conservative. */
	*ops = parport_ip32_ops;
	*priv = (struct parport_ip32_private ){
		.regs			= *regs,
		.dcr_init		= DCR_SELECT | DCR_nINIT,
		.ecr_init		= ECR_MODE_SPP |
					  ECR_nERRINTR | ECR_SERVINTR,
		.dcr_cache		= 0,
		.dcr_writable		= DCR_SELECT | DCR_nINIT |
					  DCR_AUTOFD | DCR_STROBE,
		/* Required for the first parport_ip32_dump_state */
		.ecr_present		= !!(modes & PARPORT_MODE_ECP),
		.pword			= 1,
		.fifo_depth		= 0,
		.readIntrThreshold	= 0,
		.writeIntrThreshold	= 0,
		.irq_mode		= PARPORT_IP32_IRQ_FWD,
	};

	p->private_data = priv;
	p->base_hi = base_hi;

	parport_ip32_dump_state (p, "begin init", true);

	/* First, check if we can use the Extended Control Register. */
	if (modes & PARPORT_MODE_ECP) {
		parport_ip32_ecr_supported (p);
	}
	parport_ip32_dump_state (p, "after ECR test", false);

	/* If SPP mode does not work, we can't go very far. */
	if (! parport_ip32_spp_supported (p)) {
		goto error;
	}
	parport_ip32_dump_state (p, "after SPP test", false);

	/* Is the port bidirectional? */
	parport_ip32_ps2_supported (p);
	parport_ip32_dump_state (p, "after PS2 test", false);

	if (modes & PARPORT_MODE_EPP) {
		parport_ip32_epp_supported (p);
	}
	parport_ip32_dump_state (p, "after EPP test", false);

	if (priv->ecr_present) {
		parport_ip32_ecp_supported (p);
	}
	parport_ip32_dump_state (p, "after ECP test", false);

	/* Request IRQ */
	if (p->irq != PARPORT_IRQ_NONE) {
		if (request_irq (p->irq, parport_ip32_interrupt,
				 0, p->name, p)) {
			printk (KERN_NOTICE PPIP32 "%s: irq %d in use, "
				"resorting to polled operation\n",
				p->name, p->irq);
			p->irq = PARPORT_IRQ_NONE;
/*			p->dma = PARPORT_DMA_NONE;	FIXME - Really? */
			priv->dcr_writable &= ~DCR_IRQ;
		} else {
			priv->dcr_writable |= DCR_IRQ;
		}
	}

	/* FIXME - Allocate DMA resources here. */

	/* Initialize the port with sensible values */
	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, priv->ecr_init);
	}
	parport_ip32_write_control (p, priv->dcr_init);
	parport_ip32_data_forward (p);
	parport_ip32_disable_irq (p);
	parport_ip32_write_data (p, 0x00);

	parport_ip32_dump_state (p, "end init", false);

	/* Print what we found */
	printk (KERN_INFO "%s: SGI IP32 at 0x%lx", p->name, p->base);
	if (p->base_hi) {
		printk (" (0x%lx)", p->base_hi);
	}
	if (p->irq != PARPORT_IRQ_NONE) {
		printk (", irq %d", p->irq);
	}
	if (p->dma != PARPORT_DMA_NONE) {
		printk (", dma %d", p->dma);
	}
	printk (" [");
#define printmode(x)	if (p->modes & PARPORT_MODE_##x)		\
				printk ("%s%s", f++? ",": "", #x)
	{
		unsigned int f = 0;
		printmode (PCSPP);
		printmode (TRISTATE);
		printmode (COMPAT);
		printmode (EPP);
		printmode (ECP);
		printmode (DMA);
	}
#undef printmode
#if ! defined(CONFIG_PARPORT_1284)
	printk ("(,...)");
#endif
	printk ("]\n");

	/* Now that we've told the sharing engine about the port, and found
	 * out its characteristics, let the high-level drivers know about
	 * it. */
	parport_announce_port (p);

	return p;

error:
	if (p) {
		parport_put_port(p);
	}
	kfree (priv);
	kfree (ops);
	return NULL;
}

/**
 * parport_ip32_unregister_port - unregister a parallel port
 * @p:	pointer to the &struct parport
 *
 * Unregisters a parallel port and free previously allocated resources
 * (memory, IRQ, ...).
 */
static __exit void parport_ip32_unregister_port (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	struct parport_operations *ops = p->ops;

	parport_remove_port(p);

	if (p->irq != PARPORT_IRQ_NONE)
		free_irq(p->irq, p);

	/* FIXME - Release DMA resources here. */

	parport_put_port(p);
	kfree (priv);
	kfree (ops);
}

/*--- Initialization code ----------------------------------------------*/

/**
 * parport_ip32_init - module initialization function
 */
static int __init parport_ip32_init (void)
{
	struct parport_ip32_regs regs;
	unsigned int modes;

	pr_info (PPIP32 "%s v%s\n", DRV_DESCRIPTION, DRV_VERSION);
	pr_debug1 (PPIP32 "Compiled on %s, %s\n", __DATE__, __TIME__);

	parport_ip32_iomap_mace_address ();
	if (mace == NULL) {
		printk (KERN_DEBUG PPIP32 "invalid mace pointer\n");
		return -ENXIO;
	}

	parport_ip32_make_isa_registers (&regs,
					 PARPORT_IP32_IO_ADDR,
					 PARPORT_IP32_IOHI_ADDR,
					 PARPORT_IP32_REGSHIFT);
	modes = PARPORT_MODE_EPP | PARPORT_MODE_ECP;
	this_port = parport_ip32_probe_port (PARPORT_IP32_IO,
					     PARPORT_IP32_IOHI,
					     param_irq, param_dma,
					     &regs, modes);
	if (this_port == NULL) {
		printk (KERN_DEBUG PPIP32 "failed to probe port\n");
		parport_ip32_iounmap_mace_address ();
		return -ENODEV;
	}

	return 0;
}

/**
 * parport_ip32_exit - module termination function
 */
static void __exit parport_ip32_exit (void)
{
	parport_ip32_unregister_port (this_port);
	parport_ip32_iounmap_mace_address ();
}

/*--- Module stuff -----------------------------------------------------*/

#undef bool			/* it breaks module_param if defined */

MODULE_AUTHOR (DRV_AUTHOR);
MODULE_DESCRIPTION (DRV_DESCRIPTION);
MODULE_LICENSE (DRV_LICENSE);
MODULE_VERSION (DRV_VERSION);

module_init (parport_ip32_init);
module_exit (parport_ip32_exit);

module_param_named (verbose_probing, param_verbose_probing, bool, S_IRUGO);
MODULE_PARM_DESC (verbose_probing, "Log chit-chat during initialization");

module_param_named (irq, param_irq, int, S_IRUGO);
MODULE_PARM_DESC (irq, "IRQ line (-1 to disable)");

#if 0				/* DMA is not supported yet */
module_param_named (dma, param_dma, int, S_IRUGO);
MODULE_PARM_DESC (dma, "DMA channel (-1 to disable)");
#endif

#if defined(CONFIG_PARPORT_IP32_FIFO)
module_param (use_fifo, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC (use_fifo, "Use hardware FIFO modes if available");
#endif

#if 0				/* DMA is not supported yet */
module_param (use_dma, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC (use_dma, "Use DMA if available");
#endif

/*--- Inform (X)Emacs about preferred coding style ---------------------*/
/*
 * Local Variables:
 * mode: c
 * c-file-style: "linux"
 * indent-tabs-mode: t
 * tab-width: 8
 * fill-column: 78
 * ispell-local-dictionary: "american"
 * End:
 */
