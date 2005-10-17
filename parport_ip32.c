/* Low-level parallel port routines for built in port on SGI IP32
 *
 * Author: Arnaud Giersch <arnaud.giersch@free.fr>
 *
 * $Id: parport_ip32.c,v 1.21 2005-10-17 18:39:31 arnaud Exp $
 *
 * based on parport_pc.c by
 *	Phil Blundell <philb@gnu.org>
 *	Tim Waugh <tim@cyberelk.demon.co.uk>
 *	Jose Renau <renau@acm.org>
 *	David Campbell <campbell@torque.net>
 *	Andrea Arcangeli
 *	Russell King <linux@arm.uk.linux.org>
 *	Bert De Jonghe <bert@sophis.be>
 *	Fred Barnes & Jamie Lokier
 *	Paul G.
 *	Fred Barnes
 *	Adam Belay <ambx1@neo.rr.com>
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
 *	Basic modes: PCSPP, PS2.
 *	Compatibility mode with FIFO support.
 *	FIFO can be driven with or without interrupts.
 *
 *	DMA support is not implemented.
 *	EPP and ECP modes are not implemented.
 *
 * History:
 *
 * v0.8 -- ...
 *	Corrected parport_ip32_get_fifo_residue.
 *	Added parport_ip32_drain_fifo.
 *	Timeout proportionnal to writeIntrThreshold in
 *	 parport_ip32_fifo_write_pio_wait
 *	It's ok to feed NULL pointer to kfree.
 *	Use __func__ instead of __FUNCTION__.
 *	Get rid of parport_ip32_frob_set_mode,
 *	 introduce parport_ip32_set_mode.
 *
 * v0.7 -- Thu, 13 Oct 2005 23:49:23 +0200
 *	Fix typo: left instead of len if parport_ip32_write_block_pio!
 *	Moved DRV_* at top of source file.
 *
 * v0.6 -- Thu, 13 Oct 2005 11:09:08 +0200
 *	Added possibility to disable FIFO at run-time.
 *	Added PARPORT_IP32_FIFO in KConfig.
 *	More consistent names for register macros.
 *	Check for nFault too, before starting compat_write_data.
 *	Added some module_param.
 *	Checked types.
 *	Moved {init,final}ization in parport_ip32_fifo_write_block.
 *	In parport_ip32_fifo_write_pio: check for BUSY before starting
 *	 transfer.
 *	Improved parport_ip32_fifo_write_wait.
 *	Corrected FIFO tests.
 *	Added dump_parport_state in parport_ip32_debug_irq_handler.
 *	Defined NO_OP().
 *	Added MODULE_VERSION.
 *
 * v0.5 -- Mon, 10 Oct 2005 02:29:18 +0200
 *	Improved FIFO testing.
 *	Added DEBUG_IP32_IRQ.
 *	Added FIFO support (compatibility mode).
 *	Do not initialize ECR if it is not present.
 *	Code cleanup.
 *
 * v0.4 -- Fri, 07 Oct 2005 00:57:06 +0200
 *	Major rewrite.
 *
 * v0.3 -- Tue, 04 Oct 2005 22:10:52 +0200
 *	Added Compatibility FIFO mode (PIO).
 *      Added code for EPP support (not tested).
 *      Disable interrupts: it is too slow to get an interrupt per char
 *       written!
 *
 * v0.2 -- Sun, 02 Oct 2005 19:52:04 +0200
 *	Interrupts are working in SPP mode.
 *
 * v0.1 -- Sun, 02 Oct 2005 16:18:54 +0200
 *	First working version. Only SPP/PS2 modes are supported, without
 *	 interrupts.
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
 * it is for the built-in serial ports).
 *
 * Big chunks of code were directly adapted from parport_pc. A better approach
 * would certainly be to make the corresponding code arch-independent, with
 * some generic functions for register access.
 */

/*----------------------------------------------------------------------*/

#define DRV_NAME	"parport_ip32"
#define DRV_DESCRIPTION	"SGI IP32 built-in parallel port driver"
#define DRV_AUTHOR	"Arnaud Giersch <arnaud.giersch@free.fr>"
#define DRV_LICENSE	"GPL"
#define DRV_VERSION	"0.8pre"

/*--- Some configuration defines ---------------------------------------*/

/* DEBUG_PARPORT_IP32
 *	0	disable debug
 *	1	standard level: pr_debug1 is enabled
 *	2	dump_parport_state is enabled
 *	>2	verbose level: pr_debug is enabled
 */
#define DEBUG_PARPORT_IP32	1 /* disable for production */

/* If defined, include IRQ handlers for MACEISA_PAR_{CTXA,CTXB,MERR}_IRQ
 * interrupts.  I don't know if this interrupts have any utility.  */
#undef DEBUG_IP32_IRQ		/* disable for production */

/* Un-define to disable support for a particular mode. */
#define PARPORT_IP32_PS2
#define PARPORT_IP32_EPP	/* not implemented */
#define PARPORT_IP32_ECP	/* not implemented */

/*----------------------------------------------------------------------*/

/* Setup DEBUG macros. */
#if DEBUG_PARPORT_IP32 == 0
#	undef DEBUG_PARPORT_IP32
#elif DEBUG_PARPORT_IP32 == 1
#	warning DEBUG_PARPORT_IP32 == 1
#elif DEBUG_PARPORT_IP32 == 2
#	warning DEBUG_PARPORT_IP32 == 2
#elif DEBUG_PARPORT_IP32 > 2
#	warning DEBUG_PARPORT_IP32 > 2
#	if ! defined(DEBUG)
#		define DEBUG /* enable pr_debug() in kernel.h */
#	endif
#endif

#if defined(DEBUG_IP32_IRQ)
#	warning DEBUG_IP32_IRQ enabled
#endif

#include <linux/config.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/parport.h>
#include <linux/stddef.h>
#include <asm/io.h>
#include <asm/types.h>

#define NO_OP()		do { } while (0)

/*--- Parameters for SGI O2 built-in parallel port ---------------------*/

#include <asm/ip32/mace.h>
#include <asm/ip32/ip32_ints.h>

/* Physical I/O addresses */
#define PARPORT_IP32_IO	  (MACE_BASE +				\
			   offsetof (struct sgi_mace, isa) +	\
			   offsetof (struct mace_isa, parallel))
#define PARPORT_IP32_IOHI (MACE_BASE +				\
			   offsetof (struct sgi_mace, isa) +	\
			   offsetof (struct mace_isa, ecp1284))

#define PARPORT_IP32_IRQ	MACEISA_PARALLEL_IRQ
#define PARPORT_IP32_DMA	PARPORT_DMA_NONE

/* Memory mapped I/O addresses */
#define PARPORT_IP32_IO_ADDR   ((void *)&mace->isa.parallel)
#define PARPORT_IP32_IOHI_ADDR ((void *)&mace->isa.ecp1284)

/* Registers are replicated 256 times */
#define PARPORT_IP32_REGSHIFT	8

/* Pointer mace is not exported in arch/mips/sgi-ip32/crime.c,
 * use our own pointer for modules.
 */
#if defined(MODULE)
static struct sgi_mace __iomem *mace = NULL;
static inline void iomap_mace_address (void)
{
	mace = ioremap (MACE_BASE, sizeof (struct sgi_mace));
}
static inline void iounmap_mace_address (void)
{
	iounmap (mace);
}
#else /* ! defined(MODULE) */
#define iomap_mace_address(...)		NO_OP()
#define iounmap_mace_address(...)	NO_OP()
#endif  /* ! defined(MODULE) */

/*--- Basic type definitions -------------------------------------------*/

typedef unsigned char	parport_ip32_bool;
#define bool		parport_ip32_bool
#define true		1
#define false		0

typedef unsigned int	parport_ip32_byte;
#define byte		parport_ip32_byte

/*--- Global variables -------------------------------------------------*/

/* Check dependencies accross config options. */
#if ! defined(CONFIG_PARPORT_1284)
#	undef PARPORT_IP32_PS2
#	undef PARPORT_IP32_EPP
#	undef PARPORT_IP32_ECP
#endif

#if ! defined(CONFIG_PARPORT_IP32_FIFO)
#	undef PARPORT_IP32_ECP
#endif

#if defined(DEBUG_PARPORT_IP32)
#	define DEFAULT_VERBOSE_PROBING 1
#else
#	define DEFAULT_VERBOSE_PROBING 0
#endif

#define PPIP32 DRV_NAME ": "

static int param_verbose_probing =	DEFAULT_VERBOSE_PROBING;
static int param_irq =			PARPORT_IP32_IRQ;
static int param_dma =			PARPORT_IP32_DMA;

#if defined(CONFIG_PARPORT_IP32_FIFO)
static int use_fifo =			1;
static int use_dma =			1;
#endif

/* We do not support more than one port */
static struct parport *this_port = NULL;

/*--- I/O register definitions -----------------------------------------*/

struct parport_ip32_regs {
	void __iomem *data;	/* Data Register */
	void __iomem *dsr;	/* Device Status Register */
	void __iomem *dcr;	/* Device Control Register */

	void __iomem *eppAddr;	/* EPP Address Register */
	void __iomem *eppData0;	/* EPP Data Register 0 */
	void __iomem *eppData1;	/* EPP Data Register 1 */
	void __iomem *eppData2;	/* EPP Data Register 2 */
	void __iomem *eppData3;	/* EPP Data Register 3 */

	void __iomem *cFifo;	/* Parallel Port DATA FIFO */
	void __iomem *ecpAFifo; /* ECP FIFO (Address) */
	void __iomem *ecpDFifo; /* ECP FIFO (Data) */
	void __iomem *tFifo;	/* Test FIFO */

	void __iomem *cnfgA;	/* Configuration Register A */
	void __iomem *cnfgB;	/* Configuration Register B */

	void __iomem *ecr;	/* Extended Control Register */
};

#undef BIT
#define BIT(n) (1U << (n))

/* Device Status Register */
#define DSR_nBUSY	BIT(7)	/* PARPORT_STATUS_BUSY */
#define DSR_nACK	BIT(6)	/* PARPORT_STATUS_ACK */
#define DSR_PERROR	BIT(5)	/* PARPORT_STATUS_PAPEROUT */
#define DSR_SELECT	BIT(4)	/* PARPORT_STATUS_SELECT */
#define DSR_nFAULT	BIT(3)	/* PARPORT_STATUS_ERROR */
#define DSR_nPRINT	BIT(2)	/* specific to the TL16PIR552 */
/* #define DSR_???	BIT(1) */
#define DSR_TIMEOUT	BIT(0)	/* EPP timeout */

/* Device Control Register */
/* #define DCR_???	BIT(7) */
/* #define DCR_???	BIT(6) */
#define DCR_DIR		BIT(5)	/* direction */
#define DCR_IRQ		BIT(4)	/* interrupt on nAck */
#define DCR_SELECT	BIT(3)	/* PARPORT_CONTROL_SELECT */
#define DCR_nINIT	BIT(2)	/* PARPORT_CONTROL_INIT */
#define DCR_AUTOFD	BIT(1)	/* PARPORT_CONTROL_AUTOFD */
#define DCR_STROBE	BIT(0)	/* PARPORT_CONTROL_STROBE */

/* ECP Configuration Register A */
#define CNFGA_IRQ	BIT(7)
#define CNFGA_ID_MASK	(BIT(6) | BIT(5) | BIT(4))
#define CNFGA_ID_SHIFT	4
#define CNFGA_ID_16	(0x00 << CNFGA_ID_SHIFT)
#define CNFGA_ID_8	(0x01 << CNFGA_ID_SHIFT)
#define CNFGA_ID_32	(0x02 << CNFGA_ID_SHIFT)
/* #define CNFGA_???	BIT(3) */
#define CNFGA_nBYTEINTRANS BIT(2)
#define CNFGA_PWORDLEFT	(BIT(1) | BIT(0))

/* ECP Configuration Register B */
#define CNFGB_COMPRESS	BIT(7)
#define CNFGB_INTRVAL	BIT(6)
#define CNFGB_IRQ_MASK	(BIT(5) | BIT(4) | BIT(3))
#define CNFGB_IRQ_SHIFT	3
#define CNFGB_IRQ(r)	cnfgb_irq_line(r) /* defined below */
#define CNFGB_DMA_MASK	(BIT(2) | BIT(1) | BIT(0))
#define CNFGB_DMA_SHIFT	0
#define CNFGB_DMA(r)	cnfgb_dma_channel(r) /* defined below */

static inline unsigned int cnfgb_irq_line (byte reg)
{
	static const unsigned int irq_line[] = {0, 7, 9, 10, 11, 14, 15, 5};
	return irq_line[(reg & CNFGB_IRQ_MASK) >> CNFGB_IRQ_SHIFT];
}

static inline unsigned int cnfgb_dma_channel (byte reg)
{
	unsigned int dma = (reg & CNFGB_DMA_MASK) >> CNFGB_DMA_SHIFT;
	return (dma & 0x03)? dma: 0;
}

/* Extended Control Register */
#define ECR_MODE_MASK	(BIT(7) | BIT(6) | BIT(5))
#define ECR_MODE_SHIFT	5
#define ECR_MODE_SPP	(0x00 << ECR_MODE_SHIFT)
#define ECR_MODE_PS2	(0x01 << ECR_MODE_SHIFT)
#define ECR_MODE_PPF	(0x02 << ECR_MODE_SHIFT)
#define ECR_MODE_ECP	(0x03 << ECR_MODE_SHIFT)
#define ECR_MODE_EPP	(0x04 << ECR_MODE_SHIFT)
/* #define ECR_MODE_???	(0x05 << ECR_MODE_SHIFT) */
#define ECR_MODE_TST	(0x06 << ECR_MODE_SHIFT)
#define ECR_MODE_CFG	(0x07 << ECR_MODE_SHIFT)
#define ECR_nERRINTR	BIT(4)
#define ECR_DMA		BIT(3)
#define ECR_SERVINTR	BIT(2)
#define ECR_F_FULL	BIT(1)
#define ECR_F_EMPTY	BIT(0)

/*--- Private data -----------------------------------------------------*/

struct parport_ip32_private {
	struct parport_ip32_regs regs; /* Register addresses */

	byte dcr_init;		/* Initial value for dcr */
	byte ecr_init;		/* Initial value for ecr */

	byte dcr_cache;		/* Cached contents of DCR */
	byte dcr_writable;	/* Bitmask of writable DCR bits */

	bool ecr_present; /* Is an ECR register present? */

	/* Number of PWords that FIFO will hold */
	unsigned int fifo_depth;
	/* Number of bytes per PWord */
	unsigned int pword;
	/* Minimum number of PWords we can read if we get an interrupt */
	unsigned int readIntrThreshold;
	/* Minimum number of PWords we knowwe can write if we get an
	 * interrupt */
	unsigned int writeIntrThreshold;
};

/* Fetch address of parport_ip32_private structure, p is a pointer to a
 * parport structure
 */
#define PRIV(p) ((struct parport_ip32_private *)(p)->physport->private_data)

/*--- Generic functions ------------------------------------------------*/

/* Compute register addresses, according to ISA standard */
static void make_ISA_registers (struct parport_ip32_regs *regs,
				void __iomem *base, void __iomem *base_hi,
				unsigned int regshift)
{
#define r_base(offset)    ((u8 __iomem *)base    + (offset << regshift))
#define r_base_hi(offset) ((u8 __iomem *)base_hi + (offset << regshift))
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

/* FIXME: use io{read,write}8 (and _rep) when available on mips
 * FIXME: are the memory barriers really needed?
 */
static inline byte parport_in (void __iomem *addr)
{
	byte val = readb (addr);
	rmb ();
	return val;
}

static inline void parport_out (byte val, void __iomem *addr)
{
	writeb (val, addr);
	wmb ();
}

static inline void parport_in_rep (void __iomem *addr, void *buf,
				   unsigned long count)
{
	readsb (addr, buf, count);
	rmb ();
}

static inline void parport_out_rep (void __iomem *addr, const void *buf,
				    unsigned long count)
{
	writesb (addr, buf, count);
	wmb ();
}

/*--- Debug code -------------------------------------------------------*/

#if DEBUG_PARPORT_IP32 >= 1
#	define pr_debug1(...)	printk (KERN_DEBUG __VA_ARGS__)
#else
#	define pr_debug1(...)	NO_OP()
#endif

#if DEBUG_PARPORT_IP32 >= 2
#	if ! defined(DUMP_PARPORT_STATE)
#		define DUMP_PARPORT_STATE
#	endif
#	define dump_parport_state(...)	_dump_parport_state ( __VA_ARGS__ )
#else
#	define dump_parport_state(...)	NO_OP()
#endif

#if defined(DEBUG_IP32_IRQ)
#	if ! defined (DUMP_PARPORT_STATE)
#		define DUMP_PARPORT_STATE
#	endif
#endif

#if defined(DUMP_PARPORT_STATE)

/* _dump_parport_state - print register status of parport
 */
static void _dump_parport_state (struct parport *p, char *str,
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
		byte ecr = parport_in (priv->regs.ecr);
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
		oecr = parport_in (priv->regs.ecr);
		parport_out (ECR_MODE_CFG, priv->regs.ecr);
		cnfgA = parport_in (priv->regs.cnfgA);
		cnfgB = parport_in (priv->regs.cnfgB);
		parport_out (oecr, priv->regs.ecr);
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
		byte dcr = i? priv->dcr_cache: parport_in (priv->regs.dcr);
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
		byte dsr = parport_in (priv->regs.dsr);
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

#endif /* defined(DUMP_PARPORT_STATE) */

#define _pr_probe(...)							\
	do { if (param_verbose_probing) printk ( __VA_ARGS__ ); } while (0)
#define pr_probe(p, ...)						\
	do { _pr_probe (KERN_DEBUG PPIP32 "0x%lx: ", (p)->base);	\
	     _pr_probe ( __VA_ARGS__ ); } while (0)

#define NOT_IMPLEMENTED(p, m)						\
	printk (KERN_DEBUG PPIP32					\
		"%s: %s not implemented, %s\n", (p)->name, __func__, m)

/*--- Some utility function to manipulate ECR register -----------------*/

static inline byte parport_ip32_read_econtrol (struct parport *p)
{
	byte c = parport_in (PRIV(p)->regs.ecr);
	pr_debug ("%s(%s): 0x%02x\n", __func__, p->name, c);
	return c;
}

static inline void parport_ip32_write_econtrol (struct parport *p,
						byte c)
{
	pr_debug ("%s(%s, 0x%02x)\n", __func__, p->name, c);
	parport_out (c, PRIV(p)->regs.ecr);
}

static inline void parport_ip32_frob_econtrol (struct parport *p,
					       byte mask, byte val)
{
	byte c = 0;
	pr_debug ("%s(%s, %02x, %02x)\n", __func__, p->name, mask, val);
	c = (mask == 0xff)? 0: parport_ip32_read_econtrol (p);
	parport_ip32_write_econtrol (p, (c & ~mask) ^ val);
}

/* ECR is reset in a sane state (interrupts and dma disabled), and placed in
 * mode `mode'.
 */
static inline void parport_ip32_set_mode (struct parport *p, byte mode)
{
	byte omode;
	pr_debug ("%s(%s, 0x%02x)\n", __func__, p->name, mode);

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

static inline byte parport_ip32_read_data (struct parport *p)
{
	byte d = parport_in (PRIV(p)->regs.data);
	pr_debug ("%s(%s): 0x%02x\n", __func__, p->name, d);
	return d;
}

static inline void parport_ip32_write_data (struct parport *p, byte d)
{
	pr_debug ("%s(%s, 0x%02x)\n", __func__, p->name, d);
	parport_out (d, PRIV(p)->regs.data);
}

static inline byte parport_ip32_read_status (struct parport *p)
{
	byte s = parport_in (PRIV(p)->regs.dsr);
	pr_debug ("%s(%s): 0x%02x\n", __func__, p->name, s);
	return s;
}

/* __parport_ip32_read_control differs from parport_ip32_read_control in that
 * it doesn't do any extra masking.
 */
static inline byte __parport_ip32_read_control (struct parport *p)
{
	byte c = PRIV(p)->dcr_cache; /* use soft copy */
	pr_debug ("%s(%s): 0x%02x\n", __func__, p->name, c);
	return c;
}

static inline byte parport_ip32_read_control (struct parport *p)
{
	const byte rm = DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	byte c = __parport_ip32_read_control (p) & rm;
	pr_debug ("%s(%s): 0x%02x\n", __func__, p->name, c);
	return c;
}

/* __parport_ip32_write_control differs from parport_ip32_write_control in
 * that it doesn't do any extra masking.
 */
static inline void __parport_ip32_write_control (struct parport *p, byte c)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_debug ("%s(%s, 0x%02x)\n", __func__, p->name, c);
#if defined(DEBUG_PARPORT_IP32)
	if (c & ~priv->dcr_writable) {
		pr_debug1 (PPIP32 "%s: extra bits in %s: 0x%02x/0x%02x\n",
			   p->name, __func__, c, priv->dcr_writable);
	}
#endif /* defined(DEBUG_PARPORT_IP32) */
	c &= priv->dcr_writable; /* only writable bits */
	parport_out (c, priv->regs.dcr);
	priv->dcr_cache = c;		/* update soft copy */
}

/* __parport_ip32_frob_control differs from parport_ip32_frob_control in that
 * it doesn't do any extra masking.
 */
static inline void __parport_ip32_frob_control (struct parport *p,
						byte mask, byte val)
{
	byte c;
	pr_debug ("%s(%s, 0x%02x, 0x%02x)\n", __func__, p->name, mask, val);
	c = (__parport_ip32_read_control (p) & ~mask) ^ val;
	__parport_ip32_write_control (p, c);
}

static inline void parport_ip32_write_control (struct parport *p, byte c)
{
	const byte wm = DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	pr_debug ("%s(%s, 0x%02x)\n", __func__, p->name, c);
#if defined(DEBUG_PARPORT_IP32)
	if (c & ~wm) {
		pr_debug1 (PPIP32 "%s: extra bits in %s: 0x%02x/0x%02x\n",
			   p->name, __func__, c, wm);
	}
#endif /* defined(DEBUG_PARPORT_IP32) */
	__parport_ip32_frob_control (p, wm, c & wm);
}

static inline byte parport_ip32_frob_control (struct parport *p,
					      byte mask, byte val)
{
	const byte wm = DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	pr_debug ("%s(%s, 0x%02x, 0x%02x)\n", __func__, p->name, mask, val);
#if defined(DEBUG_PARPORT_IP32)
	if (mask & ~wm || val & ~wm) {
		pr_debug1 (PPIP32 "%s: extra bits in %s: 0x%02x,0x%02x/0x%02x",
			   p->name, __func__, mask, val, wm);
	}
#endif /* defined(DEBUG_PARPORT_IP32) */
	__parport_ip32_frob_control (p, mask & wm, val & wm);
	return parport_ip32_read_control (p);
}

static inline void parport_ip32_disable_irq (struct parport *p)
{
	pr_debug ("%s(%s)\n", __func__, p->name);
	__parport_ip32_frob_control (p, DCR_IRQ, 0x00);
}

static inline void parport_ip32_enable_irq (struct parport *p)
{
	pr_debug ("%s(%s)\n", __func__, p->name);
	__parport_ip32_frob_control (p, DCR_IRQ, DCR_IRQ);
}

static inline void parport_ip32_data_forward (struct parport *p)
{
	pr_debug ("%s(%s)\n", __func__, p->name);
	__parport_ip32_frob_control (p, DCR_DIR, 0x00);
}

static inline void parport_ip32_data_reverse (struct parport *p)
{
	pr_debug ("%s(%s)\n", __func__, p->name);
	__parport_ip32_frob_control (p, DCR_DIR, DCR_DIR);
}

static inline void parport_ip32_init_state (struct pardevice *dev,
					    struct parport_state *s)
{
	struct parport_ip32_private * const priv = PRIV(dev->port);
	pr_debug ("%s(%s [%s], %p)\n",
		  __func__, dev->port->name, dev->name, s);
	s->u.ip32.dcr = priv->dcr_init;
	s->u.ip32.ecr = priv->ecr_init;
}

static inline void parport_ip32_save_state (struct parport *p,
					    struct parport_state *s)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_debug ("%s(%s, %p)\n", __func__, p->name, s);
	s->u.ip32.dcr = __parport_ip32_read_control (p);
	if (priv->ecr_present) {
		s->u.ip32.ecr = parport_ip32_read_econtrol (p);
	}
}

static inline void parport_ip32_restore_state (struct parport *p,
					       struct parport_state *s)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_debug ("%s(%s, %p)\n", __func__, p->name, s);
	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, s->u.ip32.ecr);
	}
	__parport_ip32_write_control (p, s->u.ip32.dcr);
}

/*--- Simple interrupt handler -----------------------------------------*/

static irqreturn_t parport_ip32_interrupt (int irq, void *dev_id,
					   struct pt_regs *regs)
{
	struct parport * const port = dev_id;
	pr_debug ("%s(%d, %s)\n", __func__, irq, port->name);
	parport_generic_irq (irq, port, regs);
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}

#if defined(DEBUG_IP32_IRQ)

static int maceisa_par_ctxa_irq = MACEISA_PAR_CTXA_IRQ;
static int maceisa_par_ctxb_irq = MACEISA_PAR_CTXB_IRQ;
static int maceisa_par_merr_irq = MACEISA_PAR_MERR_IRQ;

static irqreturn_t parport_ip32_debug_irq_handler (int irq, void *dev_id,
						   struct pt_regs *regs)
{
	struct parport * const port = dev_id;
	printk (KERN_DEBUG "%s(%d, %s)\n", __func__, irq, port->name);
	_dump_parport_state (port, "@ irq", false);
	return IRQ_HANDLED;
}

static __init void parport_ip32_debug_irq_init (struct parport *port)
{
#define debug_ip32_request_irq(irq)					\
	do {								\
		if (irq != PARPORT_IRQ_NONE) {				\
			const char *s = "";				\
			if (request_irq (irq, parport_ip32_debug_irq_handler, \
					 0, port->name, port)) {	\
				s = "failed to ";			\
				irq = PARPORT_IRQ_NONE;			\
			}						\
			printk (KERN_DEBUG PPIP32			\
				"%s: %sinstall IRQ handler for %s (%d)\n", \
				port->name, s, #irq, irq);		\
		}							\
	} while (0)

	debug_ip32_request_irq (maceisa_par_ctxa_irq);
	debug_ip32_request_irq (maceisa_par_ctxb_irq);
	debug_ip32_request_irq (maceisa_par_merr_irq);

#undef debug_ip32_request_irq
}

static __exit void parport_ip32_debug_irq_exit (struct parport *port)
{
#define debug_ip32_free_irq(irq)				\
	do {							\
		if (irq != PARPORT_IRQ_NONE) {			\
			printk (KERN_DEBUG PPIP32 "%s: "	\
				"freeing IRQ %s (%d)\n",	\
				port->name, #irq, irq);		\
			free_irq(irq, port);			\
		}						\
	} while (0)

	debug_ip32_free_irq (maceisa_par_merr_irq);
	debug_ip32_free_irq (maceisa_par_ctxb_irq);
	debug_ip32_free_irq (maceisa_par_ctxa_irq);

#undef debug_ip32_free_irq
}

#else /* ! defined(DEBUG_IP32_IRQ) */

#define parport_ip32_debug_irq_init(...)	NO_OP()
#define parport_ip32_debug_irq_exit(...)	NO_OP()

#endif /* ! defined(DEBUG_IP32_IRQ) */

/*--- EPP mode functions -----------------------------------------------*/

/* parport_ip32_clear_epp_timeout - clear Timeout bit in EPP mode
 *
 * Returns false if it failed to clear Timeout bit.
 *
 * This is also used in SPP and PS2 detection.
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
		parport_out (r | DSR_TIMEOUT, priv->regs.dsr);
		/* Others by writing 0 */
		parport_out (r & ~DSR_TIMEOUT, priv->regs.dsr);

		r = parport_ip32_read_status (p);
		cleared = !(r & DSR_TIMEOUT);
	}

	pr_debug ("%s(%s): %s\n", __func__, p->name,
		  cleared? "cleared": "failed");
	return cleared;
}

#if defined(PARPORT_IP32_EPP)

/*
 * TODO: insert here parport_ip32_epp_{read,write}_{data,address}
 */

#endif /* defined(PARPORT_IP32_EPP) */

/*--- ECP mode functions (FIFO) ----------------------------------------*/

#if defined(CONFIG_PARPORT_IP32_FIFO)

/* Return address for fifo register, according to mode */
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

/* parport_ip32_fifo_write_pio_wait - Wait until FIFO empties a bit.
 *
 * Returns the number of bytes that can safely be written in the FIFO.  A
 * return value of zero means that the calling function should terminate as
 * fast as possible.  If an error is returned (value less than zero), the FIFO
 * must be reset.
 */
static inline int parport_ip32_fifo_write_pio_wait (struct parport *port)
{
	static const unsigned int polling_interval = 50; /* usecs */
	static const unsigned int nfault_check_interval = 100000; /* usecs */
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	const bool polling = (port->irq == PARPORT_IRQ_NONE);
	unsigned long expire;
	unsigned long timeout;
	unsigned long nfault_timeout;
	int skip_nfault_check;
	int count;
	byte ecr;

	/* In the case of interrupt-driven waiting, we multiply the timeout by
	 * priv->writeIntrThreshold.  */
	if (polling) {
		timeout = physport->cad->timeout;
	} else {
		timeout = physport->cad->timeout * priv->writeIntrThreshold;
	}
	expire = jiffies + timeout;

	nfault_timeout = min (timeout,
			      usecs_to_jiffies (nfault_check_interval));
	skip_nfault_check = 0;

	count = 0;
	while (1) {
		/* Timed out? */
		if (time_after (jiffies, expire)) {
			printk (KERN_DEBUG PPIP32
				"%s: FIFO write timed out\n", port->name);
			break;
		}
		/* Anyone else waiting for the port? */
		if (port->waithead) {
			printk (KERN_DEBUG PPIP32
				"%s: Somebody wants the port\n", port->name);
			break;
		}
		/* Pending signal? */
		if (signal_pending (current)) {
			printk (KERN_DEBUG PPIP32
				"%s: Signal pending\n", port->name);
			break;
		}
		/* nFault? */
		if (! skip_nfault_check &&
		    ! (parport_ip32_read_status (port) & DSR_nFAULT)) {
			printk (KERN_DEBUG PPIP32
				"%s: FIFO write error\n", port->name);
			break;
		}
		/* Time to resched? */
		if (need_resched ()) {
			pr_debug (PPIP32 "%s: .. schedule\n", port->name);
			schedule ();
		}

		if (polling) {
			/* Active polling */

			/* Check FIFO state */
			ecr = parport_ip32_read_econtrol (port);
			switch (ecr & (ECR_F_FULL | ECR_F_EMPTY)) {
			case ECR_F_FULL | ECR_F_EMPTY:
				/* Something wrong happened */
				goto fifo_error;
			case ECR_F_EMPTY:
				count = priv->fifo_depth;
				break;
			case 0:
				count = 1;
				break;
			}
			if (count)
				break;

			/* Wait... */
			udelay (polling_interval);
			if (--skip_nfault_check < 0){
				skip_nfault_check = nfault_check_interval /
					polling_interval;
			}
		} else { /* (! polling) */
			/* Interrupt driven waiting */
			static bool lost_interrupt = false;
			int r;

			/* Enable serviceIntr */
			parport_ip32_frob_econtrol (port, ECR_SERVINTR, 0);
			/* Wait for interrupt */
			r = parport_wait_event (port, nfault_timeout);
			ecr = parport_ip32_read_econtrol (port);
			/* Disable serviceIntr */
			parport_ip32_frob_econtrol (port, ECR_SERVINTR,
						    ECR_SERVINTR);

			if (r == 1 && (ecr & (ECR_SERVINTR | ECR_F_EMPTY))) {
				/* We should have got an interrupt, but we did
				 * not.  */
				pr_debug1 (PPIP32 "%s: lost interrupt\n",
					   port->name);
				pr_debug (PPIP32 "%s: .. r=%d, "
					  "ecr=0x%02x, dsr=0x%02x\n",
					  port->name, r, ecr,
					  parport_ip32_read_status (port));
				lost_interrupt = true;
			} else if (r == 0 && lost_interrupt) {
				pr_debug1 (PPIP32 "%s: interrupt back\n",
					   port->name);
				lost_interrupt = false;
			}

			/* Check FIFO state */
			switch (ecr & (ECR_F_FULL | ECR_F_EMPTY)) {
			case ECR_F_FULL | ECR_F_EMPTY:
				/* Something wrong happened */
				goto fifo_error;
			case ECR_F_EMPTY:
				count = priv->fifo_depth;
				break;
			case 0:
				count = (ecr & ECR_SERVINTR)?
					priv->writeIntrThreshold: 1;
				break;
			}
			if (count)
				break;
		}  /* port->irq != PARPORT_IRQ_NONE */
	} /* while (1) */

	return count;

fifo_error:
	printk (KERN_DEBUG PPIP32 "%s: FIFO error in %s, ecr=0x%02x\n",
		port->name, __func__, parport_ip32_read_econtrol (port));
	return -1;
}

/* Write a block of data - PIO mode */
static size_t parport_ip32_fifo_write_block_pio (struct parport *port,
						 const void *buf, size_t len,
						 int flags, byte mode)
{
	void __iomem * const fifo = parport_ip32_fifo_addr (port, mode);
	const u8 *bufp = buf;
	size_t left = len;

	dump_parport_state (port, "begin fifo_write_block_pio", false);

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

		/* Write next bytes to Fifo */
		if (count == 1) {
			parport_out (*bufp, fifo);
		} else {
			parport_out_rep (fifo, bufp, count);
		}
		bufp += count;
		left -= count;
	}

	pr_debug (PPIP32 "%s: .. transfer %s (left=%lu)\n", port->name,
		  left? "aborted": "completed", (unsigned long)left);

	dump_parport_state (port, "end fifo_write_block_pio", false);

	return (len - left);
}

/* Write a block of data - DMA mode */
static size_t parport_ip32_fifo_write_block_dma (struct parport *port,
						 const void *buf, size_t len,
						 int flags, byte mode)
{
	NOT_IMPLEMENTED (port, "reverting to PIO mode");
	return parport_ip32_fifo_write_block_pio (port, buf, len, flags, mode);
}

/* Initialize a forward FIFO transfer.  Reset FIFO and set appropriate mode.
 * Returns true if the printer is ready.  Returns false otherwise.
 */
static bool parport_ip32_fifo_write_initialize (struct parport *port,
						byte mode)
{
	bool ready;

	pr_debug ("%s(%s, 0x%02x)\n", __func__, port->name, mode);

	/* Reset Fifo, go in forward mode, and disable ackIntEn */
	parport_ip32_set_mode (port, ECR_MODE_PS2);
	parport_ip32_write_control (port, DCR_SELECT | DCR_nINIT);
	parport_ip32_data_forward (port);
	parport_ip32_disable_irq (port);

	/* Go in desired mode */
	parport_ip32_set_mode (port, mode);

	/* Wait for printer to become ready */
	ready = !parport_wait_peripheral (port,
					  DSR_nBUSY | DSR_nFAULT,
					  DSR_nBUSY | DSR_nFAULT);

	return ready;
}

/* Waits for FIFO to empty.  Returns true when FIFO is empty, or false if
 * timeout is reached before, or if a signal is pending.
 */
static bool parport_ip32_drain_fifo (struct parport *port,
				     unsigned long timeout)
{
	unsigned long expire = jiffies + timeout;
	unsigned int polling_interval;
	unsigned int counter;

	pr_debug ("%s(%s, %ums)\n", __func__,
		  port->name, jiffies_to_msecs (timeout));

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

/* Resets FIFO, and returns the number of bytes remaining in it.
 */
static unsigned int parport_ip32_get_fifo_residue (struct parport *port,
						   byte mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	void __iomem * const fifo = parport_ip32_fifo_addr (port, mode);
	unsigned int residue;
	byte cnfga;

	/* FIXME: we are missing one byte if the printer is off-line.  I don't
	 * know how to detect this.  For the moment, the problem is avoided by
	 * testing for BUSY in parport_ip32_fifo_write_initialize.
	 */

	pr_debug ("%s(%s, 0x%02x)\n", __func__, port->name, mode);

	if (parport_ip32_read_econtrol (port) & ECR_F_EMPTY) {
		residue = 0;
	} else {
		printk (KERN_DEBUG PPIP32 "%s: FIFO is stuck\n", port->name);

		/* Stop all transfers.
		 *
		 * Microsoft's document instructs to drive DCR_STROBE to 0,
		 * but it doesn't work (at least in Compat mode, not tested in
		 * ECP mode).  Switching directly to Test mode (as in
		 * parport_pc) is not an option: it does confuse the port, ECP
		 * service interrupts are no more working after that.  A hard
		 * reset is then needed to revert to a sane state.
		 *
		 * Let's hope that the FIFO is really stuck and that the
		 * peripheral doesn't wake up now.
		 */
		parport_ip32_frob_control (port, DCR_STROBE, 0);

		/* Fill up FIFO */
		for (residue = priv->fifo_depth; residue > 0; residue--) {
			if (parport_ip32_read_econtrol (port) & ECR_F_FULL)
				break;
			parport_out (0x00, fifo);
		}
	}

	if (residue) {
		pr_debug1 (PPIP32 "%s: %d PWords were left in FIFO\n",
			   port->name, residue);
	}

	/* Now reset the FIFO, change to Config mode, and clean up */
	parport_ip32_set_mode (port, ECR_MODE_CFG);
	cnfga = parport_in (priv->regs.cnfgA);


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

/* Finalize a forward FIFO transfer.  Returns 1 if FIFO is empty and the
 * status of Busy is low.  Returns -count otherwise, where count is the number
 * of bytes remaining in the FIFO.  Note that zero can be returned in case
 * there is a BUSY timeout.
 */
static int parport_ip32_fifo_write_finalize (struct parport *port, int mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	bool ready;
	unsigned int residue;

	pr_debug ("%s(%s, 0x%02x)\n", __func__, port->name, mode);

	/* Wait FIFO to empty.  Timeout is proportionnal to fifo_depth.  */
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

static size_t parport_ip32_fifo_write_block (struct parport *port,
					     const void *buf, size_t len,
					     int flags, byte mode)
{
	static bool ready_before = true;
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

	pr_debug (PPIP32 "%s: -> %s: len=%lu\n",
		  port->name, __func__, (unsigned long)len);

	physport->ieee1284.phase = IEEE1284_PH_FWD_DATA;

	if (!use_dma || port->dma == PARPORT_DMA_NONE) {
		written = parport_ip32_fifo_write_block_pio (port, buf, len,
							     flags, mode);
	} else {
		written = parport_ip32_fifo_write_block_dma (port, buf, len,
							     flags, mode);
	}

	/* Finalize the transfer */
	r = parport_ip32_fifo_write_finalize (port, mode);
	if (r < 0) {
		written += r;
	}

	physport->ieee1284.phase = IEEE1284_PH_FWD_IDLE;

	pr_debug (PPIP32 "%s: <- %s: written=%lu\n",
		  port->name, __func__, (unsigned long)written);

out:
	return written;
}

static size_t parport_ip32_compat_write_data (struct parport *port,
					      const void *buf, size_t len,
					      int flags)
{
	struct parport * const physport = port->physport;
	size_t written;

	pr_debug ("%s(%s, ...)\n", __func__, port->name);

	/* Special case: a timeout of zero means we cannot call schedule().
	 * Also if O_NONBLOCK is set then use the default implementation. */
	if (!use_fifo
	    || physport->cad->timeout <= PARPORT_INACTIVITY_O_NONBLOCK)
		written = parport_ieee1284_write_compat (port, buf, len,
							 flags);
	else {
		written = parport_ip32_fifo_write_block (port, buf, len,
							 flags, ECR_MODE_PPF);
	}
	return written;
}

#if defined(PARPORT_IP32_ECP)

/*
 * TODO: insert here parport_ip32_ecp_{read,write}_{data,address}
 */

#endif /* defined(PARPORT_IP32_ECP) */

#endif /* defined(CONFIG_PARPORT_IP32_FIFO) */

/*--- Default operations -----------------------------------------------*/

/* To ensure the correct types, define wrappers around parport operations. */

#define parport_byte	unsigned char
#define READER(f)					      \
	static parport_byte f ## _wrapper (struct parport *p) \
	{ return f (p); }
#define WRITER(f)						      \
	static void f ## _wrapper (struct parport *p, parport_byte v) \
	{ f (p, v); }
#define FROBBER(f)							\
	static parport_byte f ## _wrapper (struct parport *p,		\
					   parport_byte m, parport_byte v) \
	{ return f (p, m, v); }

READER (parport_ip32_read_data)
READER (parport_ip32_read_control)
READER (parport_ip32_read_status)
WRITER (parport_ip32_write_data)
WRITER (parport_ip32_write_control)
FROBBER (parport_ip32_frob_control)

#undef FROBBER
#undef WRITER
#undef READER
#undef parport_byte

/* Do not uncomment the const qualifier: this causes compilation error
 * ("parport_ip32_ops causes a section type conflict") because of the
 * __initdata qualifier.  */
static __initdata /*const*/ struct parport_operations parport_ip32_ops = {
	.write_data	= parport_ip32_write_data_wrapper,
	.read_data	= parport_ip32_read_data_wrapper,

	.write_control	= parport_ip32_write_control_wrapper,
	.read_control	= parport_ip32_read_control_wrapper,
	.frob_control	= parport_ip32_frob_control_wrapper,

	.read_status	= parport_ip32_read_status_wrapper,

	.enable_irq	= parport_ip32_enable_irq,
	.disable_irq	= parport_ip32_disable_irq,

	.data_forward	= parport_ip32_data_forward,
	.data_reverse	= parport_ip32_data_reverse,

	.init_state	= parport_ip32_init_state,
	.save_state	= parport_ip32_save_state,
	.restore_state	= parport_ip32_restore_state,

	.epp_write_data = parport_ieee1284_epp_write_data,
	.epp_read_data	= parport_ieee1284_epp_read_data,
	.epp_write_addr = parport_ieee1284_epp_write_addr,
	.epp_read_addr	= parport_ieee1284_epp_read_addr,

	.ecp_write_data = parport_ieee1284_ecp_write_data,
	.ecp_read_data	= parport_ieee1284_ecp_read_data,
	.ecp_write_addr = parport_ieee1284_ecp_write_addr,

	.compat_write_data	= parport_ieee1284_write_compat,
	.nibble_read_data	= parport_ieee1284_read_nibble,
	.byte_read_data		= parport_ieee1284_read_byte,

	.owner		= THIS_MODULE,
};

/*--- Device detection -------------------------------------------------*/

/*
 * parport_ip32_<mode>_supported - check for supported modes
 *   p: pointer to the parport structure
 *
 * Returns false if mode is not supported.  Adjust parameters in the parport
 * structure.
 */

static inline bool parport_ip32_not_supported(void)
{
	return false;
}

/* Check if the Extended Control Register is functional.
 *
 * Old style XT ports alias io ports every 0x400, hence accessing ECR on these
 * cards actually accesses the DCR.
 *
 * Modern cards don't do this but reading from ECR will return 0xff regardless
 * of what is written here if the card does NOT support ECP.
 *
 * We first check to see if ECR is the same as DCR.  If not, the low two bits
 * of ECR aren't writable, so we check by writing ECR and reading it back to
 * see if it's what we expect.
 *
 * On return, the port is placed in SPP mode.
 */
static __init bool parport_ECR_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	byte dcr, ecr, mask;

	parport_out (DCR_SELECT | DCR_nINIT, priv->regs.dcr);
	dcr = parport_in (priv->regs.dcr);
	mask = ECR_F_FULL | ECR_F_EMPTY;
	if ((parport_in (priv->regs.ecr) & mask) == (dcr & mask)) {
		/* Toggle bit ECR_F_FULL */
		mask = ECR_F_FULL;
		parport_out (dcr ^ mask, priv->regs.dcr);
		dcr = parport_in (priv->regs.dcr);
		if ((parport_in (priv->regs.ecr) & mask) == (dcr & mask))
			goto no_reg; /* Sure that no ECR register exists */
	}

	ecr = parport_in (priv->regs.ecr) & (ECR_F_FULL | ECR_F_EMPTY);
	if (ecr != ECR_F_EMPTY)
		goto no_reg;

	ecr = ECR_MODE_PS2 | ECR_nERRINTR | ECR_SERVINTR;
	parport_out (ecr, priv->regs.ecr);
	if (parport_in (priv->regs.ecr) != (ecr | ECR_F_EMPTY))
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

/* Check if the port behaves correctly.
 */
static __init bool parport_SPP_supported (struct parport *p)
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
	w = 0x0c;
	parport_out (w, priv->regs.dcr);

	/* Is there a control register that we can read from?  Some
	 * ports don't allow reads, so read_control just returns a
	 * software copy. Some ports _do_ allow reads, so bypass the
	 * software copy here.  In addition, some bits aren't
	 * writable. */
	r = parport_in (priv->regs.dcr);
	if ((r & 0x0f) == w) {
		w = 0x0e;
		parport_out (w, priv->regs.dcr);
		r = parport_in (priv->regs.dcr);
		parport_out (0x0c, priv->regs.dcr);
		if ((r & 0x0f) != w)
			goto spp_ok;
	}

	/* Try the data register.  The data lines aren't tri-stated at
	 * this stage, so we expect back what we wrote. */
	w = 0xaa;
	parport_ip32_write_data (p, w);
	r = parport_ip32_read_data (p);
	if (r == w) {
		w = 0x55;
		parport_ip32_write_data (p, w);
		r = parport_ip32_read_data (p);
		if (r == w)
			goto spp_ok;
	}

	/* No parallel port found. */
	pr_probe (p, "SPP register not found\n");
	return false;

spp_ok:
	pr_probe (p, "Found working SPP register\n");
	p->modes |= PARPORT_MODE_PCSPP;
	/* FIXME:
	 *   What does exactly SAFEININT mean?
	 *   Is there some way to test for it?
	 * Activate it unless someone complains.
	 */
	p->modes |= PARPORT_MODE_SAFEININT;
	priv->dcr_init = DCR_SELECT | DCR_nINIT;
	parport_ip32_write_control (p, priv->dcr_init);
	return true;
}

#if defined(PARPORT_IP32_PS2)

/* Chek if the port supports bidirectionnal mode .
 *
 * Bit 5 (0x20) sets the PS/2 data direction; setting this high
 * allows us to read data from the data lines.  In theory we would get back
 * 0xff but any peripheral attached to the port may drag some or all of the
 * lines down to zero.  So if we get back anything that isn't the contents
 * of the data register we deem PS/2 support to be present.
 *
 * Some SPP ports have "half PS/2" ability - you can't turn off the line
 * drivers, but an external peripheral with sufficiently beefy drivers of
 * its own can overpower them and assert its own levels onto the bus, from
 * where they can then be read back as normal.  Ports with this property
 * and the right type of device attached are likely to fail the SPP test,
 * (as they will appear to have stuck bits) and so the fact that they might
 * be misdetected here is rather academic.
 */
static __init bool parport_PS2_supported (struct parport *p)
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
	if (parport_ip32_read_data (p) != 0x55)
		ok = true;

	parport_ip32_write_data (p, 0xaa);
	if (parport_ip32_read_data (p) != 0xaa)
		ok = true;

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
#define parport_PS2_supported(...)	parport_ip32_not_supported()
#endif

#if defined(PARPORT_IP32_EPP)

/* Check for Enhanced Parallel Port. */
static __init bool parport_EPP_supported (struct parport *p)
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
		pr_probe (p, "Support for EPP mode enabled\n");
#endif
	}

	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, priv->ecr_init);
	}

	return ok;
}

#else  /* ! defined(PARPORT_IP32_EPP) */
#define parport_EPP_supported(...)	parport_ip32_not_supported()
#endif

#if defined(CONFIG_PARPORT_IP32_FIFO)

/* Check for Extended Capabilites Port.  Actually search FIFO parameters. */
static __init bool parport_ECP_supported (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	byte configa, configb;
	unsigned int pword;
	unsigned int i;

	/* If there is no ECR, we have no hope of supporting ECP. */
	if (! priv->ecr_present)
		return false;
	/* We must be able to tristate the port. */
	if (! (p->modes & PARPORT_MODE_TRISTATE)) {
		pr_probe (p, "ECP modes not checked (can't TRISTATE)\n");
		return false;
	}

	/* Configuration mode */
	parport_ip32_set_mode (p, ECR_MODE_CFG);

	configa = parport_in (priv->regs.cnfgA);
	configb = parport_in (priv->regs.cnfgB);

	pr_probe (p, "ECP port cnfgA=0x%02x cnfgB=0x%02x\n", configa, configb);
	pr_probe (p, "Interrupts are ISA-%s\n",
		  (configa & CNFGA_IRQ)? "Level": "Pulses");
	pr_probe (p, "ECP settings irq=");
	if (CNFGB_IRQ (configb)) {
		_pr_probe ("%d", CNFGB_IRQ (configb));
	} else {
		_pr_probe ("<none or set by other means>");
	}
	_pr_probe (" dma=");
	if (CNFGB_DMA (configb)) {
		_pr_probe ("%d\n", CNFGB_DMA (configb));
	} else {
		_pr_probe ("<none or set by other means>\n");
	}

	/* Check for interrupt conflict */
	if (! (configb & CNFGB_INTRVAL) && p->irq != PARPORT_IRQ_NONE) {
		printk (KERN_NOTICE PPIP32
			"0x%lx: IRQ conflict detected, disabling IRQ\n",
			p->base);
		p->irq = PARPORT_IRQ_NONE;
	}

	/* Check for compression support */
	parport_out (configb | CNFGB_COMPRESS, priv->regs.cnfgB);
	if (parport_in (priv->regs.cnfgB) & CNFGB_COMPRESS) {
		pr_probe (p, "Hardware compression detected (unsupported)\n");
	}
	parport_out (configb & ~CNFGB_COMPRESS, priv->regs.cnfgB);

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
	if (! (parport_in (priv->regs.ecr) & ECR_F_EMPTY)) {
		pr_probe (p, "FIFO not reset\n");
		goto ecp_error;
	}

	/* Find out FIFO depth. */
	priv->fifo_depth = 0;
	for (i = 0; i < 1024; i++) {
		if (parport_in (priv->regs.ecr) & ECR_F_FULL) {
			/* FIFO full */
			priv->fifo_depth = i;
			break;
		}
		parport_out ((u8 )i, priv->regs.tFifo);
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
		if (parport_in (priv->regs.tFifo) != (u8 )i) {
			pr_probe (p, "Invalid data in FIFO\n");
			goto ecp_error;
		}
		if (! priv->writeIntrThreshold
		    && parport_in (priv->regs.ecr) & ECR_SERVINTR) {
			/* writeIntrThreshold reached */
			priv->writeIntrThreshold = i + 1;
		}
		if (i + 1 < priv->fifo_depth
		    && parport_in (priv->regs.ecr) & ECR_F_EMPTY) {
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
	if (! (parport_in (priv->regs.ecr) & ECR_F_EMPTY)) {
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
		parport_out (0xaa, priv->regs.tFifo);
		if (! priv->readIntrThreshold
		    && parport_in (priv->regs.ecr) & ECR_SERVINTR) {
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
	pr_probe (p, "Support for compatibility mode enabled\n");

#if defined(PARPORT_IP32_ECP)
#if 0				/* not implemented */
	p->ops->ecp_write_data = parport_ip32_ecp_write_data,
	p->ops->ecp_read_data  = parport_ip32_ecp_read_data,
	p->ops->ecp_write_addr = parport_ip32_ecp_write_addr,
	p->modes |= PARPORT_MODE_ECP;
	pr_probe (p, "Support for ECP modes enabled\n");
#endif
#endif /* defined(PARPORT_IP32_ECP) */

	return true;

ecp_error:
	priv->fifo_depth = 0;
	parport_ip32_write_econtrol (p, priv->ecr_init);
	return false;
}

#else /* ! defined(CONFIG_PARPORT_IP32_FIFO) */
#define parport_ECP_supported(...)	parport_ip32_not_supported()
#endif

/* parport_ip32_probe_port - probe and register a new port
 *   base: base I/O addresse
 *   base_hi:  -
 *   irq: IRQ line
 *   dma: DMA channel
 *   regs: pointer to the port register adresses structure
 *	   NOTE: the structure is copied and can sfely be freed after call
 *   modes: bitmask of extended modes to probe, combination of
 *		PARPORT_MODE_EPP	EPP Mode
 *		PARPORT_MODE_ECP	ECP Mode
 *
 * Parameters base and base_hi are only used to fill in the parport structure.
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

	ops = kmalloc (sizeof (struct parport_operations), GFP_KERNEL);
	priv = kmalloc (sizeof (struct parport_ip32_private), GFP_KERNEL);
	/* A misnomer, actually it's allocate and reserve parport number. */
	p = parport_register_port(base, irq, dma, ops);
	if (! ops || ! priv || ! p) {
		goto error;
	}

	/* Initialize parport structure. Be conservative. */
	*ops = parport_ip32_ops;
	*priv = (struct parport_ip32_private ){
		.regs =		*regs,
		.dcr_init =	DCR_SELECT | DCR_nINIT,
		.ecr_init =	ECR_MODE_SPP | ECR_nERRINTR | ECR_SERVINTR,
		.dcr_cache =	0,
		.dcr_writable =	DCR_SELECT | DCR_nINIT |
				DCR_AUTOFD | DCR_STROBE,
		/* Required for the first dump_parport_state */
		.ecr_present =	!!(modes & PARPORT_MODE_ECP),
		.pword =	1,
		.fifo_depth =	0,
		.readIntrThreshold = 0,
		.writeIntrThreshold = 0
	};
	p->private_data = priv;
	p->base_hi = base_hi;

	dump_parport_state (p, "begin init", true);

	/* First, check if we can use the Extended Control Register. */
	if (modes & PARPORT_MODE_ECP) {
		parport_ECR_supported (p);
	}
	dump_parport_state (p, "after ECR test", false);

	/* If SPP mode does not work, we can't go very far. */
	if (! parport_SPP_supported (p)) {
		goto error;
	}
	dump_parport_state (p, "after SPP test", false);

	/* Is the port bidirectional? */
	parport_PS2_supported (p);
	dump_parport_state (p, "after PS2 test", false);

	if (modes & PARPORT_MODE_EPP) {
		parport_EPP_supported (p);
	}
	dump_parport_state (p, "after EPP test", false);

	if (priv->ecr_present) {
		parport_ECP_supported (p);
	}
	dump_parport_state (p, "after ECP test", false);

	/* Request IRQ */
	if (p->irq != PARPORT_IRQ_NONE) {
		if (request_irq (p->irq, parport_ip32_interrupt,
				 0, p->name, p)) {
			printk (KERN_NOTICE PPIP32 "%s: irq %d in use, "
				"resorting to polled operation\n",
				p->name, p->irq);
			p->irq = PARPORT_IRQ_NONE;
/*			p->dma = PARPORT_DMA_NONE;	FIXME */
			priv->dcr_writable &= ~DCR_IRQ;
		} else {
			priv->dcr_writable |= DCR_IRQ;
		}
	}

	/* Initialise the port with sensible values */
	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, priv->ecr_init);
	}
	parport_ip32_write_control (p, priv->dcr_init);
	parport_ip32_data_forward (p);
	parport_ip32_disable_irq (p);
	parport_ip32_write_data (p, 0x00);

	dump_parport_state (p, "end init", false);

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

static __exit void parport_ip32_unregister_port (struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	struct parport_operations *ops = p->ops;

	parport_remove_port(p);

	if (p->irq != PARPORT_IRQ_NONE)
		free_irq(p->irq, p);

	/* FIXME: release DMA resources */

	parport_put_port(p);
	kfree (priv);
	kfree (ops); /* hope no-one cached it */
}

/*--- Initialisation code ----------------------------------------------*/

static int __init parport_ip32_init (void)
{
	struct parport_ip32_regs regs;
	unsigned int modes;

	pr_info (PPIP32 "%s v%s\n", DRV_DESCRIPTION, DRV_VERSION);
	pr_debug1 (PPIP32 "Compiled on %s, %s\n", __DATE__, __TIME__);

	iomap_mace_address ();
	if (mace == NULL) {
		printk (KERN_DEBUG PPIP32 "invalid mace pointer\n");
		return -ENXIO;
	}

	make_ISA_registers (&regs,
			    PARPORT_IP32_IO_ADDR, PARPORT_IP32_IOHI_ADDR,
			    PARPORT_IP32_REGSHIFT);
	modes = PARPORT_MODE_EPP | PARPORT_MODE_ECP;
	this_port = parport_ip32_probe_port (PARPORT_IP32_IO,
					     PARPORT_IP32_IOHI,
					     param_irq, param_dma,
					     &regs, modes);
	if (this_port == NULL) {
		printk (KERN_DEBUG PPIP32 "failed to probe port\n");
		iounmap_mace_address ();
		return -ENODEV;
	}

	parport_ip32_debug_irq_init (this_port);

	return 0;
}

static void __exit parport_ip32_exit (void)
{
	parport_ip32_debug_irq_exit (this_port);
	parport_ip32_unregister_port (this_port);
	iounmap_mace_address ();
}

/*--- Module stuff -----------------------------------------------------*/

#undef bool			/* it breaks module_param */

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

#if 0				/* unused */
module_param_named (dma, param_dma, int, S_IRUGO);
MODULE_PARM_DESC (dma, "DMA channel (-1 to disable)");
#endif

#if defined(CONFIG_PARPORT_IP32_FIFO)
module_param (use_fifo, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC (use_fifo, "Use hardare FIFO modes");
#endif

#if 0				/* unused */
module_param (use_dma, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC (use_dma, "Use DMA if available");
#endif

/*--- Inform (X)Emacs about preferred coding style ---------------------*/
/*
 * Local Variables:
 * mode: c
 * eval: (c-set-style "K&R")
 * tab-width: 8
 * indent-tabs-mode: t
 * c-basic-offset: 8
 * fill-column: 78
 * ispell-local-dictionary: "american"
 * End:
 */
