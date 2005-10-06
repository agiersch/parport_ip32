/* Low-level parallel port routines for built in port on SGI IP32
 *
 * Author: Arnaud Giersch <arnaud.giersch@free.fr>
 *
 * $Id: parport_ip32.c,v 1.5 2005-10-06 22:59:21 arnaud Exp $
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
 *
 * Current status (v0.4):
 *	Working modes: PCSPP, PS2.
 *      Interrupts work, but it is slow.
 *	FIFO test look good.
 *
 *	FIFO support is not implemented.
 *	EPP and ECP modes are not implemented.
 *	DMA support is not implemented.
 *
 * History:
 *
 * v0.4 -- Fri, 07 Oct 2005 00:57:06 +0200
 *	Major rewrite.
 *
 * v0.3 -- Tue, 04 Oct 2005 22:10:52 +0200
 *	Added Compatibility FIFO mode (PIO).
 *      Added code for EPP support (not tested).
 *      Disable interrupts: it is too slow to get an interrupt per char
 *      written!
 *
 * v0.2 -- Sun, 02 Oct 2005 19:52:04 +0200
 *	Interrupts are working in SPP mode.
 *
 * v0.1 -- Sun, 02 Oct 2005 16:18:54 +0200
 *	First working version. Only SPP/PS2 modes are supported, without
 *	interrupts.
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

/*--- Some configuration defines ---------------------------------------*/

/* DEBUG_PARPORT_IP32
 *	0	disable debug
 *	1	standard level
 *	2	dump_parport_state is enabled
 *	>2	verbose level
 */
#define DEBUG_PARPORT_IP32	1 /* disable for production */

/* Define to enable IRQ. */
#undef PARPORT_IP32_USE_IRQ

/* Define to enable DMA with FIFO modes. */
#undef PARPORT_IP32_USE_DMA	/* unimplemented */

/* Define to enable FIFO modes. */
#define PARPORT_IP32_FIFO	/* unimplemented */

/* Following modes need CONFIG_PARPORT_1284. */

/* Define to enable PS2 support. */
#define PARPORT_IP32_PS2

/* Define to enable EPP support. */
#undef PARPORT_IP32_EPP		/* unimplemented */

/* Define to enable ECP support.  Needs PARPORT_IP32_FIFO. */
#undef PARPORT_IP32_ECP	        /* unimplemented */

/* Boolean: activate verbose probing. */
#define VERBOSE_PROBING 1

/*----------------------------------------------------------------------*/

#include <linux/config.h>

/* Check dependencies accross config options. */
#ifndef CONFIG_PARPORT_1284
#  undef PARPORT_IP32_PS2
#  undef PARPORT_IP32_EPP
#  undef PARPORT_IP32_ECP
#endif

#ifndef PARPORT_IP32_FIFO
#  undef PARPORT_IP32_ECP
#endif

/* Setup DEBUG macros. */
#undef DEBUG
#ifdef DEBUG_PARPORT_IP32
#  if DEBUG_PARPORT_IP32 == 0
#    undef DEBUG_PARPORT_IP32
#  elif DEBUG_PARPORT_IP32 == 1
#    warning DEBUG_PARPORT_IP32 == 1
     /* nop */
#  elif DEBUG_PARPORT_IP32 == 2
#    warning DEBUG_PARPORT_IP32 == 2
     /* nop */
#  else /* DEBUG_PARPORT_IP32 > 1 */
#    warning DEBUG_PARPORT_IP32 > 2
#    define DEBUG			/* enable pr_debug() in kernel.h */
#  endif /* DEBUG_PARPORT_IP32 > 1 */
#endif /* ! DEBUG_PARPORT_IP32 */

/*----------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/parport.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/ip32/mace.h>
#include <asm/ip32/ip32_ints.h>

#define PPIP32 "parport_ip32: "

MODULE_AUTHOR ("Arnaud Giersch <arnaud.giersch@free.fr>");
MODULE_DESCRIPTION ("SGI IP32 built-in parallel port driver");
MODULE_SUPPORTED_DEVICE ("SGI IP32 MACE parallel port");
MODULE_LICENSE ("GPL");

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

#ifdef PARPORT_IP32_USE_IRQ
#  define PARPORT_IP32_IRQ	MACEISA_PARALLEL_IRQ
#else
#  define PARPORT_IP32_IRQ	PARPORT_IRQ_NONE
#endif

#ifdef PARPORT_IP32_USE_DMA
#  define PARPORT_IP32_DMA	PARPORT_DMA_NONE
#else
#  define PARPORT_IP32_DMA	PARPORT_DMA_NONE
#endif

/* Memory mapped I/O addresses */
#define PARPORT_IP32_IO_ADDR   ((unsigned char *)&mace->isa.parallel)
#define PARPORT_IP32_IOHI_ADDR ((unsigned char *)&mace->isa.ecp1284)

/* Registers are replicated 256 times */
#define PARPORT_IP32_REGSHIFT	8

/* Pointer mace is not exported in arch/mips/sgi-ip32/crime.c,
 * use our own pointer for modules.
 */
#ifdef MODULE
static struct sgi_mace __iomem *mace;
static inline void __init iomap_mace_address (void)
{
	mace = ioremap (MACE_BASE, sizeof (struct sgi_mace));
}
static inline void __exit iounmap_mace_address (void)
{
	iounmap (mace);
}
#else /* ! MODULE */
static inline void __init iomap_mace_address (void) { }
static inline void __exit iounmap_mace_address (void) { }
#endif  /* ! MODULE */

/*--- Global variables -------------------------------------------------*/

/* We do not support more than one port */
static struct parport *this_port;

static __initdata int verbose_probing = VERBOSE_PROBING;

#define _pr_probe(...)							\
	do { if (verbose_probing) printk ( __VA_ARGS__ ); } while (0)
#define pr_probe(p, ...)						\
	do { _pr_probe (KERN_DEBUG PPIP32 "0x%lx: ", (p)->base);	\
	     _pr_probe ( __VA_ARGS__ ); } while (0)

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
#define DSR_BUSY	BIT(7)
#define DSR_ACK		BIT(6)
#define DSR_PERROR	BIT(5)
#define DSR_SELECT	BIT(4)
#define DSR_FAULT	BIT(3)
#define DSR_PRINT	BIT(2)
/* #define DSR_???	BIT(1) */
#define DSR_TIMEOUT	BIT(0)

/* Device Control Register */
/* #define DCR_???	BIT(7) */
/* #define DCR_???	BIT(6) */
#define DCR_DIR		BIT(5)
#define DCR_IRQ		BIT(4)
#define DCR_SELECT	BIT(3)
#define DCR_INIT	BIT(2)
#define DCR_AUTOFD	BIT(1)
#define DCR_STROBE	BIT(0)

/* ECP Configuration Register A */
#define CNFGA_IRQ	BIT(7)
#define CNFGA_ID_MASK	(BIT(6) | BIT(5) | BIT(4))
#define CNFGA_ID_SHIFT	4
#define CNFGA_ID_16	(0x00 << CNFGA_ID_SHIFT)
#define CNFGA_ID_8	(0x01 << CNFGA_ID_SHIFT)
#define CNFGA_ID_32	(0x02 << CNFGA_ID_SHIFT)
/* #define CNFGA_???	BIT(3) */
#define CNFGA_HR_TRANS	BIT(2)
#define CNFGA_HR_MASK	(BIT(1) | BIT(0))

/* ECP Configuration Register B */
#define CNFGB_COMPRESS	BIT(7)
#define CNFGB_INTR	BIT(6)
#define CNFGB_IRQ_MASK	(BIT(5) | BIT(4) | BIT(3))
#define CNFGB_IRQ_SHIFT	3
#define CNFGB_IRQ(r)	cnfgb_irq_line(r) /* defined below */
#define CNFGB_DMA_MASK	(BIT(2) | BIT(1) | BIT(0))
#define CNFGB_DMA_SHIFT	0
#define CNFGB_DMA(r)	cnfgb_dma_channel(r) /* defined below */

static inline int cnfgb_irq_line (u8 reg)
{
	static const int irq_line[] = {0, 7, 9, 10, 11, 14, 15, 5};
	return irq_line[(reg & CNFGB_IRQ_MASK) >> CNFGB_IRQ_SHIFT];
}

static inline int cnfgb_dma_channel (u8 reg)
{
	int dma = (reg & CNFGB_DMA_MASK) >> CNFGB_DMA_SHIFT;
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
#define ECR_IRQ		BIT(4)
#define ECR_DMA		BIT(3)
#define ECR_SERVICE	BIT(2)
#define ECR_F_FULL	BIT(1)
#define ECR_F_EMPTY	BIT(0)

/*--- Private data -----------------------------------------------------*/

struct parport_ip32_private {
	struct parport_ip32_regs regs; /* Register addresses */

	u8 dcr_init;		/* Initial value for dcr */
	u8 ecr_init;		/* Initial value for ecr */

	u8 dcr_cache;		/* Cached contents of DCR */
	u8 dcr_writable;	/* Bitmask of writable DCR bits */

	int ecr_present;	/* Is an ECR register present? */

	int fifo_depth;		/* Number of PWords that FIFO will hold */
	int pword;		/* Number of bytes per PWord */
	int readIntrThreshold;	/* Minimum number of PWords we can read if we
				 * get an interrupt */
	int writeIntrThreshold; /* Minimum number of PWords we know we can
				 * write if we get an interrupt */
};

/* Fetch address of parport_ip32_private structure, p is a pointer to a
 * parport structure
 */
#define PRIV(p) ((struct parport_ip32_private *)(p)->physport->private_data)

/*--- Generic functions ------------------------------------------------*/

/* Compute register addresses, according to ISA standard */
static void make_ISA_registers (struct parport_ip32_regs *regs,
				void __iomem *base, void __iomem *base_hi,
				int regshift)
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

/* LDD3 says that {read,write}b usage is discouraged.  But they do not exist
 * on mips, so define them.  */
#ifndef ioread8			/* and iowrite8, and ... */
#  warning FIXME: defining io* family functions
#  define ioread8(addr)		readb (addr)
#  define iowrite8(b, addr)	writeb (b, addr)
#  define ioread8_rep(a, b, c)	readsb (a, b, c)
#  define iowrite8_rep(a, b, c)	writesb (a, b, c)
#endif

/* FIXME: are the memory barriers really needed?
*/

static inline u8 parport_in (void __iomem *addr)
{
	u8 val = ioread8 (addr);
	rmb ();
	return val;
}

static inline void parport_out (u8 val, void __iomem *addr)
{
	iowrite8 (val, addr);
	wmb ();
}

static inline void parport_in_rep (void __iomem *addr, void *buf,
				   unsigned long count)
{
	ioread8_rep (addr, buf, count);
	rmb ();
}

static inline void parport_out_rep (void __iomem *addr, /*const*/ void *buf,
				    unsigned long count)
{
	iowrite8_rep (addr, buf, count);
	wmb ();
}

/*--- Debug code -------------------------------------------------------*/

#ifdef DEBUG_PARPORT_IP32
#  define pr_debug1(...)	printk (KERN_DEBUG __VA_ARGS__)
#else
#  define pr_debug1(...)	do { } while (0)
#endif

#ifdef DEBUG_PARPORT_IP32
#if DEBUG_PARPORT_IP32 >= 2

/* dump_parport_state - print register status of parport
 */
static void dump_parport_state (char *str, struct parport *p,
				int show_ecp_config)
{
	/* here's hoping that reading these ports won't side-effect
	 * anything underneath */
	struct parport_ip32_private *priv = PRIV(p);
	int i;

	printk (KERN_DEBUG PPIP32 "parport state (%s):\n", str);
	if (priv->ecr_present) {
		static const char * const ecr_modes[] = {"SPP", "PS2", "PPF",
							"ECP", "EPP", "???",
							"TST", "CFG"};
		u8 ecr = parport_in (priv->regs.ecr);
		printk (KERN_DEBUG PPIP32 "    ecr=0x%02x", ecr);
		printk (" %s",
			ecr_modes[(ecr & ECR_MODE_MASK) >> ECR_MODE_SHIFT]);
		if (ecr & ECR_IRQ)	printk (",nErrIntrEn");
		if (ecr & ECR_DMA)	printk (",dmaEn");
		if (ecr & ECR_SERVICE)	printk (",serviceIntr");
		if (ecr & ECR_F_FULL)	printk (",f_full");
		if (ecr & ECR_F_EMPTY)	printk (",f_empty");
		printk ("\n");
	}
	if (priv->ecr_present && show_ecp_config) {
		u8 oecr, cnfgA, cnfgB;
		oecr = parport_in (priv->regs.ecr);
		parport_out (ECR_MODE_CFG, priv->regs.ecr);
		cnfgA = parport_in (priv->regs.cnfgA);
		cnfgB = parport_in (priv->regs.cnfgB);
		parport_out (oecr, priv->regs.ecr);
		printk (KERN_DEBUG PPIP32 "    cnfgA=0x%02x", cnfgA);
		printk (" ISA-%s",
			(cnfgA & CNFGA_IRQ)? "Level": "Pulses");
		switch (cnfgA & CNFGA_ID_MASK) {
		case CNFGA_ID_8:	printk (",8 bits"); break;
		case CNFGA_ID_16:	printk (",16 bits"); break;
		case CNFGA_ID_32:	printk (",32 bits"); break;
		default:		printk (",unknown ID"); break;
		}
		if (! (cnfgA & CNFGA_HR_TRANS))	printk (",ByteInTrans");
		if ((cnfgA & CNFGA_ID_MASK) != CNFGA_ID_8) {
			printk (",%d byte%s left", cnfgA & CNFGA_HR_MASK,
				((cnfgA & CNFGA_HR_MASK) > 1)? "s": "");
		}
		printk ("\n");
		printk (KERN_DEBUG PPIP32 "    cnfgB=0x%02x", cnfgB);
		printk (" irq=%d,dma=%d", CNFGB_IRQ(cnfgB), CNFGB_DMA(cnfgB));
		printk (",intrValue=%d", !!(cnfgB & CNFGB_INTR));
		if (cnfgB & CNFGB_COMPRESS)	printk (",compress");
		printk ("\n");
	}
	for (i=0; i<2; i++) {
		u8 dcr = i? priv->dcr_cache: parport_in (priv->regs.dcr);
		printk (KERN_DEBUG PPIP32 "    dcr(%s)=0x%02x",
			i? "soft": "hard", dcr);
		printk (" %s", (dcr & DCR_DIR)? "rev": "fwd");
		if (dcr & DCR_IRQ)		printk (",ackIntEn");
		if (! (dcr & DCR_SELECT))	printk (",N-SELECT-IN");
		if (dcr & DCR_INIT)		printk (",N-INIT");
		if (! (dcr & DCR_AUTOFD))	printk (",N-AUTOFD");
		if (! (dcr & DCR_STROBE))	printk (",N-STROBE");
		printk ("\n");
	}
#define sep (f++? ',': ' ')
	{
		int f =0;
		u8 dsr = parport_in (priv->regs.dsr);
		printk (KERN_DEBUG PPIP32 "    dsr=0x%02x", dsr);
		if (! (dsr & DSR_BUSY))		printk ("%cBUSY", sep);
		if (dsr & DSR_ACK)		printk ("%cN-ACK", sep);
		if (dsr & DSR_PERROR)		printk ("%cPERROR", sep);
		if (dsr & DSR_SELECT)		printk ("%cSELECT", sep);
		if (dsr & DSR_FAULT)		printk ("%cN-FAULT", sep);
		if (! (dsr & DSR_PRINT))	printk ("%cN-PRINT", sep);
		if (dsr & DSR_TIMEOUT)		printk ("%cTIMEOUT", sep);
		printk ("\n");
	}
#undef sep
}

#else /* DEBUG_PARPORT_IP32 < 2 */

#define dump_parport_state(...) do { } while (0)

#endif /* DEBUG_PARPORT_IP32 < 2 */
#endif /* ! DEBUG_PARPORT_IP32 */

/*--- Some utility function to manipulate ECR register -----------------*/

static inline unsigned char parport_ip32_read_econtrol (struct parport *p)
{
	unsigned char c = parport_in (PRIV(p)->regs.ecr);
	pr_debug ("parport_ip32_read_econtrol(%p): 0x%02x\n", p, c);
	return c;
}

static inline void parport_ip32_write_econtrol (struct parport *p,
						unsigned char c)
{
	pr_debug ("parport_ip32_write_econtrol(%p, 0x%02x)\n", p, c);
	parport_out (c, PRIV(p)->regs.ecr);
}

static inline void parport_ip32_frob_econtrol (struct parport *p,
					       unsigned char mask,
					       unsigned char val)
{
	unsigned char c = 0;
	pr_debug ("parport_ip32_frob_econtrol(%p, %02x, %02x)\n",
		  p, mask, val);
	c = (mask == 0xff)? 0: parport_ip32_read_econtrol (p);
	parport_ip32_write_econtrol (p, (c & ~mask) ^ val);
}

static inline void parport_ip32_frob_set_mode (struct parport *p, int mode)
{
	pr_debug ("parport_ip32_frob_set_mode(%p, 0x%02x)\n", p, mode);
	parport_ip32_frob_econtrol (p, ECR_MODE_MASK, mode & ECR_MODE_MASK);
}

/*--- Basic functions needed for parport -------------------------------*/

static inline unsigned char parport_ip32_read_data (struct parport *p)
{
	unsigned char d = parport_in (PRIV(p)->regs.data);
	pr_debug ("parport_ip32_read_data(%p): 0x%02x\n", p, d);
	return d;
}

static inline void parport_ip32_write_data (struct parport *p,
					    unsigned char d)
{
	pr_debug ("parport_ip32_write_data(%p, 0x%02x)\n", p, d);
	parport_out (d, PRIV(p)->regs.data);
}

static inline unsigned char parport_ip32_read_status (struct parport *p)
{
	unsigned char s = parport_in (PRIV(p)->regs.dsr);
	pr_debug ("parport_ip32_read_status(%p): 0x%02x\n", p, s);
	return s;
}

/* __parport_ip32_read_control differs from parport_ip32_read_control in that
 * it doesn't do any extra masking.
 */
static inline unsigned char __parport_ip32_read_control (struct parport *p)
{
	unsigned char c = PRIV(p)->dcr_cache; /* use soft copy */
	pr_debug ("__parport_ip32_read_control(%p): 0x%02x\n", p, c);
	return c;
}

static inline unsigned char parport_ip32_read_control (struct parport *p)
{
	const unsigned char rm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);
	unsigned char c = __parport_ip32_read_control (p) & rm;
	pr_debug ("parport_ip32_read_control(%p): 0x%02x\n", p, c);
	return c;
}

/* __parport_ip32_write_control differs from parport_ip32_write_control in
 * that it doesn't do any extra masking.
 */
static inline void __parport_ip32_write_control (struct parport *p,
						 unsigned char c)
{
	struct parport_ip32_private *priv = PRIV(p);
	pr_debug ("__parport_ip32_write_control(%p, 0x%02x)\n", p, c);
#ifdef DEBUG_PARPORT_IP32
	if (c & ~priv->dcr_writable) {
		pr_debug1 (PPIP32
			   "extra bits in __write_control: 0x%02x/0x%02x\n",
			   c, priv->dcr_writable);
	}
#endif /* DEBUG_PARPORT_IP32 */
	c &= priv->dcr_writable; /* only writable bits */
	parport_out (c, priv->regs.dcr);
	priv->dcr_cache = c;		/* update soft copy */
}

/* __parport_ip32_frob_control differs from parport_ip32_frob_control in that
 * it doesn't do any extra masking.
 */
static inline void __parport_ip32_frob_control (struct parport *p,
						unsigned char mask,
						unsigned char val)
{
	unsigned char c;
	pr_debug ("__parport_ip32_frob_control(%p, 0x%02x, 0x%02x)\n",
		  p, mask, val);
	c = (__parport_ip32_read_control (p) & ~mask) ^ val;
	__parport_ip32_write_control (p, c);
}

static inline void parport_ip32_write_control (struct parport *p,
					       unsigned char c)
{
	const unsigned char wm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);
	pr_debug ("parport_ip32_write_control(%p, 0x%02x)\n", p, c);
#ifdef DEBUG_PARPORT_IP32
	if (c & ~wm) {
		pr_debug1 (PPIP32
			   "extra bits in write_control: 0x%02x/0x%02x\n",
			   c, wm);
	}
#endif /* DEBUG_PARPORT_IP32 */
	__parport_ip32_frob_control (p, wm, c & wm);
}

static inline unsigned char parport_ip32_frob_control (struct parport *p,
						       unsigned char mask,
						       unsigned char val)
{
	const unsigned char wm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);
	pr_debug ("parport_ip32_frob_control(%p, 0x%02x, 0x%02x)\n",
		 p, mask, val);
#ifdef DEBUG_PARPORT_IP32
	if (mask & ~wm || val & ~wm) {
		pr_debug1 (PPIP32
			  "extra bits in frob_control: 0x%02x,0x%02x/0x%02x",
			  mask, val, wm);
	}
#endif /* DEBUG_PARPORT_IP32 */
	__parport_ip32_frob_control (p, mask & wm, val & wm);
	return parport_ip32_read_control (p);
}

static inline void parport_ip32_disable_irq (struct parport *p)
{
	pr_debug ("parport_ip32_disable_irq(%p)\n", p);
	__parport_ip32_frob_control (p, DCR_IRQ, 0x00);
}

static inline void parport_ip32_enable_irq (struct parport *p)
{
	pr_debug ("parport_ip32_enable_irq(%p)\n", p);
	__parport_ip32_frob_control (p, DCR_IRQ, DCR_IRQ);
}

static inline void parport_ip32_data_forward (struct parport *p)
{
	pr_debug ("parport_ip32_data_forward(%p)\n", p);
	__parport_ip32_frob_control (p, DCR_DIR, 0x00);
}

static inline void parport_ip32_data_reverse (struct parport *p)
{
	pr_debug ("parport_ip32_data_reverse(%p)\n", p);
	__parport_ip32_frob_control (p, DCR_DIR, DCR_DIR);
}

static inline void parport_ip32_init_state (struct pardevice *dev,
					    struct parport_state *s)
{
	struct parport_ip32_private *priv = PRIV(dev->port);
	pr_debug ("parport_ip32_init_state(%p, %p)\n", dev, s);
	s->u.ip32.dcr = priv->dcr_init;
	s->u.ip32.ecr = priv->ecr_init;
}

static inline void parport_ip32_save_state (struct parport *p,
					    struct parport_state *s)
{
	struct parport_ip32_private *priv = PRIV(p);
	pr_debug ("parport_ip32_save_state(%p, %p)\n", p, s);
	s->u.ip32.dcr = __parport_ip32_read_control (p);
	if (priv->ecr_present) {
		s->u.ip32.ecr = parport_ip32_read_econtrol (p);
	}
}

static inline void parport_ip32_restore_state (struct parport *p,
					       struct parport_state *s)
{
	struct parport_ip32_private *priv = PRIV(p);
	pr_debug ("parport_ip32_restore_state(%p, %p)\n", p, s);
	if (priv->ecr_present) {
		parport_ip32_write_econtrol (p, s->u.ip32.ecr);
	}
	__parport_ip32_write_control (p, s->u.ip32.dcr);
}

/*--- Simple interrupt handler -----------------------------------------*/

static irqreturn_t parport_ip32_interrupt (int irq, void *dev_id,
					   struct pt_regs *regs)
{
	pr_debug ("parport_ip32_interrupt(%d, %p, %p)\n", irq, dev_id, regs);
	parport_generic_irq (irq, (struct parport *)dev_id, regs);
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}

/*--- EPP mode functions -----------------------------------------------*/

/* clear_epp_timeout - clear Timeout bit in EPP mode
 *
 * Returns 0 if it failed to clear Timeout bit.
 *
 * This is also used in SPP detection.
 */
static int clear_epp_timeout(struct parport *p)
{
	struct parport_ip32_private *priv = PRIV(p);
	unsigned char r;

	if (! (parport_ip32_read_status (p) & DSR_TIMEOUT))
		return 1;

	/* To clear timeout some chips require double read */
	parport_ip32_read_status (p);
	r = parport_ip32_read_status (p);
	/* Some reset by writing 1 */
	parport_out (r | DSR_TIMEOUT, priv->regs.dsr);
	/* Others by writing 0 */
	parport_out (r & ~DSR_TIMEOUT, priv->regs.dsr);

	r = parport_ip32_read_status (p);

	return !(r & DSR_TIMEOUT);
}

/*--- Default operations -----------------------------------------------*/

static __initdata struct parport_operations parport_ip32_ops = {
	.write_data	= parport_ip32_write_data,
	.read_data	= parport_ip32_read_data,

	.write_control	= parport_ip32_write_control,
	.read_control	= parport_ip32_read_control,
	.frob_control	= parport_ip32_frob_control,

	.read_status	= parport_ip32_read_status,

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
 * Returns 0 if mode is not supported.  Adjust parameters in the parport
 * structure.
 *
 */

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
static __init int parport_ECR_supported (struct parport *p)
{
	struct parport_ip32_private *priv = PRIV(p);
	unsigned char r = DCR_SELECT | DCR_INIT;

	parport_out (r, priv->regs.dcr);
	if ((parport_in (priv->regs.ecr) & 0x3) == (r & 0x3)) {
		/* Toggle bit 1 */
		parport_out (r & 0x2, priv->regs.dcr);

		r = parport_in (priv->regs.dcr);
		if ((parport_in (priv->regs.ecr) & 0x2) == (r & 0x2))
			goto no_reg; /* Sure that no ECR register exists */
	}

	r = parport_in (priv->regs.ecr) & (ECR_F_FULL | ECR_F_EMPTY);
	if (r != ECR_F_EMPTY)
		goto no_reg;

	r = ECR_MODE_PS2 | ECR_IRQ | ECR_SERVICE;
	parport_out (r, priv->regs.ecr);
	if (parport_in (priv->regs.ecr) != (r | ECR_F_EMPTY))
		goto no_reg;

	pr_probe (p, "Found working ECR register\n");
	priv->ecr_present = 1;
	priv->ecr_init = ECR_MODE_SPP | ECR_IRQ | ECR_SERVICE;
	parport_ip32_write_control (p, DCR_SELECT | DCR_INIT);
	parport_ip32_frob_set_mode (p, ECR_MODE_SPP);
	return 1;

no_reg:
	pr_probe (p, "ECR register not found\n");
	priv->ecr_present = 0;
	return 0;
}

/* Check if the port behaves correctly.
 *
 * Note: with ECP port, the port must be in SPP mode.
 */
static __init int parport_SPP_supported (struct parport *p)
{
	struct parport_ip32_private *priv = PRIV(p);
	unsigned char r, w;

	/* first clear an eventually pending EPP timeout
	 * I (sailer@ife.ee.ethz.ch) have an SMSC chipset
	 * that does not even respond to SPP cycles if an EPP
	 * timeout is pending
	 */
	clear_epp_timeout (p);

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
	return 0;

spp_ok:
	pr_probe (p, "Found working SPP register\n");
	p->modes |= PARPORT_MODE_PCSPP;
	/* FIXME:
	 *   What does exactly SAFEININT mean?
	 *   Is there some way to test for it?
	 * Activate it unless someone complains.
	 */
	p->modes |= PARPORT_MODE_SAFEININT;
	priv->dcr_init = DCR_SELECT | DCR_INIT;
	parport_ip32_write_control (p, priv->dcr_init);
	return 1;
}

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
static __init int parport_PS2_supported (struct parport *p)
{
	struct parport_ip32_private *priv = PRIV(p);
	unsigned char oecr = 0;
	int ok = 0;

#ifndef PARPORT_IP32_PS2
	return 0;
#endif

	if (priv->ecr_present) {
		oecr = parport_ip32_read_econtrol (p);
		parport_ip32_write_econtrol (p, ECR_MODE_PS2);
	}

	clear_epp_timeout (p);

	/* Try to tri-state the buffer. */
	priv->dcr_writable |= DCR_DIR;
	parport_ip32_data_reverse (p);

	parport_ip32_write_data (p, 0x55);
	if (parport_ip32_read_data (p) != 0x55)
		ok++;

	parport_ip32_write_data (p, 0xaa);
	if (parport_ip32_read_data (p) != 0xaa)
		ok++;

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
		parport_ip32_write_econtrol (p, oecr);
	}

	pr_probe (p, "PS2 mode%s supported\n", ok? "": " not");
	return ok;
}

/* Check for Enhanced Parallel Port. */
static __init int parport_EPP_supported (struct parport *p)
{
#ifndef PARPORT_IP32_EPP
	return 0;
#endif
	// FIXME...
	return 0;
}

/* Check for Extended Capabilites Port.  Actually search FIFO parameters. */
static __init int parport_ECP_supported (struct parport *p)
{
	struct parport_ip32_private *priv = PRIV(p);
	int i;
	int configa, configb;
	int pword;

#ifndef PARPORT_IP32_FIFO
	return 0;
#endif

	/* If there is no ECR, we have no hope of supporting ECP. */
	if (! priv->ecr_present)
		return 0;
	/* We must be able to tristate the port. */
	if (! (p->modes & PARPORT_MODE_TRISTATE))
		return 0;

	/* Find out PWord size */
	parport_out (ECR_MODE_SPP, priv->regs.ecr); /* Reset FIFO */
	parport_out (ECR_MODE_CFG, priv->regs.ecr); /* Configuration mode */

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

	switch (configa & CNFGA_ID_MASK) {
	case CNFGA_ID_8:	pword = 1; break;
	case CNFGA_ID_16:	pword = 2; break;
	case CNFGA_ID_32:	pword = 4; break;
	default:
		pr_probe (p, "Unknown implementation ID: 0x%0x\n",
			  (configa & CNFGA_ID_MASK) >> CNFGA_ID_SHIFT);
		goto no_fifo;
		break;
	}
	if (pword != 1) {
		pr_probe (p, "Unsupported PWord size: %d\n", pword);
		goto no_fifo;
	}
	priv->pword = pword;
	pr_probe (p, "PWord is %d bits\n", 8 * priv->pword);

	/* Find out FIFO depth */
	parport_out (ECR_MODE_SPP, priv->regs.ecr); /* Reset FIFO */
	parport_out (ECR_MODE_TST, priv->regs.ecr); /* TEST FIFO */
	for (i=0; i < 1024 && !(parport_in(priv->regs.ecr) & ECR_F_FULL); i++)
		parport_out (0xaa, priv->regs.tFifo);

	/* FIFO seems too big: not supported */
	if (i == 1024) {
		pr_probe (p, "Could not get FIFO depth\n");
		goto no_fifo;
	}

	priv->fifo_depth = i;
	pr_probe (p, "FIFO is %d PWords deep\n", priv->fifo_depth);

	/* Find out writeIntrThreshold: number of PWords we know we can
	 * write if we get an interrupt. */
	parport_ip32_frob_econtrol (p, ECR_SERVICE, ECR_SERVICE);
	parport_ip32_frob_econtrol (p, ECR_SERVICE, 0);
	for (i = 1; i <= priv->fifo_depth; i++) {
		parport_in (priv->regs.tFifo);
		udelay (50);
		if (parport_in (priv->regs.ecr) & ECR_SERVICE)
			break;
	}
	if (i > priv->fifo_depth) {
		i = 0;
	}
	priv->writeIntrThreshold = i;
	pr_probe (p, "writeIntrThreshold is %d\n", priv->writeIntrThreshold);

	/* Find out readIntrThreshold: number of PWords we can read if
	 * we get an interrupt. */
	parport_ip32_frob_set_mode (p, ECR_MODE_PS2); /* Reset FIFO and enable
							 PS2 */
	parport_ip32_data_reverse (p); /* Must be in PS2 mode */
	parport_ip32_frob_set_mode (p, ECR_MODE_TST); /* Test FIFO */
	parport_ip32_frob_econtrol (p, ECR_SERVICE, ECR_SERVICE);
	parport_ip32_frob_econtrol (p, ECR_SERVICE, 0);
	for (i = 1; i <= priv->fifo_depth; i++) {
		parport_out (0xaa, priv->regs.tFifo);
		if (parport_in (priv->regs.ecr) & ECR_SERVICE)
			break;
	}
	if (i > priv->fifo_depth) {
		i = 0;
	}
	priv->readIntrThreshold = i;
	pr_probe (p, "readIntrThreshold is %d\n", priv->writeIntrThreshold);

	/* Go back to mode 000 */
	parport_ip32_write_econtrol (p, ECR_MODE_SPP);
	return 1;

no_fifo:
	priv->fifo_depth = 0;
	parport_ip32_write_econtrol (p, ECR_MODE_SPP);
	return 0;
}

/* parport_ip32_probe_port - probe and register a new port
 *   base: base I/O addresse
 *   base_hi:  -
 *   irq: IRQ line
 *   dma: DMA channel
 *   regs: pointer to the port register adresses structure
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
		printk (KERN_WARNING PPIP32 "irq probing is not supported\n");
		irq = PARPORT_IRQ_NONE;
		break;
	}

	switch (dma) {
	case PARPORT_DMA_AUTO:
		printk (KERN_WARNING PPIP32 "dma probing is not supported\n");
	case PARPORT_DMA_NOFIFO:
		dma = PARPORT_DMA_NONE;
		break;
	}

	ops = kmalloc(sizeof (struct parport_operations), GFP_KERNEL);
	if (! ops) {
		goto out1;
	}
	priv = kmalloc (sizeof (struct parport_ip32_private), GFP_KERNEL);
	if (! priv) {
		goto out2;
	}
	/* A misnomer, actually it's allocate and reserve parport number. */
	p = parport_register_port(base, irq, dma, ops);
	if (! p) {
		goto out3;
	}

	/* Initialize parport structure. Be conservative. */
	*ops = parport_ip32_ops;
	*priv = (struct parport_ip32_private ){
		.regs =		*regs,

		.dcr_init =	DCR_SELECT | DCR_INIT,
		.ecr_init =	ECR_MODE_SPP | ECR_IRQ | ECR_SERVICE,
		.dcr_cache =	0,
		.dcr_writable =	DCR_SELECT | DCR_INIT |
				DCR_AUTOFD | DCR_STROBE,
		.ecr_present =	!!(modes & PARPORT_MODE_ECP),
		.fifo_depth =	0,
		.pword =	1,
		.readIntrThreshold = 0,
		.writeIntrThreshold = 0
	};
	p->private_data = priv;
	p->base_hi = base_hi;

	dump_parport_state ("begin init", p, modes & PARPORT_MODE_ECP);

	/* First, check if we can use the Extended Control Register. */
	parport_ECR_supported (p);
	/* If SPP mode does not work, we can't go very far. */
	if (! parport_SPP_supported (p)) {
		goto out4;
	}
	/* Is the port bidirectional? */
	parport_PS2_supported (p);

	if (modes & PARPORT_MODE_EPP) {
		parport_EPP_supported (p);
	}
	if (priv->ecr_present) {
		parport_ECP_supported (p);
	}

	/* Request IRQ */
	if (p->irq != PARPORT_IRQ_NONE) {
		if (request_irq (p->irq, parport_ip32_interrupt,
				 0, p->name, p)) {
			printk (KERN_WARNING "%s: irq %d in use, "
				"resorting to polled operation\n",
				p->name, p->irq);
			p->irq = PARPORT_IRQ_NONE;
/*			p->dma = PARPORT_DMA_NONE; ??? */
			priv->dcr_writable &= ~DCR_IRQ;
		} else {
			priv->dcr_writable |= DCR_IRQ;
		}
	}

	/* Initialise the port with sensible values */
	parport_ip32_write_econtrol (p, priv->ecr_init);
	parport_ip32_write_control (p, priv->dcr_init);
	parport_ip32_write_data (p, 0x00);

	dump_parport_state ("end init", p, 0);

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
		int f = 0;
		printmode (PCSPP);
		printmode (TRISTATE);
		printmode (COMPAT);
		printmode (EPP);
		printmode (ECP);
		printmode (DMA);
	}
#undef printmode
#ifndef CONFIG_PARPORT_1284
	printk ("(,...)");
#endif
	printk ("]\n");

	/* Now that we've told the sharing engine about the port, and found
	 * out its characteristics, let the high-level drivers know about
	 * it. */
	parport_announce_port (p);

	return p;

out4:
	parport_put_port(p);
out3:
	kfree (priv);
out2:
	kfree (ops);
out1:
	return NULL;
}

static __exit void parport_ip32_unregister_port (struct parport *p)
{
	struct parport_ip32_private *priv = PRIV(p);
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
	int modes;

	printk (KERN_INFO PPIP32 "SGI IP32 MACE parallel port driver\n");

	iomap_mace_address ();
	if (mace == NULL) {
		printk (KERN_WARNING PPIP32 "invalid mace pointer\n");
		return -ENXIO;
	}

	make_ISA_registers (&regs,
			    PARPORT_IP32_IO_ADDR, PARPORT_IP32_IOHI_ADDR,
			    PARPORT_IP32_REGSHIFT);
	modes = PARPORT_MODE_EPP | PARPORT_MODE_ECP;
	this_port = parport_ip32_probe_port (PARPORT_IP32_IO,
					     PARPORT_IP32_IOHI,
					     PARPORT_IP32_IRQ,
					     PARPORT_IP32_DMA,
					     &regs, modes);

	if (this_port == NULL) {
		printk (KERN_INFO PPIP32 "parport_ip32_probe_port failed\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit parport_ip32_exit (void)
{
	parport_ip32_unregister_port (this_port);
	iounmap_mace_address ();
}

module_init (parport_ip32_init);
module_exit (parport_ip32_exit);

/* Inform (X)Emacs about preferred coding style
 *
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
