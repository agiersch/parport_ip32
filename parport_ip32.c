/* Low-level parallel port routines for built-in port on SGI IP32
 *
 * Author: Arnaud Giersch <arnaud.giersch@free.fr>
 *
 * $Id: parport_ip32.c,v 1.44 2005-11-05 16:26:23 arnaud Exp $
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

/*--- Some configuration defines ---------------------------------------*/

/* DEBUG_PARPORT_IP32
 *	0	disable debug
 *	1	standard level: pr_debug1 is enabled
 *	2	parport_ip32_dump_state is enabled
 *	>2	verbose level: pr_debug is enabled
 */
#define DEBUG_PARPORT_IP32  1	/* disable for production */

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
#	if !defined(DEBUG)
#		define DEBUG /* enable pr_debug() in kernel.h */
#	endif
#endif

#include <linux/config.h>
#include <linux/delay.h>
#include <linux/err.h>
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
#include <asm/ip32/ip32_ints.h>
#include <asm/ip32/mace.h>

#undef BIT
#define BIT(n) (1U << (n))

/*--- Global variables -------------------------------------------------*/

/* Verbose probing on by default for debugging. */
#if DEBUG_PARPORT_IP32 >= 1
#	define DEFAULT_VERBOSE_PROBING	1
#else
#	define DEFAULT_VERBOSE_PROBING	0
#endif

/* Default prefix for printk */
#define PPIP32 "parport_ip32: "

/*
 * These are the module parameters:
 * @features:		bit mask of features to enable/disable
 *			(all enabled by default)
 * @verbose_probing:	log chit-chat during initialization
 */
static unsigned int features =	~0U;
#define PARPORT_IP32_ENABLE_IRQ	BIT(0)
#define PARPORT_IP32_ENABLE_DMA	BIT(1)
#define PARPORT_IP32_ENABLE_SPP	BIT(2)
#define PARPORT_IP32_ENABLE_EPP	BIT(3)
#define PARPORT_IP32_ENABLE_ECP	BIT(4)
static int verbose_probing =	DEFAULT_VERBOSE_PROBING;

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
 * @ecpAFifo:	ECP Address FIFO
 * @fifo:	General FIFO register.  The same address is used for:
 *		- cFifo, the Parallel Port DATA FIFO
 *		- ecpDFifo, the ECP Data FIFO
 *		- tFifo, the ECP Test FIFO
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
	void __iomem *ecpAFifo;
	void __iomem *fifo;
	void __iomem *cnfgA;
	void __iomem *cnfgB;
	void __iomem *ecr;
};

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
 * @dcr_cache:		cached contents of DCR
 * @dcr_writable:	bit mask of writable DCR bits
 * @pword:		number of bytes per PWord
 * @fifo_depth:		number of PWords that FIFO will hold
 * @readIntrThreshold:	minimum number of PWords we can read
 *			if we get an interrupt
 * @writeIntrThreshold:	minimum number of PWords we can write
 *			if we get an interrupt
 * @irq_mode:		operation mode of interrupt handler for this port
 * @irq:		mutex used to wait for an interrupt
 */
struct parport_ip32_private {
	struct parport_ip32_regs regs;
	unsigned int dcr_cache;
	unsigned int dcr_writable;
	unsigned int pword;
	unsigned int fifo_depth;
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
 * &parport structure.  This happens very often, so use this macro to be sure
 * not to forget the physport part.
 */
#define PRIV(p) ((p)->physport->private_data)

/*--- I/O register access functions ------------------------------------*/

/* FIXME - Use io{read,write}8 (and _rep) when available on MIPS?  */
/* FIXME - Are the memory barriers really needed?  */

/**
 * parport_ip32_in - read a register
 * @addr:	address of register
 */
static inline u8 parport_ip32_in(void __iomem *addr)
{
	u8 val = readb(addr);
	rmb();
	return val;
}

/**
 * parport_ip32_out - write some value to a register
 * @val:	value to write
 * @addr:	address of register
 */
static inline void parport_ip32_out(u8 val, void __iomem *addr)
{
	writeb(val, addr);
	wmb();
}

/**
 * parport_ip32_out_rep - write multiple values to a register
 * @addr:	address of register
 * @buf:	buffer of values to write
 * @count:	number of bytes to write
 */
static inline void parport_ip32_out_rep(void __iomem *addr,
					const void *buf, unsigned long count)
{
	writesb(addr, buf, count);
	wmb();
}

/*--- Debug code -------------------------------------------------------*/

/**
 * pr_debug1 - print debug messages
 *
 * This is like pr_debug(), but is defined for %DEBUG_PARPORT_IP32 >= 1
 */
#if DEBUG_PARPORT_IP32 >= 1
#	define pr_debug1(...)	printk(KERN_DEBUG __VA_ARGS__)
#else /* DEBUG_PARPORT_IP32 < 1 */
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
static void parport_ip32_dump_state(struct parport *p, char *str,
				    unsigned int show_ecp_config)
{
	/* here's hoping that reading these ports won't side-effect
	 * anything underneath */
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int i;

	printk(KERN_DEBUG PPIP32 "%s: state (%s):\n", p->name, str);
	{
		static const char ecr_modes[8][4] = {"SPP", "PS2", "PPF",
						     "ECP", "EPP", "???",
						     "TST", "CFG"};
		unsigned int ecr = parport_ip32_in(priv->regs.ecr);
		printk(KERN_DEBUG PPIP32 "    ecr=0x%02x", ecr);
		printk(" %s",
		       ecr_modes[(ecr & ECR_MODE_MASK) >> ECR_MODE_SHIFT]);
		if (ecr & ECR_nERRINTR)	printk(",nErrIntrEn");
		if (ecr & ECR_DMA)	printk(",dmaEn");
		if (ecr & ECR_SERVINTR)	printk(",serviceIntr");
		if (ecr & ECR_F_FULL)	printk(",f_full");
		if (ecr & ECR_F_EMPTY)	printk(",f_empty");
		printk("\n");
	}
	if (show_ecp_config) {
		unsigned int oecr, cnfgA, cnfgB;
		oecr = parport_ip32_in(priv->regs.ecr);
		parport_ip32_out(ECR_MODE_PS2, priv->regs.ecr);
		parport_ip32_out(ECR_MODE_CFG, priv->regs.ecr);
		cnfgA = parport_ip32_in(priv->regs.cnfgA);
		cnfgB = parport_ip32_in(priv->regs.cnfgB);
		parport_ip32_out(ECR_MODE_PS2, priv->regs.ecr);
		parport_ip32_out(oecr, priv->regs.ecr);
		printk(KERN_DEBUG PPIP32 "    cnfgA=0x%02x", cnfgA);
		printk(" ISA-%s", (cnfgA & CNFGA_IRQ)? "Level": "Pulses");
		switch (cnfgA & CNFGA_ID_MASK) {
		case CNFGA_ID_8:	printk(",8 bits"); break;
		case CNFGA_ID_16:	printk(",16 bits"); break;
		case CNFGA_ID_32:	printk(",32 bits"); break;
		default:		printk(",unknown ID"); break;
		}
		if (!(cnfgA & CNFGA_nBYTEINTRANS))  printk(",ByteInTrans");
		if ((cnfgA & CNFGA_ID_MASK) != CNFGA_ID_8) {
			printk(",%d byte%s left", cnfgA & CNFGA_PWORDLEFT,
			       ((cnfgA & CNFGA_PWORDLEFT) > 1)? "s": "");
		}
		printk("\n");
		printk(KERN_DEBUG PPIP32 "    cnfgB=0x%02x", cnfgB);
		printk(" irq=%u,dma=%u", CNFGB_IRQ(cnfgB), CNFGB_DMA(cnfgB));
		printk(",intrValue=%d", !!(cnfgB & CNFGB_INTRVAL));
		if (cnfgB & CNFGB_COMPRESS)	printk(",compress");
		printk("\n");
	}
	for (i = 0; i < 2; i++) {
		unsigned int dcr =
			i? priv->dcr_cache: parport_ip32_in(priv->regs.dcr);
		printk(KERN_DEBUG PPIP32 "    dcr(%s)=0x%02x",
		       i? "soft": "hard", dcr);
		printk(" %s", (dcr & DCR_DIR)? "rev": "fwd");
		if (dcr & DCR_IRQ)		printk(",ackIntEn");
		if (!(dcr & DCR_SELECT))	printk(",nSelectIn");
		if (dcr & DCR_nINIT)		printk(",nInit");
		if (!(dcr & DCR_AUTOFD))	printk(",nAutoFD");
		if (!(dcr & DCR_STROBE))	printk(",nStrobe");
		printk("\n");
	}
#define sep (f++? ',': ' ')
	{
		unsigned int f = 0;
		unsigned int dsr = parport_ip32_in(priv->regs.dsr);
		printk(KERN_DEBUG PPIP32 "    dsr=0x%02x", dsr);
		if (!(dsr & DSR_nBUSY))		printk("%cBusy", sep);
		if (dsr & DSR_nACK)		printk("%cnAck", sep);
		if (dsr & DSR_PERROR)		printk("%cPError", sep);
		if (dsr & DSR_SELECT)		printk("%cSelect", sep);
		if (dsr & DSR_nFAULT)		printk("%cnFault", sep);
		if (!(dsr & DSR_nPRINT))	printk("%c(Print)", sep);
		if (dsr & DSR_TIMEOUT)		printk("%cTimeout", sep);
		printk("\n");
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
		unsigned int __b = (b), __m = (m);			\
		if (__b & ~__m)						\
			pr_debug1(PPIP32 "%s: extra bits in %s(%s): "	\
				  "0x%02x/0x%02x\n",			\
				  (p)->name, __func__, #b, __b, __m);	\
	} while (0)
#else /* DEBUG_PARPORT_IP32 < 1 */
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
	pr("%s: %s" fmt "\n", (p)->name, __func__ , ##__VA_ARGS__)
#define pr_trace(p, fmt, ...)	__pr_trace(pr_debug, p, fmt, __VA_ARGS__)
#define pr_trace1(p, fmt, ...)	__pr_trace(pr_debug1, p, fmt, __VA_ARGS__)

/**
 * __pr_probe, pr_probe - print message if @verbose_probing is true
 * @p:		pointer to &struct parport
 * @fmt:	printk format string
 * @...:	parameters for format string
 *
 * For new lines, use pr_probe().  Use __pr_probe() for continued lines.
 */
#define __pr_probe(...)							\
	do { if (verbose_probing) printk(__VA_ARGS__); } while (0)
#define pr_probe(p, fmt, ...)						\
	__pr_probe(KERN_INFO PPIP32 "0x%lx: " fmt, (p)->base , ##__VA_ARGS__)

/*--- Some utility function to manipulate ECR register -----------------*/

/**
 * parport_ip32_read_econtrol - read contents of the ECR register
 * @p:	pointer to &struct parport
 */
static inline unsigned int parport_ip32_read_econtrol(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int c = parport_ip32_in(priv->regs.ecr);
	pr_trace(p, "(): 0x%02x", c);
	return c;
}

/**
 * parport_ip32_write_econtrol - write new contents to the ECR register
 * @p:	pointer to &struct parport
 * @c:	new value to write
 */
static inline void parport_ip32_write_econtrol(struct parport *p,
					       unsigned int c)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_trace(p, "(0x%02x)", c);
	parport_ip32_out(c, priv->regs.ecr);
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
static inline void parport_ip32_frob_econtrol(struct parport *p,
					      unsigned int mask,
					      unsigned int val)
{
	unsigned int c;
	pr_trace(p, "(%02x, %02x)", mask, val);
	c = (parport_ip32_read_econtrol(p) & ~mask) ^ val;
	parport_ip32_write_econtrol(p, c);
}

/**
 * parport_ip32_set_mode - change mode of ECP port
 * @p:		pointer to &struct parport
 * @mode:	new ECP mode
 *
 * ECR is reset in a sane state (interrupts and DMA disabled), and placed in
 * mode @mode.  Go through PS2 mode if needed.
 */
static inline void parport_ip32_set_mode(struct parport *p, unsigned int mode)
{
	unsigned int omode;
	pr_trace(p, "(0x%02x)", mode);

	mode &= ECR_MODE_MASK;
	omode = parport_ip32_read_econtrol(p) & ECR_MODE_MASK;

	if (!(mode == ECR_MODE_SPP || mode == ECR_MODE_PS2
	      || omode == ECR_MODE_SPP || omode == ECR_MODE_PS2)) {
		/* We have to go through PS2 mode */
		unsigned int ecr = ECR_MODE_PS2 | ECR_nERRINTR | ECR_SERVINTR;
		parport_ip32_write_econtrol(p, ecr);
	}
	parport_ip32_write_econtrol(p, mode | ECR_nERRINTR | ECR_SERVINTR);
}

/*--- Basic functions needed for parport -------------------------------*/

/**
 * parport_ip32_read_data - return current contents of the DATA register
 * @p:		pointer to &struct parport
 */
static inline unsigned char parport_ip32_read_data(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int d = parport_ip32_in(priv->regs.data);
	pr_trace(p, "(): 0x%02x", d);
	return d;
}

/**
 * parport_ip32_write_data - set new contents for the DATA register
 * @p:		pointer to &struct parport
 * @d:		new value to write
 */
static inline void parport_ip32_write_data(struct parport *p, unsigned char d)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_trace(p, "(0x%02x)", d);
	parport_ip32_out(d, priv->regs.data);
}

/**
 * parport_ip32_read_status - return current contents of the DSR register
 * @p:		pointer to &struct parport
 */
static inline unsigned char parport_ip32_read_status(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int s = parport_ip32_in(priv->regs.dsr);
	pr_trace(p, "(): 0x%02x", s);
	return s;
}

/**
 * __parport_ip32_read_control - return cached contents of the DCR register
 * @p:		pointer to &struct parport
 */
static inline unsigned int __parport_ip32_read_control(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int c = priv->dcr_cache; /* use soft copy */
	pr_trace(p, "(): 0x%02x", c);
	return c;
}

/**
 * parport_ip32_read_control - return cached contents of the DCR register
 * @p:		pointer to &struct parport
 *
 * The return value is masked so as to only return the value of %DCR_STROBE,
 * %DCR_AUTOFD, %DCR_nINIT, and %DCR_SELECT.
 */
static inline unsigned char parport_ip32_read_control(struct parport *p)
{
	const unsigned int rm =
		DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	unsigned int c = __parport_ip32_read_control(p) & rm;
	pr_trace(p, "(): 0x%02x", c);
	return c;
}

/**
 * __parport_ip32_write_control - set new contents for the DCR register
 * @p:		pointer to &struct parport
 * @c:		new value to write
 */
static inline void __parport_ip32_write_control(struct parport *p,
						unsigned int c)
{
	struct parport_ip32_private * const priv = PRIV(p);
	pr_trace(p, "(0x%02x)", c);
	CHECK_EXTRA_BITS(p, c, priv->dcr_writable);
	c &= priv->dcr_writable; /* only writable bits */
	parport_ip32_out(c, priv->regs.dcr);
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
static inline void __parport_ip32_frob_control(struct parport *p,
					       unsigned int mask,
					       unsigned int val)
{
	unsigned int c;
	pr_trace(p, "(0x%02x, 0x%02x)", mask, val);
	c = (__parport_ip32_read_control(p) & ~mask) ^ val;
	__parport_ip32_write_control(p, c);
}

/**
 * parport_ip32_write_control - set new contents for the DCR register
 * @p:		pointer to &struct parport
 * @c:		new value to write
 *
 * The value is masked so as to only change the value of %DCR_STROBE,
 * %DCR_AUTOFD, %DCR_nINIT, and %DCR_SELECT.
 */
static inline void parport_ip32_write_control(struct parport *p,
					      unsigned char c)
{
	const unsigned int wm =
		DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	pr_trace(p, "(0x%02x)", c);
	CHECK_EXTRA_BITS(p, c, wm);
	__parport_ip32_frob_control(p, wm, c & wm);
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
static inline unsigned char parport_ip32_frob_control(struct parport *p,
						      unsigned char mask,
						      unsigned char val)
{
	const unsigned int wm =
		DCR_STROBE | DCR_AUTOFD | DCR_nINIT | DCR_SELECT;
	pr_trace(p, "(0x%02x, 0x%02x)\n", mask, val);
	CHECK_EXTRA_BITS(p, mask, wm);
	CHECK_EXTRA_BITS(p, val, wm);
	__parport_ip32_frob_control(p, mask & wm, val & wm);
	return parport_ip32_read_control(p);
}

/**
 * parport_ip32_disable_irq - disable interrupts on the rising edge of nACK
 * @p:		pointer to &struct parport
 */
static inline void parport_ip32_disable_irq(struct parport *p)
{
	pr_trace(p, "()");
	__parport_ip32_frob_control(p, DCR_IRQ, 0);
}

/**
 * parport_ip32_enable_irq - enable interrupts on the rising edge of nACK
 * @p:		pointer to &struct parport
 */
static inline void parport_ip32_enable_irq(struct parport *p)
{
	pr_trace(p, "()");
	__parport_ip32_frob_control(p, DCR_IRQ, DCR_IRQ);
}

/**
 * parport_ip32_data_forward - enable host-to-peripheral communications
 * @p:		pointer to &struct parport
 *
 * Enable the data line drivers, for 8-bit host-to-peripheral communications.
 */
static inline void parport_ip32_data_forward(struct parport *p)
{
	pr_trace(p, "()");
	__parport_ip32_frob_control(p, DCR_DIR, 0);
}

/**
 * parport_ip32_data_reverse - enable peripheral-to-host communications
 * @p:		pointer to &struct parport
 *
 * Place the data bus in a high impedance state, if @p->modes has the
 * PARPORT_MODE_TRISTATE bit set.
 */
static inline void parport_ip32_data_reverse(struct parport *p)
{
	pr_trace(p, "()");
	__parport_ip32_frob_control(p, DCR_DIR, DCR_DIR);
}

/**
 * parport_ip32_init_state - for core parport code
 */
static inline void parport_ip32_init_state(struct pardevice *dev,
					   struct parport_state *s)
{
	pr_trace(dev->port, "(%s, %p)", dev->name, s);
	s->u.ip32.dcr = DCR_SELECT | DCR_nINIT;
	s->u.ip32.ecr = ECR_MODE_PS2 | ECR_nERRINTR | ECR_SERVINTR;
}

/**
 * parport_ip32_save_state - for core parport code
 */
static inline void parport_ip32_save_state(struct parport *p,
					   struct parport_state *s)
{
	pr_trace(p, "(%p)", s);
	s->u.ip32.dcr = __parport_ip32_read_control(p);
	s->u.ip32.ecr = parport_ip32_read_econtrol(p);
}

/**
 * parport_ip32_restore_state - for core parport code
 */
static inline void parport_ip32_restore_state(struct parport *p,
					      struct parport_state *s)
{
	pr_trace(p, "(%p)", s);
	parport_ip32_set_mode(p,  s->u.ip32.ecr & ECR_MODE_MASK);
	parport_ip32_write_econtrol(p, s->u.ip32.ecr);
	__parport_ip32_write_control(p, s->u.ip32.dcr);
}

/*--- Interrupt handlers and associates --------------------------------*/

/**
 * parport_ip32_wakeup - wakes up code waiting for an interrupt
 * @port:	pointer to &struct parport
 */
static inline void parport_ip32_wakeup(struct parport *port)
{
	struct parport_ip32_private * const priv = PRIV(port);
	up(&priv->irq);
}

/**
 * parport_ip32_interrupt - interrupt handler
 *
 * Caught interrupts are forwarded to the upper parport layer if IRQ_mode is
 * %PARPORT_IP32_IRQ_FWD.
 */
static irqreturn_t parport_ip32_interrupt(int irq, void *dev_id,
					  struct pt_regs *regs)
{
	struct parport * const port = dev_id;
	struct parport_ip32_private * const priv = PRIV(port);
	enum parport_ip32_irq_mode irq_mode = priv->irq_mode;
	barrier();		/* ensures that priv->irq_mode is read */
	pr_trace(port, "(%d)", irq);
	switch (irq_mode) {
	case PARPORT_IP32_IRQ_FWD:
		parport_generic_irq(irq, port, regs);
		break;
	case PARPORT_IP32_IRQ_HERE:
		parport_ip32_wakeup(port);
		break;
	}
	return IRQ_HANDLED;
}

/**
 * parport_ip32_timeout - timeout handler
 */
static void parport_ip32_timeout(unsigned long data)
{
	struct parport * const port = (struct parport *)data;
	parport_ip32_wakeup(port);
}

/*--- EPP mode functions -----------------------------------------------*/

#if 0				/* FIXME - not used yet */
/**
 * parport_ip32_clear_epp_timeout - clear Timeout bit in EPP mode
 * @p:	pointer to &struct parport
 *
 * Returns 1 if the Timeout bit is clear, and 0 otherwise.
 */
static unsigned int parport_ip32_clear_epp_timeout(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int cleared;
	unsigned int r;

	if (!(parport_ip32_read_status(p) & DSR_TIMEOUT)) {
		cleared = 1;
	} else {
		/* To clear timeout some chips require double read */
		parport_ip32_read_status(p);
		r = parport_ip32_read_status(p);
		/* Some reset by writing 1 */
		parport_ip32_out(r | DSR_TIMEOUT, priv->regs.dsr);
		/* Others by writing 0 */
		parport_ip32_out(r & ~DSR_TIMEOUT, priv->regs.dsr);

		r = parport_ip32_read_status(p);
		cleared = !(r & DSR_TIMEOUT);
	}

	pr_trace(p, "(): %s", cleared? "cleared": "failed");
	return cleared;
}
#endif

/*
 * FIXME - Insert here parport_ip32_epp_{read,write}_{data,address}().
 */

/*--- ECP mode functions (FIFO) ----------------------------------------*/

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
static inline unsigned int parport_ip32_fwp_wait_break(struct parport *port,
						       unsigned long expire)
{
	/* Time to resched? */
	cond_resched();
	/* Timed out? */
	if (time_after(jiffies, expire)) {
		printk(KERN_DEBUG PPIP32
		       "%s: FIFO write timed out\n", port->name);
		return 1;
	}
	/* Pending signal? */
	if (signal_pending(current)) {
		printk(KERN_DEBUG PPIP32
		       "%s: Signal pending\n", port->name);
		return 1;
	}
	/* nFault? */
	if (!(parport_ip32_read_status(port) & DSR_nFAULT)) {
		printk(KERN_DEBUG PPIP32
		       "%s: nFault asserted low\n", port->name);
		return 1;
	}
	return 0;
}

/**
 * parport_ip32_fwp_wait_polling - wait for FIFO to empty (polling)
 * @port:	pointer to &struct parport
 *
 * Used by parport_ip32_fifo_write_pio_wait().
 */
static unsigned int parport_ip32_fwp_wait_polling(struct parport *port)
{
	static const unsigned int polling_interval = 50; /* usecs */
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	unsigned int ecr;
	unsigned long expire = jiffies + physport->cad->timeout;
	unsigned int count = 0;

	while (1) {
		if (parport_ip32_fwp_wait_break(port, expire))
			break;

		/* Check FIFO state.  We do nothing when the FIFO is nor full,
		 * nor empty.  It appears that the FIFO full bit is not always
		 * reliable, the FIFO state is sometimes wrongly reported, and
		 * the chip gets confused if we give it another byte. */
		ecr = parport_ip32_read_econtrol(port);
		if (ecr & ECR_F_EMPTY) {
			/* FIFO is empty, fill it up */
			count = priv->fifo_depth;
			break;
		}

		/* Wait a moment... */
		udelay(polling_interval);
	} /* while (1) */

	return count;
}

/**
 * parport_ip32_fwp_wait_interrupt - wait for FIFO to empty (interrupt-driven)
 * @port:	pointer to &struct parport
 *
 * Used by parport_ip32_fifo_write_pio_wait().
 */
static unsigned int parport_ip32_fwp_wait_interrupt(struct parport *port)
{
	static const unsigned int nfault_check_interval = 100; /* msecs */
	static unsigned int lost_interrupt = 0;
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	DEFINE_TIMER(timer, parport_ip32_timeout, 0, (unsigned long)port);
	unsigned int ecr;
	unsigned long nfault_timeout =
		min(msecs_to_jiffies(nfault_check_interval),
		    (unsigned long)physport->cad->timeout);
	unsigned long expire = jiffies + physport->cad->timeout;
	unsigned int count = 0;

	while (1) {
		if (parport_ip32_fwp_wait_break(port, expire))
			break;

		/* Initialize mutex used to take interrupts into account */
		init_MUTEX_LOCKED(&priv->irq);

		/* Enable serviceIntr */
		parport_ip32_frob_econtrol(port, ECR_SERVINTR, 0);

		/* Enabling serviceIntr while the FIFO is empty does not
		 * always generate an interrupt, so check for emptiness
		 * now. */
		ecr = parport_ip32_read_econtrol(port);
		if (!(ecr & ECR_F_EMPTY)) {
			/* FIFO is not empty: wait for an interrupt or a
			 * timeout to occur */
			mod_timer(&timer, jiffies + nfault_timeout);
			down_interruptible(&priv->irq);
			del_timer(&timer);
			ecr = parport_ip32_read_econtrol(port);
			if ((ecr & ECR_F_EMPTY) && !(ecr & ECR_SERVINTR)
			    && !lost_interrupt) {
				printk(KERN_WARNING PPIP32
				       "%s: lost interrupt\n", port->name);
				lost_interrupt = 1;
			}
		}

		/* Disable serviceIntr */
		parport_ip32_frob_econtrol(port, ECR_SERVINTR, ECR_SERVINTR);

		/* Check FIFO state */
		if (ecr & ECR_F_EMPTY) {
			/* FIFO is empty, fill it up */
			count = priv->fifo_depth;
			break;
		} else if (ecr & ECR_SERVINTR) {
			/* FIFO is not empty, but we know that can safely push
			 * writeIntrThreshold bytes into it*/
			count = priv->writeIntrThreshold;
			break;
		}
		/* FIFO is not empty, and we did not get any interrupt.
		 * Either it's time to check for nFault, or a signal is
		 * pending.  This is verified in parport_ip32_fwp_wait_break,
		 * so we continue the loop. */
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
static inline unsigned int
parport_ip32_fifo_write_pio_wait(struct parport *port)
{
	unsigned int r;
	if (port->irq == PARPORT_IRQ_NONE) {
		r = parport_ip32_fwp_wait_polling(port);
	} else {
		r = parport_ip32_fwp_wait_interrupt(port);
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
static size_t parport_ip32_fifo_write_block_pio(struct parport *port,
						const void *buf, size_t len,
						unsigned int mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	const u8 *bufp = buf;
	size_t left = len;

	parport_ip32_dump_state(port, "begin fifo_write_block_pio", 0);

	while (left > 0) {
		unsigned int count;

		count = parport_ip32_fifo_write_pio_wait(port);
		if (count == 0) {
			/* Transmission should be stopped */
			break;
		}
		if (count > left) {
			count = left;
		}

		pr_debug(PPIP32 "%s: .. push %lu byte%s\n", port->name,
			 (unsigned long)count, (count > 1)? "s": "");

		/* Write next byte(s) to FIFO */
		if (count == 1) {
			parport_ip32_out(*bufp, priv->regs.fifo);
			bufp++, left--;
		} else {
			parport_ip32_out_rep(priv->regs.fifo, bufp, count);
			bufp += count, left -= count;
		}
	}

	pr_debug(PPIP32 "%s: .. transfer %s (left=%lu)\n", port->name,
		 left? "aborted": "completed", (unsigned long)left);

	parport_ip32_dump_state(port, "end fifo_write_block_pio", 0);

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
 * returns 1 if the peripheral is ready, and 0 otherwise.
 */
static unsigned int parport_ip32_fifo_write_initialize(struct parport *port,
						       unsigned int mode)
{
	unsigned int ready;

	pr_trace(port, "(0x%02x)", mode);

	/* Reset FIFO, go in forward mode, and disable ackIntEn */
	parport_ip32_set_mode(port, ECR_MODE_PS2);
	parport_ip32_write_control(port, DCR_SELECT | DCR_nINIT);
	parport_ip32_data_forward(port);
	parport_ip32_disable_irq(port);

	/* Go in desired mode */
	parport_ip32_set_mode(port, mode);

	/* Wait for peripheral to become ready */
	ready = !parport_wait_peripheral(port,
					 DSR_nBUSY | DSR_nFAULT,
					 DSR_nBUSY | DSR_nFAULT);

	return ready;
}

/**
 * parport_ip32_drain_fifo - wait for FIFO to empty
 * @port:	pointer to &struct parport
 * @timeout:	timeout, in jiffies
 *
 * This function waits for FIFO to empty.  It returns 1 when FIFO is empty, or
 * 0 if the timeout @timeout is reached before, or if a signal is pending.
 */
static unsigned int parport_ip32_drain_fifo(struct parport *port,
					    unsigned long timeout)
{
	unsigned long expire = jiffies + timeout;
	unsigned int polling_interval;
	unsigned int counter;

	pr_trace(port, "(%ums)", jiffies_to_msecs(timeout));

	/* Busy wait for approx. 200us */
	for (counter = 0; counter < 40; counter++) {
		if (parport_ip32_read_econtrol(port) & ECR_F_EMPTY)
			break;
		if (time_after(jiffies, expire))
			break;
		if (signal_pending(current))
			break;
		udelay(5);
	}
	/* Poll slowly.  Polling interval starts with 1 millisecond, and is
	 * increased exponentially until 128.  */
	polling_interval = 1; /* msecs */
	while (!(parport_ip32_read_econtrol(port) & ECR_F_EMPTY)) {
		if (time_after_eq(jiffies, expire))
			break;
		msleep_interruptible(polling_interval);
		if (signal_pending(current))
			break;
		if (polling_interval < 128) {
			polling_interval *= 2;
		}
	}

	return !!(parport_ip32_read_econtrol(port) & ECR_F_EMPTY);
}

/**
 * parport_ip32_get_fifo_residue - reset FIFO
 * @port:	pointer to &struct parport
 * @mode:	operation mode (ECR_MODE_PPF or ECR_MODE_ECP)
 *
 * This function resets FIFO, and returns the number of bytes remaining in it.
 */
static unsigned int parport_ip32_get_fifo_residue(struct parport *port)
{
	struct parport_ip32_private * const priv = PRIV(port);
	unsigned int residue;
	unsigned int cnfga;

	/* FIXME - We are missing one byte if the printer is off-line.  I
	 * don't know how to detect this.  It looks that the full bit is not
	 * always reliable.  For the moment, the problem is avoided in most
	 * cases by testing for BUSY in parport_ip32_fifo_write_initialize().
	 */

	pr_trace(port, "()");

	if (parport_ip32_read_econtrol(port) & ECR_F_EMPTY) {
		residue = 0;
	} else {
		printk(KERN_DEBUG PPIP32 "%s: FIFO is stuck\n", port->name);

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
		parport_ip32_frob_control(port, DCR_STROBE, 0);

		/* Fill up FIFO */
		for (residue = priv->fifo_depth; residue > 0; residue--) {
			if (parport_ip32_read_econtrol(port) & ECR_F_FULL)
				break;
			parport_ip32_out(0x00, priv->regs.fifo);
		}
	}

	if (residue) {
		pr_debug1(PPIP32 "%s: %d PWord%s left in FIFO\n",
			  port->name, residue,
			  (residue == 1)? " was": "s were");
	}

	/* Now reset the FIFO, change to Config mode, and clean up */
	parport_ip32_set_mode(port, ECR_MODE_CFG);
	cnfga = parport_ip32_in(priv->regs.cnfgA);

	if (!(cnfga & CNFGA_nBYTEINTRANS)) {
		pr_debug1(PPIP32 "%s: cnfgA contains 0x%02x\n",
			  port->name, cnfga);
		pr_debug1(PPIP32 "%s: Accounting for extra byte\n",
			  port->name);
		residue++;
	}

	/* Don't care about partial PWords until support is added for
	 * PWord != 1 byte. */

	/* Back to PS2 mode. */
	parport_ip32_set_mode(port, ECR_MODE_PS2);

	return residue;
}

/**
 * parport_ip32_fifo_write_finalize - finalize a forward FIFO transfer
 * @port:	pointer to &struct parport
 * @mode:	operation mode (ECR_MODE_PPF or ECR_MODE_ECP)
 *
 * Finalize a forward FIFO transfer.  Returns 1 if FIFO is empty and the
 * status of BUSY is low.  Returns -residue otherwise, where residue is the
 * number of bytes remaining in the FIFO.  Note that zero can be returned in
 * case there is a BUSY timeout.
 */
static int parport_ip32_fifo_write_finalize(struct parport *port, int mode)
{
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	unsigned int ready;
	unsigned int residue;

	pr_trace(port, "(0x%02x)", mode);

	/* Wait FIFO to empty.  Timeout is proportional to FIFO_depth.  */
	parport_ip32_drain_fifo(port,
				physport->cad->timeout * priv->fifo_depth);

	/* Check for a potential residue */
	residue = parport_ip32_get_fifo_residue(port);

	/* Then, wait for BUSY to get low. */
	ready = !parport_wait_peripheral(port, DSR_nBUSY, DSR_nBUSY);
	if (!ready) {
		printk(KERN_DEBUG PPIP32 "%s: BUSY timeout\n", port->name);
	}

	/* Reset FIFO */
	parport_ip32_set_mode(port, ECR_MODE_PS2);

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
static size_t parport_ip32_fifo_write_block(struct parport *port,
					    const void *buf, size_t len,
					    unsigned int mode)
{
	static unsigned int ready_before = 1;
	struct parport_ip32_private * const priv = PRIV(port);
	struct parport * const physport = port->physport;
	size_t written = 0;
	int r;

	if (len == 0) {
		/* There is nothing to do */
		goto out;
	}
	if (!parport_ip32_fifo_write_initialize(port, mode)) {
		/* Avoid to flood the logs */
		if (ready_before) {
			printk(KERN_INFO PPIP32 "%s: not ready\n",
			       port->name);
		}
		ready_before = 0;
		goto out;
	}
	ready_before = 1;

	pr_trace(port, " -> len=%lu", (unsigned long)len);

	physport->ieee1284.phase = IEEE1284_PH_FWD_DATA;
	priv->irq_mode = PARPORT_IP32_IRQ_HERE;

	/* FIXME - Use parport_ip32_fifo_write_block_dma when available.
	 * Maybe some threshold value should be set for @len under which we
	 * revert to PIO mode?
	 */
	written = parport_ip32_fifo_write_block_pio(port, buf, len, mode);

	/* Finalize the transfer */
	r = parport_ip32_fifo_write_finalize(port, mode);
	if (r < 0) {
		written += r;
	}

	priv->irq_mode = PARPORT_IP32_IRQ_FWD;
	physport->ieee1284.phase = IEEE1284_PH_FWD_IDLE;

	pr_trace(port, " <- written=%lu", (unsigned long)written);

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
static size_t parport_ip32_compat_write_data(struct parport *port,
					     const void *buf, size_t len,
					     int flags)
{
	struct parport * const physport = port->physport;
	size_t written;

	pr_trace(port, "(...)");

	/* Special case: a timeout of zero means we cannot call schedule().
	 * Also if O_NONBLOCK is set then use the default implementation. */
	if (physport->cad->timeout <= PARPORT_INACTIVITY_O_NONBLOCK) {
		written = parport_ieee1284_write_compat(port, buf, len,
							flags);
	} else {
		written = parport_ip32_fifo_write_block(port, buf, len,
							ECR_MODE_PPF);
	}
	return written;
}

/*
 * FIXME - Insert here parport_ip32_ecp_{read,write}_{data,address}().
 */

/*--- Default operations -----------------------------------------------*/

static __initdata struct parport_operations parport_ip32_ops = {
	.write_data		= parport_ip32_write_data,
	.read_data		= parport_ip32_read_data,

	.write_control		= parport_ip32_write_control,
	.read_control		= parport_ip32_read_control,
	.frob_control		= parport_ip32_frob_control,

	.read_status		= parport_ip32_read_status,

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
 * parport_ip32_ecp_supported - check for an ECP port
 * @regs: pointer to a  &parport_ip32_regs structure
 *
 * Returns 1 if an ECP port is found, and 0 otherwise.  This function actually
 * checks if an Extended Control Register seems to be present.  On successful
 * return, the port is placed in SPP mode.
 *
 * We first check to see if ECR is the same as DCR.  If not, the low two bits
 * of ECR aren't writable, so we check by writing ECR and reading it back to
 * see if it's what we expect.
 */
static __init unsigned int
parport_ip32_ecp_supported(const struct parport_ip32_regs *regs)
{
	unsigned int dcr, ecr, mask;

	parport_ip32_out(DCR_SELECT | DCR_nINIT, regs->dcr);
	dcr = parport_ip32_in(regs->dcr);
	mask = ECR_F_FULL | ECR_F_EMPTY;
	if ((parport_ip32_in(regs->ecr) & mask) == (dcr & mask)) {
		/* Toggle bit ECR_F_FULL */
		mask = ECR_F_FULL;
		parport_ip32_out(dcr ^ mask, regs->dcr);
		dcr = parport_ip32_in(regs->dcr);
		if ((parport_ip32_in(regs->ecr) & mask) == (dcr & mask))
			return 0; /* Sure that no ECR register exists */
	}

	ecr = parport_ip32_in(regs->ecr) & (ECR_F_FULL | ECR_F_EMPTY);
	if (ecr != ECR_F_EMPTY)
		return 0;

	ecr = ECR_MODE_PS2 | ECR_nERRINTR | ECR_SERVINTR;
	parport_ip32_out(ecr, regs->ecr);
	if (parport_ip32_in(regs->ecr) != (ecr | ECR_F_EMPTY))
		return 0;

	ecr = ECR_MODE_SPP | ECR_nERRINTR | ECR_SERVINTR;
	parport_ip32_out(ecr, regs->ecr);
	parport_ip32_out(DCR_SELECT | DCR_nINIT, regs->dcr);
	return 1;
}

/**
 * parport_ip32_fifo_supported - check for FIFO parameters
 * @p:	pointer to the &parport structure
 *
 * Check for FIFO parameters of an Extended Capabilities Port.  Returns 1 on
 * success, and 0 otherwise.  Adjust FIFO parameters in the parport structure.
 * On return, the port is placed in SPP mode.
 */
static __init unsigned int parport_ip32_fifo_supported(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	unsigned int configa, configb;
	unsigned int pword;
	unsigned int i;

	/* Configuration mode */
	parport_ip32_set_mode(p, ECR_MODE_CFG);
	configa = parport_ip32_in(priv->regs.cnfgA);
	configb = parport_ip32_in(priv->regs.cnfgB);

	/* Find out PWord size */
	switch (configa & CNFGA_ID_MASK) {
	case CNFGA_ID_8:	pword = 1; break;
	case CNFGA_ID_16:	pword = 2; break;
	case CNFGA_ID_32:	pword = 4; break;
	default:
		pr_probe(p, "Unknown implementation ID: 0x%0x\n",
			 (configa & CNFGA_ID_MASK) >> CNFGA_ID_SHIFT);
		goto fail;
		break;
	}
	if (pword != 1) {
		pr_probe(p, "Unsupported PWord size: %u\n", pword);
		goto fail;
	}
	priv->pword = pword;
	pr_probe(p, "PWord is %u bits\n", 8 * priv->pword);

	/* Check for compression support */
	parport_ip32_out(configb | CNFGB_COMPRESS, priv->regs.cnfgB);
	if (parport_ip32_in(priv->regs.cnfgB) & CNFGB_COMPRESS) {
		pr_probe(p, "Hardware compression detected (unsupported)\n");
	}
	parport_ip32_out(configb & ~CNFGB_COMPRESS, priv->regs.cnfgB);

	/* Reset FIFO and go in test mode (no interrupt, no DMA) */
	parport_ip32_set_mode(p, ECR_MODE_TST);

	/* FIFO must be empty now */
	if (!(parport_ip32_in(priv->regs.ecr) & ECR_F_EMPTY)) {
		pr_probe(p, "FIFO not reset\n");
		goto fail;
	}

	/* Find out FIFO depth. */
	priv->fifo_depth = 0;
	for (i = 0; i < 1024; i++) {
		if (parport_ip32_in(priv->regs.ecr) & ECR_F_FULL) {
			/* FIFO full */
			priv->fifo_depth = i;
			break;
		}
		parport_ip32_out((u8)i, priv->regs.fifo);
	}
	if (i >= 1024) {
		pr_probe(p, "Can't fill FIFO\n");
		goto fail;
	}
	if (!priv->fifo_depth) {
		pr_probe(p, "Can't get FIFO depth\n");
		goto fail;
	}
	pr_probe(p, "FIFO is %u PWords deep\n", priv->fifo_depth);

	/* Enable interrupts */
	parport_ip32_frob_econtrol(p, ECR_SERVINTR, 0);

	/* Find out writeIntrThreshold: number of PWords we know we can write
	 * if we get an interrupt. */
	priv->writeIntrThreshold = 0;
	for (i = 0; i < priv->fifo_depth; i++) {
		if (parport_ip32_in(priv->regs.fifo) != (u8)i) {
			pr_probe(p, "Invalid data in FIFO\n");
			goto fail;
		}
		if (!priv->writeIntrThreshold
		    && parport_ip32_in(priv->regs.ecr) & ECR_SERVINTR) {
			/* writeIntrThreshold reached */
			priv->writeIntrThreshold = i + 1;
		}
		if (i + 1 < priv->fifo_depth
		    && parport_ip32_in(priv->regs.ecr) & ECR_F_EMPTY) {
			/* FIFO empty before the last byte? */
			pr_probe(p, "Data lost in FIFO\n");
			goto fail;
		}
	}
	if (!priv->writeIntrThreshold) {
		pr_probe(p, "Can't get writeIntrThreshold\n");
		goto fail;
	}
	pr_probe(p, "writeIntrThreshold is %u\n", priv->writeIntrThreshold);

	/* FIFO must be empty now */
	if (!(parport_ip32_in(priv->regs.ecr) & ECR_F_EMPTY)) {
		pr_probe(p, "Can't empty FIFO\n");
		goto fail;
	}

	/* Reset FIFO */
	parport_ip32_set_mode(p, ECR_MODE_PS2);
	/* Set reverse direction (must be in PS2 mode) */
	parport_ip32_data_reverse(p);
	/* Test FIFO, no interrupt, no DMA */
	parport_ip32_set_mode(p, ECR_MODE_TST);
	/* Enable interrupts */
	parport_ip32_frob_econtrol(p, ECR_SERVINTR, 0);

	/* Find out readIntrThreshold: number of PWords we can read if we get
	 * an interrupt. */
	priv->readIntrThreshold = 0;
	for (i = 0; i < priv->fifo_depth; i++) {
		parport_ip32_out(0xaa, priv->regs.fifo);
		if (!priv->readIntrThreshold
		    && parport_ip32_in(priv->regs.ecr) & ECR_SERVINTR) {
			/* readIntrThreshold reached */
			priv->readIntrThreshold = i + 1;
		}
	}
	if (!priv->readIntrThreshold) {
		pr_probe(p, "Can't get readIntrThreshold\n");
		goto fail;
	}
	pr_probe(p, "readIntrThreshold is %u\n", priv->readIntrThreshold);

	/* Reset ECR */
	parport_ip32_set_mode(p, ECR_MODE_PS2);
	parport_ip32_data_forward(p);
	parport_ip32_set_mode(p, ECR_MODE_SPP);
	return 1;

fail:
	priv->fifo_depth = 0;
	parport_ip32_set_mode(p, ECR_MODE_SPP);
	return 0;
}

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
static void __init
parport_ip32_make_isa_registers(struct parport_ip32_regs *regs,
				void __iomem *base, void __iomem *base_hi,
				unsigned int regshift)
{
#define r_base(offset)    ((u8 __iomem *)base    + ((offset) << regshift))
#define r_base_hi(offset) ((u8 __iomem *)base_hi + ((offset) << regshift))
	*regs = (struct parport_ip32_regs){
		.data		= r_base(0),
		.dsr		= r_base(1),
		.dcr		= r_base(2),
		.eppAddr	= r_base(3),
		.eppData0	= r_base(4),
		.eppData1	= r_base(5),
		.eppData2	= r_base(6),
		.eppData3	= r_base(7),
		.ecpAFifo	= r_base(0),
		.fifo		= r_base_hi(0),
		.cnfgA		= r_base_hi(0),
		.cnfgB		= r_base_hi(1),
		.ecr		= r_base_hi(2)
	};
#undef r_base_hi
#undef r_base
}

/**
 * parport_ip32_probe_port - probe and register IP32 built-in parallel port
 *
 * Returns the new allocated &parport structure.  On error, an error code is
 * encoded in return value with the ERR_PTR function.
 */
static __init struct parport *parport_ip32_probe_port(void)
{
	struct parport_ip32_regs regs;
	struct parport_ip32_private *priv = NULL;
	struct parport_operations *ops = NULL;
	struct parport *p = NULL;
	unsigned int fifo_supported;
	int err;

	parport_ip32_make_isa_registers(&regs, &mace->isa.parallel,
					&mace->isa.ecp1284, 8 /* regshift */);

	if (!parport_ip32_ecp_supported(&regs)) {
		err = -ENODEV;
		goto fail;
	}

	ops = kmalloc(sizeof(struct parport_operations), GFP_KERNEL);
	priv = kmalloc(sizeof(struct parport_ip32_private), GFP_KERNEL);
	p = parport_register_port(0, PARPORT_IRQ_NONE, PARPORT_DMA_NONE, ops);
	if (ops == NULL || priv == NULL || p == NULL) {
		err = -ENOMEM;
		goto fail;
	}
	p->base = MACE_BASE + offsetof(struct sgi_mace, isa.parallel);
	p->base_hi = MACE_BASE + offsetof(struct sgi_mace, isa.ecp1284);
	p->private_data = priv;

	*ops = parport_ip32_ops;
	*priv = (struct parport_ip32_private){
		.regs			= regs,
		.dcr_writable		= DCR_DIR | DCR_SELECT | DCR_nINIT |
					  DCR_AUTOFD | DCR_STROBE,
		.irq_mode		= PARPORT_IP32_IRQ_FWD,
	};

	/* We found what looks like a working ECR register.  Simply assume
	 * that all modes are correctly supported.  Enable basic modes.*/
	pr_probe(p, "Found working ECR register\n");
	p->modes = PARPORT_MODE_PCSPP | PARPORT_MODE_SAFEININT;
	p->modes |= PARPORT_MODE_TRISTATE;

	fifo_supported = parport_ip32_fifo_supported(p);

	/* Request IRQ */
	if (features & PARPORT_IP32_ENABLE_IRQ) {
		int irq = MACEISA_PARALLEL_IRQ;
		if (request_irq(irq, parport_ip32_interrupt, 0, p->name, p)) {
			printk(KERN_WARNING PPIP32 "%s: irq %d in use, "
			       "resorting to polled operation\n",
			       p->name, irq);
		} else {
			p->irq = irq;
			priv->dcr_writable |= DCR_IRQ;
		}
	}

	/* DMA cannot work without interrupts.  Furthermore, it is not needed
	 * if FIFO is not supported.  */
	if ((features & PARPORT_IP32_ENABLE_DMA)
	    && p->irq != PARPORT_IRQ_NONE && fifo_supported) {
		/* FIXME - Allocate DMA resources here. */
	}

	if ((features & PARPORT_IP32_ENABLE_SPP) && fifo_supported) {
		/* Enable compatibility FIFO mode */
		p->ops->compat_write_data = parport_ip32_compat_write_data;
		p->modes |= PARPORT_MODE_COMPAT;
		pr_probe(p,"Hardware support for SPP mode enabled\n");
	}
#if 0		/* FIXME - parport_ip32_epp_* not implemented */
	if (features & PARPORT_IP32_ENABLE_EPP) {
		/* Set up access functions to use EPP hardware. */
		p->ops->epp_read_data = parport_ip32_epp_read_data;
		p->ops->epp_write_data = parport_ip32_epp_write_data;
		p->ops->epp_read_addr = parport_ip32_epp_read_addr;
		p->ops->epp_write_addr = parport_ip32_epp_write_addr;
		p->modes |= PARPORT_MODE_EPP;
		pr_probe(p, "Hardware support for EPP mode enabled\n");
	}
#endif
#if 0		/* FIXME - parport_ip32_ecp_* not implemented */
	if ((features & PARPORT_IP32_ENABLE_ECP) && fifo_supported) {
		/* Enable ECP FIFO mode */
		p->ops->ecp_write_data = parport_ip32_ecp_write_data;
		p->ops->ecp_read_data  = parport_ip32_ecp_read_data;
		p->ops->ecp_write_addr = parport_ip32_ecp_write_addr;
		p->modes |= PARPORT_MODE_ECP;
		pr_probe(p, "Hardware support for ECP mode enabled\n");
	}
#endif

	/* Initialize the port with sensible values */
	parport_ip32_set_mode(p, ECR_MODE_PS2);
	parport_ip32_write_control(p, DCR_SELECT | DCR_nINIT);
	parport_ip32_data_forward(p);
	parport_ip32_disable_irq(p);
	parport_ip32_write_data(p, 0x00);

	parport_ip32_dump_state (p, "end init", 0);

	/* Print what we found */
	printk(KERN_INFO "%s: SGI IP32 at 0x%lx (0x%lx)",
	       p->name, p->base, p->base_hi);
	if (p->irq != PARPORT_IRQ_NONE) {
		printk(", irq %d", p->irq);
	}
	if (p->dma != PARPORT_DMA_NONE) {
		printk(", dma %d", p->dma);
	}
	printk(" [");
#define printmode(x)	if (p->modes & PARPORT_MODE_##x)		\
				printk("%s%s", f++? ",": "", #x)
	{
		unsigned int f = 0;
		printmode(PCSPP);
		printmode(TRISTATE);
		printmode(COMPAT);
		printmode(EPP);
		printmode(ECP);
		printmode(DMA);
	}
#undef printmode
	printk("]\n");

	parport_announce_port(p);
	return p;

fail:
	if (p) {
		parport_put_port(p);
	}
	kfree(priv);
	kfree(ops);
	return ERR_PTR(err);
}

/**
 * parport_ip32_unregister_port - unregister a parallel port
 * @p:	pointer to the &struct parport
 *
 * Unregisters a parallel port and free previously allocated resources
 * (memory, IRQ, ...).
 */
static __exit void parport_ip32_unregister_port(struct parport *p)
{
	struct parport_ip32_private * const priv = PRIV(p);
	struct parport_operations *ops = p->ops;

	parport_remove_port(p);

	if (p->irq != PARPORT_IRQ_NONE)
		free_irq(p->irq, p);

	/* FIXME - Release DMA resources here. */

	parport_put_port(p);
	kfree(priv);
	kfree(ops);
}

/*--- Initialization code ----------------------------------------------*/

/**
 * parport_ip32_init - module initialization function
 */
static int __init parport_ip32_init(void)
{
	if (verbose_probing) {
		printk(KERN_INFO PPIP32
		       "SGI IP32 built-in parallel port driver v0.3pre\n");
		pr_debug1(PPIP32 "Compiled on %s, %s\n", __DATE__, __TIME__);
	}
	this_port = parport_ip32_probe_port();
	if (IS_ERR(this_port)) {
		return PTR_ERR(this_port);
	}
	return 0;
}

/**
 * parport_ip32_exit - module termination function
 */
static void __exit parport_ip32_exit(void)
{
	parport_ip32_unregister_port(this_port);
}

/*--- Module stuff -----------------------------------------------------*/

MODULE_AUTHOR("Arnaud Giersch <arnaud.giersch@free.fr>");
MODULE_DESCRIPTION("SGI IP32 built-in parallel port driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3pre");	/* update in parport_ip32_init() too */

module_init(parport_ip32_init);
module_exit(parport_ip32_exit);

module_param(verbose_probing, bool, S_IRUGO);
MODULE_PARM_DESC(verbose_probing, "Log chit-chat during initialization");

module_param(features, uint, S_IRUGO);
MODULE_PARM_DESC(features,
		 "Bit mask of features to enable"
		 ", bit 0: IRQ support"
		 ", bit 1: DMA support"
		 ", bit 2: hardware SPP mode"
		 ", bit 3: hardware EPP mode"
		 ", bit 4: hardware ECP mode");


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
