/* Low-level parallel port routines for builtin port on SGI IP32
 *
 * Author: Arnaud Giersch <arnaud.giersch@free.fr>
 *
 * $Id: parport_ip32.c,v 1.4 2005-10-03 23:09:11 arnaud Exp $
 *
 * partially based on parport_pc.c by
 *	    Phil Blundell <philb@gnu.org>
 *          Tim Waugh <tim@cyberelk.demon.co.uk>
 *          Jose Renau <renau@acm.org>
 *          David Campbell <campbell@torque.net>
 *          Andrea Arcangeli
 *
 * Copyright (C) 2005 Arnaud Giersch.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 *
 * History:
 *
 * v0.3
 *	Added Compatibility FIFO mode (PIO).
 *      Added code for EPP support (not tested).
 *      Disable interrupts: it is too slow to get on interrupt per
 *      char written!
 *
 * v0.2 -- Sun, 02 Oct 2005 19:52:04 +0200
 *	Interrupts are working in SPP mode.
 *
 * v0.1 -- Sun, 02 Oct 2005 16:18:54 +0200
 *	First working version. Only SPP/PS2 modes are supported,
 *	without interrupts.
 */

/* This driver is the result of my attempts to use the builtin
 * parallel port on the SGI O2 worksation (SGI IP32).  The integrated
 * parallel port is apparently compatible with that in the IBM PC, and
 * supports ECP and EPP modes.  The main differences are the I/O
 * addresses of the various registers.  These adresses lie in the MACE
 * adress space, and are obtained via the sgi_mace structure.  They
 * are accessed with ioremap/readb/writeb.  As for serial ports on the
 * same machine, the registers are replicated 256 times (8bit
 * regshift). This is why it is currently impossible to use the
 * parport_pc module.
 *
 * As I did not found more documentation, this driver is mainly based
 * on empirical tests.
 */

/*--- Some configuration defines ----------------------------------*/

/* DEBUG_PARPORT_IP32
 *	0	disable debug
 *	1	standard level
 *	>1	verbose level
 */
#define DEBUG_PARPORT_IP32	1

/* Define to enable IRQ */
#undef PARPORT_IP32_USE_IRQ

/* Define to enable EPP support */
#define PARPORT_IP32_EPP	/* Not tested */

/* Define to enable FIFO modes */
#define PARPORT_IP32_FIFO	/* Not tested */

/* Define to enable DMA with FIFO modes */
#undef PARPORT_IP32_USE_DMA	/* Unused */

/* Self explanatory */
#define VERBOSE_PROBING 1

/*-----------------------------------------------------------------*/

/* Setup DEBUG macros */
#undef DEBUG
#ifdef DEBUG_PARPORT_IP32
#  if DEBUG_PARPORT_IP32 == 0
#    undef DEBUG_PARPORT_IP32
#  elif DEBUG_PARPORT_IP32 == 1
#    warning DEBUG_PARPORT_IP32 == 1
     /* nop */
#  else /* DEBUG_PARPORT_IP32 > 1 */
#    warning DEBUG_PARPORT_IP32 > 1
#    define DEBUG			/* enable pr_debug() in kernel.h */
#  endif /* DEBUG_PARPORT_IP32 > 1 */
#endif /* ! DEBUG_PARPORT_IP32 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/parport.h>
#include <asm/io.h>
#include <asm/ip32/mace.h>
#include <asm/ip32/ip32_ints.h>

#define PPIP32 "parport_ip32: "

MODULE_AUTHOR ("Arnaud Giersch <arnaud.giersch@free.fr>");
MODULE_DESCRIPTION ("SGI IP32 builtin parallel port driver");
MODULE_SUPPORTED_DEVICE ("SGI IP32 MACE parallel port");
MODULE_LICENSE ("GPL");

#ifdef MODULE
/* mace is not exported in arch/mips/sgi-ip32/crime.c,
 * use our own pointer for modules */
#define DEFINE_MACE
#endif

#ifdef DEFINE_MACE
static struct sgi_mace *my_mace;
#  define INIT_MACE_POINTER() \
	my_mace = ioremap (MACE_BASE, sizeof (struct sgi_mace))
#  define RELEASE_MACE_POINTER() \
	iounmap (my_mace)
#  define mace my_mace
#else
#  define INIT_MACE_POINTER() do { } while (0)
#  define RELEASE_MACE_POINTER() do { } while (0)
#endif /* DEFINE_MACE */

/* Physical I/O addresses */
#define PARPORT_IP32_IO	  (MACE_BASE + \
			   offsetof (struct sgi_mace, isa) + \
			   offsetof (struct mace_isa, parallel))
#define PARPORT_IP32_IOHI (MACE_BASE + \
			   offsetof (struct sgi_mace, isa) + \
			   offsetof (struct mace_isa, ecp1284))
#define PARPORT_IP32_IRQ  MACEISA_PARALLEL_IRQ

/* Virtual I/O addresses */
#define PARPORT_IP32_IO_V   ((unsigned char *)&mace->isa.parallel)
#define PARPORT_IP32_IOHI_V ((unsigned char *)&mace->isa.ecp1284)

/* Registers are replicated 256 times */
#define REGSHIFT 8

/* Register definitions, names picked from parport_pc.h. */
#define ECONTROL (PARPORT_IP32_IOHI_V + (0x2 << REGSHIFT))
#define CONFIGB  (PARPORT_IP32_IOHI_V + (0x1 << REGSHIFT))
#define CONFIGA  (PARPORT_IP32_IOHI_V + (0x0 << REGSHIFT))
#define FIFO     (PARPORT_IP32_IOHI_V + (0x0 << REGSHIFT))
#define EPPDATA  (PARPORT_IP32_IO_V   + (0x4 << REGSHIFT))
#define EPPADDR  (PARPORT_IP32_IO_V   + (0x3 << REGSHIFT))
#define CONTROL  (PARPORT_IP32_IO_V   + (0x2 << REGSHIFT))
#define STATUS   (PARPORT_IP32_IO_V   + (0x1 << REGSHIFT))
#define DATA     (PARPORT_IP32_IO_V   + (0x0 << REGSHIFT))

/* ECR modes */
#define ECR_MODE_SHIFT 5
#define ECR_MODE_SPP (00 << ECR_MODE_SHIFT)
#define ECR_MODE_PS2 (01 << ECR_MODE_SHIFT)
#define ECR_MODE_PPF (02 << ECR_MODE_SHIFT)
#define ECR_MODE_ECP (03 << ECR_MODE_SHIFT)
#define ECR_MODE_EPP (04 << ECR_MODE_SHIFT)
#define ECR_MODE_VND (05 << ECR_MODE_SHIFT)
#define ECR_MODE_TST (06 << ECR_MODE_SHIFT)
#define ECR_MODE_CNF (07 << ECR_MODE_SHIFT)
#define ECR_MODE_MASK 0xe0
#define ECR_IRQ       (1 << 4)
#define ECR_DMA       (1 << 3)
#define ECR_INTR      (1 << 2)
#define ECR_F_FULL    (1 << 1)
#define ECR_F_EMPTY   (1 << 0)

/* Initial values for CTR and ECR */
#define INIT_CTR 0x0c		/* PARPORT_CONTROL_INIT |
				   PARPORT_CONTROL_SELECT */
#define INIT_ECR 0x34		/* ECR_MODE_PS2 |
				   ECR_IRQ |
				   ECR_INTR */

/* We do not support more than one port. */
static struct {
	/* Contents of CTR. */
	unsigned char ctr;

	/* Bitmask of writable CTR bits. */
	unsigned char ctr_writable;

#ifdef PARPORT_IP32_FIFO
	/* Number of PWords that FIFO will hold. */
	int fifo_depth;

	/* Number of bytes per PWord. */
	int pword;

	/* Not used yet. */
	int readIntrThreshold;
	int writeIntrThreshold;
#endif /* Enable FIFO support */

	struct parport *port;
} this;

/*--- I/O port access function ------------------------------------*/

#if defined (parport_in) || defined (parport_out)
#  error parport_in or parport_out already defined !
#endif

static inline unsigned int parport_in (void *addr)
{
	unsigned int val = readb (addr);
	rmb ();			/* is it needed? */
	return val;
}

static inline void parport_out (u8 val, void *addr)
{
	writeb (val, addr);
	wmb ();			/* is it needed? */
}

/*--- Debug code --------------------------------------------------*/

#ifdef DEBUG_PARPORT_IP32

#define pr_debug1(fmt,arg...) \
	printk (KERN_DEBUG fmt,##arg)

static inline void dump_parport_state (char *str, struct parport *p)
{
	/* here's hoping that reading these ports won't side-effect
	 * anything underneath */
	unsigned char ecr, dcr, dsr;
	static char *ecr_modes[] = 
		{"SPP", "PS2", "PPFIFO", "ECP", "EPP", "xXx", "TST", "CFG"};
	int i;

	ecr = parport_in (ECONTROL);
	printk (KERN_DEBUG PPIP32 "parport state (%s):\n", str);
	printk (KERN_DEBUG PPIP32 "    ecr=[%s", ecr_modes[(ecr & 0xe0) >> 5]);
	if (ecr & 0x10) printk (",nErrIntrEn");
	if (ecr & 0x08) printk (",dmaEn");
	if (ecr & 0x04) printk (",serviceIntr");
	if (ecr & 0x02) printk (",f_full");
	if (ecr & 0x01) printk (",f_empty");
	for (i=0; i<2; i++) {
		printk ("]\n");
		printk (KERN_DEBUG PPIP32 
			"    dcr(%s)=[", i ? "soft" : "hard");
		dcr = i? this.ctr: parport_in (CONTROL);
	
		printk ("0x%02x", dcr & 0xc0);
		if (dcr & 0x20) {
			printk (",rev");
		} else {
			printk (",fwd");
		}
		if (dcr & 0x10)	   printk (",ackIntEn");
		if (!(dcr & 0x08)) printk (",N-SELECT-IN");
		if (dcr & 0x04)	   printk (",N-INIT");
		if (!(dcr & 0x02)) printk (",N-AUTOFD");
		if (!(dcr & 0x01)) printk (",N-STROBE");
	}
	dsr = parport_in (STATUS );
	printk ("]\n");
	printk (KERN_DEBUG PPIP32 "    dsr=[");
	if (!(dsr & 0x80)) printk ("BUSY");
	if (dsr & 0x40)	   printk (",N-ACK");
	if (dsr & 0x20)	   printk (",PERROR");
	if (dsr & 0x10)	   printk (",SELECT");
	if (dsr & 0x08)	   printk (",N-FAULT");
	if (dsr & 0x07)    printk (",0x%02x", dsr & 0x07);
	printk ("]\n");
	return;
}

#else /* !DEBUG_PARPORT_IP32 */

#define pr_debug1(fmt,arg...) \
	do { } while (0)

#define dump_parport_state(...) \
	do { } while (0)

#endif /* !DEBUG_PARPORT_IP32 */

/*--- Basic functions needed for parport --------------------------*/

static inline unsigned char parport_ip32_read_data (struct parport *p)
{
	unsigned char d = parport_in (DATA);
	pr_debug ("parport_ip32_read_data(%p): 0x%02x\n", p, d);
	return d;
}

static inline void parport_ip32_write_data (struct parport *p, 
					    unsigned char d)
{
	pr_debug ("parport_ip32_write_data(%p, 0x%02x)\n", p, d);
	parport_out (d, DATA);
}

static inline unsigned char parport_ip32_read_status (struct parport *p)
{
	unsigned char s = parport_in (STATUS);
	pr_debug ("parport_ip32_read_status(%p): 0x%02x\n", p, s);
	return s;
}

/* __parport_ip32_read_control differs from parport_ip32_read_control 
 * in that it doesn't do any extra masking. */
static inline unsigned char __parport_ip32_read_control (struct parport *p)
{
	unsigned char c = this.ctr; /* use soft copy */
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

/* __parport_ip32_write_control differs from parport_ip32_write_control 
 * in that it doesn't do any extra masking. */
static inline void __parport_ip32_write_control (struct parport *p, 
						 unsigned char c)
{
	pr_debug ("__parport_ip32_write_control(%p, 0x%02x)\n", p, c);
#ifdef DEBUG_PARPORT_IP32
	if (c & ~this.ctr_writable) {
		pr_debug1 (PPIP32 
			   "extra bits in __write_control: 0x%02x/0x%02x\n", 
			   c, this.ctr_writable);
	}
#endif /* DEBUG_PARPORT_IP32 */
	c &= this.ctr_writable;	/* only writable bits */
	parport_out (c, CONTROL);
	this.ctr = c;		/* update soft copy */
}

/* __parport_ip32_frob_control differs from parport_ip32_frob_control
 * in that it doesn't do any extra masking. */
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
	c &= wm;
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
	/* Restrict mask and val to control lines */
	mask &= wm;
	val &= wm;
	__parport_ip32_frob_control (p, mask, val);
	return parport_ip32_read_control (p);
}

static inline void parport_ip32_disable_irq (struct parport *p)
{
	pr_debug ("parport_ip32_disable_irq(%p)\n", p);
	__parport_ip32_frob_control (p, 0x10, 0x00);
}

static inline void parport_ip32_enable_irq (struct parport *p)
{
	pr_debug ("parport_ip32_enable_irq(%p)\n", p);
	__parport_ip32_frob_control (p, 0x10, 0x10);
}

static inline void parport_ip32_data_forward (struct parport *p)
{
	pr_debug ("parport_ip32_data_forward(%p)\n", p);
	__parport_ip32_frob_control (p, 0x20, 0x00);
}

static inline void parport_ip32_data_reverse (struct parport *p)
{
	pr_debug ("parport_ip32_data_reverse(%p)\n", p);
	__parport_ip32_frob_control (p, 0x20, 0x20);
}

static inline void parport_ip32_init_state (struct pardevice *dev, 
					   struct parport_state *s)
{
	pr_debug ("parport_ip32_init_state(%p, %p)\n", dev, s);
	s->u.ip32.ctr = INIT_CTR;
	if (dev->irq_func && dev->port->irq != PARPORT_IRQ_NONE) {
		/* Set ackIntEn */
		s->u.ip32.ctr |= 0x10;
	}
	s->u.ip32.ecr = INIT_ECR;
}

static inline void parport_ip32_save_state (struct parport *p, 
					   struct parport_state *s)
{
	pr_debug ("parport_ip32_save_state(%p, %p)\n", p, s);
	s->u.ip32.ctr = __parport_ip32_read_control (p);
	s->u.ip32.ecr = parport_in (ECONTROL);
}

static inline void parport_ip32_restore_state (struct parport *p, 
					      struct parport_state *s)
{
	pr_debug ("parport_ip32_restore_state(%p, %p)\n", p, s);
	__parport_ip32_write_control (p, s->u.ip32.ctr);
	parport_out (s->u.ip32.ecr, ECONTROL);
}

/*--- Some utility definitions for ECR ----------------------------*/

/* frob_control, but for ECR */
static inline void frob_econtrol (unsigned char mask, unsigned char val)
{
	unsigned char ectr = 0;
 	pr_debug (PPIP32 "frob_econtrol(%02x,%02x)\n", mask, val);

	if (mask != 0xff) {
		ectr = parport_in (ECONTROL);
	}
	parport_out ((ectr & ~mask) ^ val, ECONTROL);
}

static inline void frob_set_mode (int mode)
{
	frob_econtrol (ECR_MODE_MASK, mode & ECR_MODE_MASK);
}

/*--- Simple interrupt handler ------------------------------------*/

static irqreturn_t parport_ip32_interrupt (int irq, void *dev_id, 
					   struct pt_regs *regs)
{
	pr_debug ("parport_ip32_interrupt(%d, %p, %p)\n", irq, dev_id, regs);
	parport_generic_irq (irq, (struct parport *)dev_id, regs);
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}

/*--- IEEE 1284 functions -----------------------------------------*/

#ifdef PARPORT_IP32_EPP
/* Clear TIMEOUT BIT in EPP MODE */
static int clear_epp_timeout (void)
{
	unsigned char r;

	if (!(parport_in (STATUS) & 0x01))
		return 1;

	/* To clear timeout some chips require double read */
	parport_in (STATUS);
	r = parport_in (STATUS);
	parport_out (r | 0x01, STATUS); /* Some reset by writing 1 */
	parport_out (r & 0xfe, STATUS); /* Others by writing 0 */
	r = parport_in (STATUS);

	return !(r & 0x01);
}

static size_t parport_ip32_epp_read_data (struct parport *port, 
					  void *buf, size_t length, 
					  int flags)
{
	size_t got = 0;
	pr_debug1 ("parport_ip32_epp_read_data(...)\n");
	frob_set_mode (ECR_MODE_EPP);
	parport_ip32_data_reverse (port);
	parport_ip32_write_control (port, 0x4);
	for (; got < length; got++) {
		*((char*)buf) = parport_in (EPPDATA);
		buf++;
		if (parport_in (STATUS) & 0x01) {
			/* EPP timeout */
			clear_epp_timeout ();
			break;
		}
	}
	frob_set_mode (ECR_MODE_PS2);

	return got;
}

static size_t parport_ip32_epp_write_data (struct parport *port, 
					   const void *buf, size_t length,
					   int flags)
{
	size_t written = 0;
	pr_debug1 ("parport_ip32_epp_write_data(...)\n");
	frob_set_mode (ECR_MODE_EPP);
	parport_ip32_write_control (port, 0x4);
	parport_ip32_data_forward (port);
	for (; written < length; written++) {
		parport_out (*((char*)buf), EPPDATA);
		buf++;
		if (parport_in (STATUS) & 0x01) {
			clear_epp_timeout ();
			break;
		}
	}
	frob_set_mode (ECR_MODE_PS2);

	return written;
}

static size_t parport_ip32_epp_read_addr (struct parport *port,
					  void *buf, size_t length, 
					  int flags)
{
	size_t got = 0;
	pr_debug1 ("parport_ip32_epp_read_addr(...)\n");
	frob_set_mode (ECR_MODE_EPP);
	parport_ip32_data_reverse (port);
	parport_ip32_write_control (port, 0x4);
	for (; got < length; got++) {
		*((char*)buf) = parport_in (EPPADDR);
		buf++;
		if (parport_in (STATUS) & 0x01) {
			clear_epp_timeout ();
			break;
		}
	}
	frob_set_mode (ECR_MODE_PS2);

	return got;
}

static size_t parport_ip32_epp_write_addr (struct parport *port,
					   const void *buf, size_t length,
					   int flags)
{
	size_t written = 0;
	pr_debug1 ("parport_ip32_epp_write_addr(...)\n");
	frob_set_mode (ECR_MODE_EPP);
	parport_ip32_write_control (port, 0x4);
	parport_ip32_data_forward (port);
	for (; written < length; written++) {
		parport_out (*((char*)buf), EPPADDR);
		buf++;
		if (parport_in (STATUS) & 0x01) {
			clear_epp_timeout ();
			break;
		}
	}
	frob_set_mode (ECR_MODE_PS2);

	return written;
}
#endif /* Enable EPP support */

#ifdef PARPORT_IP32_FIFO
/* Safely change the mode bits in the ECR 
   Returns:
	    0    : Success
	   -EBUSY: Could not drain FIFO in some finite amount of time,
		   mode not changed!
 */
static int change_mode(int m)
{
	unsigned char oecr;
	int mode;

	pr_debug1 ("parport change_mode ECP-ISA to mode 0x%02x\n", m);

	/* Bits <7:5> contain the mode. */
	oecr = parport_in (ECONTROL);
	mode = (oecr & ECR_MODE_MASK);
	if (mode == m) return 0;

	if (mode >= ECR_MODE_PPF && !(this.ctr & 0x20)) {
		/* This mode resets the FIFO, so we may
		 * have to wait for it to drain first. */
		unsigned long expire = 
			jiffies + this.port->physport->cad->timeout;
		int counter;
		switch (mode) {
		case ECR_MODE_PPF: /* Parallel Port FIFO mode */
		case ECR_MODE_ECP: /* ECP Parallel Port mode */
			/* Busy wait for 200us */
			for (counter = 0; counter < 40; counter++) {
				if (parport_in (ECONTROL) & ECR_F_EMPTY)
					break;
				if (signal_pending (current)) break;
				udelay (5);
			}

			/* Poll slowly. */
			while (!(parport_in (ECONTROL) & ECR_F_EMPTY)) {
				if (time_after_eq (jiffies, expire))
					/* The FIFO is stuck. */
					return -EBUSY;
				schedule_timeout_interruptible (
					msecs_to_jiffies (10));
				if (signal_pending (current))
					break;
			}
		}
	}

	if (mode >= ECR_MODE_PPF && m >= ECR_MODE_PPF) {
		/* We have to go through mode 001 */
		oecr &= ~ECR_MODE_MASK;
		oecr |= ECR_MODE_PS2;
		parport_out (oecr, ECONTROL);
	}

	/* Set the mode. */
	oecr &= ~ECR_MODE_MASK;
	oecr |= m;
	parport_out (oecr, ECONTROL);
	return 0;
}

static size_t parport_ip32_fifo_write_block_pio (struct parport *port,
						 const void *buf, 
						 size_t length)
{
	int ret = 0;
	const unsigned char *bufp = buf;
	size_t left = length;
	unsigned long expire = jiffies + port->physport->cad->timeout;
	void * const fifo = FIFO;
	int poll_for = 8; /* 80 usecs */
	const int fifo_depth = this.fifo_depth;

	pr_debug1 ("parport_ip32_fifo_write_block_pio(...)\n");

	port = port->physport;

	/* We don't want to be interrupted every character. */
	parport_ip32_disable_irq (port);
	/* set nErrIntrEn and serviceIntr */
	frob_econtrol (ECR_IRQ | ECR_INTR, ECR_IRQ | ECR_INTR);

	/* Forward mode. */
	parport_ip32_data_forward (port); /* Must be in PS2 mode */

	while (left) {
		unsigned char byte;
		unsigned char ecrval = parport_in (ECONTROL);
		int i = 0;

		if (need_resched () && time_before (jiffies, expire))
			/* Can't yield the port. */
			schedule ();

		/* Anyone else waiting for the port? */
		if (port->waithead) {
			printk (KERN_DEBUG "Somebody wants the port\n");
			break;
		}

		if (ecrval & ECR_F_FULL) {
			/* FIFO is full. Wait for interrupt. */

			/* Clear serviceIntr */
			parport_out (ecrval & ~ECR_INTR, ECONTROL);
		false_alarm:
			ret = parport_wait_event (port, HZ);
			if (ret < 0) break;
			ret = 0;
			if (!time_before (jiffies, expire)) {
				/* Timed out. */
				printk (KERN_DEBUG "FIFO write timed out\n");
				break;
			}
			ecrval = parport_in (ECONTROL);
			if (!(ecrval & ECR_INTR)) {
				if (need_resched() &&
				    time_before (jiffies, expire))
					schedule ();

				goto false_alarm;
			}

			continue;
		}

		/* Can't fail now. */
		expire = jiffies + port->cad->timeout;

	poll:
		if (signal_pending (current))
			break;

		if (ecrval & ECR_F_EMPTY) {
			/* FIFO is empty. Blast it full. */
			int n = left < fifo_depth ? left : fifo_depth;
			while (n-- > 0) {
				parport_out (*bufp++, fifo);
				left--;
			}

			/* Adjust the poll time. */
			if (i < (poll_for - 2)) poll_for--;
			continue;
		} else if (i++ < poll_for) {
			udelay (10);
			ecrval = parport_in (ECONTROL);
			goto poll;
		}

		/* Half-full (call me an optimist) */
		byte = *bufp++;
		parport_out (byte, fifo);
		left--;
        }

	dump_parport_state ("leave fifo_write_block_pio", port);
	return length - left;
}

/* #ifdef HAS_DMA */
/* static size_t parport_ip32_fifo_write_block_dma (struct parport *port, */
/* 					       const void *buf, size_t length) */
/* { */
/* 	... */
/* } */
/* #endif */

static inline size_t parport_ip32_fifo_write_block(struct parport *port,
						   const void *buf,
						   size_t length)
{
	pr_debug1 ("parport_ip32_fifo_write_block(...)\n");
/* #ifdef HAS_DMA */
/* 	if (port->dma != PARPORT_DMA_NONE) */
/* 		return parport_ip32_fifo_write_block_dma (port, buf, length); */
/* #endif */
	return parport_ip32_fifo_write_block_pio (port, buf, length);
}

/* Parallel Port FIFO mode (ECP chipsets) */
static size_t parport_ip32_compat_write_block (struct parport *port,
					       const void *buf, size_t length,
					       int flags)
{
	size_t written;
	int r;
	unsigned long expire;

	pr_debug1 ("parport_ip32_compat_write_block(...)\n");

	/* Special case: a timeout of zero means we cannot call schedule().
	 * Also if O_NONBLOCK is set then use the default implementation. */
	if (port->physport->cad->timeout <= PARPORT_INACTIVITY_O_NONBLOCK)
		return parport_ieee1284_write_compat (port, buf,
						      length, flags);

	/* Set up parallel port FIFO mode.*/
	parport_ip32_data_forward (port); /* Must be in PS2 mode */
	parport_ip32_frob_control (port, PARPORT_CONTROL_STROBE, 0);
	r = change_mode (ECR_MODE_PPF); /* Parallel port FIFO */
	if (r)  
		printk (KERN_DEBUG "%s: Warning change_mode ECR_PPF failed\n", 
			port->name);

	port->physport->ieee1284.phase = IEEE1284_PH_FWD_DATA;

	/* Write the data to the FIFO. */
	written = parport_ip32_fifo_write_block(port, buf, length);

	/* Finish up. */
	/* For some hardware we don't want to touch the mode until
	 * the FIFO is empty, so allow 4 seconds for each position
	 * in the fifo.
	 */
        expire = jiffies + (this.fifo_depth * HZ * 4);
	do {
		/* Wait for the FIFO to empty */
		r = change_mode (ECR_MODE_PS2);
		if (r != -EBUSY) {
			break;
		}
	} while (time_before (jiffies, expire));
	if (r == -EBUSY) {

		printk (KERN_DEBUG "%s: FIFO is stuck\n", port->name);

		/* Prevent further data transfer. */
		frob_set_mode (ECR_MODE_TST);

		/* Adjust for the contents of the FIFO. */
		for (written -= this.fifo_depth; ; written++) {
			if (parport_in (ECONTROL) & ECR_F_FULL) {
				/* Full up. */
				break;
			}
			parport_out (0, FIFO);
		}

		/* Reset the FIFO and return to PS2 mode. */
		frob_set_mode (ECR_MODE_PS2);
	}

	r = parport_wait_peripheral (port,
				     PARPORT_STATUS_BUSY,
				     PARPORT_STATUS_BUSY);
	if (r)
		printk (KERN_DEBUG
			"%s: BUSY timeout (%d) in compat_write_block_pio\n", 
			port->name, r);

	port->physport->ieee1284.phase = IEEE1284_PH_FWD_IDLE;

	return written;
}
#endif /* Enable FIFO support */

/*--- Default operations ------------------------------------------*/

static struct parport_operations parport_ip32_ops = 
{
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

/*--- Device detection --------------------------------------------*/

/* Check for ECR */
static int __init parport_ECR_present (void)
{
	unsigned char r = 0x0c;

	parport_out (r, CONTROL);
	if ((parport_in (ECONTROL) & 0x03) == (r & 0x03)) {
		parport_out (r ^ 0x02, CONTROL); /* Toggle bit 1 */
		r = parport_in (CONTROL);
		if ((parport_in (ECONTROL) & 0x02) == (r & 0x02))
			goto no_reg; /* Sure that no ECR register exists */
	}

	if ((parport_in (ECONTROL) & 0x03) != 0x01)
		goto no_reg;

	parport_out (0x34, ECONTROL);
	if (parport_in (ECONTROL) != 0x35)
		goto no_reg;

	parport_out (0x0c, CONTROL);

	/* Go to mode 000 */
	frob_set_mode (ECR_MODE_SPP);

	return 1;

no_reg:
	parport_out (0x0c, CONTROL);
	return 0;
}

#ifdef PARPORT_IP32_FIFO
/* Try to find ECP settings (essentially FIFO parameters). 
 *
 * Returns 0 if something unexpected happened.
 */
static int __init parport_ECP_supported (void)
{
	int i;
	int config, configb;
	int pword;

	/* Find out FIFO depth */
	parport_out (ECR_MODE_SPP, ECONTROL); /* Reset FIFO */
	parport_out (ECR_MODE_TST, ECONTROL); /* TEST FIFO */
	for (i=0; i < 1024 && !(parport_in (ECONTROL) & 0x02); i++)
		parport_out (0xaa, FIFO);

	/* FIFO seems too big: not supported */
	if (i == 1024) {
		printk (KERN_WARNING PPIP32 "could not get FIFO depth\n");
		parport_out (ECR_MODE_SPP, ECONTROL);
		return 0;
	}
	this.fifo_depth = i;

	/* Find out writeIntrThreshold: number of bytes we know we can
	 * write if we get an interrupt. */
	frob_econtrol (ECR_INTR, ECR_INTR);
	frob_econtrol (ECR_INTR, 0);
	for (i = 1; i <= this.fifo_depth; i++) {
		parport_in (FIFO);
		udelay (50);
		if (parport_in (ECONTROL) & ECR_INTR)
			break;
	}
	if (i > this.fifo_depth) {
		i = 0;
	}
	this.writeIntrThreshold = i;

	/* Find out readIntrThreshold: number of bytes we can read if
	 * we get an interrupt. */
	frob_set_mode (ECR_MODE_PS2); /* Reset FIFO and enable PS2 */
	parport_ip32_data_reverse (this.port); /* Must be in PS2 mode */
	frob_set_mode (ECR_MODE_TST); /* Test FIFO */
	frob_econtrol (ECR_INTR, ECR_INTR);
	frob_econtrol (ECR_INTR, 0);
	for (i = 1; i <= this.fifo_depth; i++) {
		parport_out (0xaa, FIFO);
		if (parport_in (ECONTROL) & ECR_INTR)
			break;
	}
	if (i > this.fifo_depth) {
		i = 0;
	}
	this.readIntrThreshold = i;

	parport_out (ECR_MODE_SPP, ECONTROL); /* Reset FIFO */
	parport_out (ECR_MODE_CNF, ECONTROL); /* Configuration mode */
	config = parport_in (CONFIGA);
	pword = (config >> 4) & 0x7;
	switch (pword) {
	case 0:
		pword = 2;
		printk (KERN_WARNING PPIP32 "Unsupported pword size: %d\n",
			pword);
		break;
	case 2:
		pword = 4;
		printk (KERN_WARNING PPIP32 "Unsupported pword size: %d\n",
			pword);
		break;
	default:
		printk (KERN_WARNING PPIP32 "Unknown pword ID: 0x%x\n", pword);
		/* Assume 1 */
	case 1:
		pword = 1;
	}
	this.pword = pword;

	if (VERBOSE_PROBING) {
		printk (KERN_DEBUG PPIP32 "* FIFO is %d bytes\n", 
		       this.fifo_depth);
		printk (KERN_DEBUG PPIP32 "* writeIntrThreshold is %d\n",
		       this.writeIntrThreshold);
		printk (KERN_DEBUG PPIP32 "* readIntrThreshold is %d\n",
		       this.readIntrThreshold);
		printk (KERN_DEBUG PPIP32 "* PWord is %d bits\n", 8 * pword);
		printk (KERN_DEBUG PPIP32 "* Interrupts are %s triggered\n",
		       config & 0x80 ? "level" : "edge");

		configb = parport_in (CONFIGB);
		printk (KERN_DEBUG PPIP32 
		       "* ECP port cfgA=0x%02x cfgB=0x%02x\n",
		       config, configb);
		printk (KERN_DEBUG PPIP32 "* ECP settings irq=(%d) dma=(%d)\n",
		       (configb >>3) & 0x07, configb & 0x07);
	}

	return 1;
}
#endif /* Enable FIFO suppport */

/*--- Initialisation code -----------------------------------------*/

static int __init parport_ip32_init (void)
{
	printk (KERN_INFO PPIP32 "SGI IP32 MACE parallel port driver\n");

	INIT_MACE_POINTER ();
	if (mace == NULL) {
		printk (KERN_INFO PPIP32 "invalid mace pointer\n");
		return -ENXIO;
	}

	pr_debug (PPIP32 "    DATA     @ %p\n", DATA);
	pr_debug (PPIP32 "    STATUS   @ %p\n", STATUS);
	pr_debug (PPIP32 "    CONTROL  @ %p\n", CONTROL);
	pr_debug (PPIP32 "    CONFIGA  @ %p\n", CONFIGA);
	pr_debug (PPIP32 "    CONFIGB  @ %p\n", CONFIGB);
	pr_debug (PPIP32 "    ECONTROL @ %p\n", ECONTROL);
#ifdef DEBUG_PARPORT_IP32
#define print_register(x) printk ("%s%s=0x%02x", f++?", ":"", #x, parport_in (x))
	{
		int f;
		unsigned char oectr;
		oectr = parport_in (ECONTROL);
		pr_debug1 (PPIP32 "* SPP registers: "); f = 0;
		print_register (DATA);
		print_register (STATUS);
		print_register (CONTROL);
		printk ("\n");
		pr_debug1 (PPIP32 "* EPP registers: "); f = 0;
		print_register (EPPADDR);
		print_register (EPPDATA);
		printk ("\n");
		pr_debug1 (PPIP32 "* ECP registers: "); f = 0;
		print_register (CONFIGA);
		print_register (CONFIGB);
		print_register (ECONTROL);
		printk ("\n");
		parport_out (0xe0, ECONTROL);
		pr_debug1 (PPIP32 "* ECP config:    "); f = 0;
		print_register (CONFIGA);
		print_register (CONFIGB);
		print_register (ECONTROL);
		printk ("\n");
		parport_out (oectr, ECONTROL);
	}
#undef print_register
#endif /* DEBUG_PARPORT_IP32 */
	dump_parport_state ("begin init", this.port);

	/* Initialize private variables */
	this.ctr = INIT_CTR;
	this.ctr_writable = 0x2f;
#if 0
	this.fifo_depth = 0;
	this.pword = 1;
	this.readIntrThreshold = 0;
	this.writeIntrThreshold = 0;
#endif /* 0 */

	/* Note: the `base' parameter is not used, we just put here
	   some arbitrary value */
	this.port = parport_register_port (PARPORT_IP32_IO, /* base */
					   PARPORT_IRQ_NONE, /* irq */
					   PARPORT_DMA_NONE, /* dma */
					   &parport_ip32_ops);
	if (this.port == NULL) {
		printk (KERN_INFO PPIP32 "parport_register_port failed\n");
		return -ENOMEM;
	}
	this.port->modes = 0;
	this.port->base_hi = PARPORT_IP32_IOHI;

	/* We only support ECP ports.  Thus we need the ECR to be
	 * present */
	if (! parport_ECR_present ()) {
		printk (KERN_INFO PPIP32 "ECR not found\n");
		parport_put_port (this.port);
		return -ENODEV;
	}

	/* 
	 * Initialize supported modes. 
	 */
	this.port->modes |= PARPORT_MODE_PCSPP;
	/* don't know if it is really "SAFEININT", try it until
	 * someone complains */
	this.port->modes |= PARPORT_MODE_SAFEININT;
	this.port->modes |= PARPORT_MODE_TRISTATE;
#ifdef PARPORT_IP32_USE_IRQ
	this.port->irq = PARPORT_IP32_IRQ;
#endif /* Enable interrupts */
#ifdef PARPORT_IP32_EPP
	this.port->modes |= PARPORT_MODE_EPP;
#endif /* Enable EPP support */
#ifdef PARPORT_IP32_FIFO
	if (parport_ECP_supported ()) {
		this.port->modes |= PARPORT_MODE_COMPAT;
	}
#endif /* Enable FIFO support */

	/* Adjust ctr_writable */
	if (! this.port->modes & PARPORT_MODE_TRISTATE) {
		this.ctr_writable &= ~0x20;
	}
	if (this.port->irq != PARPORT_IRQ_NONE) {
		this.ctr_writable |= 0x10;
	}

#ifdef PARPORT_IP32_FIFO
	if (this.port->modes & PARPORT_MODE_COMPAT) {
		struct parport_operations *ops = this.port->ops;
		ops->compat_write_data = parport_ip32_compat_write_block;
	}
#endif /* Enable FIFO support */
#ifdef PARPORT_IP32_EPP
	if (this.port->modes & PARPORT_MODE_EPP) {
		/* Set up access functions to use EPP hardware. */
		struct parport_operations *ops = this.port->ops;
		ops->epp_read_data  = parport_ip32_epp_read_data;
		ops->epp_write_data = parport_ip32_epp_write_data;
		ops->epp_read_addr  = parport_ip32_epp_read_addr;
		ops->epp_write_addr = parport_ip32_epp_write_addr;
	}
#endif /* Enable EPP support */

	/* Print essential informations */
	printk (KERN_INFO "%s: SGI IP32", this.port->name);
	printk (" irq=");
	if (this.port->irq != PARPORT_IRQ_NONE) {
		printk ("%d", this.port->irq);
	} else {
		printk ("(none)");
	}
	printk (" dma=");
	if (this.port->dma != PARPORT_DMA_NONE) {
		printk ("%d", this.port->dma);
	} else {
		printk ("(none)");
	}
	printk (" [");
#define printmode(x)					\
	if (this.port->modes & PARPORT_MODE_##x)	\
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
	printk ("]\n");

	/* Request configured IRQ */
	if (this.port->irq != PARPORT_IRQ_NONE) {
		if (request_irq (this.port->irq, parport_ip32_interrupt,
				 0, this.port->name, this.port)) {
			printk (KERN_WARNING "%s: irq %d in use, "
				"resorting to polled operation\n",
				this.port->name, this.port->irq);
			this.port->irq = PARPORT_IRQ_NONE;
			this.ctr_writable &= ~0x10;
			this.port->dma = PARPORT_DMA_NONE;
		}
	}

	/* Initialise the port with sensible values */  
        parport_out (INIT_ECR, ECONTROL);
	__parport_ip32_write_control (this.port, INIT_CTR);
	parport_ip32_disable_irq (this.port);
	parport_ip32_data_forward (this.port);
	parport_ip32_write_data (this.port, 0x00);

	dump_parport_state ("end init", this.port);
	parport_announce_port (this.port);

	return 0;
}

static void __exit parport_ip32_exit (void)
{
	parport_remove_port (this.port);

	if (this.port->irq != PARPORT_IRQ_NONE)
		free_irq (this.port->irq, this.port);

	parport_put_port (this.port);  
	RELEASE_MACE_POINTER ();
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
 * ispell-local-dictionary: "american"
 * End:
 */
