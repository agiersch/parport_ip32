/* Low-level parallel port routines for builtin port on SGI IP32
 *
 * Author: Arnaud Giersch <arnaud.giersch@free.fr>
 *
 * $Id: parport_ip32.c,v 1.1 2005-10-02 14:17:57 arnaud Exp $
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
 * October 2005
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

/* DEBUG_PARPORT
 *	0	disable debug
 *	1	standard level
 *	>1	verbose level
 */
#define DEBUG_PARPORT 1

#undef DEBUG
#ifdef DEBUG_PARPORT
#  if DEBUG_PARPORT == 0
#    undef DEBUG_PARPORT
#  elif DEBUG_PARPORT == 1
#    warning DEBUG_PARPORT == 1
     /* nop */
#  else /* DEBUG_PARPORT > 1 */
#    warning DEBUG_PARPORT > 1
#    define DEBUG			/* enable pr_debug() in kernel.h */
#  endif /* DEBUG_PARPORT > 1 */
#endif /* ! DEBUG_PARPORT */

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

/* ECR modes */
#define ECR_MODE_SPP (00 << 5)
#define ECR_MODE_PS2 (01 << 5)
#define ECR_MODE_PPF (02 << 5)
#define ECR_MODE_ECP (03 << 5)
#define ECR_MODE_EPP (04 << 5)
#define ECR_MODE_VND (05 << 5)
#define ECR_MODE_TST (06 << 5)
#define ECR_MODE_CNF (07 << 5)
#define ECR_MODE_MASK 0xe0
#define ECR_IRQ       (1 << 4)
#define ECR_DMA       (1 << 3)
#define ECR_INTR      (1 << 2)
#define ECR_F_FULL    (1 << 1)
#define ECR_F_EMPTY   (1 << 0)

/* Initial values for CTR and ECR */
#define INIT_CTR \
	(PARPORT_CONTROL_INIT | PARPORT_CONTROL_SELECT)
#define INIT_ECR \
	(((this.port->modes & PARPORT_MODE_TRISTATE)?	\
	  ECR_MODE_PS2: ECR_MODE_SPP))

MODULE_AUTHOR("Arnaud Giersch <arnaud.giersch@free.fr>");
MODULE_DESCRIPTION("SGI IP32 builtin parallel port driver");
MODULE_SUPPORTED_DEVICE("SGI IP32 MACE parallel port");
MODULE_LICENSE("GPL");

#ifdef MODULE
/* mace is not exported in arch/mips/sgi-ip32/crime.c,
 * use our own pointer for modules */
#define DEFINE_MACE
#endif

#ifdef DEFINE_MACE
static struct sgi_mace *my_mace;
#  define INIT_MACE_POINTER() \
	my_mace = ioremap(MACE_BASE, sizeof(struct sgi_mace))
#  define RELEASE_MACE_POINTER() \
	iounmap(my_mace)
#  define mace my_mace
#else
#  define INIT_MACE_POINTER() do { } while (0)
#  define RELEASE_MACE_POINTER() do { } while (0)
#endif /* DEFINE_MACE */

/* Physical I/O addresses */
#define PARPORT_IP32_IO	  (MACE_BASE + \
			   offsetof(struct sgi_mace, isa) + \
			   offsetof(struct mace_isa, parallel))
#define PARPORT_IP32_IOHI (MACE_BASE + \
			   offsetof(struct sgi_mace, isa) + \
			   offsetof(struct mace_isa, ecp1284))

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

static int verbose_probing = 1;

/* We do not support more than one port. */
static struct {
	/* Contents of CTR. */
	unsigned char ctr;

	/* Bitmask of writable CTR bits. */
	unsigned char ctr_writable;

	/* Number of PWords that FIFO will hold. */
	int fifo_depth;

	/* Number of bytes per PWord. */
	int pword;

	/* Not used yet. */
	int readIntrThreshold;
	int writeIntrThreshold;

	struct parport *port;
} this;

/*--- I/O port access function ------------------------------------*/

#if defined(io_read) || defined (io_write)
#  error io_read or io_write already defined !
#endif

static inline unsigned int io_read(void *addr)
{
	unsigned int val = readb(addr);
	rmb();
	return val;
}

static inline void io_write(u8 val, void *addr)
{
	writeb(val, addr);
	wmb();
}

/*--- Debug code --------------------------------------------------*/

#ifdef DEBUG_PARPORT

#define pr_debug1(fmt,arg...) \
	printk(KERN_DEBUG fmt,##arg)

static inline void dump_parport_state(char *str, struct parport *p)
{
	/* here's hoping that reading these ports won't side-effect
	 * anything underneath */
	unsigned char ecr, dcr, dsr;
	static char *ecr_modes[] = 
		{"SPP", "PS2", "PPFIFO", "ECP", "EPP", "xXx", "TST", "CFG"};
	int i;

	ecr = io_read(ECONTROL);
	printk(KERN_DEBUG PPIP32 "parport state (%s):\n", str);
	printk(KERN_DEBUG PPIP32 "    ecr=[%s", ecr_modes[(ecr & 0xe0) >> 5]);
	if (ecr & 0x10) printk(",nErrIntrEn");
	if (ecr & 0x08) printk(",dmaEn");
	if (ecr & 0x04) printk(",serviceIntr");
	if (ecr & 0x02) printk(",f_full");
	if (ecr & 0x01) printk(",f_empty");
	for (i=0; i<2; i++) {
		printk("]\n");
		printk(KERN_DEBUG PPIP32 "    dcr(%s)=[", i ? "soft" : "hard");
		dcr = i? this.ctr: io_read(CONTROL);
	
		printk("0x%02x", dcr & 0xc0);
		if (dcr & 0x20) {
			printk(",rev");
		} else {
			printk(",fwd");
		}
		if (dcr & 0x10) printk(",ackIntEn");
		if (!(dcr & 0x08)) printk(",N-SELECT-IN");
		if (dcr & 0x04) printk(",N-INIT");
		if (!(dcr & 0x02)) printk(",N-AUTOFD");
		if (!(dcr & 0x01)) printk(",N-STROBE");
	}
	dsr = io_read(STATUS );
	printk("]\n");
	printk(KERN_DEBUG PPIP32 "    dsr=[");
	if (!(dsr & 0x80)) printk("BUSY");
	if (dsr & 0x40) printk(",N-ACK");
	if (dsr & 0x20) printk(",PERROR");
	if (dsr & 0x10) printk(",SELECT");
	if (dsr & 0x08) printk(",N-FAULT");
	if (dsr & 0x07) printk(",0x%02x", dsr & 0x07);
	printk("]\n");
	return;
}

#else /* !DEBUG_PARPORT */

#define pr_debug1(fmt,arg...) \
	do { } while (0)

#define dump_parport_state(...) \
	do { } while (0)

#endif /* !DEBUG_PARPORT */

/*--- Basic functions needed for parport --------------------------*/

static inline unsigned char parport_ip32_read_data(struct parport *p)
{
	unsigned char d = io_read(DATA);
	pr_debug("parport_ip32_read_data(%p): 0x%02x\n", p, d);
	return d;
}

static inline void parport_ip32_write_data(struct parport *p, 
					   unsigned char d)
{
	pr_debug("parport_ip32_write_data(%p, 0x%02x)\n", p, d);
	io_write(d, DATA);
}

static inline unsigned char parport_ip32_read_status(struct parport *p)
{
	unsigned char s = io_read(STATUS);
	pr_debug("parport_ip32_read_status(%p): 0x%02x\n", p, s);
	return s;
}

static inline unsigned char parport_ip32_read_control(struct parport *p)
{
	unsigned char c = this.ctr; /* use soft copy */
	pr_debug("parport_ip32_read_control(%p): 0x%02x\n", p, c);
	return c;
}

/* __parport_ip32_write_control differs from parport_ip32_write_control in that
 * it doesn't do any extra masking. */
static inline void __parport_ip32_write_control(struct parport *p, 
						unsigned char c)
{
#ifdef DEBUG_PARPORT
	if (c & ~this.ctr_writable) {
		pr_debug1(PPIP32 
			  "extra bits in __write_control: 0x%02x/0x%02x\n", 
			  c, this.ctr_writable);
	}
#endif
	c &= this.ctr_writable;	/* only writable bits */
	io_write(c, CONTROL);
	this.ctr = c;		/* update soft copy */
}

static inline void parport_ip32_write_control(struct parport *p, 
					      unsigned char c)
{
	const unsigned char wm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);

	pr_debug("parport_ip32_write_control(%p, 0x%02x)\n", p, c);
#ifdef DEBUG_PARPORT
	if (c & ~wm) {
		pr_debug1(PPIP32 
			  "extra bits in write_control: 0x%02x/0x%02x\n", 
			  c, wm);
	}
#endif 
	c &= wm;
	__parport_ip32_write_control(p, c);
}

/* __parport_ip32_frob_control differs from parport_ip32_frob_control in that
 * it doesn't do any extra masking. */
static inline unsigned char __parport_ip32_frob_control(struct parport *p,
							unsigned char mask,
							unsigned char val)
{
	unsigned char c = (parport_ip32_read_control(p) & ~mask) ^ val;
	__parport_ip32_write_control(p, c);
	return c;
}

static inline unsigned char parport_ip32_frob_control(struct parport *p, 
						      unsigned char mask, 
						      unsigned char val)
{
	const unsigned char wm = (PARPORT_CONTROL_STROBE |
				  PARPORT_CONTROL_AUTOFD |
				  PARPORT_CONTROL_INIT |
				  PARPORT_CONTROL_SELECT);
	pr_debug("parport_ip32_frob_control(%p, 0x%02x, 0x%02x)\n", 
		 p, mask, val);
#ifdef DEBUG_PARPORT
	if (val & ~wm) {
		pr_debug1(PPIP32
			  "extra bits in frob_control: 0x%02x,0x%02x/0x%02x",
			  mask, val, wm);
	}
#endif
	val &= wm;
	return __parport_ip32_frob_control(p, mask, val);
}

static inline void parport_ip32_disable_irq(struct parport *p)
{
	pr_debug1("parport_ip32_disable_irq(%p)\n", p);
	__parport_ip32_frob_control(p, 0x10, 0x00);
}

static inline void parport_ip32_enable_irq(struct parport *p)
{
	pr_debug1("parport_ip32_enable_irq(%p)\n", p);
	__parport_ip32_frob_control(p, 0x10, 0x10);
}

static inline void parport_ip32_data_forward(struct parport *p)
{
	pr_debug("parport_ip32_data_forward(%p)\n", p);
	__parport_ip32_frob_control(p, 0x20, 0x00);
}

static inline void parport_ip32_data_reverse(struct parport *p)
{
	pr_debug("parport_ip32_data_reverse(%p)\n", p);
	__parport_ip32_frob_control(p, 0x20, 0x20);
}

static inline void parport_ip32_init_state(struct pardevice *dev, 
					   struct parport_state *s)
{
	pr_debug("parport_ip32_init_state(%p, %p)\n", dev, s);
	s->u.ip32.ctr = INIT_CTR;
	if (dev->irq_func && dev->port->irq != PARPORT_IRQ_NONE) {
		/* Set ackIntEn */
		s->u.ip32.ctr |= 0x10;
	}
	s->u.ip32.ecr = INIT_ECR;
}

static inline void parport_ip32_save_state(struct parport *p, 
					   struct parport_state *s)
{
	pr_debug("parport_ip32_save_state(%p, %p)\n", p, s);
	s->u.ip32.ctr = parport_ip32_read_control(p);
	s->u.ip32.ecr = io_read(ECONTROL);
}

static inline void parport_ip32_restore_state(struct parport *p, 
					      struct parport_state *s)
{
	pr_debug("parport_ip32_restore_state(%p, %p)\n", p, s);
	__parport_ip32_write_control(p, s->u.ip32.ctr);
	io_write(s->u.ip32.ecr, ECONTROL);
}

/*--- Simple interrupt handler ------------------------------------*/

#if 0
static irqreturn_t parport_ip32_interrupt(int irq, void *dev_id, 
					  struct pt_regs *regs)
{
#ifdef DEBUG_PARPORT
	printk(KERN_DEBUG PPIP32 "caught IRQ %d\n", irq);
#endif
	parport_generic_irq(irq, (struct parport *) dev_id, regs);
	/* FIXME! Was it really ours? */
	return IRQ_HANDLED;
}
#endif

/*--- Some utility definitions for ECR ----------------------------*/

/* frob_control, but for ECR */
static inline void frob_econtrol(unsigned char mask, unsigned char val)
{
	unsigned char ectr = 0;
 	pr_debug(PPIP32 "frob_econtrol(%02x,%02x)\n", mask, val);

	if (mask != 0xff) {
		ectr = io_read(ECONTROL);
	}
	io_write((ectr & ~mask) ^ val, ECONTROL);
}

static inline void frob_set_mode(int mode)
{
	frob_econtrol(ECR_MODE_MASK, mode & ECR_MODE_MASK);
}

/*--- Mode detection ----------------------------------------------*/

/* Check for ECR
 *
 * Old style XT ports alias io ports every 0x400, hence accessing ECR
 * on these cards actually accesses the CTR.
 *
 * Modern cards don't do this but reading from ECR will return 0xff
 * regardless of what is written here if the card does NOT support
 * ECP.
 *
 * We first check to see if ECR is the same as CTR.  If not, the low
 * two bits of ECR aren't writable, so we check by writing ECR and
 * reading it back to see if it's what we expect.
 */
static int parport_ECR_present(void)
{
	unsigned char r = 0x0c;

	io_write(r, CONTROL);
	if ((io_read(ECONTROL) & 0x03) == (r & 0x03)) {
		io_write(r ^ 0x02, CONTROL); /* Toggle bit 1 */
		r = io_read(CONTROL);
		if ((io_read(ECONTROL) & 0x02) == (r & 0x02))
			goto no_reg; /* Sure that no ECR register exists */
	}

	if ((io_read(ECONTROL) & 0x03) != 0x01)
		goto no_reg;

	io_write(0x34, ECONTROL);
	if (io_read(ECONTROL) != 0x35)
		goto no_reg;

	io_write(0x0c, CONTROL);

	/* Go to mode 000 */
	frob_set_mode(ECR_MODE_SPP);

	return 1;

no_reg:
	io_write(0x0c, CONTROL);
	return 0;
}

/* parport_<mode>_supported
 *
 * Check if <mode> is supported.  Returns 0 if it is not.  Do various
 * initializations according to detected capabilities.
 *
 * ECR is assumed to be present.  Status of registers is undefined
 * before *and* after call.
 */

/* Detect SPP support
 *
 * All ports support SPP mode, so we only initialize this.port->modes.
 */
static int parport_SPP_supported(void)
{
	/* simply assume that SPP mode is supported */
	this.port->modes |= PARPORT_MODE_PCSPP;
	/* don't know if it is really "SAFEININT", try it until
	 * someone complains */
	this.port->modes |= PARPORT_MODE_SAFEININT;
	return 1;
}

/* Detect PS/2 support.
 *
 * Bit 5 (0x20) sets the PS/2 data direction; setting this high allows
 * us to read data from the data lines.  In theory we would get back
 * 0xff but any peripheral attached to the port may drag some or all
 * of the lines down to zero.  So if we get back anything that isn't
 * the contents of the data register we deem PS/2 support to be
 * present.
 *
 * Some SPP ports have "half PS/2" ability - you can't turn off the
 * line drivers, but an external peripheral with sufficiently beefy
 * drivers of its own can overpower them and assert its own levels
 * onto the bus, from where they can then be read back as normal.
 * Ports with this property and the right type of device attached are
 * likely to fail the SPP test, (as they will appear to have stuck
 * bits) and so the fact that they might be misdetected here is rather
 * academic.
 */
static int parport_PS2_supported(void)
{
	int ok = 0;

	/* go to PS2 mode */
	io_write(ECR_MODE_PS2, ECONTROL);
	parport_ip32_write_control(this.port, 0x0c);

	/* try to tri-state the buffer */
	parport_ip32_data_reverse(this.port);
	
	parport_ip32_write_data(this.port, 0x55);
	if (parport_ip32_read_data(this.port) != 0x55) ok++;

	parport_ip32_write_data(this.port, 0xaa);
	if (parport_ip32_read_data(this.port) != 0xaa) ok++;

	/* cancel input mode */
	parport_ip32_data_forward(this.port);

	if (ok) {
		this.port->modes |= PARPORT_MODE_TRISTATE;
	} else {
		/* disable direction bit */
		this.ctr_writable &= ~0x20;
	}

	return ok;
}

static int parport_ECP_supported(void)
{
	int i;
	int config, configb;
	int pword;

	/* Find out FIFO depth */
	io_write(ECR_MODE_SPP, ECONTROL); /* Reset FIFO */
	io_write(ECR_MODE_TST, ECONTROL); /* TEST FIFO */

	for (i=0; i < 1024 && !(io_read(ECONTROL) & 0x02); i++)
		io_write(0xaa, FIFO);

	/* FIFO seems too big: not supported */
	if (i == 1024) {
		return 0;
	}
	this.fifo_depth = i;

	/* Find out writeIntrThreshold: number of bytes we know we can
	 * write if we get an interrupt. */
	frob_econtrol (ECR_INTR, ECR_INTR);
	frob_econtrol (ECR_INTR, 0);
	for (i = 1; i <= this.fifo_depth; i++) {
		io_read(FIFO);
		udelay (50);
		if (io_read(ECONTROL) & ECR_INTR)
			break;
	}
	if (i > this.fifo_depth) {
		i = 0;
	}
	this.writeIntrThreshold = i;

	/* Find out readIntrThreshold: number of bytes we can read if
	 * we get an interrupt. */
	frob_set_mode(ECR_MODE_PS2); /* Reset FIFO and enable PS2 */
	parport_ip32_data_reverse(this.port); /* Must be in PS2 mode */
	frob_set_mode(ECR_MODE_TST); /* Test FIFO */
	frob_econtrol(ECR_INTR, ECR_INTR);
	frob_econtrol(ECR_INTR, 0);
	for (i = 1; i <= this.fifo_depth; i++) {
		io_write(0xaa, FIFO);
		if (io_read(ECONTROL) & ECR_INTR)
			break;
	}
	if (i > this.fifo_depth) {
		i = 0;
	}
	this.readIntrThreshold = i;

	io_write(ECR_MODE_SPP, ECONTROL); /* Reset FIFO */
	io_write(ECR_MODE_CNF, ECONTROL); /* Configuration mode */
	config = io_read(CONFIGA);
	pword = (config >> 4) & 0x7;
	switch (pword) {
	case 0:
		pword = 2;
		printk(KERN_WARNING PPIP32 "Unsupported pword size: %d\n",
			pword);
		break;
	case 2:
		pword = 4;
		printk(KERN_WARNING PPIP32 "Unsupported pword size: %d\n",
			pword);
		break;
	default:
		printk(KERN_WARNING PPIP32 "Unknown pword ID: 0x%x\n", pword);
		/* Assume 1 */
	case 1:
		pword = 1;
	}
	this.pword = pword;

	if (verbose_probing) {
		/* Translate ECP intrLine to ISA irq value */	
		/* Not sure this values are significant on IP32! */
		static const int intrline[] = {0, 7, 9, 10, 11, 14, 15, 5};

		printk(KERN_DEBUG PPIP32 "* FIFO is %d bytes\n", 
		       this.fifo_depth);
		printk(KERN_DEBUG PPIP32 "* writeIntrThreshold is %d\n",
		       this.writeIntrThreshold);
		printk(KERN_DEBUG PPIP32 "* readIntrThreshold is %d\n",
		       this.readIntrThreshold);
		printk(KERN_DEBUG PPIP32 "* PWord is %d bits\n", 8 * pword);
		printk(KERN_DEBUG PPIP32 "* Interrupts are %s triggered\n",
		       config & 0x80 ? "level" : "edge");

		configb = io_read(CONFIGB);
		printk(KERN_DEBUG PPIP32 
		       "* ECP port cfgA=0x%02x cfgB=0x%02x\n",
		       config, configb);
		printk(KERN_DEBUG PPIP32 "* ECP settings irq=");
		if ((configb >>3) & 0x07)
			printk("%d",intrline[(configb >>3) & 0x07]);
		else
			printk("<none or set by other means>");
		printk (" dma=");
		if( (configb & 0x03 ) == 0x00)
			printk("<none or set by other means>\n");
		else
			printk("%d\n",configb & 0x07);
	}

	return 1;
}

/*--- Initialisation ----------------------------------------------*/

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

/*--- Initialisation code -----------------------------------------*/

static int __init parport_ip32_init(void)
{
	printk(KERN_INFO PPIP32 "SGI IP32 MACE parallel port driver\n");

	INIT_MACE_POINTER();
	if (mace == NULL) {
		printk(KERN_INFO PPIP32 "invalid mace pointer\n");
		return -ENXIO;
	}

#ifdef DEBUG_PARPORT
	pr_debug(PPIP32 "    DATA     @ %p\n", DATA);
	pr_debug(PPIP32 "    STATUS   @ %p\n", STATUS);
	pr_debug(PPIP32 "    CONTROL  @ %p\n", CONTROL);
	pr_debug(PPIP32 "    CONFIGA  @ %p\n", CONFIGA);
	pr_debug(PPIP32 "    CONFIGB  @ %p\n", CONFIGB);
	pr_debug(PPIP32 "    ECONTROL @ %p\n", ECONTROL);
#define print_register(x) printk("%s%s=0x%02x", f++?", ":"", #x, io_read(x))
	{
		int f;
		unsigned char oectr;
		oectr = io_read(ECONTROL);
		pr_debug1(PPIP32 "* SPP registers: "); f = 0;
		print_register(DATA);
		print_register(STATUS);
		print_register(CONTROL);
		printk("\n");
		pr_debug1(PPIP32 "* EPP registers: "); f = 0;
		print_register(EPPADDR);
		print_register(EPPDATA);
		printk("\n");
		pr_debug1(PPIP32 "* ECP registers: "); f = 0;
		print_register(CONFIGA);
		print_register(CONFIGB);
		print_register(ECONTROL);
		printk("\n");
		io_write(0xe0, ECONTROL);
		pr_debug1(PPIP32 "* ECP config:    "); f = 0;
		print_register(CONFIGA);
		print_register(CONFIGB);
		print_register(ECONTROL);
		printk("\n");
		io_write(oectr, ECONTROL);
	}
#undef print_register
	dump_parport_state("begin init", this.port);
#endif /* DEBUG_PARPORT */

	/* We need the ECR to be present */
	if (! parport_ECR_present()) {
		printk(KERN_INFO PPIP32 "ECR not found\n");
		return -ENODEV;
	}

	/* Note: the `base' parameter is not used, we just put here
	   some arbitrary value */
	this.port = parport_register_port (PARPORT_IP32_IO, /* base */
					   PARPORT_IRQ_NONE, /* irq */
					   PARPORT_DMA_NONE, /* dma */
					   &parport_ip32_ops);
	if (this.port == NULL) {
		printk(KERN_INFO PPIP32 "parport_register_port failed\n");
		return -ENOMEM;
	}
	this.port->modes = 0;
	this.port->base_hi = PARPORT_IP32_IOHI;
	this.ctr_writable = 0x2f;

	parport_SPP_supported();
	parport_PS2_supported();
	parport_ECP_supported();

/*	this.port->modes |= PARPORT_MODE_EPP; */
/*	this.port->modes |= PARPORT_MODE_ECP; */
/*	this.port->modes |= PARPORT_MODE_COMPAT; */
/*	this.port->modes |= PARPORT_MODE_DMA; */
/* 	this.port->irq = MACEISA_PARALLEL_IRQ;	 */

#if 0
	if (request_irq (this.port->irq, parport_ip32_interrupt,
			 0, this.port->name, this.port)) {
		printk (KERN_WARNING "%s: irq %d in use, "
			"resorting to polled operation\n",
			this.port->name, this.port->irq);
		this.port->irq = PARPORT_IRQ_NONE;
		this.port->dma = PARPORT_DMA_NONE;
	}
	request_irq (50, parport_ip32_interrupt,
		     0, this.port->name, this.port);
	request_irq (51, parport_ip32_interrupt,
		     0, this.port->name, this.port);
	request_irq (52, parport_ip32_interrupt,
		     0, this.port->name, this.port);
#endif

	/* Print essential informations */
	printk(KERN_INFO "%s: PC-style (IP32)", this.port->name);
	printk(" irq=");
	if (this.port->irq != PARPORT_IRQ_NONE) {
		printk("%d", this.port->irq);
	} else {
		printk("(none)");
	}
	printk(" dma=");
	if (this.port->dma != PARPORT_DMA_NONE) {
		printk("%d", this.port->dma);
		this.port->modes |= PARPORT_MODE_DMA;
	} else {
		printk("(none)");
	}
	printk(" [");
#define printmode(x)					\
	if (this.port->modes & PARPORT_MODE_##x)	\
		printk("%s%s", f++? ",": "", #x)
	{
		int f = 0;
		printmode(PCSPP);
		printmode(TRISTATE);
		printmode(COMPAT);
		printmode(EPP);
		printmode(ECP);
		printmode(DMA);
	}
#undef printmode
#ifndef CONFIG_PARPORT_1284
	printk ("(,...)");
#endif /* CONFIG_PARPORT_1284 */
	printk("]\n");

	/* Initialise the port with sensible values */  
	pr_debug1("%s: INIT_ECR=0x%02x, INIT_CTR=0x%02x\n",
		  this.port->name, INIT_ECR, INIT_CTR);
        io_write(INIT_ECR, ECONTROL);
	parport_ip32_write_control(this.port, INIT_CTR);
	parport_ip32_write_data(this.port, 0x00);

	dump_parport_state("end init", this.port);
	parport_announce_port (this.port);

	return 0;
}

static void __exit parport_ip32_exit(void)
{
	parport_remove_port(this.port);
#if 0
	if (this.port->irq != PARPORT_IRQ_NONE)
		free_irq(this.port->irq, this.port);
	free_irq(50, this.port);
	free_irq(51, this.port);
	free_irq(52, this.port);
#endif
	parport_put_port(this.port);  
	RELEASE_MACE_POINTER();
}

module_init(parport_ip32_init);
module_exit(parport_ip32_exit);

/* Inform (X)Emacs about preferred coding style
 *
 * Local Variables:
 * mode: c
 * eval: (c-set-style "K&R")
 * tab-width: 8
 * indent-tabs-mode: t
 * c-basic-offset: 8
 * ispell-local-dictionary: american
 * End:
 */
