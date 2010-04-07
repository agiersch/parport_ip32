/* Debugging routines for parport_ip32
 *
 * Copyright (C) 2005-2007 Arnaud Giersch.
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

/* DEBUG_PARPORT_IP32
 *	0	disable debug
 *	1	standard level: pr_debug1 is enabled
 *	2	parport_ip32_dump_state is enabled
 *	>=3	verbose level: pr_debug is enabled
 */

#ifndef PARPORT_IP32_DEBUG_H
#define PARPORT_IP32_DEBUG_H

/*
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
	struct parport_ip32_private * const priv = p->physport->private_data;
	unsigned int i;

	printk(KERN_DEBUG PPIP32 "%s: state (%s):\n", p->name, str);
	{
		static const char ecr_modes[8][4] = {"SPP", "PS2", "PPF",
						     "ECP", "EPP", "???",
						     "TST", "CFG"};
		unsigned int ecr = ioread8(priv->regs.ecr);
		printk(KERN_DEBUG PPIP32 "    ecr=0x%02x", ecr);
		printk(" %s",
		       ecr_modes[(ecr & ECR_MODE_MASK) >> ECR_MODE_SHIFT]);
		if (ecr & ECR_nERRINTR)
			printk(",nErrIntrEn");
		if (ecr & ECR_DMAEN)
			printk(",dmaEn");
		if (ecr & ECR_SERVINTR)
			printk(",serviceIntr");
		if (ecr & ECR_F_FULL)
			printk(",f_full");
		if (ecr & ECR_F_EMPTY)
			printk(",f_empty");
		printk("\n");
	}
	if (show_ecp_config) {
		unsigned int oecr, cnfgA, cnfgB;
		oecr = ioread8(priv->regs.ecr);
		iowrite8(ECR_MODE_PS2, priv->regs.ecr);
		iowrite8(ECR_MODE_CFG, priv->regs.ecr);
		cnfgA = ioread8(priv->regs.cnfgA);
		cnfgB = ioread8(priv->regs.cnfgB);
		iowrite8(ECR_MODE_PS2, priv->regs.ecr);
		iowrite8(oecr, priv->regs.ecr);
		printk(KERN_DEBUG PPIP32 "    cnfgA=0x%02x", cnfgA);
		printk(" ISA-%s", (cnfgA & CNFGA_IRQ) ? "Level" : "Pulses");
		switch (cnfgA & CNFGA_ID_MASK) {
		case CNFGA_ID_8:
			printk(",8 bits");
			break;
		case CNFGA_ID_16:
			printk(",16 bits");
			break;
		case CNFGA_ID_32:
			printk(",32 bits");
			break;
		default:
			printk(",unknown ID");
			break;
		}
		if (!(cnfgA & CNFGA_nBYTEINTRANS))
			printk(",ByteInTrans");
		if ((cnfgA & CNFGA_ID_MASK) != CNFGA_ID_8)
			printk(",%d byte%s left", cnfgA & CNFGA_PWORDLEFT,
			       ((cnfgA & CNFGA_PWORDLEFT) > 1) ? "s" : "");
		printk("\n");
		printk(KERN_DEBUG PPIP32 "    cnfgB=0x%02x", cnfgB);
		printk(" irq=%u,dma=%u",
		       (cnfgB & CNFGB_IRQ_MASK) >> CNFGB_IRQ_SHIFT,
		       (cnfgB & CNFGB_DMA_MASK) >> CNFGB_DMA_SHIFT);
		printk(",intrValue=%d", !!(cnfgB & CNFGB_INTRVAL));
		if (cnfgB & CNFGB_COMPRESS)
			printk(",compress");
		printk("\n");
	}
	for (i = 0; i < 2; i++) {
		unsigned int dcr = i ? priv->dcr_cache
				     : ioread8(priv->regs.dcr);
		printk(KERN_DEBUG PPIP32 "    dcr(%s)=0x%02x",
		       i ? "soft" : "hard", dcr);
		printk(" %s", (dcr & DCR_DIR) ? "rev" : "fwd");
		if (dcr & DCR_IRQ)
			printk(",ackIntEn");
		if (!(dcr & DCR_SELECT))
			printk(",nSelectIn");
		if (dcr & DCR_nINIT)
			printk(",nInit");
		if (!(dcr & DCR_AUTOFD))
			printk(",nAutoFD");
		if (!(dcr & DCR_STROBE))
			printk(",nStrobe");
		printk("\n");
	}
#define sep (f++ ? ',' : ' ')
	{
		unsigned int f = 0;
		unsigned int dsr = ioread8(priv->regs.dsr);
		printk(KERN_DEBUG PPIP32 "    dsr=0x%02x", dsr);
		if (!(dsr & DSR_nBUSY))
			printk("%cBusy", sep);
		if (dsr & DSR_nACK)
			printk("%cnAck", sep);
		if (dsr & DSR_PERROR)
			printk("%cPError", sep);
		if (dsr & DSR_SELECT)
			printk("%cSelect", sep);
		if (dsr & DSR_nFAULT)
			printk("%cnFault", sep);
		if (!(dsr & DSR_nPRINT))
			printk("%c(Print)", sep);
		if (dsr & DSR_TIMEOUT)
			printk("%cTimeout", sep);
		printk("\n");
	}
#undef sep
}
#else /* DEBUG_PARPORT_IP32 < 2 */
#define parport_ip32_dump_state(...)	do { } while (0)
#endif

/*
 * CHECK_EXTRA_BITS - track and log extra bits
 * @p:		pointer to &struct parport
 * @b:		byte to inspect
 * @m:		bit mask of authorized bits
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
#define CHECK_EXTRA_BITS(...)	do { } while (0)
#endif

#endif /* !PARPORT_IP32_DEBUG_H */
