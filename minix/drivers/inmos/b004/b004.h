#ifndef __B004_H
#define __B004_H

#define B004_IRQ	5
#define B004_BASE	0x150

#define B004_IDR	(B004_BASE +  0x0)	/* Input data register */
#define B004_ODR	(B004_BASE +  0x1)	/* Output data register */
#define B004_ISR	(B004_BASE +  0x2)	/* Input status register */
#define B004_OSR	(B004_BASE +  0x3)	/* Output status register */
#define B004_RESET	(B004_BASE + 0x10)	/* Reset register (write) */
#define B004_ERROR	(B004_BASE + 0x10)	/* Error register (read) */
#define B004_ANALYSE	(B004_BASE + 0x11)	/* Analyse register */
#define B008_DMA	(B004_BASE + 0x12)	/* B008 DMA request register */
#define B008_INT	(B004_BASE + 0x13)	/* B008 interrupt ctl. reg. */

#define B004_READY	0x1
#define B004_INT_ENA	0x2
#define B004_INT_DIS	0x0

#define B008_DMAINT_ENA	0x1
#define B008_ERRINT_ENA	0x2
#define B008_OUTINT_ENA	0x4
#define B008_INPINT_ENA	0x8
#define B008_INT_MASK	0xf
#define B008_INT_DIS	0x0

#define B004		4
#define B008		8

#define DMA_SIZE	4096		/* Max bytes per DMA transfer */
#define DMA_ALIGN	(64*1024)	/* DMA must stay within a click */

#define B004_IO_DELAY	10000		/* 10 milliseconds (usleep) */
#define B004_RST_DELAY	100000		/* 100 milliseconds (usleep) */

#endif /* __B004_H */
