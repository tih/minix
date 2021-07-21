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
#define B004_HAS_ERROR	0x1
#define B004_INT_ENA	0x2
#define B004_INT_DIS	0x0

#define B004_RST_DELAY	100000		/* 100 milliseconds (usleep) */

#define B008_DMAINT_ENA	0x1
#define B008_ERRINT_ENA	0x2
#define B008_OUTINT_ENA	0x4
#define B008_INPINT_ENA	0x8
#define B008_INT_MASK	0xf
#define B008_INT_DIS	0x0

#define B008_DMAWRITE	0x0		/* DMA transfer to B008 board */
#define B008_DMAREAD	0x1		/* DMA transfer from B008 board */

#define LINKBUF_SIZE	(64*1024)

#define DMA_ALIGN	(64*1024)	/* DMA must stay within a click */

#define DMA_THRESHOLD	16		/* Polled I/O for smaller xfers */

#define DMA_ADDR	0x002		/* DMA chan 1 port; low 16 addr bits */
#define DMA_TOP		0x083		/* DMA chan 1 port; high 8 addr bits */
#define DMA_COUNT	0x003		/* DMA chan 1 port; byte count - 1 */

#define DMA_FLIPFLOP	0x00C		/* DMA byte pointer flip-flop */
#define DMA_MODE	0x00B		/* DMA mode port */
#define DMA_INIT	0x00A		/* DMA init port */

#define DMA_MASK	0x05		/* set mask for DMA channel 1 */
#define DMA_UNMASK	0x01		/* unset mask for DMA channel 1 */

#define DMA_READ	0x05		/* DMA read; chan 1 demand mode */
#define DMA_WRITE	0x09		/* DMA write; chan 1 demand mode */

#endif /* __B004_H */
