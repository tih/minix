#ifndef __B004_H
#define __B004_H

#define B004_MESSAGE "b004, World!\n"

#define B004_IDR	 0x0		/* Input data register */
#define B004_ODR	 0x1		/* Output data register */
#define B004_ISR	 0x2		/* Input status register */
#define B004_OSR	 0x3		/* Output status register */
#define B004_RESET	0x10		/* Reset register (write) */
#define B004_ERROR	0x10		/* Error register (read) */
#define B004_ANALYSE	0x11		/* Analyse register */

#define B008_DMA	0x12		/* B008 DMA request register */
#define B008_INT	0x13		/* B008 interrupt control register */

#define B004_IRQ	5

#define DMA_SIZE	4096		/* Max bytes per DMA transfer */
#define DMA_ALIGN	(64*1024)	/* DMA must stay within a click */


#endif /* __B004_H */
