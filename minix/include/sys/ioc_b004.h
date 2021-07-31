/*
 *	INMOS B004/B008 Transputer TRAM Motherboard character device
 */

#ifndef _S_I_B004_H
#define _S_I_B004_H

#include <minix/ioctl.h>

#define B004	4
#define B008	8

struct b004_flags {
  unsigned int  b004_board;
  unsigned int  b004_timeout;
  unsigned char b004_readable;
  unsigned char b004_writeable;
  unsigned char b004_error;
  unsigned char b004_dma_ok;
};

#define B004GETTIMEOUT	_IOR ('T',  1, int)
#define B004SETTIMEOUT	_IOW ('T',  2, int)
#define B004TIMEOUT	_IO  ('T',  3)
#define B004BOARDTYPE	_IO  ('T',  4)
#define B004READABLE	_IO  ('T',  5)
#define B004WRITEABLE	_IO  ('T',  6)
#define B004ERROR	_IO  ('T',  7)
#define B004GETFLAGS	_IOR ('T',  8, struct b004_flags)
#define B004DMADISABLE	_IO  ('T',  9)
#define B004DMAENABLE	_IO  ('T', 10)
#define B004RESET	_IO  ('T', 11)
#define B004ANALYSE	_IO  ('T', 12)

#endif /* _S_I_B004_H */
