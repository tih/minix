/*	sys/ioc_b004.h - Inmos B004 ioctls */

#ifndef _S_I_B004_H
#define _S_I_B004_H

#include <minix/ioctl.h>

#define B004	4
#define B008	8

struct b004_flags {
  unsigned int  b004_board;
  unsigned char b004_readable;
  unsigned char b004_writeable;
  unsigned char b004_error;
  unsigned char b004_readintr;
  unsigned char b004_writeintr;
  unsigned char b008_intrmask;
};

#define B004RESET	_IO  ('b', 1)
#define B004ANALYSE	_IO  ('b', 2)
#define B004GETFLAGS	_IOR ('b', 3, struct b004_flags)

#endif /* _S_I_B004_H */
