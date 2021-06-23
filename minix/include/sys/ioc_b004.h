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
};

#define B004RESET	_IO  ('b', 1)
#define B004ANALYSE	_IO  ('b', 2)
#define B004GETFLAGS	_IOR ('b', 3, struct b004_flags)
#define B004GETTIMEOUT	_IOR ('b', 4, int)
#define B004SETTIMEOUT	_IOW ('b', 5, int)

#endif /* _S_I_B004_H */
