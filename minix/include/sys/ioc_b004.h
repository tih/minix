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

#define B004RESET	_IO  ('T',  1)
#define B004ANALYSE	_IO  ('T',  2)
#define B004GETFLAGS	_IOR ('T',  3, struct b004_flags)
#define B004GETTIMEOUT	_IOR ('T',  4, int)
#define B004SETTIMEOUT	_IOW ('T',  5, int)
#define B004ERROR	_IO  ('T',  6)
#define B004READABLE	_IO  ('T',  7)
#define B004WRITEABLE	_IO  ('T',  8)
#define B004TIMEOUT	_IO  ('T',  9)
#define B004NODMA	_IO  ('T', 10)

#ifdef PERFDATA

#define PERFMAXLEN 64

struct perfdata {
  int threshold;
  struct {
    int count;
    int ticks;
  } r[PERFMAXLEN];
  struct {
    int count;
    int ticks;
  } w[PERFMAXLEN];
};

#define B004GETPERF	_IOR ('T',  11, struct perfdata)

#endif /* PERFDATA */

#endif /* _S_I_B004_H */
