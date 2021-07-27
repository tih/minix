# Makefile for the b004 driver.
PROG=	b004
SRCS=	b004.c

FILES=${PROG}.conf
FILESNAME=${PROG}
FILESDIR= /etc/system.conf.d

CPPFLAGS+= -DPERFDATA

DPADD+=	${LIBCHARDRIVER} ${LIBSYS}
LDADD+=	-lchardriver -lsys

.include <minix.service.mk>
