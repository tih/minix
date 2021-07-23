DEFS= -DMINIX -DDEFAULTLINK=/dev/link0
CPPFLAGS+= ${DEFS}

PROGS= ispy mtest
SRCS.ispy= check.c cklib.c b004link.c
SRCS.mtest= mtest.c cklib.c b004link.c

.include <bsd.prog.mk>
