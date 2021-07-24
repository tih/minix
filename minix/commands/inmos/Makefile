DEFS= -DMINIX -DDEFAULTLINK=\"/dev/link0\" -DBOARD_ID=B008
CPPFLAGS+= ${DEFS}

PROGS= ispy mtest iserver

SRCS.ispy= check.c cklib.c b004link.c
SRCS.mtest= mtest.c cklib.c b004link.c
SRCS.iserver= iserver.c filec.c hostc.c serverc.c b004link.c

.include <bsd.prog.mk>
