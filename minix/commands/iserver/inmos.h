
/*
 --   ---------------------------------------------------------------------------
 --
 --      ISERVER  -  INMOS standard file server
 --
 --      inmos.h
 --
 --      Some global definitions
 --
 --      Copyright (c) INMOS Ltd., 1988.
 --      All Rights Reserved.
 --
 --   ---------------------------------------------------------------------------
*/



#ifndef _INMOS_H
#define _INMOS_H
#endif

#ifndef BYTE
#define BYTE unsigned char
#endif
#define BOOL int

#ifdef MSC
#define INT32 long
#else
#define INT32 int
#endif

#ifdef MINIX
#include <sys/endian.h>
#if BYTE_ORDER == BIG_ENDIAN
#undef LITTLE_ENDIAN
#else
#undef BIG_ENDIAN
#endif
#else
#ifdef sun3
#define BIG_ENDIAN
#else
#ifdef sun4
#define BIG_ENDIAN
#else
#define LITTLE_ENDIAN
#endif
#endif
#endif

#ifndef TRUE 
#define TRUE 1
#endif
#ifndef FALSE 
#define FALSE 0
#endif

#ifndef PUBLIC
#define PUBLIC
#endif
#ifndef PRIVATE
#define PRIVATE static
#endif
#define VOID void
#define EXTERN extern

#define LINK int

static char Copyright[] = "Copyright (c) INMOS Ltd, 1988.  All Rights Reserved.\n";


/*
 *   Eof
 */

