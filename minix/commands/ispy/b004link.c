
/*
 --   ---------------------------------------------------------------------------
 --
 --      ISERVER  -  INMOS standard file server
 --
 --      b004link.c
 --
 --      Link module for B004 type boards in IBM PCs running MINIX
 --
 --      Copyright (c) INMOS Ltd., 1988.
 --      Copyright (c) Christoph Niemann, 1993
 --      Copyright (c) Tom Ivar Helbekkmo, 2021
 --      All Rights Reserved.
 --
 --   ---------------------------------------------------------------------------
*/

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/ioc_b004.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "linkio.h"
#include "inmos.h"


#define NULL_LINK -1

PRIVATE LINK ActiveLink = NULL_LINK;
PRIVATE int CurrentTimeout = -1;

/*
 *   Open Link
 */

PUBLIC LINK OpenLink ( Name )
   char *Name;
{
   
   if ( ActiveLink != NULL_LINK )
      return( ER_LINK_CANT );
   if ( *Name == '\0')
   {
     if ( ( ActiveLink = open( "/dev/link0" , O_RDWR ) ) < 0 )
     {
        perror( "/dev/link");
        return( ER_LINK_BUSY );
     }
   }
   else
   {
     if ( ( ActiveLink = open( Name , O_RDWR ) ) < 0 )
     {
        perror( Name );
        return( ER_LINK_BUSY );
     }
   }

   /* ioctl(ActiveLink, B004NODMA); */

   return(ActiveLink);
}




/*
 *   Close Link
 */

int CloseLink ( LinkId )
   int LinkId;
{
   if ( LinkId != ActiveLink )
      return( -1 );
   close(ActiveLink);
   ActiveLink = NULL_LINK;
   return(TRUE);
}





/*
 *   Read Link
 */

int ReadLink ( LinkId, Buffer, Count, Timeout )
   LINK LinkId;
   char *Buffer;
   unsigned int Count;
   int Timeout;
{
   if (Timeout && (Timeout != CurrentTimeout)) {
      if (ioctl(LinkId, B004SETTIMEOUT, &Timeout) != 0)
         return -1;

      CurrentTimeout = Timeout;
   }

  return (read(LinkId, Buffer, Count));
}




/*
 *   Write Link
 */

int WriteLink ( LinkId, Buffer, Count, Timeout )
   LINK LinkId;
   char *Buffer;
   unsigned int Count;
   int Timeout;
{
   if (Timeout && (Timeout != CurrentTimeout)) {
      if (ioctl(LinkId, B004SETTIMEOUT, &Timeout) != 0)
         return -1;

      CurrentTimeout = Timeout;
   }

  return (write(LinkId, Buffer, Count));
}



/*
 *   Reset Link
 */

PUBLIC int ResetLink ( LinkId )
   LINK LinkId;
{

   if ( LinkId != ActiveLink )
      return( -1 );
   if ( ioctl( LinkId, B004RESET) != 0 )
      return( -1 );

   return( 1 );
}



/*
 *   Analyse Link
 */
 
PUBLIC int AnalyseLink ( LinkId )
   LINK LinkId;
{

   if ( LinkId != ActiveLink )
      return( -1 );
   if ( ioctl( LinkId, B004ANALYSE) != 0 )
      return( -1 );

   return( 1 );
}



/*
 *   Test Error
 */
 
PUBLIC int TestError ( LinkId )
   LINK LinkId;
{
   struct b004_flags flag;

   if ( LinkId != ActiveLink )
      return( -1 );

   if (ioctl(LinkId, B004GETFLAGS, &flag) != 0)
      return( -1 );

   return (flag.b004_error);
}




/*
 *   Test Read
 */

PUBLIC int TestRead ( LinkId )
   LINK LinkId;
{
   struct b004_flags flag;

   if ( LinkId != ActiveLink )
      return( -1 );

   if (ioctl(LinkId, B004GETFLAGS, &flag) != 0)
      return( -1 );

   return (flag.b004_readable);
}




/*
 *   Test Write
 */

PUBLIC int TestWrite ( LinkId )
   LINK LinkId;
{
   struct b004_flags flag;

   if ( LinkId != ActiveLink )
      return( -1 );

   if (ioctl(LinkId, B004GETFLAGS, &flag) != 0)
      return( -1 );

   return (flag.b004_writeable);
}



/*
 *   Eof
 */


