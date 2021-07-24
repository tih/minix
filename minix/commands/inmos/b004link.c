
/*
 --   ---------------------------------------------------------------------------
 --
 --      link.c   -   INMOS standard link interface
 --
 --   ---------------------------------------------------------------------------
*/

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/ioc_b004.h>

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/errno.h>

#include "iserver.h"

#define NULL_LINK -1

extern int errno;

static int ActiveLink = NULL_LINK;
static int TheCurrentTimeout = -1;

/*
 *
 *   OpenLink
 *
 *   Ready the link associated with `Name'.
 *   If `Name' is NULL or "" then any free link can be used.
 *   Returns any positive integer as a link id or
 *   a negative value if the open fails.
 *
 */

int OpenLink ( Name )
   char *Name;
{
   static char     DefaultDevice[] = "/dev/link0";

   /* Already open ? */
   if (ActiveLink != NULL_LINK)
      return ER_LINK_CANT;

   /* Use default name ? */
   if ((Name == NULL) || (*Name == '\0')) {
      if ((ActiveLink = open(DefaultDevice, O_RDWR)) >= 0)
         return ActiveLink;
   }
   else {
      if ((ActiveLink = open(Name, O_RDWR)) >= 0)
         return ActiveLink;
   }

   return ER_LINK_CANT;
}



/*
 *
 *   CloseLink
 *
 *   Close the active link `LinkId'.
 *   Returns 1 on success or negative if the close failed.
 *
 */

int CloseLink ( LinkId )
   int LinkId;
{
   if (LinkId != ActiveLink)
      return -1;

   close(ActiveLink);
   ActiveLink = NULL_LINK;

   return SUCCEEDED;
}



/*
 *
 *   ReadLink
 *
 *   Read `Count' chars into `Buffer' from the specified link.
 *   LinkId is a vaild link identifier, opened with OpenLink.
 *   `Timeout' is a non negative integer representing tenths
 *   of a second.  A `Timeout' of zero is an infinite timeout.
 *   The timeout is for the complete operation.
 *   If `Timeout' is positive then ReadLink may return having
 *   read less that the number of chars asked for.
 *   Returns the number of chars placed in `Buffer' (which may
 *   be zero) or negative to indicate an error.
 *
 */
 
int ReadLink ( LinkId, Buffer, Count, Timeout )
   int LinkId;
   char *Buffer;
   unsigned int Count;
   int Timeout;
{
   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (Count < 1)
      return ER_LINK_CANT;

   if (Timeout && (Timeout != TheCurrentTimeout)) {
      if (ioctl(LinkId, B004SETTIMEOUT, &Timeout) != 0)
         return ER_LINK_BAD;

      TheCurrentTimeout = Timeout;
   }

   return read(LinkId, Buffer, Count);
}



/*
 *
 *   WriteLink
 *
 *   Write `Count' chars from `Buffer' to the specified link.
 *   LinkId is a vaild link identifier, opened with OpenLink.
 *   `Timeout' is a non negative integer representing tenths
 *   of a second.  A `Timeout' of zero is an infinite timeout.
 *   The timeout is for the complete operation.
 *   If `Timeout' is positive then WriteLink may return having
 *   written less that the number of chars asked for.
 *   Returns the number of chars actually written (which may
 *   be zero) or negative to indicate an error.
 *
 */
 
int WriteLink ( LinkId, Buffer, Count, Timeout )
   int LinkId;
   char *Buffer;
   unsigned int Count;
   int Timeout;
{
   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (Count < 1)
      return ER_LINK_BAD;

   if (Timeout && (Timeout != TheCurrentTimeout)) {
      if (ioctl(LinkId, B004SETTIMEOUT, &Timeout) != 0)
         return ER_LINK_BAD;

      TheCurrentTimeout = Timeout;
   }

   return write(LinkId, Buffer, Count);
}



/*
 *
 *   ResetLink
 *
 *   Reset the subsystem associated with the specified link.
 *   Returns 1 if the reset is successful, negative otherwise.
 *
 */
 
int ResetLink ( LinkId )
   int LinkId;
{
   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (ioctl(LinkId, B004RESET) != 0)
      return ER_LINK_BAD;

   return SUCCEEDED;
}



/*
 *
 *   AnalyseLink
 *
 *   Analyse the subsystem associated with the specified link.
 *   Returns 1 if the analyse is successful, negative otherwise.
 *
 */
 
int AnalyseLink ( LinkId )
   int LinkId;
{
   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (ioctl(LinkId, B004ANALYSE) != 0)
      return ER_LINK_BAD;

   return SUCCEEDED;
}



/*
 *
 *   TestError
 *
 *   Test the error status associated with the specified link.
 *   Returns 1 if error is set, 0 if it is not and
 *   negative to indicate an error.
 *
 */
 
int TestError ( LinkId )
   int LinkId;
{
   struct b004_flags flag;

   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (ioctl(LinkId, B004GETFLAGS, &flag) != 0)
      return ER_LINK_BAD;

   return flag.b004_error;
}



/*
 *
 *   TestRead
 *
 *   Test input status of the link.
 *   Returns 1 if ReadLink will return one byte without timeout,
 *   0 if it may not and negative to indicate an error.
 *
 */

int TestRead ( LinkId )
   int LinkId;
{
   struct b004_flags flag;

   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (ioctl(LinkId, B004GETFLAGS, &flag) != 0)
      return ER_LINK_BAD;

   return flag.b004_readable;
}



/*
 *
 *   TestWrite
 *
 *   Test output status of the link.
 *   Returns 1 if WriteLink can write one byte without timeout,
 *   0 if it may not and negative to indicate an error.
 *
 */

int TestWrite ( LinkId )
   int LinkId;
{
   struct b004_flags flag;

   if (LinkId != ActiveLink)
      return ER_LINK_BAD;

   if (ioctl(LinkId, B004GETFLAGS, &flag) != 0)
      return ER_LINK_BAD;

   return flag.b004_writeable;
}



/*
 *   Eof
 */
