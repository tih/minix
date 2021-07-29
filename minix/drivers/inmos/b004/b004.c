/*
 *	INMOS B004/B008 Transputer TRAM Motherboard character device
 *
 *	This driver implements the B004 interface, using the I/O port
 *	block starting at 0x150, for polled communication with the
 *	Transputer network.  It will also detect the presence of the
 *	B008 extensions, and use those to use DMA for transfers over
 *	a minimum size, worth the DMA setup overhead.
 *
 *	Only one device is supported, and half duplex communication.
 */

#include <minix/drivers.h>
#include <minix/chardriver.h>
#include <sys/ioc_b004.h>
#include <minix/ds.h>

#include <stdio.h>
#include <stdlib.h>

#include "b004.h"

static int b004_open(devminor_t minor, int access, endpoint_t user_endpt);

static int b004_close(devminor_t minor);

static ssize_t b004_read(devminor_t minor, u64_t position, endpoint_t endpt,
			 cp_grant_id_t grant, size_t size, int flags,
			 cdev_id_t id);

static ssize_t b004_write(devminor_t minor, u64_t position, endpoint_t endpt,
			  cp_grant_id_t grant, size_t size, int flags,
			  cdev_id_t id);

static int b004_ioctl(devminor_t minor, unsigned long request,
		      endpoint_t endpt, cp_grant_id_t grant, int flags,
		      endpoint_t user_endpt, cdev_id_t id);

static int b004_cancel(devminor_t minor, endpoint_t endpt, cdev_id_t id);

static void b004_alarm(clock_t stamp);

static void b004_intr(unsigned int mask);

static void b004_probe(void);
static void b004_initialize(void);
static void b004_reset(void);
static void b004_analyse(void);

static void dma_read(void);
static void dma_write(void);
static int dma_transfer(phys_bytes dmabuf_phys, size_t count, int do_write);

static void sef_local_startup(void);
static int sef_cb_init(int type, sef_init_info_t *info);
static int sef_cb_lu_state_save(int, int);
static int lu_state_restore(void);

static struct chardriver b004_tab = {
  .cdr_open   = b004_open,
  .cdr_close  = b004_close,
  .cdr_read   = b004_read,
  .cdr_write  = b004_write,
  .cdr_ioctl  = b004_ioctl,
  .cdr_cancel = b004_cancel,
  .cdr_alarm  = b004_alarm,
  .cdr_intr   = b004_intr
};

static int board_type = 0;
static int board_busy = 1;

static unsigned char *linkbuf;
static int linkbuf_busy = 0;

static unsigned char *dmabuf;
static phys_bytes dmabuf_phys;
static size_t dmabuf_len = 0;
static int dma_available = 0;
static int dma_disabled = 0;

static struct {
  endpoint_t endpt;
  cdev_id_t id;
  cp_grant_id_t grant;
  int writing;
  size_t size;
  size_t done;
  size_t chunk;
} dma;

static int probe_active = 0;

static u32_t system_hz;
static int io_timeout = 0;

static int irq_hook_id;

/*
 *	OPEN: permits only one client at a time.
 */

static int b004_open(devminor_t UNUSED(minor), int UNUSED(access),
		     endpoint_t UNUSED(user_endpt)) {

  if (board_busy)
    return EAGAIN;

  board_busy = 1;

  return OK;
}

/*
 *	CLOSE: resets timeout to default; re-enables DMA if disabled.
 */

static int b004_close(devminor_t UNUSED(minor)) {

  io_timeout = system_hz;
  if (dma_disabled)
    dma_available = 1;
  dma.endpt = 0;

  board_busy = 0;

  return OK;
}

/*
 *	READ: if DMA available, sets up and initiates a DMA transfer,
 *	      and suspends the caller.  Otherwise, does a polled read.
 */

static ssize_t b004_read(devminor_t UNUSED(minor), u64_t UNUSED(position),
			 endpoint_t endpt, cp_grant_id_t grant, size_t size,
			 int UNUSED(flags), cdev_id_t id) {
  int ret = OK, b;
  size_t i, j, copied;
  clock_t now, deadline;

  if (size <= 0)
    return EINVAL;

  if (linkbuf_busy || (dma.endpt != 0))
    return EIO;

  if ((dma_available) && (size >= DMA_THRESHOLD)) {
    dma.endpt = endpt;
    dma.id = id;
    dma.grant = grant;
    dma.writing = 0;
    dma.size = size;
    dma.done = 0;
    dma.chunk = 0;
    sys_setalarm(io_timeout, 0);
    dma_read();
    return EDONTREPLY;
  }

  linkbuf_busy = 1;

  getuptime(&now, NULL, NULL);
  deadline = now + io_timeout;

  copied = 0;
  for (i = 0, j = 0; i < size; i++) {
    while (1) {
      sys_inb(B004_ISR, &b);
      if (b & B004_READY) {
	sys_inb(B004_IDR, &b);
	linkbuf[j++] = b;
	break;
      } else {
	if (io_timeout > 0) {
	  getuptime(&now, NULL, NULL);
	  if (now > deadline)
	    goto out;
	}
      }
    }
    if (j == LINKBUF_SIZE) {
      ret = sys_safecopyto(endpt, grant, copied,
			   (vir_bytes)linkbuf, LINKBUF_SIZE);
      j = 0;
      if (ret == OK)
	copied += LINKBUF_SIZE;
      else
	goto out;
    }
  }

 out:
  if (j > 0) {
    ret = sys_safecopyto(endpt, grant, copied, (vir_bytes)linkbuf, j);
    if (ret == OK)
      copied += j;
  }

  linkbuf_busy = 0;

  if (ret != OK)
    return ret;

  return copied;
}

/*
 *	WRITE: if DMA available, sets up and initiates a DMA transfer,
 *	       and suspends the caller.  Otherwise, does a polled write.
 */

static ssize_t b004_write(devminor_t UNUSED(minor), u64_t UNUSED(position),
			  endpoint_t endpt, cp_grant_id_t grant, size_t size,
			  int UNUSED(flags), cdev_id_t id) {
  int ret = OK, b;
  size_t i, j, copied, chunk;
  clock_t now, deadline;

  if (size <= 0)
    return EINVAL;

  if (linkbuf_busy || (dma.endpt != 0))
    return EIO;

  if ((dma_available) && (size >= DMA_THRESHOLD)) {
    dma.endpt = endpt;
    dma.id = id;
    dma.grant = grant;
    dma.writing = 1;
    dma.size = size;
    dma.done = 0;
    dma.chunk = 0;
    sys_setalarm(io_timeout, 0);
    dma_write();
    return EDONTREPLY;
  }

  linkbuf_busy = 1;

  getuptime(&now, NULL, NULL);
  deadline = now + io_timeout;

  copied = 0;
  for (i = 0, j = 0; i < size; i++) {
    if (j == 0) {
      chunk = MIN((size - copied), LINKBUF_SIZE);
      ret = sys_safecopyfrom(endpt, grant, copied, (vir_bytes)linkbuf, chunk);
      if (ret == OK)
	copied += chunk;
      else
	goto out;
    }
    while (1) {
      sys_inb(B004_OSR, &b);
      if (b & B004_READY) {
	sys_outb(B004_ODR, linkbuf[j++]);
	break;
      } else {
	if (io_timeout > 0) {
	  getuptime(&now, NULL, NULL);
	  if (now > deadline)
	    goto out;
	}
      }
    }
    if (j == LINKBUF_SIZE)
      j = 0;
  }

 out:
  linkbuf_busy = 0;

  if (ret != OK)
    return ret;

  return i;
}

/*
 *	DMA_READ: simple state machine for doing DMA read.  Called once
 *	          from READ, above, then one or more times from INTR.
 */

static void dma_read(void) {
  int ret = OK;

  if (dma.chunk > 0) {
    ret = sys_safecopyto(dma.endpt, dma.grant, dma.done,
			 (vir_bytes)dmabuf, dma.chunk);
    if (ret == OK) {
      dma.done += dma.chunk;
      dma.chunk = 0;
    }
  }

  if (dma.done == dma.size) {
    chardriver_reply_task(dma.endpt, dma.id, dma.size);
    dma.endpt = 0;
    return;
  }

  if (dma.chunk == 0) {
    dma.chunk = MIN((dma.size - dma.done), dmabuf_len);
    ret = dma_transfer(dmabuf_phys, dma.chunk, 0);
  }

  if (ret != OK) {
    chardriver_reply_task(dma.endpt, dma.id, ret);
    dma.endpt = 0;
  }
}

/*
 *	DMA_WRITE: simple state machine for doing DMA write.  Called once
 *	           from WRITE, above, then one or more times from INTR.
 */

static void dma_write(void) {
  int ret = OK;

  if (dma.chunk > 0) {
    dma.done += dma.chunk;
    dma.chunk = 0;
  }

  if (dma.done == dma.size) {
    chardriver_reply_task(dma.endpt, dma.id, dma.size);
    dma.endpt = 0;
    return;
  }

  if (dma.chunk == 0) {
    dma.chunk = MIN((dma.size - dma.done), dmabuf_len);
    ret = sys_safecopyfrom(dma.endpt, dma.grant, dma.done,
			   (vir_bytes)dmabuf, dma.chunk);
    if (ret == OK)
      ret = dma_transfer(dmabuf_phys, dma.chunk, 1);
  }

  if (ret != OK) {
    chardriver_reply_task(dma.endpt, dma.id, ret);
    dma.endpt = 0;
  }
}

/*
 *	DMA_TRANSFER: program the DMA controller for a transfer, then
 *		      enable interrupts and tell the B008 to start it.
 */

static int dma_transfer(phys_bytes dmabuf_phys, size_t count, int do_write) {
  pvb_pair_t byte_out[9];
  int ret;

  pv_set(byte_out[0], DMA_INIT, DMA_MASK);
  pv_set(byte_out[1], DMA_FLIPFLOP, 0);
  pv_set(byte_out[2], DMA_MODE, do_write ? DMA_WRITE : DMA_READ);
  pv_set(byte_out[3], DMA_ADDR, (unsigned) (dmabuf_phys >>  0) & 0xff);
  pv_set(byte_out[4], DMA_ADDR, (unsigned) (dmabuf_phys >>  8) & 0xff);
  pv_set(byte_out[5], DMA_TOP,  (unsigned) (dmabuf_phys >> 16) & 0xff);
  pv_set(byte_out[6], DMA_COUNT, (((count - 1) >> 0)) & 0xff);
  pv_set(byte_out[7], DMA_COUNT, (count - 1) >> 8);
  pv_set(byte_out[8], DMA_INIT, DMA_UNMASK);

  if ((ret = sys_voutb(byte_out, 9)) != OK)
    panic("dma_setup: failed to program DMA chip (%d)", ret);

  pv_set(byte_out[0], B004_ISR, B004_INT_ENA);
  pv_set(byte_out[1], B004_OSR, B004_INT_ENA);
  pv_set(byte_out[2], B008_INT, B008_DMAINT_ENA);

  if ((ret = sys_voutb(byte_out, 3)) != OK)
    panic("dma_setup: failed to enable interrupts (%d)", ret);

  sys_irqenable(&irq_hook_id);

  sys_outb(B008_DMA, do_write ? B008_DMAWRITE : B008_DMAREAD);

  return OK;
}

/*
 *	IOCTL: handle the various ioctl() calls the driver supports.
 */

static int b004_ioctl(devminor_t UNUSED(minor), unsigned long request,
		      endpoint_t endpt, cp_grant_id_t grant, int UNUSED(flags),
		      endpoint_t UNUSED(user_endpt), cdev_id_t UNUSED(id)) {
  int ret;
  unsigned int b;
  struct b004_flags flag;
  int timeout;

  switch (request) {
  case B004RESET:
    b004_reset();
    ret = OK;
    break;
  case B004ANALYSE:
    b004_analyse();
    ret = OK;
    break;
  case B004GETFLAGS:
    flag.b004_board = board_type;
    sys_inb(B004_ISR, &b);
    flag.b004_readable = b & B004_READY;
    sys_inb(B004_OSR, &b);
    flag.b004_writeable = b & B004_READY;
    sys_inb(B004_ERROR, &b);
    flag.b004_error = b & B004_HAS_ERROR;
    ret = sys_safecopyto(endpt, grant, 0, (vir_bytes) &flag, sizeof flag);
    break;
  case B004GETTIMEOUT:
    timeout = (io_timeout * 10) / system_hz;
    ret = sys_safecopyto(endpt, grant,
			 0, (vir_bytes)&timeout, sizeof timeout);
    break;
  case B004SETTIMEOUT:
    ret = sys_safecopyfrom(endpt, grant,
			   0, (vir_bytes)&timeout, sizeof timeout);
    if ((ret == OK) && (timeout >= 0))
      io_timeout = (timeout * system_hz) / 10;
    else
      ret = EINVAL;
    break;
  case B004ERROR:
    sys_inb(B004_ERROR, &b);
    ret = b & B004_HAS_ERROR;
    break;
  case B004READABLE:
    sys_inb(B004_ISR, &b);
    ret = b & B004_READY;
    break;
  case B004WRITEABLE:
    sys_inb(B004_OSR, &b);
    ret = b & B004_READY;
    break;
  case B004TIMEOUT:
    ret = (io_timeout * 10) / system_hz;
    break;
  case B004NODMA:
    dma_disabled |= dma_available;
    dma_available = 0;
    ret = OK;
    break;
  default:
    ret = EINVAL;
  }

  return ret;
}

/*
 *	CANCEL: if the indicated operation is still in progress, cancel
 *		it, log this, and return EINTR to the client.  If not,
 *		just ignore the CANCEL message; a response has been sent.
 */

static int b004_cancel(devminor_t UNUSED(minor),
			endpoint_t endpt, cdev_id_t id) {

  if (dma.endpt == endpt && dma.id == id) {
    sys_setalarm(0, 0);
    printf("b004: cancelling %d byte %s operation\n",
           dma.size, dma.writing ? "write" : "read");
    dma.endpt = 0;
    return EINTR;
  }

  return EDONTREPLY;
}

/*
 *	ALARM: if the indicated operation is still in progress, time it
 *	       out, log this, and return the count of bytes that have 
 *	       already been transfered.  Otherwise, ignore the alarm.
 */

static void b004_alarm(clock_t UNUSED(stamp)) {

  if (dma.endpt != 0) {
    printf("b004: timing out a %d byte %s operation\n",
           dma.size, dma.writing ? "write" : "read");
    chardriver_reply_task(dma.endpt, dma.id, dma.done);
    dma.endpt = 0;
  }
}

/*
 *	INTR: acknowledge each possible interrupt source, then run the
 *	      next step in the relevant state machine.  Note that this
 *	      is where DMA operation timeout alarms are canceled, and
 *	      that we also detect the interrupt from the experimental
 *	      DMA attempt during the initial probe, switching DMA on.
 */

static void b004_intr(unsigned int UNUSED(mask)) {
  pvb_pair_t byte_out[3];

  pv_set(byte_out[0], B004_ISR, B004_INT_DIS);
  pv_set(byte_out[1], B004_OSR, B004_INT_DIS);
  pv_set(byte_out[2], B008_INT, B008_INT_DIS);

  if (sys_voutb(byte_out, 3) != OK)
    panic("b004: failed to acknowledge interrupt");

  if (probe_active) {
    printf("b004: DMA verified; switching to B008 mode\n");
    board_type = B008;
    dma_available = 1;
    probe_active = 0;
    return;
  }

  if (dma.endpt == 0) {
    printf("b004: unexpected hardware interrupt\n");
    return;
  }

  if (dma.writing)
    dma_write();
  else
    dma_read();

  if (dma.endpt == 0)
    sys_setalarm(0, 0);

  return;
}

/*
 *	Live update: save state.  Not tested; not production ready.
 */

static int sef_cb_lu_state_save(int UNUSED(state), int UNUSED(flags)) {

  ds_publish_u32("board_type", board_type, DSF_OVERWRITE);
  ds_publish_u32("io_timeout", io_timeout, DSF_OVERWRITE);

  return OK;
}

/*
 *	Live update: restore state.  Not tested; not production ready.
 */

static int lu_state_restore() {
  u32_t value;

  if (ds_retrieve_u32("board_type", &value) == OK) {
    board_type = (int) value;
    ds_delete_u32("board_type");
  }

  if (ds_retrieve_u32("io_timeout", &value) == OK) {
    io_timeout = (int) value;
    ds_delete_u32("io_timeout");
  }

  return OK;
}

/*
 *	SEF startup routine; establish callbacks, and fire startup.
 */

static void sef_local_startup() {

  sef_setcb_init_fresh(sef_cb_init);
  sef_setcb_init_lu(sef_cb_init);
  sef_setcb_init_restart(sef_cb_init);

  sef_setcb_lu_state_save(sef_cb_lu_state_save);

  sef_startup();
}

/*
 *	SEF initialization: set up default timeout value, allocate
 *	buffers for communication (polled and DMA), and probe for the
 *	physical device.  Note the strategy for getting a sizeable
 *	low memory DMA buffer: prefer a properly aligned allocation,
 *	but accept a double size non-aligned one, and use an aligned
 *	part of it.  The latter is wasteful, but we want the space.
 */

static int sef_cb_init(int type, sef_init_info_t *UNUSED(info)) {
  int off, k;

  if (type == SEF_INIT_LU)
    lu_state_restore();

  if (sys_getinfo(GET_HZ, &system_hz, sizeof(system_hz), 0, 0) != OK)
    panic("sef_cb_init: couldn't get system HZ value");

  if (io_timeout == 0)
    io_timeout = system_hz;

  if (!(linkbuf = malloc(LINKBUF_SIZE)))
    panic("sef_cb_init: couldn't allocate link buffer");

  for (k = 64; k >= 1; k /= 2) {
    if ((dmabuf = alloc_contig(k * 1024, AC_LOWER16M | AC_ALIGN64K,
			       &dmabuf_phys)))
      break;
    if ((dmabuf = alloc_contig(2 * k * 1024, AC_LOWER16M | AC_ALIGN4K,
			       &dmabuf_phys)))
      break;
  }

  if (k == 0)
    panic("sef_cb_init: couldn't allocate DMA buffer");

  if (dmabuf_phys/DMA_ALIGN != (dmabuf_phys+dmabuf_len-1)/DMA_ALIGN) {
    off = dmabuf_phys % DMA_ALIGN;
    dmabuf += (DMA_ALIGN - off);
    dmabuf_phys += (DMA_ALIGN - off);
  }

  dmabuf_len = k * 1024;

  printf("b004: allocated a %d byte DMA buffer\n", dmabuf_len);

  dma.endpt = 0;

  if (board_type == 0)
    b004_probe();

  if (type != SEF_INIT_LU)
    chardriver_announce();

  if (board_type != 0)
    board_busy = 0;

  return OK;
}

/*
 *	PROBE: reset the expected B004 compatible hardware, then check
 *	       that the Output Status Register indicates being ready to
 *	       transmit.  Once this is established, verify that we can
 *	       enable interrupts properly, then disable them, and fire
 *	       off a blind DMA write of one byte.  If this works, the
 *	       interrupt handler will detect it, and enable DMA use.
 */

void b004_probe(void) {
  unsigned int b;

  b004_reset();

  if (sys_outb(B004_OSR, 0) == OK) {
    if (sys_inb(B004_OSR, &b) == OK) {
      if (b & B004_READY) {
	board_type = B004;
	irq_hook_id = B004_IRQ;
	sys_outb(B004_ISR, B004_INT_ENA);
	sys_outb(B004_OSR, B004_INT_ENA);
	sys_outb(B008_INT, B008_DMAINT_ENA);
	if ((sys_irqsetpolicy(B004_IRQ, 0, &irq_hook_id) != OK) ||
	    (sys_irqenable(&irq_hook_id) != OK))
	  panic("sef_cb_init: couldn't enable interrupts");
	sys_irqdisable(&irq_hook_id);
	sys_outb(B004_ISR, B004_INT_DIS);
	sys_outb(B004_OSR, B004_INT_DIS);
	sys_outb(B008_INT, B008_INT_DIS);
	printf("b004: probe found a B004 compatible device.\n");
	board_busy = 0;
	probe_active = 1;
	dmabuf[0] = 0;
	dma_transfer(dmabuf_phys, 1, 1);
      }
    }
  }
}

/*
 *	RESET: the I/O operations to reset a B004 compatible device.
 */

void b004_reset(void) {

  sys_outb(B004_ANALYSE, 0);
  usleep(B004_RST_DELAY);
  sys_outb(B004_RESET, 0);
  usleep(B004_RST_DELAY);
  sys_outb(B004_RESET, 1);
  usleep(B004_RST_DELAY);
  sys_outb(B004_RESET, 0);
  usleep(B004_RST_DELAY);
}

/*
 *	ANALYSE: the I/O operations to switch a B004 compatible device
 *		 into analyse mode; used by debuggers to talk directly
 *		 to the first Transputer on the physical device.
 */

void b004_analyse(void) {

  sys_outb(B004_ANALYSE, 0);
  usleep(B004_RST_DELAY);
  sys_outb(B004_RESET, 0);
  usleep(B004_RST_DELAY);
  sys_outb(B004_ANALYSE, 1);
  usleep(B004_RST_DELAY);
  sys_outb(B004_RESET, 1);
  usleep(B004_RST_DELAY);
  sys_outb(B004_RESET, 0);
  usleep(B004_RST_DELAY);
  sys_outb(B004_ANALYSE, 0);
  usleep(B004_RST_DELAY);
}

/*
 *	MAIN: run the startup routine, then pass control to the library
 *	      routine that handles character driver messages, and calls
 *	      our various message handler functions, above.
 */

int main(void) {

  sef_local_startup();

  chardriver_task(&b004_tab);

  return OK;
}
