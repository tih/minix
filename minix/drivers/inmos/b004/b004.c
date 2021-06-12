#include <minix/drivers.h>
#include <minix/chardriver.h>
#include <stdio.h>
#include <stdlib.h>
#include <minix/ds.h>

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

static void b004_intr(unsigned int mask);

static void b004_probe(void);
static void b004_initialize(void);

static void sef_local_startup(void);
static int sef_cb_init(int type, sef_init_info_t *info);
static int sef_cb_lu_state_save(int, int);
static int lu_state_restore(void);

static struct chardriver b004_tab = {
  .cdr_open  = b004_open,
  .cdr_close = b004_close,
  .cdr_read  = b004_read,
  .cdr_write = b004_write,
  .cdr_ioctl = b004_ioctl,
  .cdr_intr  = b004_intr,
};

static int board_type = 0;

static vir_bytes rlinkbuf, wlinkbuf;
static phys_bytes rlinkbuf_phys, wlinkbuf_phys;
static int rbuf_read_offset, rbuf_write_offset;
static int wbuf_read_offset, wbuf_write_offset;
static int rlink_busy, wlink_busy;

static int irq_hook_id;

static int open_counter;

static int b004_open(devminor_t UNUSED(minor), int UNUSED(access),
		     endpoint_t UNUSED(user_endpt)) {

  printf("b004_open(). Called %d time(s).\n", ++open_counter);

  return OK;
}

static int b004_close(devminor_t UNUSED(minor)) {

  printf("b004_close()\n");

  return OK;
}

static ssize_t b004_read(devminor_t UNUSED(minor), u64_t position,
			 endpoint_t endpt, cp_grant_id_t grant, size_t size,
			 int flags, cdev_id_t UNUSED(id)) {
  int ret;
  int avail, xfer;

  if (rlink_busy)		return EIO;
  if (size <= 0)		return EINVAL;
  if (size > DMA_SIZE)		return EINVAL;

  rlink_busy = 1;

  avail = rbuf_write_offset - rbuf_read_offset;

  if (flags & CDEV_NONBLOCK) {
    if (avail == 0) {
      rlink_busy = 0;
      return EAGAIN;
    }
    xfer = MIN(avail, size);
    if ((ret = sys_safecopyto(endpt, grant,
			      rbuf_read_offset, rlinkbuf, xfer)) != OK) {
      rlink_busy = 0;
      return ret;
    }
    rbuf_read_offset += xfer;
    if (rbuf_read_offset == rbuf_write_offset) {
      rbuf_read_offset = 0;
      rbuf_write_offset = 0;
    }
    rlink_busy = 0;
    return xfer;
  }

  for (;;) {
    if (avail >= size) {
      if ((ret = sys_safecopyto(endpt, grant,
				rbuf_read_offset, rlinkbuf, size)) != OK) {
	rlink_busy = 0;
	return ret;
      }
      rbuf_read_offset += size;
      if (rbuf_read_offset == rbuf_write_offset) {
	rbuf_read_offset = 0;
	rbuf_write_offset = 0;
      }
      rlink_busy = 0;
      return size;
    } else {
      usleep(10000);
      avail = rbuf_write_offset - rbuf_read_offset;
    }
  }
  /* NOTREACHED */
}

static ssize_t b004_write(devminor_t UNUSED(minor), u64_t UNUSED(position),
			  endpoint_t endpt, cp_grant_id_t grant, size_t size,
			  int flags, cdev_id_t UNUSED(id)) {
  int ret;

  if (wlink_busy)		return EIO;
  if (size <= 0)		return EINVAL;
  if (size > DMA_SIZE)		return EINVAL;

  wlink_busy = 1;

  if ((ret = sys_safecopyfrom(endpt, grant, 0, wlinkbuf, size)) != OK)
    return ret;

  wbuf_read_offset = 0;
  wbuf_write_offset = size;

  b004_intr(0);

  if (flags & CDEV_NONBLOCK)
    return size;

  while (wbuf_read_offset != wbuf_write_offset)
    usleep(10000);

  return size;
}

static int b004_ioctl(devminor_t UNUSED(minor), unsigned long UNUSED(request),
		      endpoint_t UNUSED(endpt), cp_grant_id_t UNUSED(grant),
		      int UNUSED(flags), endpoint_t UNUSED(user_endpt),
		      cdev_id_t UNUSED(id)) {

  /* Should have reset, analyse, getflags, and setflags */

  return EINVAL;
}

static void b004_intr(unsigned int UNUSED(mask)) {

  if(sys_irqenable(&irq_hook_id) != OK) {
	panic("sys_irqenable failed");
  }

  return;
}

static int sef_cb_lu_state_save(int UNUSED(state), int UNUSED(flags)) {

  ds_publish_u32("open_counter", open_counter, DSF_OVERWRITE);

  return OK;
}

static int lu_state_restore() {
  u32_t value;

  ds_retrieve_u32("open_counter", &value);
  ds_delete_u32("open_counter");
  open_counter = (int) value;

  return OK;
}

static void sef_local_startup() {

  sef_setcb_init_fresh(sef_cb_init);
  sef_setcb_init_lu(sef_cb_init);
  sef_setcb_init_restart(sef_cb_init);

  sef_setcb_lu_state_save(sef_cb_lu_state_save);

  sef_startup();
}

static int sef_cb_init(int type, sef_init_info_t *UNUSED(info)) {
  int off;

  if (!(rlinkbuf = (vir_bytes)alloc_contig(2*DMA_SIZE, AC_LOWER16M|AC_ALIGN4K,
				&rlinkbuf_phys)))
    panic("couldn't allocate DMA buffer");

  if (rlinkbuf / DMA_ALIGN != (rlinkbuf + DMA_SIZE - 1) / DMA_ALIGN) {
    off = rlinkbuf % DMA_ALIGN;
    rlinkbuf += (DMA_ALIGN - off);
    rlinkbuf_phys += (DMA_ALIGN - off);
  }

  if (!(wlinkbuf = (vir_bytes)alloc_contig(2*DMA_SIZE, AC_LOWER16M|AC_ALIGN4K,
				&wlinkbuf_phys)))
    panic("couldn't allocate DMA buffer");

  if (wlinkbuf / DMA_ALIGN != (wlinkbuf + DMA_SIZE - 1) / DMA_ALIGN) {
    off = wlinkbuf % DMA_ALIGN;
    wlinkbuf += (DMA_ALIGN - off);
    wlinkbuf_phys += (DMA_ALIGN - off);
  }

  rbuf_read_offset = 0;
  rbuf_write_offset = 0;
  wbuf_read_offset = 0;
  wbuf_write_offset = 0;

  if (board_type == 0) {
    b004_probe();
    if ((board_type != 0) && (type == SEF_INIT_FRESH)) {
      b004_initialize();
    }
  }

  irq_hook_id = 0;
  if(sys_irqsetpolicy(B004_IRQ, 0, &irq_hook_id) != OK ||
     sys_irqenable(&irq_hook_id) != OK) {
    panic("do_initialize: irq enabling failed");
  }

  if (type == SEF_INIT_LU)
    lu_state_restore();

  if (type == SEF_INIT_FRESH)
    chardriver_announce();

  return OK;
}

void b004_probe(void) {
  unsigned char b;

  if (sys_outb(B004_OSR, 0) == OK) {
    usleep(10000);
    if (sys_inb(B004_OSR, &b) == OK) {
      if (b & B004_READY) {
	board_type = B004;
	usleep(10000);
	if (sys_inb(B008_INT, &b) == OK) {
	  if ((b & B008_INT_MASK) == 0) {
	    board_type = B008;
	  }
	}
      }
    }
  }

  if (board_type)
    printf("Found a %s device!\n", board_type == B004 ? "B004" : "B008");
}

void b004_initialize(void) {

}

int main(void) {

  sef_local_startup();

  if (board_type != 0)
    chardriver_task(&b004_tab);

  return OK;
}
