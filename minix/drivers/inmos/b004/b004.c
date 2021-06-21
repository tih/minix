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

static void b004_intr(unsigned int mask);

static void b004_probe(void);
static void b004_initialize(void);
static void b004_reset(void);
static void b004_analyse(void);

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
static int probe_active;
static volatile int probe_int_seen;

static unsigned char *rlinkbuf, *wlinkbuf;
static phys_bytes rlinkbuf_phys, wlinkbuf_phys;
static int rlink_busy, wlink_busy;

static u32_t system_hz;
static int b004_io_timeout;

static unsigned int b008_intmask = B008_INT_MASK;

static int irq_hook_id;

static int board_busy;

static int b004_open(devminor_t UNUSED(minor), int UNUSED(access),
		     endpoint_t UNUSED(user_endpt)) {

  if (board_busy)
    return EAGAIN;

  if (board_type == 0)
    b004_probe();

  if (board_type == 0)
    return EINVAL;

  board_busy = 1;

  return OK;
}

static int b004_close(devminor_t UNUSED(minor)) {

  board_busy = 0;

  return OK;
}

static ssize_t b004_read(devminor_t UNUSED(minor), u64_t position,
			 endpoint_t endpt, cp_grant_id_t grant, size_t size,
			 int flags, cdev_id_t UNUSED(id)) {
  int ret, i, b;
  clock_t now, deadline;

  if (rlink_busy)		return EIO;
  if (size <= 0)		return EINVAL;
  if (size > DMA_SIZE)		return EINVAL;

  rlink_busy = 1;

  getuptime(&now, NULL, NULL);
  deadline = now + b004_io_timeout;

  for (i = 0; i < size; i++) {
    while (1) {
      sys_inb(B004_ISR, &b);
      if (b & B004_READY) {
	sys_inb(B004_IDR, &b);
	rlinkbuf[i] = b;
	break;
      } else {
	getuptime(&now, NULL, NULL);
	if (now > deadline) {
	  goto out;
	}
      }
    }
  }

 out:
  rlink_busy = 0;

  if (i > 0)
    ret = sys_safecopyto(endpt, grant,
			 0, (vir_bytes)rlinkbuf, i);
  if (ret == OK)
    return i;
  else
    return ret;
}

static ssize_t b004_write(devminor_t UNUSED(minor), u64_t UNUSED(position),
			  endpoint_t endpt, cp_grant_id_t grant, size_t size,
			  int flags, cdev_id_t UNUSED(id)) {
  int ret, i, b;
  clock_t now, deadline;

  if (wlink_busy)		return EIO;
  if (size <= 0)		return EINVAL;
  if (size > DMA_SIZE)		return EINVAL;

  if ((ret = sys_safecopyfrom(endpt, grant,
			      0, (vir_bytes)wlinkbuf, size)) != OK)
    return ret;

  wlink_busy = 1;

  getuptime(&now, NULL, NULL);
  deadline = now + b004_io_timeout;

  for (i = 0; i < size; i++) {
    while (1) {
      sys_inb(B004_OSR, &b);
      if (b & B004_READY) {
	sys_outb(B004_ODR, wlinkbuf[i]);
	break;
      } else {
	getuptime(&now, NULL, NULL);
	if (now > deadline) {
	  goto out;
	}
      }
    }
  }

 out:
  wlink_busy = 0;

  return i;
}

static int b004_ioctl(devminor_t UNUSED(minor), unsigned long request,
		      endpoint_t endpt, cp_grant_id_t grant, int UNUSED(flags),
		      endpoint_t UNUSED(user_endpt), cdev_id_t UNUSED(id)) {
  int ret;
  unsigned int b;
  struct b004_flags flag;

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
    ret = sys_safecopyto(endpt, grant, 0, (vir_bytes) &flag,
			 sizeof(struct b004_flags));
    break;
  default:
    ret = EINVAL;
  }

  return ret;
}

static void b004_intr(unsigned int mask) {
  unsigned int b;

  printf("b004_intr(%d)\n", mask);

  sys_outb(B008_INT, B008_INT_DIS);

  if (probe_active)
    probe_int_seen = 1;

  sys_outb(B008_INT, b008_intmask);

  return;
}

static int sef_cb_lu_state_save(int UNUSED(state), int UNUSED(flags)) {

  ds_publish_u32("board_busy", board_busy, DSF_OVERWRITE);

  return OK;
}

static int lu_state_restore() {
  u32_t value;

  ds_retrieve_u32("board_busy", &value);
  ds_delete_u32("board_busy");
  board_busy = (int) value;

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

  if (!(rlinkbuf = alloc_contig(2*DMA_SIZE, AC_LOWER16M|AC_ALIGN4K,
				&rlinkbuf_phys)))
    panic("sef_cb_init: couldn't allocate DMA buffer");

  if (rlinkbuf_phys/DMA_ALIGN != (rlinkbuf_phys+DMA_SIZE-1)/DMA_ALIGN) {
    off = rlinkbuf_phys % DMA_ALIGN;
    rlinkbuf += (DMA_ALIGN - off);
    rlinkbuf_phys += (DMA_ALIGN - off);
  }

  if (!(wlinkbuf = alloc_contig(2*DMA_SIZE, AC_LOWER16M|AC_ALIGN4K,
				&wlinkbuf_phys)))
    panic("sef_cb_init: couldn't allocate DMA buffer");

  if (wlinkbuf_phys/DMA_ALIGN != (wlinkbuf_phys+DMA_SIZE-1)/DMA_ALIGN) {
    off = wlinkbuf_phys % DMA_ALIGN;
    wlinkbuf += (DMA_ALIGN - off);
    wlinkbuf_phys += (DMA_ALIGN - off);
  }

  if (sys_getinfo(GET_HZ, &system_hz, sizeof(system_hz), 0, 0) != OK)
    panic("sef_cb_init: couldn't get system HZ value");
  
  b004_io_timeout = system_hz;

  if (type == SEF_INIT_LU)
    lu_state_restore();

  if (type == SEF_INIT_FRESH)
    chardriver_announce();

  return OK;
}

void b004_probe(void) {
  unsigned int b;

  b004_reset();

  if (sys_outb(B004_OSR, 0) == OK) {
    if (sys_inb(B004_OSR, &b) == OK) {
      if (b & B004_READY) {
	board_type = B004;
	probe_active = 1;
	probe_int_seen = 0;
	sys_outb(B008_INT, B008_INT_DIS);
	sys_outb(B004_OSR, B004_INT_DIS);
	irq_hook_id = B004_IRQ;
	if ((sys_irqsetpolicy(B004_IRQ, IRQ_REENABLE, &irq_hook_id) != OK) ||
	    (sys_irqenable(&irq_hook_id) != OK))
	  panic("sef_cb_init: couldn't enable interrupts");
	sys_outb(B004_OSR, B004_INT_ENA);
	sys_outb(B004_ODR, 0);
	usleep(B004_RST_DELAY);
	sys_outb(B004_OSR, B004_INT_DIS);
	if (probe_int_seen) {
	  printf("got the B004 interrupt\n");
	  board_type = B004;
	} else {
	  usleep(B004_IO_DELAY);
	  sys_outb(B008_INT, B008_OUTINT_ENA);
	  sys_outb(B004_OSR, B004_INT_ENA);
	  sys_outb(B004_ODR, 0);
	  usleep(B004_RST_DELAY);
	  sys_outb(B004_OSR, B004_INT_DIS);
	  sys_outb(B004_OSR, B004_INT_DIS);
	  if (probe_int_seen) {
	    printf("got the B008 interrupt\n");
	    board_type = B008;
	  }
	}
	probe_active = 0;
      }
    }
  }

  if (board_type) {
    printf("b004: found a %s device.\n",
	   board_type == B004 ? "B004" : "B008");
    sys_outb(B004_OSR, B004_INT_ENA);
    sys_outb(B004_ISR, B004_INT_ENA);
    if (board_type == B008)
      sys_outb(B008_INT, b008_intmask);
    board_busy = 0;
  }
}

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

int main(void) {

  sef_local_startup();

  chardriver_task(&b004_tab);

  return OK;
}
