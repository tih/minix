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
static int board_busy = 1;
static int probe_active = 0;
static volatile int probe_int_seen;

static unsigned char *linkbuf;
static int link_busy = 0;

static unsigned char *dmabuf;
static phys_bytes dmabuf_phys;
static int dmabuf_len = 0;

static u32_t system_hz;
static int io_timeout = 0;

static unsigned int b008_intmask = B008_INT_MASK;

static int irq_hook_id;

static int b004_open(devminor_t UNUSED(minor), int UNUSED(access),
		     endpoint_t UNUSED(user_endpt)) {

  if (board_busy)
    return EAGAIN;

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
  int ret, i, j, b, copied;
  clock_t now, deadline;

  if (link_busy)		return EIO;
  if (size <= 0)		return EINVAL;

  link_busy = 1;

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
	  if (now > deadline) {
	    goto out;
	  }
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

  link_busy = 0;

  if (ret != OK)
    return ret;

  return copied;
}

static ssize_t b004_write(devminor_t UNUSED(minor), u64_t UNUSED(position),
			  endpoint_t endpt, cp_grant_id_t grant, size_t size,
			  int flags, cdev_id_t UNUSED(id)) {
  int ret, i, j, b, copied, chunk;
  clock_t now, deadline;

  if (link_busy)		return EIO;
  if (size <= 0)		return EINVAL;

  link_busy = 1;

  getuptime(&now, NULL, NULL);
  deadline = now + io_timeout;

  copied = 0;
  for (i = 0, j = 0; i < size; i++) {
    if (j == 0) {
      chunk = MIN((size - copied), LINKBUF_SIZE);
      ret = sys_safecopyfrom(endpt, grant, copied,
			     (vir_bytes)linkbuf, chunk);
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
	  if (now > deadline) {
	    goto out;
	  }
	}
      }
    }
    if (j == LINKBUF_SIZE)
      j = 0;
  }

 out:
  link_busy = 0;

  if (ret != OK)
    return ret;

  return i;
}

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
  default:
    ret = EINVAL;
  }

  return ret;
}

static void b004_intr(unsigned int mask) {
  unsigned int b;

  printf("b004_intr(%d)\n", mask);

  if (probe_active)
    probe_int_seen = 1;

  return;
}

static int sef_cb_lu_state_save(int UNUSED(state), int UNUSED(flags)) {

  ds_publish_u32("board_type", board_type, DSF_OVERWRITE);
  ds_publish_u32("board_busy", board_busy, DSF_OVERWRITE);
  ds_publish_u32("io_timeout", io_timeout, DSF_OVERWRITE);

  return OK;
}

static int lu_state_restore() {
  u32_t value;

  if (ds_retrieve_u32("board_type", &value) == OK)
    board_type = (int) value;
  if (ds_retrieve_u32("board_busy", &value) == OK)
    board_busy = (int) value;
  if (ds_retrieve_u32("io_timeout", &value) == OK)
    io_timeout = (int) value;

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
  int off, i;

  if (type == SEF_INIT_LU)
    lu_state_restore();

  if (sys_getinfo(GET_HZ, &system_hz, sizeof(system_hz), 0, 0) != OK)
    panic("sef_cb_init: couldn't get system HZ value");
  
  if (io_timeout == 0)
    io_timeout = system_hz;

  if (!(linkbuf = malloc(LINKBUF_SIZE)))
    panic("sef_cb_init: couldn't allocate link buffer");

  if (board_type == 0)
    b004_probe();

  if (board_type == B008) {
    for (i = 64; i >= 1; i /= 2) {
      if ((dmabuf = alloc_contig(i * 1024, AC_LOWER16M | AC_ALIGN64K,
				 &dmabuf_phys)))
	break;
      if ((dmabuf = alloc_contig(2 * i * 1024, AC_LOWER16M | AC_ALIGN4K,
				 &dmabuf_phys)))
	break;
    }

    if (i == 0)
      panic("sef_cb_init: couldn't allocate DMA buffer");

    if (dmabuf_phys/DMA_ALIGN != (dmabuf_phys+dmabuf_len-1)/DMA_ALIGN) {
      off = dmabuf_phys % DMA_ALIGN;
      dmabuf += (DMA_ALIGN - off);
      dmabuf_phys += (DMA_ALIGN - off);
    }

    dmabuf_len = i * 1024;

    printf("b004: allocated a %d byte DMA buffer\n", dmabuf_len);
  }

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
	sys_outb(B004_ISR, B004_INT_ENA);
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
	} else {
	  sys_outb(B008_INT, B008_OUTINT_ENA);
	  sys_outb(B004_OSR, B004_INT_ENA);
	  sys_outb(B004_ODR, 0);
	  usleep(B004_RST_DELAY);
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
    printf("b004: probe found a %s device.\n",
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
