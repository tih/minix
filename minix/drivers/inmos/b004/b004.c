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

static ssize_t dma_read(endpoint_t endpt, cp_grant_id_t grant, size_t size);
static ssize_t dma_write(endpoint_t endpt, cp_grant_id_t grant, size_t size);
static int dma_transfer(phys_bytes dmabuf_phys, int count, int do_write);

static int expect_intr(void);

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

static unsigned char *linkbuf;

static unsigned char *dmabuf;
static phys_bytes dmabuf_phys;
static int dmabuf_len = 0;
static int dma_available = 0;
static int dma_disabled = 0;

static u32_t system_hz;
static int io_timeout = 0;

static int irq_hook_id;

static int b004_open(devminor_t UNUSED(minor), int UNUSED(access),
		     endpoint_t UNUSED(user_endpt)) {

  if (board_busy)
    return EAGAIN;

  board_busy = 1;

  return OK;
}

static int b004_close(devminor_t UNUSED(minor)) {

  io_timeout = system_hz;
  if (dma_disabled)
    dma_available = 1;

  board_busy = 0;

  return OK;
}

static ssize_t b004_read(devminor_t UNUSED(minor), u64_t UNUSED(position),
			 endpoint_t endpt, cp_grant_id_t grant, size_t size,
			 int UNUSED(flags), cdev_id_t UNUSED(id)) {
  int ret, i, j, b, copied;
  clock_t now, deadline;

  if (size <= 0)		return EINVAL;


  if ((dma_available) && (size > DMA_THRESHOLD))
    return dma_read(endpt, grant, size);

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

  if (ret != OK)
    return ret;

  return copied;
}

static ssize_t b004_write(devminor_t UNUSED(minor), u64_t UNUSED(position),
			  endpoint_t endpt, cp_grant_id_t grant, size_t size,
			  int UNUSED(flags), cdev_id_t UNUSED(id)) {
  int ret, i, j, b, copied, chunk;
  clock_t now, deadline;

  if (size <= 0)		return EINVAL;

  if ((dma_available) && (size > DMA_THRESHOLD))
    return dma_write(endpt, grant, size);

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
  if (ret != OK)
    return ret;

  return i;
}

static ssize_t dma_read(endpoint_t endpt, cp_grant_id_t grant, size_t size) {
  int ret, i, chunk, copied;
  struct itimerval itimer;

  sys_setalarm(io_timeout, 0);

  copied = 0;
  while (copied < size) {
    chunk = MIN((size - copied), dmabuf_len);
    ret = dma_transfer(dmabuf_phys, chunk, 0);
    if (ret != OK)
      break;
    ret = sys_safecopyto(endpt, grant, copied, (vir_bytes)dmabuf, chunk);
    if (ret == OK)
      copied += chunk;
    else
      break;
  }

  sys_setalarm(0, 0);

  if ((ret != OK) && (ret != EINTR))
    return ret;

  return copied;
}

static ssize_t dma_write(endpoint_t endpt, cp_grant_id_t grant, size_t size) {
  int ret, i, chunk, copied;

  sys_setalarm(io_timeout, 0);

  copied = 0;
  while (copied < size) {
    chunk = MIN((size - copied), dmabuf_len);
    ret = sys_safecopyfrom(endpt, grant, copied, (vir_bytes)dmabuf, chunk);
    if (ret != OK)
      break;
    ret = dma_transfer(dmabuf_phys, chunk, 1);
    if (ret == OK)
      copied += chunk;
    else
      break;
  }

  sys_setalarm(0, 0);
  
  if ((ret != OK) && (ret != EINTR))
    return ret;

  return copied;
}

static int dma_transfer(phys_bytes dmabuf_phys, int count, int do_write) {
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

  if ((ret=sys_voutb(byte_out, 9)) != OK)
    panic("dma_setup: failed to program DMA chip (%d)", ret);

  pv_set(byte_out[0], B004_ISR, B004_INT_ENA);
  pv_set(byte_out[1], B004_OSR, B004_INT_ENA);
  pv_set(byte_out[2], B008_INT, B008_DMAINT_ENA);

  if ((ret=sys_voutb(byte_out, 3)) != OK)
    panic("dma_setup: failed to enable interrupts (%d)", ret);

  sys_irqenable(&irq_hook_id);

  sys_outb(B008_DMA, do_write ? B008_DMAWRITE : B008_DMAREAD);

  ret = expect_intr();

  sys_irqdisable(&irq_hook_id);

  pv_set(byte_out[0], B004_ISR, B004_INT_DIS);
  pv_set(byte_out[1], B004_OSR, B004_INT_DIS);
  pv_set(byte_out[2], B008_INT, B008_INT_DIS);

  if (sys_voutb(byte_out, 3) != OK)
    panic("dma_setup: failed to reset interrupts");

  return ret;
}

static int expect_intr(void) {
  message msg;
  int status, ret;

  while (1) {
    ret = driver_receive(ANY, &msg, &status);
    if (ret != OK) {
      if (ret == EINTR)
	return ret;
      printf("b004: expect_intr: unexpected %d from driver_receive()", ret);
    }

    switch (msg.m_source) {
    case HARDWARE:
      return OK;
    case CLOCK:
      return EINTR;
    case VFS_PROC_NR:
      if (msg.m_type == CDEV_CLOSE)
	chardriver_process(&b004_tab, &msg, status);
	return EINTR;
    default:
      printf("b004: unexpected %d message from %d\n",
	     msg.m_type, msg.m_source);
    }
  }

  return OK;
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

static void b004_intr(unsigned int mask) {
  unsigned int b;

  printf("b004: unexpected hardware interrupt\n");

  return;
}

static int sef_cb_lu_state_save(int UNUSED(state), int UNUSED(flags)) {

  ds_publish_u32("board_type", board_type, DSF_OVERWRITE);
  ds_publish_u32("io_timeout", io_timeout, DSF_OVERWRITE);

  return OK;
}

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
    dma_available = 1;

    printf("b004: allocated a %d byte DMA buffer\n", dmabuf_len);
  }

  if (type != SEF_INIT_LU)
    chardriver_announce();

  if (board_type != 0)
    board_busy = 0;

  return OK;
}

void b004_probe(void) {
  unsigned int b, ret;

  b004_reset();

  if (sys_outb(B004_OSR, 0) == OK) {
    if (sys_inb(B004_OSR, &b) == OK) {
      if (b & B004_READY) {
	sys_outb(B004_OSR, B004_INT_ENA);
	sys_outb(B008_INT, B008_OUTINT_ENA);
	irq_hook_id = B004_IRQ;
	if ((sys_irqsetpolicy(B004_IRQ, 0, &irq_hook_id) != OK) ||
	    (sys_irqenable(&irq_hook_id) != OK))
	  panic("sef_cb_init: couldn't enable interrupts");
	sys_irqdisable(&irq_hook_id);
	sys_setalarm(system_hz, 0);
	sys_outb(B004_ODR, 0);
	ret = expect_intr();
	if (ret == OK) {
	  board_type = B004;
	  if ((dmabuf = alloc_contig(4, AC_LOWER16M | AC_ALIGN4K,
				     &dmabuf_phys))) {
	    sys_setalarm(system_hz, 0);
	    dmabuf[0] = 0;
	    ret = dma_transfer(dmabuf_phys, 1, 1);
	    free_contig(dmabuf, 4);
	    if (ret == OK)
	      board_type = B008;
	  }
	}
	sys_setalarm(0, 0);
      }
    }
  }

  if (board_type) {
    printf("b004: probe found a %s compatible device.\n",
	   board_type == B004 ? "B004" : "B008");
    sys_outb(B004_OSR, B004_INT_ENA);
    sys_outb(B004_ISR, B004_INT_ENA);
    if (board_type == B008)
      sys_outb(B008_INT, B008_DMAINT_ENA);
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
