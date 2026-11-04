/****************************************************************************
 * drivers/syslog/ramlog.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <ctype.h>
#include <sys/boardctl.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/syslog/ramlog.h>
#include <nuttx/compiler.h>
#include <nuttx/list.h>
#include <nuttx/irq.h>

#ifdef CONFIG_RAMLOG

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RAMLOG_MAGIC_NUMBER 0x12345678

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ramlog_ratelimit_s
{
  unsigned int  interval; /* The interval in seconds */
  unsigned int  burst;    /* The max allowed note number during interval */
  unsigned int  printed;  /* The number of printed note during interval */
  unsigned long begin;    /* The timestamp in seconds */
};

struct ramlog_header_s
{
  uint32_t          rl_magic;    /* The rl_magic number for ramlog buffer init */
  volatile uint32_t rl_head;     /* The head index (where data is added,natural growth) */
  char              rl_buffer[]; /* Circular RAM buffer */
};

struct ramlog_user_s
{
  dq_entry_t        rl_entry;      /* The dq entry of reader */
  volatile uint32_t rl_tail;       /* The tail index (where data is removed) */
  uint32_t          rl_threashold; /* The threashold of the reader to read log */
#ifndef CONFIG_RAMLOG_NONBLOCKING
  sem_t             rl_waitsem;    /* Used to wait for data */
#endif

  /* The following the poll structures of threads waiting for driver events.
   * The 'struct pollfd' reference for each open is also  retained in the
   * f_priv field of the 'struct file'.
   */

  FAR struct pollfd *rl_fds;
};

struct ramlog_dev_s
{
  /* The following is the header of the RAM buffer,
   * Store the RAM BUFFER init rl_magic number,
   * and read/write pointers
   */

  FAR struct ramlog_header_s *rl_header;

  uint32_t                   rl_bufsize;   /* Size of the circular buffer */
  dq_queue_t                 rl_queue;     /* The dq of ramlog_user_s */
  struct ramlog_ratelimit_s  rl_ratelimit; /* The ratelimit for ramlog */
#if CONFIG_RAMLOG_POLLTIMEOUT_MS > 0
  struct wdog_s              rl_wdog;      /* The wdog for poll timeout */
#endif
  spinlock_t                 rl_lock;      /* The lock for ramlog */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

#ifndef CONFIG_RAMLOG_NONBLOCKING
static void    ramlog_readnotify(FAR struct ramlog_dev_s *priv);
#endif
static void    ramlog_pollnotify(FAR struct ramlog_dev_s *priv);
#if CONFIG_RAMLOG_POLLTIMEOUT_MS > 0
static void    ramlog_timeout_handler(wdparm_t arg);
#endif

/* Character driver methods */

static int     ramlog_file_open(FAR struct file *filep);
static int     ramlog_file_close(FAR struct file *filep);
static ssize_t ramlog_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t ramlog_file_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int     ramlog_file_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg);
static int     ramlog_file_poll(FAR struct file *filep,
                                FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_ramlogfops =
{
  ramlog_file_open,  /* open */
  ramlog_file_close, /* close */
  ramlog_file_read,  /* read */
  ramlog_file_write, /* write */
  NULL,              /* seek */
  ramlog_file_ioctl, /* ioctl */
  NULL,              /* mmap */
  NULL,              /* truncate */
  ramlog_file_poll   /* poll */
};

/* This is the pre-allocated buffer used for the console RAM log and/or
 * for the syslogging function.
 *
 * [V633] This array is static, so it lands in .dram0.bss, which on this board
 * is the very same internal DRAM that backs the DMA-capable heap the WiFi
 * blob needs for its TX descriptors.  The default 32 KB was buying log depth
 * with radio headroom: association fails outright once that heap runs dry
 * (see esp32s3_allocateheap.c and the v619/v629-v632 bisection).  Trading
 * 24 KB of scrollback for it is a good deal - the serial console is the real
 * log transport here.
 */

#define V633_RAMLOG_BUFSIZE 8192

#ifdef CONFIG_RAMLOG_SYSLOG
#  ifdef RAMLOG_BUFFER_SECTION
static uint32_t g_sysbuffer[V633_RAMLOG_BUFSIZE / 4]
                       locate_data(CONFIG_RAMLOG_BUFFER_SECTION);
#  else
static uint32_t g_sysbuffer[V633_RAMLOG_BUFSIZE / 4];
#  endif

/* This is the device structure for the console or syslogging function.  It
 * must be statically initialized because the RAMLOG ramlog_putc function
 * could be called before the driver initialization logic executes.
 */

static DEFINE_PER_CPU_BMP(struct ramlog_dev_s, g_sysdev) =
{
  (FAR struct ramlog_header_s *)g_sysbuffer,            /* rl_buffer */
  sizeof(g_sysbuffer) - sizeof(struct ramlog_header_s), /* rl_bufsize */
};
#define g_sysdev this_cpu_var_bmp(g_sysdev)

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if CONFIG_RAMLOG_POLLTIMEOUT_MS > 0
static inline_function
void ramlog_timeout_reset(FAR struct ramlog_dev_s * dev)
{
  /* We need locked to ensure the wdog status and dq status aligned. */

  irqstate_t flags = spin_lock_irqsave(&dev->rl_lock);
  if (dq_empty(&dev->rl_queue))
    {
      wd_cancel(&dev->rl_wdog);
    }
  else
    {
      wd_start(&dev->rl_wdog, MSEC2TICK(CONFIG_RAMLOG_POLLTIMEOUT_MS),
               ramlog_timeout_handler, (wdparm_t)dev);
    }

  spin_unlock_irqrestore(&dev->rl_lock, flags);
}

static void ramlog_timeout_handler(wdparm_t arg)
{
  FAR struct ramlog_dev_s *dev = (FAR struct ramlog_dev_s *)arg;

  ramlog_pollnotify(dev);
  ramlog_timeout_reset(dev);
}
#else
#  define ramlog_timeout_reset(dev)
#endif

/****************************************************************************
 * Name: ramlog_ratelimit
 *
 * Description:
 *   Check whether the log is limited.
 *
 * Input Parameters:
 *   dev - The pointer of ramlog device.
 *
 * Returned Value:
 *   True is returned if the log is limited.
 *
 ****************************************************************************/

static bool ramlog_ratelimit(FAR struct ramlog_dev_s *dev)
{
  bool ret;
  clock_t ticks;
  uint32_t seconds;
  FAR struct ramlog_ratelimit_s *limit;

  limit = &dev->rl_ratelimit;

  if (limit->interval == 0)
    {
      return false;
    }

  ticks = clock_systime_ticks();
  seconds = ticks * CONFIG_USEC_PER_TICK / 1000000;

  if (limit->begin == 0)
    {
      limit->begin = seconds;
    }

  /* Reset statistical information */

  if ((seconds - limit->begin) >= limit->interval)
    {
      limit->begin = seconds;
      limit->printed = 0;
    }

  /* Check if the note is limited */

  if (limit->burst && limit->burst > limit->printed)
    {
      limit->printed++;
      ret = false;
    }
  else
    {
      ret = true;
    }

  return ret;
}

/****************************************************************************
 * Name: ramlog_bufferused
 ****************************************************************************/

static uint32_t ramlog_bufferused(FAR struct ramlog_dev_s *priv,
                                  FAR struct ramlog_user_s *upriv)
{
  uint32_t used = priv->rl_header->rl_head - upriv->rl_tail;
  return used > priv->rl_bufsize ? priv->rl_bufsize : used;
}

/****************************************************************************
 * Name: ramlog_readnotify
 ****************************************************************************/

#ifndef CONFIG_RAMLOG_NONBLOCKING
static void ramlog_readnotify(FAR struct ramlog_dev_s *priv)
{
  FAR dq_entry_t *entry;

  /* Notify all waiting readers that they can read from the FIFO */

  dq_for_every(&priv->rl_queue, entry)
    {
      FAR struct ramlog_user_s *upriv =
        container_of(entry, struct ramlog_user_s, rl_entry);
      upriv->rl_tail = 0;

      for (; ; )
        {
          int semcount = 0;

          nxsem_get_value(&upriv->rl_waitsem, &semcount);
          if (semcount >= 0)
            {
              break;
            }

          nxsem_post(&upriv->rl_waitsem);
        }
    }
}
#endif

/****************************************************************************
 * Name: ramlog_pollnotify
 ****************************************************************************/

static void ramlog_pollnotify(FAR struct ramlog_dev_s *priv)
{
  FAR dq_entry_t *entry;

  /* This function may be called from an interrupt handler */

  dq_for_every(&priv->rl_queue, entry)
    {
      FAR struct ramlog_user_s *upriv =
        container_of(entry, struct ramlog_user_s, rl_entry);
      if (ramlog_bufferused(priv, upriv) >= upriv->rl_threashold)
        {
          /* Notify all poll/select waiters that they can read from
           * the FIFO
           */

          poll_notify(&upriv->rl_fds, 1, POLLIN);
        }
    }
}

/****************************************************************************
 * Name: ramlog_flush
 ****************************************************************************/

static void ramlog_bufferflush(FAR struct ramlog_dev_s *priv)
{
  FAR dq_entry_t *entry;

  priv->rl_header->rl_head = 0;

  dq_for_every(&priv->rl_queue, entry)
    {
      FAR struct ramlog_user_s *upriv =
        container_of(entry, struct ramlog_user_s, rl_entry);
      upriv->rl_tail = 0;
    }
}

/****************************************************************************
 * Name: ramlog_copybuf
 ****************************************************************************/

static void ramlog_copybuf(FAR struct ramlog_dev_s *priv,
                          FAR const char *buffer, size_t len)
{
  FAR struct ramlog_header_s *header = priv->rl_header;
  FAR char *buf = header->rl_buffer;
  uint32_t offset;
  uint32_t tail;

  if (len <= 0)
    {
      return;
    }

  offset = header->rl_head % priv->rl_bufsize;
  tail = priv->rl_bufsize - offset;

  if (len > tail)
    {
      memcpy(&buf[offset], buffer, tail);
      memcpy(buf, buffer + tail, len - tail);
    }
  else
    {
      memcpy(&buf[offset], buffer, len);
    }

  header->rl_head += len;
}

/****************************************************************************
 * Name: ramlog_addbuf
 ****************************************************************************/

static ssize_t ramlog_addbuf(FAR struct ramlog_dev_s *priv,
                             FAR const char *buffer, size_t len)
{
#ifdef CONFIG_RAMLOG_SYSLOG
  FAR struct ramlog_header_s *header = priv->rl_header;
#endif
  size_t buflen = len;
  irqstate_t flags;

  /* Disable interrupts (in case we are NOT called from interrupt handler) */

  flags = spin_lock_irqsave_nopreempt(&priv->rl_lock);

  if (ramlog_ratelimit(priv))
    {
      spin_unlock_irqrestore_nopreempt(&priv->rl_lock, flags);
      return len;
    }

#ifdef CONFIG_RAMLOG_SYSLOG
  if (header->rl_magic != RAMLOG_MAGIC_NUMBER && priv == &g_sysdev)
    {
      memset(header, 0, sizeof(g_sysbuffer));
      header->rl_magic = RAMLOG_MAGIC_NUMBER;
    }
#endif

  if (buflen > priv->rl_bufsize)
    {
      buffer += buflen - priv->rl_bufsize;
      buflen = priv->rl_bufsize;
    }

  ramlog_copybuf(priv, buffer, buflen);

  /* Was anything written? */

  if (len > 0)
    {
#ifndef CONFIG_RAMLOG_NONBLOCKING
      /* Are there threads waiting for read data? */

      ramlog_readnotify(priv);
#endif
      /* Notify all poll/select waiters that they can read from the FIFO */

      ramlog_pollnotify(priv);
    }

  /* We always have to return the number of bytes requested and NOT the
   * number of bytes that were actually written.  Otherwise, callers
   * probably retry, causing same error condition again.
   */

  spin_unlock_irqrestore_nopreempt(&priv->rl_lock, flags);
  ramlog_timeout_reset(priv);
  return len;
}

/****************************************************************************
 * Name: ramlog_read
 ****************************************************************************/

static ssize_t ramlog_file_read(FAR struct file *filep, FAR char *buffer,
                                size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_header_s *header = priv->rl_header;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  irqstate_t flags;
  uint32_t ncopy;
  ssize_t nread;
  uint32_t tail;
  uint32_t pos;

  /* Get exclusive access to the rl_tail index */

  flags = spin_lock_irqsave(&priv->rl_lock);

  /* Loop until something is read */

  for (nread = 0; (size_t)nread < len; )
    {
      /* Get the next byte from the buffer */

      if (header->rl_head == upriv->rl_tail)
        {
          /* The circular buffer is empty. */

          /* If the function be called from an interrupt handler,
           * then just break out of the loop.
           */

          if (up_interrupt_context())
            {
              break;
            }

#ifdef CONFIG_RAMLOG_NONBLOCKING
          /* Return what we have (with zero mean the end-of-file) */

          break;
#else
          int ret;

          /* Did we read anything? */

          if (nread > 0)
            {
              /* Yes.. break out to return what we have. */

              break;
            }

          /* If the driver was opened with O_NONBLOCK option, then don't
           * wait.
           */

          if (filep->f_oflags & O_NONBLOCK)
            {
              nread = -EAGAIN;
              break;
            }

          /* We may now be pre-empted!  But that should be okay because we
           * have already incremented nwaiters.  Pre-emptions is disabled
           * but will be re-enabled while we are waiting.
           */

          spin_unlock_irqrestore(&priv->rl_lock, flags);
          ret = nxsem_wait(&upriv->rl_waitsem);

          /* Did we successfully get the rl_waitsem? */

          if (ret < 0)
            {
              /* No.. nxsem_wait's failed. */

              /* Return the error. We did handle the case where we read
               * anything already before waiting.
               */

              return ret;
            }
          else
            {
              flags = spin_lock_irqsave(&priv->rl_lock);
            }
#endif /* CONFIG_RAMLOG_NONBLOCKING */
        }
      else
        {
          /* Determine whether the read pointer is overwritten */

          if (header->rl_head - upriv->rl_tail > priv->rl_bufsize)
            {
              upriv->rl_tail = header->rl_head - priv->rl_bufsize;
            }

          /* The circular buffer is not empty, get the next byte from the
           * tail index.
           */

          pos = upriv->rl_tail % priv->rl_bufsize;
          ncopy = header->rl_head - upriv->rl_tail;
          tail = priv->rl_bufsize - pos;

          if (ncopy > len - nread)
            {
              ncopy = len - nread;
            }

          if (ncopy > tail)
            {
              memcpy(&buffer[nread], &header->rl_buffer[pos], tail);
              memcpy(&buffer[nread + tail], header->rl_buffer, ncopy - tail);
            }
          else
            {
              memcpy(&buffer[nread], &header->rl_buffer[pos], ncopy);
            }

          upriv->rl_tail += ncopy;
          nread += ncopy;
        }
    }

  spin_unlock_irqrestore(&priv->rl_lock, flags);

  /* Return the number of characters actually read */

  return nread;
}

/****************************************************************************
 * Name: ramlog_file_write
 ****************************************************************************/

static ssize_t ramlog_file_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;

  return ramlog_addbuf(priv, buffer, len);
}

/****************************************************************************
 * Name: ramlog_file_ioctl
 ****************************************************************************/

static int ramlog_file_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  irqstate_t flags;
  int ret = 0;

  flags = spin_lock_irqsave(&priv->rl_lock);

  switch (cmd)
    {
      case FIONREAD:
        *(FAR int *)((uintptr_t)arg) = ramlog_bufferused(priv, upriv);
        break;
      case PIPEIOC_POLLINTHRD:
        upriv->rl_threashold = (uint32_t)arg;
        break;
      case BIOC_FLUSH:
        ramlog_bufferflush(priv);
        break;

      case SYSLOGIOC_SETRATELIMIT:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            FAR struct syslog_ratelimit_s *limit =
              (FAR struct syslog_ratelimit_s *)arg;

            priv->rl_ratelimit.interval = limit->interval;
            priv->rl_ratelimit.burst = limit->burst;
            ret = 0;
          }
        break;

      case SYSLOGIOC_GETRATELIMIT:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            FAR struct syslog_ratelimit_s *limit =
              (FAR struct syslog_ratelimit_s *)arg;

            limit->interval = priv->rl_ratelimit.interval;
            limit->burst = priv->rl_ratelimit.burst;
            ret = 0;
          }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  spin_unlock_irqrestore(&priv->rl_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: ramlog_file_poll
 ****************************************************************************/

static int ramlog_file_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  pollevent_t eventset = POLLOUT;
  irqstate_t flags;

  /* Get exclusive access to the poll structures */

  flags = spin_lock_irqsave(&priv->rl_lock);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference.
       */

      if (!upriv->rl_fds)
        {
          upriv->rl_fds = fds;
          fds->priv     = &upriv->rl_fds;
        }

      /* Should immediately notify on any of the requested events? */

      /* Check if the receive buffer is not empty. */

      if (ramlog_bufferused(priv, upriv) >= upriv->rl_threashold)
        {
          eventset |= POLLIN;
        }

      spin_unlock_irqrestore(&priv->rl_lock, flags);
      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
      spin_unlock_irqrestore(&priv->rl_lock, flags);
    }
  else
    {
      spin_unlock_irqrestore(&priv->rl_lock, flags);
    }

  ramlog_timeout_reset(priv);
  return 0;
}

/****************************************************************************
 * Name: ramlog_file_open
 ****************************************************************************/

static int ramlog_file_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_header_s *header = priv->rl_header;
  FAR struct ramlog_user_s *upriv;
  irqstate_t flags;

  /* Get exclusive access to the rl_tail index */

  upriv = kmm_zalloc(sizeof(FAR struct ramlog_user_s));
  if (upriv == NULL)
    {
      return -ENOMEM;
    }

  upriv->rl_threashold = CONFIG_RAMLOG_POLLTHRESHOLD;
#ifndef CONFIG_RAMLOG_NONBLOCKING
  nxsem_init(&upriv->rl_waitsem, 0, 0);
#endif

  flags = spin_lock_irqsave(&priv->rl_lock);
  dq_addlast(&upriv->rl_entry, &priv->rl_queue);
  upriv->rl_tail = header->rl_head > priv->rl_bufsize ?
                   header->rl_head - priv->rl_bufsize : 0;
  spin_unlock_irqrestore(&priv->rl_lock, flags);

  filep->f_priv = upriv;
  return 0;
}

/****************************************************************************
 * Name: ramlog_file_close
 ****************************************************************************/

static int ramlog_file_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct ramlog_dev_s *priv = inode->i_private;
  FAR struct ramlog_user_s *upriv = filep->f_priv;
  irqstate_t flags;

  /* Get exclusive access to the rl_tail index */

  flags = spin_lock_irqsave(&priv->rl_lock);
  dq_rem(&upriv->rl_entry, &priv->rl_queue);
  spin_unlock_irqrestore(&priv->rl_lock, flags);

#ifndef CONFIG_RAMLOG_NONBLOCKING
  nxsem_destroy(&upriv->rl_waitsem);
#endif
  kmm_free(upriv);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramlog_register
 *
 * Description:
 *   Create the RAM logging device and register it at the specified path.
 *
 ****************************************************************************/

int ramlog_register(FAR const char *devpath, FAR char *buffer, size_t buflen)
{
  FAR struct ramlog_dev_s *priv;
  int ret = -ENOMEM;

  /* Sanity checking */

  DEBUGASSERT(devpath && buffer && buflen > sizeof(struct ramlog_header_s));

  /* Allocate a RAM logging device structure */

  priv = kmm_zalloc(sizeof(struct ramlog_dev_s));
  if (priv != NULL)
    {
      /* Initialize the non-zero values in the RAM logging device structure */

      priv->rl_bufsize = buflen - sizeof(struct ramlog_header_s);
      priv->rl_header = (FAR struct ramlog_header_s *)buffer;

      /* Register the character driver */

      ret = register_driver(devpath, &g_ramlogfops, 0666, priv);
      if (ret < 0)
        {
          kmm_free(priv);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ramlog_syslog_register
 *
 * Description:
 *   Use a pre-allocated RAM logging device and register it at the path
 *   specified by CONFIG_RAMLOG_SYSLOG
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
void ramlog_syslog_register(void)
{
  /* Register the syslog character driver */

  register_driver(CONFIG_SYSLOG_DEVPATH, &g_ramlogfops, 0666, &g_sysdev);
}
#endif

/****************************************************************************
 * Name: ramlog_putc
 *
 * Description:
 *   This is the low-level system logging interface.
 *
 ****************************************************************************/

#ifdef CONFIG_RAMLOG_SYSLOG
int ramlog_putc(FAR syslog_channel_t *channel, int ch)
{
  char cch = ch;

  UNUSED(channel);

  /* Add the character to the RAMLOG */

  ramlog_addbuf(&g_sysdev, &cch, 1);

  /* Return the character added on success */

  return ch;
}

ssize_t ramlog_write(FAR syslog_channel_t *channel,
                     FAR const char *buffer, size_t buflen)
{
  return ramlog_addbuf(&g_sysdev, buffer, buflen);
}
#endif

#endif /* CONFIG_RAMLOG */
