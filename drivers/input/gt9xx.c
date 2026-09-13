/****************************************************************************
 * drivers/input/gt9xx.c
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

/* Reference:
 * "NuttX RTOS for PinePhone: Touch Panel"
 * https://lupyuen.github.io/articles/touch2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/touchscreen.h>
#include <nuttx/wqueue.h>
#include <nuttx/input/gt9xx.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Default I2C Frequency is 400 kHz */

#ifndef CONFIG_INPUT_GT9XX_I2C_FREQUENCY
#  define CONFIG_INPUT_GT9XX_I2C_FREQUENCY 400000
#endif

/* Default Number of Poll Waiters is 1 */

#ifndef CONFIG_INPUT_GT9XX_NPOLLWAITERS
#  define CONFIG_INPUT_GT9XX_NPOLLWAITERS 1
#endif

/* I2C Registers for Goodix GT9XX Touch Panel */

#define GTP_REG_VERSION    0x8140  /* Product ID */
#define GTP_READ_COOR_ADDR 0x814e  /* Touch Panel Status */
#define GTP_POINT1         0x8150  /* Touch Point 1 */

/* Period of the poll worker. The frame acknowledge must reach the
 * controller within a bounded time, and the cadence also keeps the
 * I2C interface alive */

#define GT911_POLL_PERIOD_MS 20

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Touch Panel Device */

struct gt9xx_dev_s
{
  /* I2C bus and address for device */

  FAR struct i2c_master_s *i2c;
  uint8_t addr;

  /* Callback for Board-Specific Operations */

  FAR const struct gt9xx_board_s *board;

  /* Device State */

  mutex_t devlock;  /* Mutex to prevent concurrent reads */
  uint8_t cref;     /* Reference Counter for device */
  bool int_pending; /* True if a Touch Interrupt is pending processing */

  /* Periodic poll worker state: the worker owns all I2C traffic and
   * publishes the decoded sample for the reader */

  struct work_s poll_work;      /* Re-arming poll work item */
  volatile bool polling;        /* Worker scheduling control */
  struct touch_sample_s cached; /* Last decoded sample */
  bool touched;                 /* A contact is currently held */
  int unacked_x;                /* X of a frame whose ack failed, -1 none */
  int unacked_y;                /* Y of a frame whose ack failed, -1 none */
  uint16_t x;       /* X Coordinate of Last Touch Point */
  uint16_t y;       /* Y Coordinate of Last Touch Point */
  uint8_t flags;    /* Touch Up or Touch Down for Last Touch Point */

  /* Poll Waiters for device */

  FAR struct pollfd *fds[CONFIG_INPUT_GT9XX_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep);
static int gt9xx_close(FAR struct file *filep);
static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static void gt9xx_poll_work(FAR void *arg);
static int gt9xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct gt9xx_dev_s *priv = inode->i_private;
  int ret = OK;

  DEBUGASSERT(priv != NULL);

  switch (cmd)
    {
      case TSIOC_GETMAXPOINTS:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          DEBUGASSERT(ptr != NULL);
          *ptr = 1;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int gt9xx_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File Operations for Touch Panel */

static const struct file_operations g_gt9xx_fileops =
{
  gt9xx_open,   /* open */
  gt9xx_close,  /* close */
  gt9xx_read,   /* read */
  NULL,         /* write */
  NULL,         /* seek */
  gt9xx_ioctl,  /* ioctl */
  NULL,         /* truncate */
  NULL,         /* mmap */
  gt9xx_poll,   /* poll */
  NULL,         /* readv */
  NULL          /* writev */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL        /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_i2c_read
 *
 * Description:
 *   Read a Touch Panel Register over I2C.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   reg    - I2C Register to be read
 *   buf    - Receive Buffer
 *   buflen - Number of bytes to be read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_i2c_read(FAR struct gt9xx_dev_s *dev,
                          uint16_t reg,
                          uint8_t *buf,
                          size_t buflen)
{
  int retries;
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Receive the I2C Register Values */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  DEBUGASSERT(dev && dev->i2c && buf);

  /* Execute the I2C transfer. The Goodix controller briefly NACKs
   * accesses while its internal MCU refreshes the coordinate buffer,
   * so a rejected transfer is retried a few times before it is
   * reported upstream */

  ret = -EIO;
  for (retries = 0; retries < CONFIG_INPUT_GT9XX_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
      if (ret >= 0)
        {
          break;
        }

      nxsig_usleep(5 * 1000);
    }

  if (ret < 0)
    {
      ierr("I2C Read failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_i2c_write
 *
 * Description:
 *   Write to a Touch Panel Register over I2C.
 *
 * Input Parameters:
 *   dev - Touch Panel Device
 *   reg - I2C Register to be written
 *   val - Value to be written
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_i2c_write(FAR struct gt9xx_dev_s *dev,
                           uint16_t reg,
                           uint8_t val)
{
  int ret;

  /* Send the Register Address, MSB first */

  uint8_t regbuf[2] =
  {
    reg >> 8,   /* First Byte: MSB */
    reg & 0xff  /* Second Byte: LSB */
  };

  /* Send the Register Value */

  uint8_t buf[1] =
  {
    val  /* Value to be written */
  };

  /* Compose the I2C Messages */

  struct i2c_msg_s msgv[2] =
  {
    {
      /* Send the I2C Register Address */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = regbuf,
      .length    = sizeof(regbuf)
    },
    {
      /* Send the I2C Register Value */

      .frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_NOSTART,
      .buffer    = buf,
      .length    = sizeof(buf)
    }
  };

  const int msgv_len = sizeof(msgv) / sizeof(msgv[0]);

  iinfo("reg=0x%x, val=%d\n", reg, val);
  DEBUGASSERT(dev && dev->i2c);

  /* Execute the I2C Transfer */

  ret = I2C_TRANSFER(dev->i2c, msgv, msgv_len);
  if (ret < 0)
    {
      ierr("I2C Write failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: gt9xx_probe_device
 *
 * Description:
 *   Read the Product ID from the Touch Panel over I2C.
 *
 * Input Parameters:
 *   dev - Touch Panel Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_probe_device(FAR struct gt9xx_dev_s *dev)
{
  int ret;
  uint8_t id[4];

  /* Read the Product ID */

  ret = gt9xx_i2c_read(dev, GTP_REG_VERSION, id, sizeof(id));

#ifdef CONFIG_INPUT_GT9XX_ALT_ADDR
  /* The controller latches its I2C address from the INT level while
   * leaving reset; panels differ between 0x5d and 0x14. Fall back to
   * the complementary address when the primary one is silent */

  if (ret < 0)
    {
      dev->addr = (dev->addr == 0x5d) ? 0x14 : 0x5d;
      iinfo("Primary address silent, retrying at 0x%02x\n", dev->addr);

      ret = gt9xx_i2c_read(dev, GTP_REG_VERSION, id, sizeof(id));
    }
#endif

  if (ret < 0)
    {
      ierr("I2C Probe failed: %d\n", ret);
      return ret;
    }

  /* For GT917S: Product ID will be 39 31 37 53, i.e. "917S" */

#ifdef CONFIG_DEBUG_INPUT_INFO
  iinfodumpbuffer("gt9xx_probe_device", id, sizeof(id));
#endif /* CONFIG_DEBUG_INPUT_INFO */

  return OK;
}

/****************************************************************************
 * Name: gt9xx_ack_frame
 *
 * Description:
 *   Retire the frame the controller is holding by clearing the status
 *   register (0x814E).
 *
 *   The write is issued exactly once per polled frame. A transient
 *   NACK on this write is normal and the next poll cycle simply tries
 *   again; hammering the acknowledge back-to-back instead drives the
 *   controller into a state where every data write is rejected while
 *   reads keep working, and only a power cycle recovers it.
 *
 *   The register pointer and the value travel as one four-byte
 *   transaction (pointer + status clear + track byte): three-byte
 *   data writes are rejected by this I2C master.
 *
 ****************************************************************************/

static int gt9xx_ack_frame(FAR struct gt9xx_dev_s *dev)
{
  struct i2c_msg_s msgv[1];
  uint8_t buf[4];
  int ret;

  DEBUGASSERT(dev);

  buf[0] = GTP_READ_COOR_ADDR >> 8;
  buf[1] = GTP_READ_COOR_ADDR & 0xff;
  buf[2] = 0x00;
  buf[3] = 0x00;

  msgv[0].frequency = CONFIG_INPUT_GT9XX_I2C_FREQUENCY;
  msgv[0].addr      = dev->addr;
  msgv[0].flags     = 0;
  msgv[0].buffer    = buf;
  msgv[0].length    = sizeof(buf);

  ret = I2C_TRANSFER(dev->i2c, msgv, 1);
  if (ret < 0)
    {
      ierr("Ack frame failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: gt9xx_poll_work
 *
 * Description:
 *   Periodic poll work item (HPWORK, re-arms itself every
 *   GT911_POLL_PERIOD_MS). Samples the status register, decodes the
 *   frame and acknowledges it - the acknowledge must reach the
 *   controller within a bounded time or the controller wedges, which
 *   is why this runs independent of application reads.
 *
 *   The decoded result is published into the cached sample that
 *   gt9xx_read() copies out; readers never touch the I2C bus.
 *
 ****************************************************************************/

static void gt9xx_poll_work(FAR void *arg)
{
  FAR struct gt9xx_dev_s *priv = (FAR struct gt9xx_dev_s *)arg;
  struct touch_sample_s sample;
  uint8_t status[1];
  uint8_t touch[8];
  uint8_t status_code;
  uint8_t touched_points;
  uint16_t x;
  uint16_t y;

  DEBUGASSERT(priv != NULL);

  memset(&sample, 0, sizeof(sample));

  /* Read the frame status (one byte at 0x814E) */

  if (gt9xx_i2c_read(priv, GTP_READ_COOR_ADDR, status, sizeof(status)) < 0)
    {
      goto out_reschedule;
    }

  status_code = status[0] & 0x80;
  touched_points = status[0] & 0x0f;

  if (status_code == 0)
    {
      /* No new frame latched. Acknowledge anyway, matching the vendor
       * touch driver: the write is harmless when nothing is pending */

      gt9xx_ack_frame(priv);

      /* If a contact was held, report its release once */

      if (priv->touched)
        {
          priv->touched = false;

          sample.npoints = 1;
          sample.point[0].id = 0;
          sample.point[0].x = priv->x;
          sample.point[0].y = priv->y;
          sample.point[0].flags = TOUCH_UP | TOUCH_ID_VALID |
                                  TOUCH_POS_VALID;
        }
    }
  else if (touched_points >= 1 && touched_points <= 5)
    {
      /* Frame with contacts: the point payload sits behind the status
       * byte, 8 bytes from 0x814F (track, xL, xH, yL, yH, sizeL,
       * sizeH, reserved) */

      if (gt9xx_i2c_read(priv, GTP_READ_COOR_ADDR + 1, touch,
                         sizeof(touch)) < 0)
        {
          gt9xx_ack_frame(priv);
          goto out_reschedule;
        }

      x = touch[1] + (touch[2] << 8);
      y = touch[3] + (touch[4] << 8);

#ifdef CONFIG_INPUT_GT9XX_X_INVERT
      x = CONFIG_INPUT_GT9XX_X_INVERT_MAX - x;
#endif
#ifdef CONFIG_INPUT_GT9XX_Y_INVERT
      y = CONFIG_INPUT_GT9XX_Y_INVERT_MAX - y;
#endif

      /* A frame that could not be acknowledged is re-presented by the
       * controller; report nothing for the re-presentation */

      if ((int)x == priv->unacked_x && (int)y == priv->unacked_y)
        {
          gt9xx_ack_frame(priv);
          goto out_reschedule;
        }

      sample.npoints = 1;
      sample.point[0].id = 0;
      sample.point[0].x = x;
      sample.point[0].y = y;
      sample.point[0].flags = TOUCH_DOWN | TOUCH_ID_VALID |
                              TOUCH_POS_VALID;

      priv->touched = true;
      priv->x = x;
      priv->y = y;
    }
  else
    {
      /* bit 7 set with an out-of-range point count: acknowledge and
       * report nothing */

      gt9xx_ack_frame(priv);
      goto out_reschedule;
    }

  /* Publish the decoded sample for readers */

  nxmutex_lock(&priv->devlock);
  priv->cached = sample;
  nxmutex_unlock(&priv->devlock);

  priv->int_pending = true;
  poll_notify(priv->fds, CONFIG_INPUT_GT9XX_NPOLLWAITERS, POLLIN);

out_reschedule:
  /* Re-arm while the driver is registered */

  if (priv->polling)
    {
      work_queue(HPWORK, &priv->poll_work, gt9xx_poll_work,
                 priv, MSEC2TICK(GT911_POLL_PERIOD_MS));
    }
}

/****************************************************************************
 * Name: gt9xx_read
 *
 * Description:
 *   Read a Touch Sample from Touch Panel. Returns either 0 or 1
 *   Touch Points.
 *
 * Input Parameters:
 *   dev    - Touch Panel Device
 *   buffer - Returned Touch Sample (0 or 1 Touch Points)
 *   buflen - Size of buffer
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static ssize_t gt9xx_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  const size_t outlen = sizeof(struct touch_sample_s);

  if (buflen < outlen)
    {
      ierr("Buffer should be at least %ld bytes, got %ld bytes\n",
           outlen, buflen);
      return -EINVAL;
    }

  /* Get the Touch Panel Device */

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Copy the sample the poll worker last published. All I2C traffic
   * happens on the poll worker; the reader never blocks on the bus */

  nxmutex_lock(&priv->devlock);
  memcpy(buffer, &priv->cached, outlen);
  priv->int_pending = false;
  nxmutex_unlock(&priv->devlock);

  return outlen;
}

/****************************************************************************
 * Name: gt9xx_open
 *
 * Description:
 *   Open the Touch Panel Device.  If this is the first open, we power on
 *   the Touch Panel, probe for the Touch Panel and enable Touch Panel
 *   Interrupts.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  unsigned int use_count;
  int ret;

  /* Get the Touch Panel Device */

  iinfo("\n");
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Reference Count */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  /* Get next Reference Count */

  use_count = priv->cref + 1;
  DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);
  if (use_count == 1)
    {
      /* If first user, power on the Touch Panel */

      DEBUGASSERT(priv->board->set_power != NULL);
      ret = priv->board->set_power(priv->board, true);
      if (ret < 0)
        {
          goto out_lock;
        }

      /* Let Touch Panel power up before probing */

      nxsig_usleep(100 * 1000);

      /* Check that Touch Panel exists on I2C */

      ret = gt9xx_probe_device(priv);
      if (ret < 0)
        {
          /* No such device, power off the Touch Panel */

          priv->board->set_power(priv->board, false);
          goto out_lock;
        }

      /* Enable Touch Panel Interrupts */

      DEBUGASSERT(priv->board->irq_enable);
      priv->board->irq_enable(priv->board, true);
    }

  /* Set the Reference Count */

  priv->cref = use_count;

  /* End Mutex: Unlock to allow update to Reference Count */

out_lock:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: gt9xx_close
 *
 * Description:
 *   Close the Touch Panel Device.  If this is the final close, we disable
 *   Touch Panel Interrupts and power off the Touch Panel.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct gt9xx_dev_s *priv;
  int use_count;
  int ret;

  /* Get the Touch Panel Device */

  iinfo("\n");
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Reference Count */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  /* Decrement the Reference Count */

  use_count = priv->cref - 1;
  DEBUGASSERT(use_count >= 0);
  if (use_count == 0)
    {
      /* If final user, disable Touch Panel Interrupts */

      DEBUGASSERT(priv->board && priv->board->irq_enable);
      priv->board->irq_enable(priv->board, false);

      /* Power off the Touch Panel */

      DEBUGASSERT(priv->board->set_power);
      priv->board->set_power(priv->board, false);
    }

  /* Set the Reference Count */

  priv->cref = use_count;

  /* End Mutex: Unlock to allow update to Reference Count */

  nxmutex_unlock(&priv->devlock);
  return OK;
}

/****************************************************************************
 * Name: gt9xx_poll
 *
 * Description:
 *   Setup or teardown a poll for the Touch Panel Device.
 *
 * Input Parameters:
 *   filep - File Struct for Touch Panel
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup the poll; false: Teardown the poll
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_poll(FAR struct file *filep, FAR struct pollfd *fds,
                      bool setup)
{
  FAR struct gt9xx_dev_s *priv;
  FAR struct inode *inode;
  bool pending;
  int ret = 0;
  int i;

  /* Get the Touch Panel Device */

  iinfo("setup=%d\n", setup);
  DEBUGASSERT(fds);
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Begin Mutex: Lock to prevent concurrent update to Poll Waiters */

  ret = nxmutex_lock(&priv->devlock);
  if (ret < 0)
    {
      ierr("Lock Mutex failed: %d\n", ret);
      return ret;
    }

  if (setup)
    {
      /* If Poll Setup: Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* Find an available slot for the Poll Waiter */

      for (i = 0; i < CONFIG_INPUT_GT9XX_NPOLLWAITERS; i++)
        {
          /* Found an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_INPUT_GT9XX_NPOLLWAITERS)
        {
          /* No slots available */

          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          /* If Interrupt Pending is set, notify the Poll Waiters */

          pending = priv->int_pending;
          if (pending)
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
    }
  else if (fds->priv)
    {
      /* If Poll Teardown: Remove the poll setup */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      *slot = NULL;
      fds->priv = NULL;
    }

  /* End Mutex: Unlock to allow update to Poll Waiters */

out:
  nxmutex_unlock(&priv->devlock);
  return ret;
}

/****************************************************************************
 * Name: gt9xx_isr_handler
 *
 * Description:
 *   Interrupt Handler for Touch Panel.
 *
 * Input Parameters:
 *   irq     - IRQ Number
 *   context - IRQ Context
 *   arg     - Touch Panel Device
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int gt9xx_isr_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct gt9xx_dev_s *priv = (FAR struct gt9xx_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* Begin Critical Section */

  flags = enter_critical_section();

  /* Set the Interrupt Pending Flag */

  priv->int_pending = true;

  /* End Critical Section */

  leave_critical_section(flags);

  /* Notify the Poll Waiters */

  poll_notify(priv->fds, CONFIG_INPUT_GT9XX_NPOLLWAITERS, POLLIN);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gt9xx_register
 *
 * Description:
 *   Register the driver for Goodix GT9XX Touch Panel.  Attach the
 *   Interrupt Handler for the Touch Panel and disable Touch Interrupts.
 *
 * Input Parameters:
 *   devpath      - Device Path (e.g. "/dev/input0")
 *   dev          - I2C Bus
 *   i2c_devaddr  - I2C Address of Touch Panel
 *   board_config - Callback for Board-Specific Operations
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int gt9xx_register(FAR const char *devpath,
                   FAR struct i2c_master_s *i2c_dev,
                   uint8_t i2c_devaddr,
                   const struct gt9xx_board_s *board_config)
{
  struct gt9xx_dev_s *priv;
  int ret = 0;

  iinfo("devpath=%s, i2c_devaddr=%d\n", devpath, i2c_devaddr);
  DEBUGASSERT(devpath != NULL && i2c_dev != NULL && board_config != NULL);

  /* Allocate the Touch Panel Device Structure */

  priv = kmm_zalloc(sizeof(struct gt9xx_dev_s));
  if (!priv)
    {
      ierr("GT9XX Memory Allocation failed\n");
      return -ENOMEM;
    }

  /* Setup the Touch Panel Device Structure */

  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;
  priv->board = board_config;
  priv->unacked_x = -1;
  priv->unacked_y = -1;
  nxmutex_init(&priv->devlock);

  /* Register the Touch Input Driver */

  ret = register_driver(devpath, &g_gt9xx_fileops, 0666, priv);
  if (ret < 0)
    {
      nxmutex_destroy(&priv->devlock);
      kmm_free(priv);
      ierr("GT9XX Registration failed: %d\n", ret);
      return ret;
    }

  /* Attach the Interrupt Handler */

  DEBUGASSERT(priv->board->irq_attach);
  priv->board->irq_attach(priv->board, gt9xx_isr_handler, priv);

  /* Disable Touch Panel Interrupts */

  DEBUGASSERT(priv->board->irq_enable);
  priv->board->irq_enable(priv->board, false);

  iinfo("GT9XX Touch Panel registered\n");

  /* Start the periodic poll worker: it owns all I2C traffic and runs
   * for the lifetime of the driver */

  priv->polling = true;
  work_queue(HPWORK, &priv->poll_work, gt9xx_poll_work,
             priv, MSEC2TICK(GT911_POLL_PERIOD_MS));

  return OK;
}
