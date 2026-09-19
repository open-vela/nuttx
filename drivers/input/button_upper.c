/****************************************************************************
 * drivers/input/button_upper.c
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

/* This file provides a driver for a button input devices.
 *
 * The buttons driver exports a standard character driver interface. By
 * convention, the button driver is registered as an input device at
 * /dev/btnN where N uniquely identifies the driver instance.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/input/buttons.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the state of one button driver */

struct btn_upperhalf_s
{
  /* Saved binding to the lower half button driver */

  FAR const struct btn_lowerhalf_s *bu_lower;

  btn_buttonset_t bu_sample; /* Last sampled button states */
  bool bu_enabled;
  spinlock_t lock;           /* Spinlock for this button driver */
  mutex_t mutex;             /* Mutex for this button driver */

  /* The following is a singly linked list of open references to the
   * button device.
   */

  FAR struct btn_open_s *bu_open;

#if CONFIG_INPUT_BUTTONS_DEBOUNCE_DELAY
  struct wdog_s bu_wdog;
#endif
};

/* This structure describes the state of one open button driver instance */

struct btn_open_s
{
  /* Supports a singly linked list */

  FAR struct btn_open_s *bo_flink;

  /* Button event notification information */

  pid_t bo_pid;
  struct btn_notify_s bo_notify;
  struct sigwork_s bo_work;

  /* Poll event information */

  struct btn_pollevents_s bo_pollevents;

  /* The following is a list if poll structures of threads waiting for
   * driver events.
   */

  bool bo_pending;
  FAR struct pollfd *bo_fds[CONFIG_INPUT_BUTTONS_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Sampling and Interrupt handling */

static void    btn_enable(FAR struct btn_upperhalf_s *priv);
static void    btn_interrupt(FAR const struct btn_lowerhalf_s *lower,
                             FAR void *arg);

/* Sampling */

static void    btn_sample(wdparm_t arg);

/* Character driver methods */

static int     btn_open(FAR struct file *filep);
static int     btn_close(FAR struct file *filep);
static ssize_t btn_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t btn_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     btn_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);
static int     btn_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_btn_fops =
{
  btn_open,  /* open */
  btn_close, /* close */
  btn_read,  /* read */
  btn_write, /* write */
  NULL,      /* seek */
  btn_ioctl, /* ioctl */
  NULL,      /* mmap */
  NULL,      /* truncate */
  btn_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btn_enable
 ****************************************************************************/

static void btn_enable(FAR struct btn_upperhalf_s *priv)
{
  FAR const struct btn_lowerhalf_s *lower;
  FAR struct btn_open_s *opriv;
  btn_buttonset_t press;
  btn_buttonset_t release;
  irqstate_t flags;

  DEBUGASSERT(priv && priv->bu_lower);
  lower = priv->bu_lower;
  DEBUGASSERT(lower->bl_enable);

  /* This routine is called both task level and interrupt level, so
   * interrupts must be disabled.
   */

  flags = spin_lock_irqsave(&priv->lock);

  /* Visit each opened reference to the device */

  press   = 0;
  release = 0;

  for (opriv = priv->bu_open; opriv; opriv = opriv->bo_flink)
    {
      /* OR in the poll event buttons */

      press   |= opriv->bo_pollevents.bp_press;
      release |= opriv->bo_pollevents.bp_release;

      /* OR in the signal events */

      press   |= opriv->bo_notify.bn_press;
      release |= opriv->bo_notify.bn_release;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  nxmutex_lock(&priv->mutex);

  /* Enable/disable button interrupts */

  if (press != 0 || release != 0)
    {
      /* Update last sampled button states when enabling interrupts for
       * the first time.
       */

      if (!priv->bu_enabled)
        {
          priv->bu_enabled = true;
          priv->bu_sample = lower->bl_buttons(lower);
        }

      /* Enable interrupts with the new button set */

      lower->bl_enable(lower, press, release,
                       (btn_handler_t)btn_interrupt, priv);
    }
  else
    {
      priv->bu_enabled = false;

      /* Disable further interrupts */

      lower->bl_enable(lower, 0, 0, NULL, NULL);
    }

    nxmutex_unlock(&priv->mutex);
}

/****************************************************************************
 * Name: btn_interrupt
 ****************************************************************************/

static void btn_interrupt(FAR const struct btn_lowerhalf_s *lower,
                          FAR void *arg)
{
  FAR struct btn_upperhalf_s *priv = (FAR struct btn_upperhalf_s *)arg;

  DEBUGASSERT(priv);

  /* Process the next sample */

#if CONFIG_INPUT_BUTTONS_DEBOUNCE_DELAY
  wd_start(&priv->bu_wdog, MSEC2TICK(CONFIG_INPUT_BUTTONS_DEBOUNCE_DELAY),
           btn_sample, (wdparm_t)priv);
#else
  btn_sample((wdparm_t)priv);
#endif
}

/****************************************************************************
 * Name: btn_sample
 ****************************************************************************/

static void btn_sample(wdparm_t arg)
{
  FAR struct btn_upperhalf_s *priv;
  FAR const struct btn_lowerhalf_s *lower;
  FAR struct btn_open_s *opriv;
  btn_buttonset_t sample;
  btn_buttonset_t change;
  btn_buttonset_t press;
  btn_buttonset_t release;
  irqstate_t flags;

  priv = (FAR struct btn_upperhalf_s *)arg;
  DEBUGASSERT(priv && priv->bu_lower);
  lower = priv->bu_lower;

  /* Sample the new button state */

  DEBUGASSERT(lower->bl_buttons);
  sample = lower->bl_buttons(lower);

  add_ui_randomness(sample);

  /* Determine which buttons have been newly pressed and which have been
   * newly released.
   */

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  change = sample ^ priv->bu_sample;
  press  = change & sample;

  DEBUGASSERT(lower->bl_supported);
  release = change & (lower->bl_supported(lower) & ~sample);

  /* Visit each opened reference to the device */

  for (opriv = priv->bu_open; opriv; opriv = opriv->bo_flink)
    {
      /* Always set bo_pending true, only clear it after button read */

      opriv->bo_pending = true;

      /* Have any poll events occurred? */

      if ((press & opriv->bo_pollevents.bp_press)     != 0 ||
          (release & opriv->bo_pollevents.bp_release) != 0)
        {
          /* Yes.. Notify all waiters */

          poll_notify(opriv->bo_fds, CONFIG_INPUT_BUTTONS_NPOLLWAITERS,
                      POLLIN);
        }

      /* Have any signal events occurred? */

      if ((press & opriv->bo_notify.bn_press)     != 0 ||
          (release & opriv->bo_notify.bn_release) != 0)
        {
          /* Yes.. Signal the waiter */

          opriv->bo_notify.bn_event.sigev_value.sival_int = sample;
          nxsig_notification(opriv->bo_pid, &opriv->bo_notify.bn_event,
                             SI_QUEUE, &opriv->bo_work);
        }
    }

  priv->bu_sample = sample;
  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
}

/****************************************************************************
 * Name: btn_open
 ****************************************************************************/

static int btn_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  FAR const struct btn_lowerhalf_s *lower;
  btn_buttonset_t supported;
  irqstate_t flags;

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  /* Allocate a new open structure */

  opriv = kmm_zalloc(sizeof(struct btn_open_s));
  if (!opriv)
    {
      ierr("ERROR: Failed to allocate open structure\n");
      return -ENOMEM;
    }

  /* Initialize the open structure */

  lower = priv->bu_lower;
  DEBUGASSERT(lower && lower->bl_supported);

  supported = lower->bl_supported(lower);
  opriv->bo_pollevents.bp_press   = supported;
  opriv->bo_pollevents.bp_release = supported;
  opriv->bo_pending = true;

  /* Attach the open structure to the device */

  flags = spin_lock_irqsave(&priv->lock);

  opriv->bo_flink = priv->bu_open;
  priv->bu_open = opriv;

  spin_unlock_irqrestore(&priv->lock, flags);

  /* Attach the open structure to the file structure */

  filep->f_priv = (FAR void *)opriv;

  /* Enable/disable interrupt handling */

  btn_enable(priv);

  return OK;
}

/****************************************************************************
 * Name: btn_close
 ****************************************************************************/

static int btn_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  FAR struct btn_open_s *curr;
  FAR struct btn_open_s *prev;
  irqstate_t flags;

  DEBUGASSERT(filep->f_priv);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = inode->i_private;

  /* Get exclusive access to the driver structure */

#if CONFIG_INPUT_BUTTONS_DEBOUNCE_DELAY
  wd_cancel(&priv->bu_wdog);
#endif

  /* Find the open structure in the list of open structures for the device */

  flags = spin_lock_irqsave(&priv->lock);

  for (prev = NULL, curr = priv->bu_open;
       curr && curr != opriv;
       prev = curr, curr = curr->bo_flink);

  DEBUGASSERT(curr);
  if (!curr)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      ierr("ERROR: Failed to find open entry\n");
      return -ENOENT;
    }

  /* Remove the structure from the device */

  if (prev)
    {
      prev->bo_flink = opriv->bo_flink;
    }
  else
    {
      priv->bu_open = opriv->bo_flink;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  /* Enable/disable interrupt handling */

  btn_enable(priv);

  /* Cancel any pending notification */

  nxsig_cancel_notification(&opriv->bo_work);

  /* And free the open structure */

  kmm_free(opriv);
  return OK;
}

/****************************************************************************
 * Name: btn_read
 ****************************************************************************/

static ssize_t btn_read(FAR struct file *filep, FAR char *buffer,
                        size_t len)
{
  FAR struct inode *inode;
  FAR struct btn_open_s *opriv;
  FAR struct btn_upperhalf_s *priv;
  FAR const struct btn_lowerhalf_s *lower;
  irqstate_t flags;

  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   *
   * REVISIT:  Should also check buffer alignment.
   */

  if (len < sizeof(btn_buttonset_t))
    {
      ierr("ERROR: buffer too small: %zu\n", len);
      return -EINVAL;
    }

  /* Get exclusive access to the driver structure */

  nxmutex_lock(&priv->mutex);

  /* A non-blocking reader must not consume the same current state forever.
   * The buttons example drains read() until it gets EAGAIN; return that
   * result once the notification that woke the reader has already been
   * consumed.
   */

  flags = spin_lock_irqsave(&priv->lock);
  if (!opriv->bo_pending && (filep->f_oflags & O_NONBLOCK) != 0)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      nxmutex_unlock(&priv->mutex);
      return -EAGAIN;
    }

  /* Consume the notification that caused this read before sampling the
   * hardware.  A GPIO interrupt may run while bl_buttons() is sampling the
   * pins and set bo_pending again.  Clearing it after the sample would race
   * with that interrupt and lose the new event before the next poll().
   */

  opriv->bo_pending = false;
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Read and return the current state of the buttons */

  lower = priv->bu_lower;
  DEBUGASSERT(lower && lower->bl_buttons);
  *(FAR btn_buttonset_t *)buffer = lower->bl_buttons(lower);

  nxmutex_unlock(&priv->mutex);
  return (ssize_t)sizeof(btn_buttonset_t);
}

/****************************************************************************
 * Name: btn_write
 ****************************************************************************/

static ssize_t btn_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR const struct btn_lowerhalf_s *lower;
  int ret;

  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = inode->i_private;

  /* Make sure that the buffer is sufficiently large to hold at least one
   * complete sample.
   *
   * REVISIT:  Should also check buffer alignment.
   */

  if (buflen < sizeof(btn_buttonset_t))
    {
      ierr("ERROR: buffer too small: %zu\n", buflen);
      return -EINVAL;
    }

  /* Get exclusive access to the driver structure */

  nxmutex_lock(&priv->mutex);

  /* Write the current state of the buttons */

  lower = priv->bu_lower;
  DEBUGASSERT(lower);
  if (lower->bl_write)
    {
      ret = lower->bl_write(lower, buffer, buflen);
    }
  else
    {
      ret = -ENOSYS;
    }

  nxmutex_unlock(&priv->mutex);
  return (ssize_t)ret;
}

/****************************************************************************
 * Name: btn_ioctl
 ****************************************************************************/

static int btn_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  FAR const struct btn_lowerhalf_s *lower;
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(filep->f_priv);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = inode->i_private;

  /* Handle the ioctl command */

  ret = -EINVAL;
  switch (cmd)
    {
    /* Command:     BTNIOC_SUPPORTED
     * Description: Report the set of button events supported by the
     *              hardware;
     * Argument:    A pointer to writeable integer value in which to return
     *              the set of supported buttons.
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case BTNIOC_SUPPORTED:
      {
        FAR btn_buttonset_t *supported =
          (FAR btn_buttonset_t *)((uintptr_t)arg);

        if (supported)
          {
            lower = priv->bu_lower;
            DEBUGASSERT(lower && lower->bl_supported);

            *supported = lower->bl_supported(lower);
            ret = OK;
          }
      }
      break;

    /* Command:     BTNIOC_POLLEVENTS
     * Description: Specify the set of button events that can cause a poll()
     *              to awaken.  The default is all button depressions and
     *              all button releases (all supported buttons);
     * Argument:    A read-only pointer to an instance of struct
     *              btn_pollevents_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case BTNIOC_POLLEVENTS:
      {
        FAR struct btn_pollevents_s *pollevents =
          (FAR struct btn_pollevents_s *)((uintptr_t)arg);

        if (pollevents)
          {
            /* Save the poll events */

            flags = spin_lock_irqsave(&priv->lock);

            opriv->bo_pollevents.bp_press   = pollevents->bp_press;
            opriv->bo_pollevents.bp_release = pollevents->bp_release;

            spin_unlock_irqrestore(&priv->lock, flags);

            /* Enable/disable interrupt handling */

            btn_enable(priv);
            ret = OK;
          }
      }
      break;

    /* Command:     BTNIOC_REGISTER
     * Description: Register to receive a signal whenever there is a change
     *              in any of the discrete buttone inputs.  This feature,
     *              of course, depends upon interrupt GPIO support from the
     *              platform.
     * Argument:    A read-only pointer to an instance of struct
     *              btn_notify_s
     * Return:      Zero (OK) on success.  Minus one will be returned on
     *              failure with the errno value set appropriately.
     */

    case BTNIOC_REGISTER:
      {
        FAR struct btn_notify_s *notify =
          (FAR struct btn_notify_s *)((uintptr_t)arg);

        if (notify)
          {
            /* Save the notification events */

            flags = spin_lock_irqsave(&priv->lock);

            opriv->bo_notify.bn_press   = notify->bn_press;
            opriv->bo_notify.bn_release = notify->bn_release;
            opriv->bo_notify.bn_event   = notify->bn_event;
            opriv->bo_pid               = nxsched_getpid();

            spin_unlock_irqrestore(&priv->lock, flags);

            /* Enable/disable interrupt handling */

            btn_enable(priv);
            ret = OK;
          }
      }
      break;

    default:
      iinfo("ERROR: Unrecognized command: %d\n", cmd);
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: btn_poll
 ****************************************************************************/

static int btn_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct inode *inode;
  FAR struct btn_upperhalf_s *priv;
  FAR struct btn_open_s *opriv;
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep->f_priv);
  opriv = filep->f_priv;
  inode = filep->f_inode;
  DEBUGASSERT(inode->i_private);
  priv  = inode->i_private;

  /* Get exclusive access to the driver structure */

  flags = spin_lock_irqsave_nopreempt(&priv->lock);

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_INPUT_BUTTONS_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!opriv->bo_fds[i])
            {
              /* Bind the poll structure and this slot */

              opriv->bo_fds[i] = fds;
              fds->priv = &opriv->bo_fds[i];

              /* Report if the event is pending */

              if (opriv->bo_pending)
                {
                  poll_notify(&fds, 1, POLLIN);
                }

              break;
            }
        }

      if (i >= CONFIG_INPUT_BUTTONS_NPOLLWAITERS)
        {
          spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
          ierr("ERROR: Too many poll waiters\n");
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

#ifdef CONFIG_DEBUG_FEATURES
      if (!slot)
        {
          spin_unlock_irqrestore_nopreempt(&priv->lock, flags);
          ierr("ERROR: Poll slot not found\n");
          ret = -EIO;
          goto errout;
        }
#endif

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
    }

  spin_unlock_irqrestore_nopreempt(&priv->lock, flags);

  btn_enable(priv);

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: btn_register
 *
 * Description:
 *   Bind the lower half button driver to an instance of the upper half
 *   button driver and register the composite character driver as the
 *   specified device.
 *
 * Input Parameters:
 *   devname - The name of the button device to be registered.
 *     This should be a string of the form "/dev/btnN" where N is the
 *     minor device number.
 *   lower - An instance of the platform-specific button lower half driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int btn_register(FAR const char *devname,
                 FAR const struct btn_lowerhalf_s *lower)
{
  FAR struct btn_upperhalf_s *priv;
  int ret;

  DEBUGASSERT(devname && lower);

  /* Allocate a new button driver instance */

  priv = (FAR struct btn_upperhalf_s *)
    kmm_zalloc(sizeof(struct btn_upperhalf_s));
  if (!priv)
    {
      ierr("ERROR: Failed to allocate device structure\n");
      return -ENOMEM;
    }

  spin_lock_init(&priv->lock);
  nxmutex_init(&priv->mutex);

  /* Make sure that all button interrupts are disabled */

  DEBUGASSERT(lower->bl_enable);
  lower->bl_enable(lower, 0, 0, NULL, NULL);

  /* Initialize the new button driver instance */

  priv->bu_lower = lower;

  /* And register the button driver */

  ret = register_driver(devname, &g_btn_fops, 0666, priv);
  if (ret < 0)
    {
      ierr("ERROR: register_driver failed: %d\n", ret);
      nxmutex_destroy(&priv->mutex);
      kmm_free(priv);
    }

  return ret;
}
