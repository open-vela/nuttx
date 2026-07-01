/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_usb.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_usb.h"
#include "rk3576_usb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB timeout (microseconds) */

#define USB_TIMEOUT_US              100000
#define USB_RESET_TIMEOUT_US        10000

/* USB FIFO sizes (in words) */

#define USB_RX_FIFO_SIZE            256
#define USB_NPTX_FIFO_SIZE          256
#define USB_PTX_FIFO_SIZE           256

/* Maximum endpoints */

#define USB_MAX_EP                  4

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB controller base addresses */

const uint32_t g_usb_base[RK3576_USB_COUNT] =
{
  RK3576_USB0_ADDR,
  RK3576_USB1_ADDR,
};

/* Per-controller state */

struct rk3576_usb_dev_s
{
  int mode;                     /* Current mode */
  int speed;                    /* Current speed */
  int addr;                     /* Device address */
  bool connected;               /* Connected status */
  bool initialized;             /* Initialized flag */
};

static struct rk3576_usb_dev_s g_usb_dev[RK3576_USB_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_usb_validate
 *
 * Description:
 *   Validate USB port number.
 *
 ****************************************************************************/

static inline int rk3576_usb_validate(int port)
{
  if (port < 0 || port >= RK3576_USB_COUNT)
    {
      uerr("USB: invalid port %d\n", port);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_wait_ahb_idle
 *
 * Description:
 *   Wait for AHB master to become idle.
 *
 ****************************************************************************/

static int rk3576_usb_wait_ahb_idle(int port)
{
  uint32_t base = g_usb_base[port];
  int timeout = USB_TIMEOUT_US;

  while (timeout--)
    {
      if (getreg32(base + GRSTCTL) & GRSTCTL_AHBIDLE)
        {
          return OK;
        }

      up_udelay(1);
    }

  uerr("USB%d: AHB idle timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_usb_wait_soft_reset
 *
 * Description:
 *   Wait for core soft reset to complete.
 *
 ****************************************************************************/

static int rk3576_usb_wait_soft_reset(int port)
{
  uint32_t base = g_usb_base[port];
  int timeout = USB_TIMEOUT_US;

  while (timeout--)
    {
      if (!(getreg32(base + GRSTCTL) & GRSTCTL_CSFTRST))
        {
          return OK;
        }

      up_udelay(1);
    }

  uerr("USB%d: soft reset timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_usb_flush_fifo
 *
 * Description:
 *   Flush TX or RX FIFO.
 *
 ****************************************************************************/

static int rk3576_usb_flush_fifo(int port, bool tx, int fifo_num)
{
  uint32_t base = g_usb_base[port];
  uint32_t grstctl;
  int timeout = USB_TIMEOUT_US;

  grstctl = getreg32(base + GRSTCTL);

  if (tx)
    {
      grstctl |= GRSTCTL_TXFFLSH | GRSTCTL_TXFNUM(fifo_num);
    }
  else
    {
      grstctl |= GRSTCTL_RXFFLSH;
    }

  putreg32(grstctl, base + GRSTCTL);

  /* Wait for flush to complete */

  while (timeout--)
    {
      grstctl = getreg32(base + GRSTCTL);
      if (!(grstctl & (GRSTCTL_TXFFLSH | GRSTCTL_RXFFLSH)))
        {
          return OK;
        }

      up_udelay(1);
    }

  uerr("USB%d: FIFO flush timeout\n", port);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_usb_core_reset
 *
 * Description:
 *   Perform core soft reset.
 *
 ****************************************************************************/

static int rk3576_usb_core_reset(int port)
{
  uint32_t base = g_usb_base[port];
  int ret;

  /* Wait for AHB idle */

  ret = rk3576_usb_wait_ahb_idle(port);
  if (ret < 0)
    {
      return ret;
    }

  /* Assert core soft reset */

  putreg32(GRSTCTL_CSFTRST, base + GRSTCTL);

  /* Wait for reset to complete */

  ret = rk3576_usb_wait_soft_reset(port);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for AHB idle again */

  ret = rk3576_usb_wait_ahb_idle(port);
  if (ret < 0)
    {
      return ret;
    }

  uinfo("USB%d: core reset complete\n", port);
  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_set_fifo_sizes
 *
 * Description:
 *   Configure FIFO sizes.
 *
 ****************************************************************************/

static void rk3576_usb_set_fifo_sizes(int port)
{
  uint32_t base = g_usb_base[port];

  /* Set RX FIFO size */

  putreg32(USB_RX_FIFO_SIZE, base + GRXFSIZ);

  /* Set Non-Periodic TX FIFO size and address */

  putreg32((USB_NPTX_FIFO_SIZE << 16) | USB_RX_FIFO_SIZE, base + GNPTXFSIZ);

  /* Set Host Periodic TX FIFO size and address */

  putreg32((USB_PTX_FIFO_SIZE << 16) | (USB_RX_FIFO_SIZE + USB_NPTX_FIFO_SIZE),
           base + HPTXFSIZ);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_usb_init
 *
 * Description:
 *   Initialize USB controller.
 *
 ****************************************************************************/

void rk3576_usb_init(int port)
{
  uint32_t base;
  int ret;

  if (rk3576_usb_validate(port) < 0)
    {
      return;
    }

  base = g_usb_base[port];

  /* Initialize state */

  memset(&g_usb_dev[port], 0, sizeof(struct rk3576_usb_dev_s));
  g_usb_dev[port].mode = RK3576_USB_MODE_HOST;
  g_usb_dev[port].speed = RK3576_USB_SPEED_HIGH;

  /* Disable all interrupts */

  putreg32(0, base + GINTMSK);
  putreg32(0xffffffff, base + GINTSTS);

  /* Core reset */

  ret = rk3576_usb_core_reset(port);
  if (ret < 0)
    {
      uerr("USB%d: init failed\n", port);
      return;
    }

  /* Configure FIFO sizes */

  rk3576_usb_set_fifo_sizes(port);

  /* Configure AHB:
   * - Single burst
   * - Global interrupt enable
   */

  putreg32(GAHBCFG_HBSTLEN_SINGLE | GAHBCFG_GLBINTMASK_EN,
           base + GAHBCFG);

  /* Configure USB:
   * - PHY 8-bit interface
   * - SRP capable
   */

  putreg32(GUSBCFG_PHYIF, base + GUSBCFG);

  /* Enable VBUS */

  putreg32(GOTGCTL_VBVALOEN | GOTGCTL_VBVALOVAL, base + GOTGCTL);

  /* Flush all FIFOs */

  rk3576_usb_flush_fifo(port, false, 0);   /* RX FIFO */
  rk3576_usb_flush_fifo(port, true, 0);    /* TX FIFO 0 */

  g_usb_dev[port].initialized = true;

  uinfo("USB%d: initialized\n", port);
}

/****************************************************************************
 * Name: rk3576_usb_deinit
 *
 * Description:
 *   Deinitialize USB controller.
 *
 ****************************************************************************/

void rk3576_usb_deinit(int port)
{
  uint32_t base;

  if (rk3576_usb_validate(port) < 0)
    {
      return;
    }

  base = g_usb_base[port];

  /* Disable all interrupts */

  putreg32(0, base + GINTMSK);
  putreg32(0, base + GAHBCFG);

  /* Power down */

  putreg32(PCGCCTL_STOPCLK | PCGCCTL_GATEHCLK, base + PCGCCTL);

  g_usb_dev[port].initialized = false;

  uinfo("USB%d: deinitialized\n", port);
}

/****************************************************************************
 * Name: rk3576_usb_set_mode
 *
 * Description:
 *   Set USB mode (host, device, or OTG).
 *
 ****************************************************************************/

int rk3576_usb_set_mode(int port, int mode)
{
  uint32_t base;
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  base = g_usb_base[port];

  /* Core reset */

  ret = rk3576_usb_core_reset(port);
  if (ret < 0)
    {
      return ret;
    }

  switch (mode)
    {
      case RK3576_USB_MODE_HOST:
        /* Configure for host mode */

        putreg32(HCFG_FSLSPCLKSEL_48MHZ | HCFG_FSLSSUPP, base + HCFG);
        g_usb_dev[port].mode = RK3576_USB_MODE_HOST;
        break;

      case RK3576_USB_MODE_DEVICE:
        /* Configure for device mode */

        {
          uint32_t dcfg = getreg32(base + DCFG);
          dcfg &= ~DCFG_DEVSPD_MASK;
          dcfg |= DCFG_DEVSPD_HIGH;
          putreg32(dcfg, base + DCFG);
        }

        /* Enable device interrupts */

        putreg32(GINTSTS_OINT | GINTSTS_IINT, base + GINTMSK);
        g_usb_dev[port].mode = RK3576_USB_MODE_DEVICE;
        break;

      case RK3576_USB_MODE_OTG:
        /* OTG mode - auto detect */

        g_usb_dev[port].mode = RK3576_USB_MODE_OTG;
        break;

      default:
        return -EINVAL;
    }

  /* Enable global interrupts */

  putreg32(GAHBCFG_GLBINTMASK_EN, base + GAHBCFG);

  uinfo("USB%d: mode %d\n", port, mode);
  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_host_reset
 *
 * Description:
 *   Perform USB host port reset.
 *
 ****************************************************************************/

int rk3576_usb_host_reset(int port)
{
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  /* Host port reset sequence:
   * 1. Assert reset
   * 2. Wait 10ms
   * 3. Deassert reset
   * 4. Wait for enumeration
   */

  uinfo("USB%d: host reset\n", port);

  /* Note: Actual port reset requires specific register handling
   * based on the exact DWC2 version. This is a placeholder. */

  g_usb_dev[port].connected = false;

  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_host_get_speed
 *
 * Description:
 *   Get the speed of the connected USB device.
 *
 ****************************************************************************/

int rk3576_usb_host_get_speed(int port)
{
  if (rk3576_usb_validate(port) < 0)
    {
      return -EINVAL;
    }

  return g_usb_dev[port].speed;
}

/****************************************************************************
 * Name: rk3576_usb_host_connect
 *
 * Description:
 *   Handle USB device connection in host mode.
 *
 ****************************************************************************/

int rk3576_usb_host_connect(int port)
{
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  g_usb_dev[port].connected = true;
  g_usb_dev[port].speed = RK3576_USB_SPEED_HIGH;

  uinfo("USB%d: device connected (speed=%s)\n", port,
          g_usb_dev[port].speed == 0 ? "full" : "high");

  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_host_disconnect
 *
 * Description:
 *   Handle USB device disconnection in host mode.
 *
 ****************************************************************************/

int rk3576_usb_host_disconnect(int port)
{
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  g_usb_dev[port].connected = false;

  uinfo("USB%d: device disconnected\n", port);

  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_device_set_address
 *
 * Description:
 *   Set USB device address.
 *
 ****************************************************************************/

int rk3576_usb_device_set_address(int port, int addr)
{
  uint32_t base;
  uint32_t dcfg;
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  base = g_usb_base[port];

  /* Set device address */

  dcfg = getreg32(base + DCFG);
  dcfg &= ~DCFG_DAD_MASK;
  dcfg |= (addr << DCFG_DAD_SHIFT) & DCFG_DAD_MASK;
  putreg32(dcfg, base + DCFG);

  g_usb_dev[port].addr = addr;

  uinfo("USB%d: device address set to %d\n", port, addr);
  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_device_connect
 *
 * Description:
 *   Connect USB device (pull-up D+).
 *
 ****************************************************************************/

int rk3576_usb_device_connect(int port)
{
  uint32_t base;
  uint32_t dctl;
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  base = g_usb_base[port];

  /* Clear soft disconnect */

  dctl = getreg32(base + DCTL);
  dctl &= ~DCTL_SFTDIS;
  putreg32(dctl, base + DCTL);

  g_usb_dev[port].connected = true;

  uinfo("USB%d: device connected\n", port);
  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_device_disconnect
 *
 * Description:
 *   Disconnect USB device (drive D- low).
 *
 ****************************************************************************/

int rk3576_usb_device_disconnect(int port)
{
  uint32_t base;
  uint32_t dctl;
  int ret;

  ret = rk3576_usb_validate(port);
  if (ret < 0)
    {
      return ret;
    }

  base = g_usb_base[port];

  /* Set soft disconnect */

  dctl = getreg32(base + DCTL);
  dctl |= DCTL_SFTDIS;
  putreg32(dctl, base + DCTL);

  g_usb_dev[port].connected = false;

  uinfo("USB%d: device disconnected\n", port);
  return OK;
}

/****************************************************************************
 * Name: rk3576_usb_get_status
 *
 * Description:
 *   Get USB controller status.
 *
 ****************************************************************************/

int rk3576_usb_get_status(int port)
{
  if (rk3576_usb_validate(port) < 0)
    {
      return -EINVAL;
    }

  return g_usb_dev[port].initialized ? 0 : -ENODEV;
}
