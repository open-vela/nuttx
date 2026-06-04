/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_boardusb.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbhost.h>

#include "arm_internal.h"
#include "t113_usbhost.h"

#ifdef CONFIG_T113_USBHOST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Enumeration/connect waiter kthread priority.  Matches the convention
 * used by the STM32H7 / i.MX RT / Kinetis boards of using
 * CONFIG_USBHOST_DEFPRIO so the waiter runs at the same priority as other
 * USB class threads; priority-inverted user tasks would otherwise delay
 * hot-plug enumeration visibly.  CONFIG_USBHOST_DEFPRIO is not a Kconfig
 * symbol; each board supplies a local fallback.
 */

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 100
#endif

#define T113_USBHOST_WAITER_PRIO  CONFIG_USBHOST_DEFPRIO
#define T113_USBHOST_WAITER_STACK 3072

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_T113_EHCI
static struct usbhost_connection_s *g_ehciconn;
#endif

#ifdef CONFIG_T113_OHCI
static struct usbhost_connection_s *g_ohciconn;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int usbhost_waiter(struct usbhost_connection_s *conn, const char *tag)
{
  struct usbhost_hubport_s *hport;

  for (; ; )
    {
      DEBUGVERIFY(CONN_WAIT(conn, &hport));
      syslog(LOG_INFO, "t113_usbhost[%s]: port %s\n",
             tag, hport->connected ? "connected" : "disconnected");
      if (hport->connected)
        {
          int ret = CONN_ENUMERATE(conn, hport);
          if (ret < 0)
            {
              syslog(LOG_ERR, "t113_usbhost[%s]: enumerate failed: %d\n",
                     tag, ret);
            }
        }
    }

  return 0;
}

#ifdef CONFIG_T113_EHCI
static int ehci_waiter(int argc, char *argv[])
{
  return usbhost_waiter(g_ehciconn, "EHCI");
}
#endif

#ifdef CONFIG_T113_OHCI
static int ohci_waiter(int argc, char *argv[])
{
  return usbhost_waiter(g_ohciconn, "OHCI");
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_usbhost_initialize(void)
{
  int ret;

  /* Bring up the SoC USB infrastructure (CCU clocks, PHY, reset) before
   * anything else touches the controller or class-driver stacks.  Matches
   * the ordering in other NuttX boards (SAMA5, LPC43) and avoids a future
   * regression if any class registration grows a dependency on PHY state.
   */

  ret = t113_usbhost_common_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "board_usbhost: common_init failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_USBHOST_HUB
  ret = usbhost_hub_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "board_usbhost: hub init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_FT232R
  ret = usbhost_ft232r_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "board_usbhost: ft232r init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_MSC
  ret = usbhost_msc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "board_usbhost: msc init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_T113_EHCI
  g_ehciconn = t113_ehci_initialize(0);
  if (g_ehciconn == NULL)
    {
      syslog(LOG_ERR, "board_usbhost: EHCI init returned NULL\n");
    }
  else
    {
      syslog(LOG_INFO, "board_usbhost: EHCI ready\n");
      ret = kthread_create("EHCI Waiter",
                           T113_USBHOST_WAITER_PRIO,
                           T113_USBHOST_WAITER_STACK,
                           ehci_waiter, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "board_usbhost: ehci_waiter spawn failed: %d\n",
                 ret);
        }
    }
#endif

#ifdef CONFIG_T113_OHCI
  g_ohciconn = t113_ohci_initialize(0);
  if (g_ohciconn == NULL)
    {
      syslog(LOG_ERR, "board_usbhost: OHCI init returned NULL\n");
    }
  else
    {
      syslog(LOG_INFO, "board_usbhost: OHCI ready\n");
      ret = kthread_create("OHCI Waiter",
                           T113_USBHOST_WAITER_PRIO,
                           T113_USBHOST_WAITER_STACK,
                           ohci_waiter, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "board_usbhost: ohci_waiter spawn failed: %d\n",
                 ret);
        }
    }
#endif

  return OK;
}

#endif /* CONFIG_T113_USBHOST */
