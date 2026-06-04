/****************************************************************************
 * arch/arm/src/t113/t113_usbhost.c
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

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_usbhost.h"
#include "t113_ccu.h"
#include "t113_usbhost.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void usb_clrbits(uintptr_t reg, uint32_t bits)
{
  putreg32(getreg32(reg) & ~bits, reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int t113_usbhost_common_init(void)
{
  /* 1) Bus gating: ungate EHCI1 and OHCI1 AHB clocks */

  t113_ccu_modify(T113_CCU_USB_BGR, 0,
                  USB_BGR_USBEHCI1_GATING | USB_BGR_USBOHCI1_GATING);

  /* 2) Reset de-assert: EHCI1, OHCI1, PHY1 */

  t113_ccu_modify(T113_CCU_USB_BGR, 0,
                  USB_BGR_USBEHCI1_RST | USB_BGR_USBOHCI1_RST);
  t113_ccu_modify(T113_CCU_USB1_CLK, 0, USB1_CLK_USBPHY1_RSTN);

  /* 3) OHCI 12 MHz special clock (mainline: missing this is the #1
   *    port-hang cause).  USB1_CLK12M_SEL = 00 selects 12M from 48M,
   *    which is the default; clear+leave is defensive in case a
   *    warm-boot left stale bits.
   */

  t113_ccu_modify(T113_CCU_USB1_CLK, USB1_CLK_CLK12M_SEL_MASK, 0);
  t113_ccu_modify(T113_CCU_USB1_CLK, 0, USB1_CLK_USB1_CLKEN);

  up_udelay(10);

  /* 4) PHY power-on: clear SIDDQ bit (default is 1 = powered down).
   *    PHY_CTRL is not a CCU register, so the bare RMW helper is fine.
   */

  usb_clrbits(T113_USB1_HCICTRL_BASE + T113_USB1_PHY_CTRL,
              PHY_CTRL_SIDDQ);

  /* 5) AHB burst + UTMI (ULPI bypass selects the internal UTMI PHY) */

  putreg32(USB_CTRL_INIT_VALUE,
           T113_USB1_HCICTRL_BASE + T113_USB1_HCI_INTERFACE);

  up_udelay(100);   /* PHY stabilization */

  syslog(LOG_INFO, "t113_usbhost: CCU + PHY ready\n");
  return OK;
}
