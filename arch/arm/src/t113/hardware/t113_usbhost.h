/****************************************************************************
 * arch/arm/src/t113/hardware/t113_usbhost.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_USBHOST_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_USBHOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/t113_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_USB1_BASE          0x04200000
#define T113_USB1_EHCI_OFFSET   0x0000
#define T113_USB1_OHCI_OFFSET   0x0400
#define T113_USB1_HCICTRL_OFFSET 0x0800

#define T113_USB1_EHCI_BASE     (T113_USB1_BASE + T113_USB1_EHCI_OFFSET)
#define T113_USB1_OHCI_BASE     (T113_USB1_BASE + T113_USB1_OHCI_OFFSET)
#define T113_USB1_HCICTRL_BASE  (T113_USB1_BASE + T113_USB1_HCICTRL_OFFSET)

/* HCI/PHY control registers (offsets from T113_USB1_HCICTRL_BASE) */

#define T113_USB1_HCI_INTERFACE 0x00   /* USB_CTRL: AHB burst, ULPI bypass */
#define T113_USB1_HCI_CTRL3     0x08   /* Line-state change detect */
#define T113_USB1_PHY_CTRL      0x10   /* SIDDQ + VC-bus strobe (vc_addr/vc_di/vc_clk) */
#define T113_USB1_PHY_STATUS    0x24   /* vc_do (read-only) */
#define T113_USB1_USB_SPDCR     0x28   /* SIE port disable control */

/* HCI_INTERFACE (USB_CTRL) bits */

#define USB_CTRL_ULPI_BYPASS    (1 << 0)   /* 1 = UTMI (use internal PHY) */
#define USB_CTRL_INCRX_ALIGN    (1 << 8)
#define USB_CTRL_INCR4          (1 << 9)
#define USB_CTRL_INCR8          (1 << 10)
#define USB_CTRL_INCR16         (1 << 11)
#define USB_CTRL_PP2VBUS        (1 << 12)
#define USB_CTRL_DMA_STATUS_EN  (1 << 28)  /* default 1, RO */

/* Target init value: INCR8 | INCR4 | INCRx align | ULPI bypass */
#define USB_CTRL_INIT_VALUE \
  (USB_CTRL_INCR8 | USB_CTRL_INCR4 | USB_CTRL_INCRX_ALIGN | USB_CTRL_ULPI_BYPASS)

/* PHY_CTRL bits */

#define PHY_CTRL_VC_CLK         (1 << 0)
#define PHY_CTRL_VC_DI          (1 << 7)
#define PHY_CTRL_VC_ADDR_SHIFT  8
#define PHY_CTRL_VC_ADDR_MASK   (0xff << 8)
#define PHY_CTRL_SIDDQ          (1 << 3)    /* 1 = PHY powered down (default!) */
#define PHY_CTRL_BIST_EN_A      (1 << 16)

/* PHY_STATUS bits */

#define PHY_STATUS_VC_DO        (1 << 0)
#define PHY_STATUS_BIST_DONE    (1 << 16)
#define PHY_STATUS_BIST_ERROR   (1 << 17)

/* CCU register bits (refer to hardware/t113_ccu.h for register addresses).
 * USB1_CLK_REG (0x0A74):
 */

#define USB1_CLK_USB1_CLKEN     (1u << 31)  /* OHCI1 12 MHz clock */
#define USB1_CLK_USBPHY1_RSTN   (1u << 30)  /* 1 = PHY1 reset de-asserted */
#define USB1_CLK_CLK12M_SEL_SHIFT 24
#define USB1_CLK_CLK12M_SEL_MASK  (3u << 24)
#define USB1_CLK_CLK12M_SEL_48M   (0u << 24)
#define USB1_CLK_CLK12M_SEL_24M   (1u << 24)

/* USB_BGR_REG (0x0A8C): */

#define USB_BGR_USBEHCI1_RST    (1u << 21)
#define USB_BGR_USBOHCI1_RST    (1u << 17)
#define USB_BGR_USBEHCI1_GATING (1u << 5)
#define USB_BGR_USBOHCI1_GATING (1u << 1)

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_USBHOST_H */
