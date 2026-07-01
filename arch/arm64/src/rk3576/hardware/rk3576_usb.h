/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_usb.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_USB_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_USB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB controller count */

#define RK3576_USB_COUNT            2   /* USB0 ~ USB1 */

/* DWC2 OTG Core Global Registers */

#define GOTGCTL                    0x000   /* OTG Control */
#define GOTGINT                    0x004   /* OTG Interrupt */
#define GAHBCFG                    0x008   /* AHB Configuration */
#define GUSBCFG                    0x00c   /* USB Configuration */
#define GRSTCTL                    0x010   /* Reset Control */
#define GINTSTS                    0x014   /* Interrupt Status */
#define GINTMSK                    0x018   /* Interrupt Mask */
#define GRXSTSR                    0x01c   /* RX Status (Read) */
#define GRXSTSP                    0x020   /* RX Status (Pop) */
#define GRXFSIZ                    0x024   /* RX FIFO Size */
#define GNPTXFSIZ                  0x028   /* Non-Periodic TX FIFO Size */
#define GNPTXSTS                   0x02c   /* Non-Periodic TX Status */
#define HPTXFSIZ                  0x030   /* Host Periodic TX FIFO Size */

/* Host Mode Registers (0x400-0x4FF) */

#define HCFG                       0x400   /* Host Configuration */
#define HFIR                       0x404   /* Host Frame Interval */
#define HFNUM                      0x408   /* Host Frame Number */
#define HPTXSTS                    0x40c   /* Host Periodic TX Status */
#define HAINT                      0x414   /* Host All Channels Interrupt */
#define HAINTMSK                   0x418   /* Host All Channels Interrupt Mask */

/* Host Channel Registers (per channel, 0x500-0x5FF) */

#define HCCHAR(n)                  (0x500 + (n) * 0x20)   /* Host Channel n */
#define HCINT(n)                   (0x508 + (n) * 0x20)   /* Host Channel n Interrupt */
#define HCINTMSK(n)               (0x50c + (n) * 0x20)   /* Host Channel n Interrupt Mask */
#define HCTSIZ(n)                  (0x510 + (n) * 0x20)   /* Host Channel n Transfer Size */
#define HCDMA(n)                   (0x514 + (n) * 0x20)   /* Host Channel n DMA Address */

/* Device Mode Registers (0x800-0xBFF) */

#define DCFG                       0x800   /* Device Configuration */
#define DCTL                       0x804   /* Device Control */
#define DSTS                       0x808   /* Device Status */
#define DIEPMSK                    0x810   /* Device IN Endpoint Interrupt Mask */
#define DOEPMSK                    0x814   /* Device OUT Endpoint Interrupt Mask */
#define DAINT                      0x818   /* Device All Endpoints Interrupt */
#define DAINTMSK                   0x81c   /* Device All Endpoints Interrupt Mask */

/* Device IN Endpoint Registers */

#define DIEPCTL(n)                 (0x900 + (n) * 0x20)
#define DIEPINT(n)                 (0x908 + (n) * 0x20)
#define DIEPTSIZ(n)                (0x910 + (n) * 0x20)
#define DIEPDMA(n)                 (0x914 + (n) * 0x20)

/* Device OUT Endpoint Registers */

#define DOEPCTL(n)                 (0xb00 + (n) * 0x20)
#define DOEPINT(n)                 (0xb08 + (n) * 0x20)
#define DOEPTSIZ(n)                (0xb10 + (n) * 0x20)
#define DOEPDMA(n)                 (0xb14 + (n) * 0x20)

/* Power and Clock Control Registers */

#define PCGCCTL                    0xe00   /* Power and Clock Control */

/* FIFO Addresses (word-addressed) */

#define FIFO_ADDR(ep)              ((ep) * 0x1000)

/****************************************************************************
 * GOTGCTL bit definitions
 ****************************************************************************/

#define GOTGCTL_BSESVLD            (1 << 19)   /* B-Session Valid */
#define GOTGCTL_ASESVLD            (1 << 18)   /* A-Session Valid */
#define GOTGCTL_VBVALOEN           (1 << 3)    /* VBUS Valid Enable */
#define GOTGCTL_VBVALOVAL          (1 << 2)    /* VBUS Valid Override Value */
#define GOTGCTL_SRPMSEL            (1 << 1)    /* SRP Select */
#define GOTGCTL_SRPREQ             (1 << 0)    /* SRP Request */

/****************************************************************************
 * GAHBCFG bit definitions
 ****************************************************************************/

#define GAHBCFG_PTXFIFOCFG_SHIFT   8
#define GAHBCFG_PTXFIFOCFG_MASK    (3 << GAHBCFG_PTXFIFOCFG_SHIFT)

#define GAHBCFG_HBSTLEN_SHIFT      4
#define GAHBCFG_HBSTLEN_MASK       (7 << GAHBCFG_HBSTLEN_SHIFT)
#define GAHBCFG_HBSTLEN_SINGLE     (0 << GAHBCFG_HBSTLEN_SHIFT)
#define GAHBCFG_HBSTLEN_INCR       (1 << GAHBCFG_HBSTLEN_SHIFT)
#define GAHBCFG_HBSTLEN_INCR4      (3 << GAHBCFG_HBSTLEN_SHIFT)
#define GAHBCFG_HBSTLEN_INCR8      (5 << GAHBCFG_HBSTLEN_SHIFT)
#define GAHBCFG_HBSTLEN_INCR16     (7 << GAHBCFG_HBSTLEN_SHIFT)

#define GAHBCFG_TXFIFOEMPTY_EN     (1 << 6)
#define GAHBCFG_GLBINTMASK_EN      (1 << 0)

/****************************************************************************
 * GUSBCFG bit definitions
 ****************************************************************************/

#define GUSBCFG_TOUTCAL_SHIFT      11
#define GUSBCFG_TOUTCAL_MASK       (7 << GUSBCFG_TOUTCAL_SHIFT)
#define GUSBCFG_PHYIF              (1 << 3)    /* PHY Interface (0=8bit, 1=16bit) */
#define GUSBCFG_ULPI_CLKSUSP       (1 << 19)   /* ULPI Clock Suspend */

/****************************************************************************
 * GRSTCTL bit definitions
 ****************************************************************************/

#define GRSTCTL_CSFTRST            (1 << 0)    /* Core Soft Reset */
#define GRSTCTL_HSFTRST            (1 << 1)    /* HCLK Soft Reset */
#define GRSTCTL_RXFFLSH            (1 << 4)    /* RX FIFO Flush */
#define GRSTCTL_TXFFLSH            (1 << 5)    /* TX FIFO Flush */
#define GRSTCTL_TXFNUM_SHIFT       6
#define GRSTCTL_TXFNUM_MASK        (0x1f << GRSTCTL_TXFNUM_SHIFT)
#define GRSTCTL_TXFNUM(n)          ((n) << GRSTCTL_TXFNUM_SHIFT)
#define GRSTCTL_AHBIDLE            (1 << 31)   /* AHB Master Idle */

/****************************************************************************
 * GINTSTS/GINTMSK bit definitions
 ****************************************************************************/

#define GINTSTS_CURMOD             (1 << 21)
#define GINTSTS_OINT               (1 << 2)
#define GINTSTS_IINT               (1 << 3)
#define GINTSTS_FETSUSP            (1 << 22)   /* Fetch Suspended */
#define GINTSTS_PRTINT             (1 << 24)   /* Host Port Interrupt */
#define GINTSTS_HCHINT             (1 << 25)   /* Host Channel Interrupt */
#define GINTSTS_PTXFEMP            (1 << 26)   /* Periodic TX FIFO Empty */
#define GINTSTS_RXFLVL             (1 << 4)    /* RX FIFO Level */

/****************************************************************************
 * HCFG bit definitions
 ****************************************************************************/

#define HCFG_FSLSPCLKSEL_SHIFT     0
#define HCFG_FSLSPCLKSEL_MASK      (3 << HCFG_FSLSPCLKSEL_SHIFT)
#define HCFG_FSLSPCLKSEL_48MHZ     (1 << HCFG_FSLSPCLKSEL_SHIFT)
#define HCFG_FSLSPCLKSEL_48MHZ_ULPI (2 << HCFG_FSLSPCLKSEL_SHIFT)

#define HCFG_FSLSSUPP              (1 << 2)    /* FS-Only Support */

/****************************************************************************
 * DCFG bit definitions
 ****************************************************************************/

#define DCFG_DEVSPD_SHIFT          0
#define DCFG_DEVSPD_MASK           (3 << DCFG_DEVSPD_SHIFT)
#define DCFG_DEVSPD_HIGH           (0 << DCFG_DEVSPD_SHIFT)
#define DCFG_DEVSPD_FULL           (3 << DCFG_DEVSPD_SHIFT)

#define DCFG_DAD_SHIFT             4
#define DCFG_DAD_MASK              (0x7f << DCFG_DAD_SHIFT)

#define DCFG_EPMISCNT_SHIFT        18
#define DCFG_EPMISCNT_MASK         (0x1f << DCFG_EPMISCNT_SHIFT)

/****************************************************************************
 * DCTL bit definitions
 ****************************************************************************/

#define DCTL_SFTDIS                (1 << 1)    /* Soft Disconnect */
#define DCTL_SGINNAK               (1 << 2)    /* Set GIN NAK */
#define DCTL_CGINNAK               (1 << 3)    /* Clear GIN NAK */
#define DCTL_SGOUTNAK              (1 << 4)    /* Set GOUT NAK */
#define DCTL_CGOUTNAK              (1 << 5)    /* Clear GOUT NAK */

/****************************************************************************
 * DSTS bit definitions
 ****************************************************************************/

#define DSTS_SUSPSTS               (1 << 22)   /* Suspend Status */
#define DSTS_ENUMSPD_SHIFT         1
#define DSTS_ENUMSPD_MASK          (3 << DSTS_ENUMSPD_SHIFT)
#define DSTS_ENUMSPD_HIGH          (0 << DSTS_ENUMSPD_SHIFT)
#define DSTS_ENUMSPD_FULL          (3 << DSTS_ENUMSPD_SHIFT)
#define DSTS_EERR                  (1 << 3)    /* Enumerated Speed Error */

/****************************************************************************
 * PCGCCTL bit definitions
 ****************************************************************************/

#define PCGCCTL_STOPCLK            (1 << 0)
#define PCGCCTL_GATEHCLK           (1 << 1)
#define PCGCCTL_PWRCLMP            (1 << 2)
#define PCGCCTL_RSTPDWNMODULE      (1 << 3)
#define PCGCCTL_PHYSUSP            (1 << 4)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_usb_base[RK3576_USB_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_USB_H */
