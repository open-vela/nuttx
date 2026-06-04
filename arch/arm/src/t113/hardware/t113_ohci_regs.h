/****************************************************************************
 * arch/arm/src/t113/hardware/t113_ohci_regs.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_OHCI_REGS_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_OHCI_REGS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/ohci.h>

#include "hardware/t113_usbhost.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T113 has one root hub port on USB1 */

#define T113_OHCI_NRHPORT       1

/* Register addresses (base + OHCI standard offsets) ************************/

#define T113_USBHOST_HCIREV     (T113_USB1_OHCI_BASE + OHCI_HCIREV_OFFSET)
#define T113_USBHOST_CTRL       (T113_USB1_OHCI_BASE + OHCI_CTRL_OFFSET)
#define T113_USBHOST_CMDST      (T113_USB1_OHCI_BASE + OHCI_CMDST_OFFSET)
#define T113_USBHOST_INTST      (T113_USB1_OHCI_BASE + OHCI_INTST_OFFSET)
#define T113_USBHOST_INTEN      (T113_USB1_OHCI_BASE + OHCI_INTEN_OFFSET)
#define T113_USBHOST_INTDIS     (T113_USB1_OHCI_BASE + OHCI_INTDIS_OFFSET)

/* Memory pointers (section 7.2) */

#define T113_USBHOST_HCCA       (T113_USB1_OHCI_BASE + OHCI_HCCA_OFFSET)
#define T113_USBHOST_PERED      (T113_USB1_OHCI_BASE + OHCI_PERED_OFFSET)
#define T113_USBHOST_CTRLHEADED (T113_USB1_OHCI_BASE + OHCI_CTRLHEADED_OFFSET)
#define T113_USBHOST_CTRLED     (T113_USB1_OHCI_BASE + OHCI_CTRLED_OFFSET)
#define T113_USBHOST_BULKHEADED (T113_USB1_OHCI_BASE + OHCI_BULKHEADED_OFFSET)
#define T113_USBHOST_BULKED     (T113_USB1_OHCI_BASE + OHCI_BULKED_OFFSET)
#define T113_USBHOST_DONEHEAD   (T113_USB1_OHCI_BASE + OHCI_DONEHEAD_OFFSET)

/* Frame counters (section 7.3) */

#define T113_USBHOST_FMINT      (T113_USB1_OHCI_BASE + OHCI_FMINT_OFFSET)
#define T113_USBHOST_FMREM      (T113_USB1_OHCI_BASE + OHCI_FMREM_OFFSET)
#define T113_USBHOST_FMNO       (T113_USB1_OHCI_BASE + OHCI_FMNO_OFFSET)
#define T113_USBHOST_PERSTART   (T113_USB1_OHCI_BASE + OHCI_PERSTART_OFFSET)

/* Root hub (section 7.4) */

#define T113_USBHOST_LSTHRES    (T113_USB1_OHCI_BASE + OHCI_LSTHRES_OFFSET)
#define T113_USBHOST_RHDESCA    (T113_USB1_OHCI_BASE + OHCI_RHDESCA_OFFSET)
#define T113_USBHOST_RHDESCB    (T113_USB1_OHCI_BASE + OHCI_RHDESCB_OFFSET)
#define T113_USBHOST_RHSTATUS   (T113_USB1_OHCI_BASE + OHCI_RHSTATUS_OFFSET)

#define T113_USBHOST_RHPORTST(n) (T113_USB1_OHCI_BASE + OHCI_RHPORTST_OFFSET(n))
#define T113_USBHOST_RHPORTST1  (T113_USB1_OHCI_BASE + OHCI_RHPORTST1_OFFSET)

/* Register bit definitions: see <nuttx/usb/ohci.h> */

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_OHCI_REGS_H */
