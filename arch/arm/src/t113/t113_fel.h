/****************************************************************************
 * arch/arm/src/t113/t113_fel.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_FEL_H
#define __ARCH_ARM_SRC_T113_T113_FEL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_T113_FEL_RESCUE_THRESHOLD) && \
    CONFIG_T113_FEL_RESCUE_THRESHOLD > 0

/****************************************************************************
 * Name: t113_fel_rescue
 *
 * Description:
 *   Abnormal-reboot guard, called once from t113_boardinitialize()
 *   during arm_boot() before nx_start().  Increments a counter (RTC
 *   GP_DATA[1]) on every non-USER reset that happened within
 *   HEALTHY_WINDOW_SEC of the previous boot, and forces the device
 *   into BROM FEL mode once the count reaches
 *   CONFIG_T113_FEL_RESCUE_THRESHOLD.  Resets the counter when the
 *   system stayed up long enough to be considered recovered.
 *
 *   Safe to call before IRQ enable: only touches RTC MMIO and the
 *   WDT reset path on overflow.
 *
 ****************************************************************************/

void t113_fel_rescue(void);

#else

#  define t113_fel_rescue() ((void)0)

#endif

#endif /* __ARCH_ARM_SRC_T113_T113_FEL_H */
