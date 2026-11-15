/****************************************************************************
 * boards/arm64/h616/yuzuki-h616/src/yuzuki-h616.h
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

#ifndef __BOARDS_ARM64_H616_YUZUKI_H616_SRC_YUZUKI_H616_H
#define __BOARDS_ARM64_H616_YUZUKI_H616_SRC_YUZUKI_H616_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: yuzuki_h616_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 ****************************************************************************/

int yuzuki_h616_bringup(void);

/****************************************************************************
 * Name: yuzuki_h616_sdhci_initialize
 *
 * Description:
 *   Initialize the SDIO interface for the on-board eMMC (sdc2) and bind it
 *   to the MMC/SD driver.
 *
 ****************************************************************************/

#ifdef CONFIG_H616_SDHCI
int yuzuki_h616_sdhci_initialize(void);
#endif

#endif /* __BOARDS_ARM64_H616_YUZUKI_H616_SRC_YUZUKI_H616_H */
