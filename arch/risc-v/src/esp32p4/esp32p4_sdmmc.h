/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_sdmmc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP32P4_SDMMC_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP32P4_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp32p4_sdmmc_init
 *
 * Description:
 *   Initialize the SDMMC Host Controller on ESP32-P4 and configure the
 *   specified slot (Slot 0 for MicroSD 4-bit bus).
 *
 * Parameters:
 *   slotno - SDMMC slot number (0 for default Slot 0 MicroSD).
 *
 * Returned Value:
 *   Pointer to struct sdio_dev_s on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *esp32p4_sdmmc_init(int slotno);

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   NuttX standard SDIO initialization wrapper for ESP32-P4.
 *
 * Parameters:
 *   slotno - SDMMC slot number.
 *
 * Returned Value:
 *   Pointer to struct sdio_dev_s on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP32P4_SDMMC_H */
