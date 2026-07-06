/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_sdmmc.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SDMMC_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_SDMMC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SDMMC controller indices */

#define RK3576_SDMMC0              0
#define RK3576_SDMMC1              1
#define RK3576_EMMC                 2

/* Card types */

#define CARD_TYPE_NONE             0
#define CARD_TYPE_MMC              1
#define CARD_TYPE_SD               2
#define CARD_TYPE_SDIO             3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

int  rk3576_sdmmc_init(int ctrl);

/* Card operations */

int  rk3576_sdmmc_card_detect(int ctrl);
int  rk3576_sdmmc_card_type(int ctrl);

/* Data transfer */

int  rk3576_sdmmc_read(int ctrl, uint32_t sector,
                        uint32_t count, uint8_t *buf);
int  rk3576_sdmmc_write(int ctrl, uint32_t sector,
                         uint32_t count, const uint8_t *buf);

/* Control */

void rk3576_sdmmc_set_clock(int ctrl, uint32_t freq);
int  rk3576_sdmmc_reset(int ctrl);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_SDMMC_H */
