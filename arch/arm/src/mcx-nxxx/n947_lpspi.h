/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_lpspi.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_N947_LPSPI_H
#define __ARCH_ARM_SRC_MCX_NXXX_N947_LPSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/spi/spi.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct spi_dev_s;

/****************************************************************************
 * Name: n947_lpspibus_initialize
 *
 * Description:
 *   Initialize the selected LPSPI bus.
 *
 ****************************************************************************/

struct spi_dev_s *n947_lpspibus_initialize(int bus);

/****************************************************************************
 * Name: n947_lpspiNselect/status/cmddata/register
 *
 * Description:
 *   Board-specific chip-select/status hooks.  The common driver provides
 *   weak no-op defaults, so a board may override only the buses it uses.
 *
 ****************************************************************************/

#ifdef CONFIG_N947_LPSPI0
void n947_lpspi0select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi0status(struct spi_dev_s *dev, uint32_t devid);
#ifdef CONFIG_SPI_CMDDATA
int n947_lpspi0cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif
#ifdef CONFIG_SPI_CALLBACK
int n947_lpspi0register(struct spi_dev_s *dev, spi_mediachange_t callback,
                        void *arg);
#endif
#endif

#ifdef CONFIG_N947_LPSPI1
void n947_lpspi1select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi1status(struct spi_dev_s *dev, uint32_t devid);
#endif

#ifdef CONFIG_N947_LPSPI2
void n947_lpspi2select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi2status(struct spi_dev_s *dev, uint32_t devid);
#endif

#ifdef CONFIG_N947_LPSPI3
void n947_lpspi3select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi3status(struct spi_dev_s *dev, uint32_t devid);
#endif

#ifdef CONFIG_N947_LPSPI4
void n947_lpspi4select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi4status(struct spi_dev_s *dev, uint32_t devid);
#endif

#ifdef CONFIG_N947_LPSPI5
void n947_lpspi5select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi5status(struct spi_dev_s *dev, uint32_t devid);
#endif

#ifdef CONFIG_N947_LPSPI6
void n947_lpspi6select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi6status(struct spi_dev_s *dev, uint32_t devid);
#endif

#ifdef CONFIG_N947_LPSPI7
void n947_lpspi7select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t n947_lpspi7status(struct spi_dev_s *dev, uint32_t devid);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MCX_NXXX_N947_LPSPI_H */
