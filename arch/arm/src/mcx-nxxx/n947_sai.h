/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_sai.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_N947_SAI_H
#define __ARCH_ARM_SRC_MCX_NXXX_N947_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/audio/i2s.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct n947_sai_stats_s
{
  uint32_t requested_rate;
  uint32_t actual_rate;
  uint32_t bit_clock;
  uint32_t tx_underruns;
  uint32_t tx_underruns_busy;
  uint32_t tx_underruns_idle;
  uint32_t tx_boundary_empty;
  uint32_t tx_queue_max;
  uint32_t tx_dma_chunks;
  uint32_t tx_dma_errors;
  uint32_t tx_dma_recoveries;
  uint32_t rx_overruns;
  uint32_t tx_buffers;
  uint32_t rx_buffers;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize one MCX-Nxxx SAI controller and return its standard NuttX I2S
 * lower-half.  The initial implementation supports SAI1 on MCXN947.
 */

FAR struct i2s_dev_s *n947_sai_initialize(int port);

/* Read non-destructive diagnostics for a previously initialized
 * controller.
 */

int n947_sai_getstats(FAR struct i2s_dev_s *dev,
                      FAR struct n947_sai_stats_s *stats);

#endif /* __ARCH_ARM_SRC_MCX_NXXX_N947_SAI_H */
