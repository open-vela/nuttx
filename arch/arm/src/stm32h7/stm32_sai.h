/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sai.h
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

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_SAI_H
#define __ARCH_ARM_SRC_STM32H7_STM32_SAI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/stm32_sai.h"

#include <nuttx/audio/i2s.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Logical interface numbers.  They identify one SAI sub-block each and are
 * the values passed to stm32_sai_initialize().
 */

#define SAI1_BLOCK_A     0
#define SAI1_BLOCK_B     1
#define SAI2_BLOCK_A     2
#define SAI2_BLOCK_B     3
#define SAI4_BLOCK_A     4
#define SAI4_BLOCK_B     5

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

typedef uint32_t (*stm32_sai_sampleratecb_t)(struct i2s_dev_s *dev,
                                             uint32_t rate);

/****************************************************************************
 * Name: stm32_sai_initialize
 *
 * Description:
 *   Initialize the selected SAI block.
 *
 *   The kernel clock is taken from PLL2P, which the board must have enabled
 *   and sized so that the sample rate dividers land on integers.  Returns
 *   NULL when the interface number is not enabled in the configuration.
 *
 * Input Parameters:
 *   intf - I2S interface number (one of the SAIx_BLOCK_y values above)
 *   sampleratecb - Optional callback invoked when the sample rate changes,
 *                  which may return a different kernel clock frequency
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

/* Hold master TX clocks for codec setup independently of DMA buffers.  Only
 * asynchronous, non-PDM master TX with NOMCK=0 is supported.  Queues must be
 * idle when changing this hold.  Release after gating the codec AIF clock.
 */

int stm32_sai_mclk_enable(struct i2s_dev_s *dev, bool enable);

struct i2s_dev_s *stm32_sai_initialize(int intf,
                  stm32_sai_sampleratecb_t sampleratecb);

/****************************************************************************
 * Name: stm32_sai_staterecord
 *
 * Description:
 *   Copy the register snapshot that the driver took when the last transfer
 *   ended into the caller's array.
 *
 *   SAI reports overrun and a wrong clock configuration through status
 *   bits that the DMA controller does not expose, so a failed transfer
 *   surfaces as a bare -EIO without saying why.  This accessor hands back
 *   what the hardware was showing at that moment.
 *
 * Input Parameters:
 *   dev  - Device from stm32_sai_initialize(), or NULL for "the only device"
 *   regs - Receives STM32H7_SAI_DIAG_NREGS words, in the order documented at
 *          the driver's definition.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure.
 *
 ****************************************************************************/

#define STM32H7_SAI_DIAG_NREGS  6
#define STM32H7_SAI_DIAG_NALL   11

int stm32_sai_staterecord(struct i2s_dev_s *dev, uint32_t *regs, int nregs);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32H7_STM32_SAI_H */
