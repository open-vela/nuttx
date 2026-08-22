/****************************************************************************
 * arch/risc-v/include/esp32p4/esp32p4_i2s.h
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

#ifndef __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_I2S_H
#define __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_I2S_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/audio/i2s.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32P4_I2S0 0
#define ESP32P4_I2S1 1
#define ESP32P4_I2S2 2

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp32p4_i2sbus_initialize
 *
 * Description:
 *   Initialize the selected I2S port
 *
 * Input Parameters:
 *   port - Port number (0 for I2S0, 1 for I2S1, 2 for I2S2)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct i2s_dev_s *esp32p4_i2sbus_initialize(int port);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_I2S_H */
