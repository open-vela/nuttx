/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/include/board.h
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

#ifndef __BOARDS_XTENSA_ESP32S3_ESP32S3_EYE_INCLUDE_BOARD_H
#define __BOARDS_XTENSA_ESP32S3_ESP32S3_EYE_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The ESP32-S3-EYE board is fitted with a 40MHz crystal */

#define BOARD_XTAL_FREQUENCY    40000000

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#  define BOARD_CLOCK_FREQUENCY (CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ * 1000000)
#else
#  define BOARD_CLOCK_FREQUENCY 80000000
#endif

/* LED definitions **********************************************************/

/* Define how many LEDs this board has (needed by userleds) */

#define BOARD_NLEDS             1

/* GPIO pins used by the GPIO Subsystem */

/* [COMPANION] 重排为桌面小跟班外设：
 *   OUT: GPIO4 台灯继电器 / GPIO48 震动马达
 *   IN : GPIO42 TTP223 触摸
 *   INT: GPIO0  BOOT 键
 * 注意 GPIO3 已让给俯仰舵机 PWM，不能再当普通 GPIO 输出。*/
/* [COMPANION] 引脚方向以 doc20 为准：触摸和震动都是**输入**传感器。
 *   OUT : GPIO4  台灯继电器
 *   IN  : GPIO42 TTP223 触摸 / GPIO48 SW-420 震动传感器（被拍）
 *   INT : GPIO0  BOOT 键
 * 注意 GPIO3 已让给俯仰舵机 PWM，不能再当普通 GPIO 输出。 */
#define BOARD_NGPIOOUT    1 /* Amount of GPIO Output pins */
#define BOARD_NGPIOIN     2
#define BOARD_NGPIOINT    1

#endif /* __BOARDS_XTENSA_ESP32S3_ESP32S3_EYE_INCLUDE_BOARD_H */
