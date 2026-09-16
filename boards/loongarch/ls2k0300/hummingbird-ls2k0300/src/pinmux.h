/****************************************************************************
 * boards/loongarch/ls2k0300/hummingbird-ls2k0300/src/pinmux.h
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

#ifndef __LS_PINMUX_H__
#define __LS_PINMUX_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Just for 2k0300
 * pin 0 - 105
 */

#define LS_PINMUX_MODE_AS_GPIO        0x0
#define LS_PINMUX_MODE_AS_FIRST_FUNC  0x1
#define LS_PINMUX_MODE_AS_SECOND_FUNC 0x2
#define LS_PINMUX_MODE_AS_MAIN_FUNC   0x3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int ls_pinmux_pin_setup(unsigned int pin, unsigned int mode);

#endif /* __LS_PINMUX_H__ */
