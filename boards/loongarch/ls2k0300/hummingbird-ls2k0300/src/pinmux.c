/****************************************************************************
 * boards/loongarch/ls2k0300/hummingbird-ls2k0300/src/pinmux.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "pinmux.h"
#include "ls2k0300.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ls_pinmux_pin_setup(unsigned int pin, unsigned int mode)
{
  volatile unsigned int *cfg_base_reg;
  uint32_t group;
  uint32_t group_num;
  uint32_t temp;

  if (pin > LS_GPIO_MAX)
    {
      printf("Error %s pin not in [0-105] (%d)\r\n", __func__, pin);
      return -1;
    }

  group = (pin >> 4);
  group_num = (pin & 0xf);
  group_num <<= 1;  /* multiply by 2 to get the starting bit for the pin */

  cfg_base_reg = (unsigned int *)LS_GPIO_0_15_MULTI_CFG;
  cfg_base_reg += group;
  temp = readl(cfg_base_reg);
  temp |= (0x3 << group_num);  /* clear bits */
  temp ^= (0x3 << group_num);  /* clear bits */
  temp |= (mode << group_num); /* set the pin to the desired mode */

  /* printf("===>base: %p temp: 0x%x \r\n", cfg_base_reg, temp); */

  writel(temp, cfg_base_reg);

  return 0;
}
