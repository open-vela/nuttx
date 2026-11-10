/****************************************************************************
 * boards/arm/mx8mp/ok8mp-m7/src/mx8mp_boot.c
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

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_board_initialize
 *
 * Description:
 *   The architecture code initializes clocks and the UART4 console before
 *   calling this hook.  The basic NSH configuration has no additional board
 *   devices to initialize.
 *
 ****************************************************************************/

void mx8mp_board_initialize(void)
{
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Provide the boardctl(BOARDIOC_INIT) hook when board control is enabled.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL
int board_app_initialize(uintptr_t arg)
{
  return 0;
}
#endif
