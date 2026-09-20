/****************************************************************************
 * arch/loongarch/src/common/loongarch_signal_handler.S
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
#include <nuttx/arch.h>
#include <arch/syscall.h>

#include "loongarch_internal.h"

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

  .file   "loongarch_signal_handler.S"

  .text
  .globl up_signal_handler
  .type  up_signal_handler, function

up_signal_handler:

  addi.d  $sp, $sp, -STACK_FRAME_SIZE
  st.d    $ra, $sp, 8

  move    $t0, $a0
  move    $a0, $a1
  move    $a1, $a2
  move    $a2, $a3
  jirl    $ra, $t0, 0

  ld.d    $ra, $sp, 8
  addi.d  $sp, $sp, STACK_FRAME_SIZE

  li.d    $a0, SYS_signal_handler_return
  syscall 0
  nop

  .size up_signal_handler, .-up_signal_handler
  .end

#endif
