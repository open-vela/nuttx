/****************************************************************************
 * boards/xtensa/t113/t113-evb-dsp/src/t113_libc_stubs.c
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

#include <reent.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __getreent
 *
 * Description:
 *   Provide the newlib per-thread reentrancy accessor expected by the GNU
 *   xtensa toolchain's libsupc++ verbose terminate handler (vterminate.o),
 *   which is the default std::terminate path pulled in when linking the
 *   toolchain libstdc++/libsupc++ (CONFIG_LIBCXXTOOLCHAIN).  NuttX does not
 *   use newlib's reentrancy model, so this symbol is otherwise undefined.
 *
 *   With CONFIG_CXX_EXCEPTION disabled the verbose terminate handler is
 *   effectively dead code; this stub exists only to satisfy the link.  It
 *   returns a single static _reent so that, even if reached, no NULL pointer
 *   is dereferenced.
 *
 ****************************************************************************/

struct _reent *__getreent(void)
{
  /* A plain zero-initialized _reent is sufficient: it resolves the link
   * without dragging in newlib's fake stdio objects (_REENT_INIT references
   * __sf_fake_std{in,out,err}, which NuttX does not provide), and the only
   * caller (the verbose terminate handler) is dead code with exceptions
   * disabled.
   */

  static struct _reent g_reent;

  return &g_reent;
}
