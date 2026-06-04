/****************************************************************************
 * boards/xtensa/t113/t113-evb-dsp/src/t113_cxx_locale_init.cc
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
 * Description
 *
 * Eagerly bootstrap the toolchain libstdc++ classic locale during board
 * late-init (before nsh_main / any builtin task), called from
 * board_late_initialize().
 *
 * Runtime evidence (locprobe on silicon) showed the GNU xtensa-hifi4-elf
 * GCC 10.3 libstdc++ classic-locale lazy bootstrap leaves its two private
 * statics inconsistent on this single-threaded target: after the first
 * std::ifstream, std::locale::_S_global held a heap _Impl while
 * std::locale::_S_classic was still NULL.  std::ios_base::_M_init() then
 * default-constructs a std::locale and assigns it into the stream's
 * _M_ios_locale via std::locale::operator=, whose
 * std::locale::_Impl::_M_remove_reference() unconditionally dereferences the
 * old impl.  With the two statics out of sync the old impl is NULL ->
 * LoadStoreError (EXCCAUSE=000d, VADDR=0) inside _M_remove_reference.
 *
 * std::locale::_S_initialize() runs std::locale::_S_initialize_once(), which
 * constructs the single static classic _Impl and stores it into BOTH
 * _S_classic and _S_global (single-threaded, no race), making the two statics
 * consistent up front.  After this runs once at boot, every later locale /
 * stream operation in every task sees a fully and consistently initialized
 * classic locale, and the operator= teardown never sees a NULL impl.
 *
 * Notes:
 *  - This is called from board_late_initialize() (an always-linked, always-
 *    run path) rather than relying on a C++ static-init (_GLOBAL__sub_I)
 *    object: the board static archive is not linked with --whole-archive, so
 *    a self-referencing-only static-init object would be dropped by the
 *    linker and never executed.
 *  - We call std::locale::_S_initialize() through its mangled linker symbol
 *    rather than including <locale>: the board library is compiled with
 *    -D__KERNEL__ and a kernel include set under which libstdc++'s <locale>
 *    -> <cstdlib> chain fails to declare ::system.  A direct extern
 *    declaration of the (parameterless, void) static member avoids pulling in
 *    any STL header.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* std::locale::_S_initialize() - static, takes no args, returns void.
 * Mangled: _ZNSt6locale13_S_initializeEv.  Constructs the classic _Impl and
 * installs it into both _S_classic and _S_global.
 */

extern "C" void _ZNSt6locale13_S_initializeEv(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* C entry point called from t113_boardinit.c (board_late_initialize). */

extern "C" void t113_cxx_locale_bootstrap(void)
{
  _ZNSt6locale13_S_initializeEv();
}
