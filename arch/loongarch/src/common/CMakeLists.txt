# ##############################################################################
# arch/loongarch/src/common/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS)

list(APPEND SRCS loongarch_vectors.S loongarch_macros.S)
list(APPEND SRCS loongarch_saveusercontext.S loongarch_fork_func.S)
list(APPEND SRCS loongarch_allocateheap.c loongarch_cpuidlestack.c)
list(APPEND SRCS loongarch_createstack.c loongarch_doirq.c
     loongarch_exception.c)
list(APPEND SRCS loongarch_exit.c loongarch_getintstack.c
     loongarch_getnewintctx.c)
list(APPEND SRCS loongarch_initialize.c loongarch_initialstate.c
     loongarch_modifyreg32.c)
list(APPEND SRCS loongarch_nputs.c loongarch_registerdump.c)
list(APPEND SRCS loongarch_releasestack.c loongarch_schedulesigaction.c
     loongarch_sigdeliver.c)
list(APPEND SRCS loongarch_stackframe.c loongarch_tcbinfo.c loongarch_swint.c)
list(APPEND SRCS loongarch_switchcontext.c loongarch_usestack.c
     loongarch_fork.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS loongarch_idle.c)
endif()

if(NOT CONFIG_ALARM_ARCH)
  if(NOT CONFIG_TIMER_ARCH)
    list(APPEND SRCS loongarch_mdelay.c loongarch_udelay.c)
  endif()
endif()

if(NOT CONFIG_BUILD_FLAT)
  list(APPEND SRCS loongarch_task_start.c loongarch_pthread_start.c
       loongarch_signal_dispatch.c)
  if(CONFIG_BUILD_PROTECTED)
    target_sources(arch_interface PRIVATE loongarch_signal_handler.S)
  endif()
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS loongarch_checkstack.c)
endif()

if(CONFIG_SCHED_THREAD_LOCAL)
  list(APPEND SRCS loongarch_tls.c)
endif()

target_sources(arch PRIVATE ${SRCS})
