# ##############################################################################
# arch/arm/src/bk7258/CMakeLists.txt
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

set(SRCS
    bk7258_start.c
    bk7258_vectors.c
    bk7258_irq.c
    bk7258_serial.c
    bk7258_allocateheap.c
    bk7258_timer.c
    bk7258_lowputc.c
    bk7258_syslog.c
    bk7258_mbox0.c
    bk7258_mailbox_channel.c
    bk7258_mb_uart.c
    bk7258_idle.c
    bk7258_ipc_heartbeat.c
    bk7258_pm_pwc.c
    bk7258_wifi_probe.c)

if(CONFIG_BK7258_WIFI)
  list(APPEND SRCS bk7258_wifi.c)
endif()

if(CONFIG_BK7258_CP_TRNG)
  list(APPEND SRCS bk7258_random.c)
endif()

if(CONFIG_BK7258_AUDIO)
  list(APPEND SRCS bk7258_aud.c bk7258_audio_diag.c)
endif()

if(CONFIG_BK7258_PCM)
  list(APPEND SRCS bk7258_pcm.c)
endif()

if(CONFIG_BK7258_PSRAM)
  list(APPEND SRCS bk7258_psram.c)
endif()

target_sources(arch PRIVATE ${SRCS})
target_include_directories(arch PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                                        ${CMAKE_CURRENT_SOURCE_DIR}/hardware)
