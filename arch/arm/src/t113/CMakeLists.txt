# ##############################################################################
# arch/arm/src/t113/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
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
    t113_boot.c
    t113_ccu.c
    t113_clk.c
    t113_irq.c
    t113_lowputc.c
    t113_serial.c
    t113_idle.c
    t113_timerisr.c)

if(CONFIG_T113_BOOT0)
  list(APPEND SRCS t113_boot0.S t113_load.c)
  set_source_files_properties(
    t113_load.c PROPERTIES COMPILE_OPTIONS "-fno-lto;-Os;-fno-stack-protector")
endif()

if(CONFIG_T113_DMA)
  list(APPEND SRCS t113_dma.c)
endif()

if(NOT CONFIG_UP OR CONFIG_T113_RPTUN_CORE1)
  # SMP needs the up_cpu_start/arm_cpu_boot hooks; the cpu1 rptun ELF master
  # runs in CONFIG_UP=y but still needs t113_release_cpu1 to start CPU1 after
  # its own bringup.
  list(APPEND SRCS t113_cpuboot.c)
endif()

if(CONFIG_T113_SPI0 OR CONFIG_T113_SPI1)
  list(APPEND SRCS t113_spi.c)
endif()

if(CONFIG_T113_TWI0
   OR CONFIG_T113_TWI1
   OR CONFIG_T113_TWI2
   OR CONFIG_T113_TWI3)
  list(APPEND SRCS t113_i2c.c)
endif()

if(CONFIG_T113_USBDEV)
  list(APPEND SRCS t113_usbdev.c)
endif()

if(CONFIG_T113_WDT)
  list(APPEND SRCS t113_wdt.c)
endif()

list(APPEND SRCS t113_systemreset.c)

list(APPEND SRCS t113_gpio.c)

if(CONFIG_T113_RTC OR CONFIG_T113_FEL_RESCUE_THRESHOLD GREATER 0)
  list(APPEND SRCS t113_rtc.c)
endif()

if(CONFIG_T113_FEL_RESCUE_THRESHOLD GREATER 0)
  list(APPEND SRCS t113_fel.c)
endif()

if(CONFIG_T113_PWM)
  list(APPEND SRCS t113_pwm.c)
endif()

if(CONFIG_T113_GPADC)
  list(APPEND SRCS t113_adc.c)
endif()

if(CONFIG_T113_LRADC)
  list(APPEND SRCS t113_lradc.c)
endif()

if(CONFIG_T113_CE)
  list(APPEND SRCS t113_ce.c)
endif()

if(CONFIG_T113_DSI)
  list(APPEND SRCS t113_dsi.c)
endif()

if(CONFIG_T113_TCON)
  list(APPEND SRCS t113_tcon.c)
endif()

if(CONFIG_T113_DE)
  list(APPEND SRCS t113_de.c)
endif()

if(CONFIG_T113_CAN0 OR CONFIG_T113_CAN1)
  list(APPEND SRCS t113_can.c)
endif()

if(CONFIG_T113_HSTIMER0 OR CONFIG_T113_HSTIMER1)
  list(APPEND SRCS t113_hstimer.c)
endif()

if(CONFIG_T113_USBHOST)
  list(APPEND SRCS t113_usbhost.c t113_usbhost_trace.c)
endif()
if(CONFIG_T113_EHCI)
  list(APPEND SRCS t113_ehci.c)
endif()
if(CONFIG_T113_OHCI)
  list(APPEND SRCS t113_ohci.c)
endif()

if(CONFIG_T113_HWSPINLOCK)
  list(APPEND SRCS t113_hwspinlock.c)
endif()

if(CONFIG_T113_RPTUN_DSP)
  list(APPEND SRCS t113_dsp.c)

  # The MSGBOX mailbox is the rptun doorbell, used only by the rptun transport,
  # so it follows CONFIG_RPTUN rather than the DSP clock/reset control on its
  # own.
  if(CONFIG_RPTUN)
    list(APPEND SRCS t113_msgbox.c t113_addrenv.c)
  endif()
endif()

if(CONFIG_T113_SMHC)
  list(APPEND SRCS t113_smhc.c)
endif()

if(CONFIG_T113_AUDIO)
  list(APPEND SRCS t113_audio.c)
endif()

if(CONFIG_T113_DMIC)
  list(APPEND SRCS t113_dmic.c)
endif()

target_sources(arch PRIVATE ${SRCS})
