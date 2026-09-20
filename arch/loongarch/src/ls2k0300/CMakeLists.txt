# ##############################################################################
# arch/loongarch/src/ls2k0300/CMakeLists.txt
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

set(SRCS ls2k0300_head.S ls2k0300_start.c ls2k0300_irq.c)

list(APPEND SRCS ls2k0300_allocateheap.c ls2k0300_clockconfig.c)

list(APPEND SRCS ls2k0300_lowputc.c ls2k0300_serial.c)

if(CONFIG_TIMER)
  list(APPEND SRCS ls2k0300_timerisr.c)
endif()

if(CONFIG_LS2K0300_ADC)
  list(APPEND SRCS ls2k0300_adc.c)
endif()

if(CONFIG_LS2K0300_RTC)
  list(APPEND SRCS ls2k0300_rtc.c)
endif()

if(CONFIG_LS2K0300_I2C)
  list(APPEND SRCS ls2k0300_i2c.c)
endif()

if(CONFIG_LS2K0300_PWM)
  list(APPEND SRCS ls2k0300_pwm.c)
endif()

if(CONFIG_LS2K0300_SPI)
  list(APPEND SRCS ls2k0300_spi.c)
endif()

if(CONFIG_LS2K0300_SPIIO)
  list(APPEND SRCS ls2k0300_spiio.c)
endif()

if(CONFIG_LS2K0300_THERMAL)
  list(APPEND SRCS ls2k0300_thermal.c)
endif()

if(CONFIG_LS2K0300_WDT)
  list(APPEND SRCS ls2k0300_wdt.c)
endif()

if(CONFIG_LS2K0300_GPIO)
  list(APPEND SRCS ls2k0300_gpio.c)
endif()

if(CONFIG_LS2K0300_PINCTRL)
  list(APPEND SRCS ls2k0300_pinctrl.c)
endif()

if(CONFIG_LS2K0300_ETHERNET)
  list(APPEND SRCS ls2k0300_gmac.c ls2k0300_ethernet.c ls2k0300_dma_cache.S)
endif()

target_sources(arch PRIVATE ${SRCS})
