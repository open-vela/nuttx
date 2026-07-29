# ##############################################################################
# boards/arm/mcx-nxxx/frdm-mcxn947/src/CMakeLists.txt
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

# The MCXN947 image starts with an NXP boot header at the beginning of flash.
# GNU ld's default build-id note would otherwise be placed before that header.
target_link_options(nuttx PRIVATE -Wl,--build-id=none)

set(SRCS n94x_boardinit.c n94x_bringup.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS n94x_autoleds.c)
elseif(CONFIG_USERLED)
  list(APPEND SRCS n94x_userleds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS n94x_appinit.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS n94x_buttons.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS n94x_reset.c)
endif()

if(CONFIG_N947_LPSPI)
  list(APPEND SRCS n94x_spi.c)
endif()

if(CONFIG_N947_LPADC)
  list(APPEND SRCS n94x_lpadc.c)
endif()

if(CONFIG_N947_CTIMER_PWM)
  list(APPEND SRCS n94x_pwm.c)
endif()

if(CONFIG_N94X_ENS160_AHT21)
  list(APPEND SRCS n94x_airquality.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
