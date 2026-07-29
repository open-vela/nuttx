############################################################################
# boards/arm/mcx-nxxx/frdm-mcxn947/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = n94x_boardinit.c n94x_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += n94x_autoleds.c
else ifeq ($(CONFIG_USERLED),y)
CSRCS += n94x_userleds.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += n94x_appinit.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += n94x_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += n94x_reset.c
endif

ifeq ($(CONFIG_N947_LPSPI),y)
CSRCS += n94x_spi.c
endif

ifeq ($(CONFIG_N947_LPADC),y)
CSRCS += n94x_lpadc.c
endif

ifeq ($(CONFIG_N947_CTIMER_PWM),y)
CSRCS += n94x_pwm.c
endif

ifeq ($(CONFIG_N94X_ENS160_AHT21),y)
CSRCS += n94x_airquality.c
endif

include $(TOPDIR)/boards/Board.mk
