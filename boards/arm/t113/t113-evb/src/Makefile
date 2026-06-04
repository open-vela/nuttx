############################################################################
# boards/arm/t113/t113-evb/src/Makefile
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
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = t113_boot.c t113_bringup.c t113_boot_jump.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += t113_appinit.c
endif

ifeq ($(CONFIG_BOARDCTL_BOOT_IMAGE),y)
CSRCS += t113_boot_image.c
endif

ifeq ($(CONFIG_ARCH_MINIMAL_VECTORTABLE),y)
CSRCS += t113_irqmap.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += t113_composite.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += t113_usbmsc.c
endif

ifeq ($(CONFIG_LCD_GC9503CV_DSI),y)
CSRCS += t113_lcd_board.c
endif

ifeq ($(CONFIG_T113_RPTUN_DSP),y)
CSRCS += t113_dsp_uart.c
endif

include $(TOPDIR)/boards/Board.mk
