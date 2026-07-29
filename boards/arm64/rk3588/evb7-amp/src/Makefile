############################################################################
# boards/arm64/rk3588/evb7_amp/src/Makefile
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

CSRCS = evb7_amp_boardinit.c evb7_amp_bringup.c

ifeq ($(CONFIG_RPTUN),y)
CSRCS += evb7_amp_shm.c
endif

# The framebuffer sits on top of the shared area, so it needs both the fb
# framework and the tunnel that carries the frame notifications.

ifeq ($(CONFIG_RPTUN),y)
ifeq ($(CONFIG_VIDEO_FB),y)
CSRCS += evb7_amp_fb.c
# Driving the VOP's Esmart3 window directly, instead of asking Linux to blit.
CSRCS += evb7_amp_vop.c
endif

# Touch events arrive over the same rpmsg endpoint as the frame signalling.

ifeq ($(CONFIG_INPUT_TOUCHSCREEN),y)
CSRCS += evb7_amp_touch.c
endif
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += evb7_amp_appinit.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += evb7_amp_reset.c
endif
endif

include $(TOPDIR)/boards/Board.mk
