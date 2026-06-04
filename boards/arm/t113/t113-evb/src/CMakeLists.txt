# ##############################################################################
# boards/arm/t113/t113-evb/src/CMakeLists.txt
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

set(SRCS t113_boot.c t113_bringup.c t113_boot_jump.c)

# slave and BMP both reuse the rptun_bmp backend (RPTUN_BMP); they are
# build-time exclusive (slave sets T113_RPTUN_SLAVE, BMP does not).
if(CONFIG_RPTUN_BMP AND CONFIG_T113_RPTUN_SLAVE)
  list(APPEND SRCS t113_rptun_slave.c)
elseif(CONFIG_RPTUN_BMP)
  list(APPEND SRCS t113_rptun_bmp.c)
endif()

# core0 master loader for the remote cores it loads (core1 and/or DSP).
if(CONFIG_T113_RPTUN_MASTER)
  list(APPEND SRCS t113_rptun_master.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS t113_appinit.c)
endif()

if(CONFIG_BOARDCTL_BOOT_IMAGE)
  list(APPEND SRCS t113_boot_image.c)
endif()

if(CONFIG_ARCH_MINIMAL_VECTORTABLE)
  list(APPEND SRCS t113_irqmap.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS t113_composite.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS t113_usbmsc.c)
endif()

if(CONFIG_T113_LRADC_KEYPAD)
  list(APPEND SRCS t113_keypad.c)
endif()

if(CONFIG_T113_USBHOST)
  list(APPEND SRCS t113_boardusb.c)
endif()

if(CONFIG_T113_CTP)
  list(APPEND SRCS t113_ctp_board.c)
endif()

if(CONFIG_LCD_GC9503CV_DSI)
  list(APPEND SRCS t113_lcd_board.c)
endif()

if(CONFIG_T113_RPTUN_DSP)
  list(APPEND SRCS t113_dsp_uart.c)
endif()

if(CONFIG_T113_SMHC)
  list(APPEND SRCS t113_smhc.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_T113_BOOT0)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/sram.ld")
elseif(CONFIG_SYSTEM_OFLOADER)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/ofloader.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/sdram.ld.S")
endif()

if(CONFIG_ETC_ROMFS)
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCSRCS
    etc/init.d/rcS
    etc/init.d/rc.sysinit
    RCRAWS
    etc/example
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)

  target_link_libraries(board PRIVATE romfs_etc)
endif()
