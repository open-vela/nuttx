/****************************************************************************
 * boards/arm64/rk3588/evb7_amp/src/evb7_amp.h
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

#ifndef __BOARDS_ARM64_RK3588_PINEPHONE_SRC_PINEPHONEPRO_H
#define __BOARDS_ARM64_RK3588_PINEPHONE_SRC_PINEPHONEPRO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: evb7_amp_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int evb7_amp_bringup(void);
#endif

/****************************************************************************
 * Name: evb7_amp_fb_init
 *
 * Description:
 *   Register /dev/fb0 backed by the shared frame area, so applications draw
 *   with ordinary framebuffer calls and Linux composites the result.
 *
 ****************************************************************************/

#if defined(CONFIG_RPTUN) && defined(CONFIG_VIDEO_FB)
int evb7_amp_fb_init(void);
#endif

/****************************************************************************
 * Name: evb7_amp_touch_init
 *
 * Description:
 *   Register /dev/input0 fed by touch events Linux forwards over rpmsg.
 *
 ****************************************************************************/

#if defined(CONFIG_RPTUN) && defined(CONFIG_INPUT_TOUCHSCREEN)
int evb7_amp_touch_init(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM64_RK3588_PINEPHONE_SRC_PINEPHONEPRO_H */
