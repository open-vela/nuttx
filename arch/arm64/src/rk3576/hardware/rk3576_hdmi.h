/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_hdmi.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_HDMI_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_HDMI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HDMI controller base */

#define RK3576_HDMI_COUNT           1

/****************************************************************************
 * HDMI System Registers (0x0000-0x00FF) - GRF
 ****************************************************************************/

#define HDMI_SYS_CTRL              0x0000   /* System control */
#define HDMI_SYS_STATUS            0x0004   /* System status */
#define HDMI_VIDEO_CTRL            0x0008   /* Video control */
#define HDMI_AUDIO_CTRL            0x000c   /* Audio control */
#define HDMI_I2C_ADDR              0x0010   /* I2C target address */
#define HDMI_I2C_DATA              0x0014   /* I2C data */
#define HDMI_I2C_CTRL              0x0018   /* I2C control */
#define HDMI_I2C_STATUS            0x001c   /* I2C status */
#define HDMI_INT_MASK              0x0020   /* Interrupt mask */
#define HDMI_INT_STATUS            0x0024   /* Interrupt status */
#define HDMI_INT_CLEAR             0x0028   /* Interrupt clear */

/****************************************************************************
 * HDMI Video Input Registers (0x0100-0x01FF)
 ****************************************************************************/

#define HDMI_VI_CTRL               0x0100   /* Video input control */
#define HDMI_VI_STATUS             0x0104   /* Video input status */
#define HDMI_VI_FORMAT             0x0108   /* Video input format */
#define HDMI_VI_TIMING_H           0x010c   /* H timing */
#define HDMI_VI_TIMING_V           0x0110   /* V timing */
#define HDMI_VI_TIMING_F           0x0114   /* Frame timing */

/****************************************************************************
 * HDMI TX Controller Registers (0x1000-0x1FFF) - DesignWare HDMI
 ****************************************************************************/

/* PHY Registers */

#define HDMI_PHY_CTRL              0x1000   /* PHY control */
#define HDMI_PHY_STATUS            0x1004   /* PHY status */
#define HDMI_PHY_CFG               0x1008   /* PHY configuration */

/* Main Controller */

#define HDMI_TX_CTRL               0x1100   /* TX control */
#define HDMI_TX_STATUS             0x1104   /* TX status */
#define HDMI_TX_CLK_CTRL           0x1108   /* TX clock control */

/* Audio Registers */

#define HDMI_AUD_CTRL              0x1200   /* Audio control */
#define HDMI_AUD_STATUS            0x1204   /* Audio status */
#define HDMI_AUD_SAMPLE            0x1208   /* Audio sample */

/* HDCP Registers */

#define HDCP_CTRL                  0x1400   /* HDCP control */
#define HDCP_STATUS                0x1404   /* HDCP status */

/****************************************************************************
 * HDMI_SYS_CTRL bit definitions
 ****************************************************************************/

#define HDMI_SYS_CTRL_SW_RESET     (1 << 0)   /* Software reset */
#define HDMI_SYS_CTRL_ENABLE       (1 << 1)   /* HDMI enable */
#define HDMI_SYS_CTRL_CLK_EN       (1 << 2)   /* Clock enable */

/****************************************************************************
 * HDMI_SYS_STATUS bit definitions
 ****************************************************************************/

#define HDMI_SYS_STATUS_PLL_LOCK   (1 << 0)   /* PLL locked */
#define HDMI_SYS_STATUS_TX_READY   (1 << 1)   /* TX ready */
#define HDMI_SYS_STATUS_HPD        (1 << 4)   /* Hotplug detect */

/****************************************************************************
 * HDMI_VIDEO_CTRL bit definitions
 ****************************************************************************/

#define HDMI_VIDEO_CTRL_ENABLE     (1 << 0)   /* Video enable */
#define HDMI_VIDEO_CTRL_FORMAT_SHIFT 4
#define HDMI_VIDEO_CTRL_FORMAT_MASK  (0xf << HDMI_VIDEO_CTRL_FORMAT_SHIFT)
#define HDMI_VIDEO_CTRL_FORMAT_RGB888   (0 << HDMI_VIDEO_CTRL_FORMAT_SHIFT)
#define HDMI_VIDEO_CTRL_FORMAT_RGB565   (1 << HDMI_VIDEO_CTRL_FORMAT_SHIFT)
#define HDMI_VIDEO_CTRL_FORMAT_YCbCr444 (2 << HDMI_VIDEO_CTRL_FORMAT_SHIFT)
#define HDMI_VIDEO_CTRL_FORMAT_YCbCr422 (3 << HDMI_VIDEO_CTRL_FORMAT_SHIFT)

/****************************************************************************
 * HDMI_INT_STATUS bit definitions
 ****************************************************************************/

#define HDMI_INT_HPD               (1 << 0)   /* Hotplug detect */
#define HDMI_INT_TX_READY          (1 << 1)   /* TX ready */
#define HDMI_INT_PHY_LOCK          (1 << 2)   /* PHY locked */
#define HDMI_INT_AUDIO             (1 << 3)   /* Audio interrupt */

/****************************************************************************
 * HDMI PHY_CTRL bit definitions
 ****************************************************************************/

#define HDMI_PHY_CTRL_ENABLE       (1 << 0)
#define HDMI_PHY_CTRL_RESET        (1 << 1)
#define HDMI_PHY_CTRL_MODE_SHIFT   4
#define HDMI_PHY_CTRL_MODE_MASK    (0xf << HDMI_PHY_CTRL_MODE_SHIFT)
#define HDMI_PHY_CTRL_MODE_TMDS    (0 << HDMI_PHY_CTRL_MODE_SHIFT)
#define HDMI_PHY_CTRL_MODE_HDMI    (1 << HDMI_PHY_CTRL_MODE_SHIFT)

/****************************************************************************
 * HDMI PHY_STATUS bit definitions
 ****************************************************************************/

#define HDMI_PHY_STATUS_LOCKED     (1 << 0)
#define HDMI_PHY_STATUS_READY      (1 << 1)

/****************************************************************************
 * Common video resolutions
 ****************************************************************************/

#define HDMI_RESOLUTION_640x480    0
#define HDMI_RESOLUTION_800x600    1
#define HDMI_RESOLUTION_1024x768   2
#define HDMI_RESOLUTION_1280x720   3
#define HDMI_RESOLUTION_1280x1024  4
#define HDMI_RESOLUTION_1920x1080  5

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_hdmi_base;

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_HDMI_H */
