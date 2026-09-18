/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp_mipi_csi.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_MIPI_CSI_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_MIPI_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

#include <nuttx/video/imgdata.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Static CSI link parameters, fixed by the attached image sensor mode */

struct esp_mipi_csi_config_s
{
  uint8_t data_lanes;        /* number of CSI data lanes (2) */
  uint32_t lane_rate_mbps;   /* per-lane bit rate from the sensor mode */
  int phy_ldo_chan;          /* DPHY LDO channel (board: 3) */
  uint32_t phy_ldo_mv;       /* DPHY LDO voltage (board: 2500) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mipi_csi_initialize
 *
 * Description:
 *   Bind the MIPI CSI-2 camera capture pipeline (CSI D-PHY -> DWC CSI-2
 *   host -> ISP inline RAW8-to-RGB565 conversion -> CSI bridge -> DW-GDMA)
 *   to a NuttX imgdata lower half.  The heavy hardware bring-up is
 *   deferred to the imgdata init operation, invoked by the video capture
 *   framework when the device is opened; this function only latches the
 *   link parameters and returns the singleton imgdata instance for
 *   capture_register()/v4l2 registration by the board logic.
 *
 * Input Parameters:
 *   config - Static CSI link description (lanes, lane rate, DPHY LDO).
 *            The contents are copied; the caller's structure may live on
 *            the stack.
 *
 * Returned Value:
 *   Pointer to the imgdata lower half on success; NULL on invalid
 *   arguments.
 *
 ****************************************************************************/

FAR struct imgdata_s *esp_mipi_csi_initialize(
    FAR const struct esp_mipi_csi_config_s *config);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_MIPI_CSI_H */
