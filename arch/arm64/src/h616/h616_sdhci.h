/****************************************************************************
 * arch/arm64/src/h616/h616_sdhci.h
 *
 * H616 (sun50iw9) SD/MMC host controller driver public interface.
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

#ifndef __ARCH_ARM64_SRC_H616_H616_SDHCI_H
#define __ARCH_ARM64_SRC_H616_H616_SDHCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sdio.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sdio_initialize
 *
 * Description:
 *   Initialize the SDIO interface for a slot.
 *
 ****************************************************************************/

struct sdio_dev_s *sdio_initialize(int slotno);

/****************************************************************************
 * Name: sdio_mediachange
 *
 * Description:
 *   Called by board-specific logic to signal a card insertion/removal.
 *
 ****************************************************************************/

void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot);

/****************************************************************************
 * Name: sdio_wrprotect
 *
 * Description:
 *   Called by board-specific logic to report write protection.
 *
 ****************************************************************************/

void sdio_wrprotect(FAR struct sdio_dev_s *dev, bool wrprotect);

/****************************************************************************
 * Name: h616_sdhci_set_datapath
 *
 * Description:
 *   Select the data path used by the mmcsd I/O layer: 1 = CPU/AHB FIFO
 *   (the verified boot default), 0 = internal DMA (IDMAC).  Used by the
 *   board bringup IDMAC probe to try the DMA path once the AHB path has
 *   already brought the card up.
 *
 ****************************************************************************/

void h616_sdhci_set_datapath(int use_ahb);

#endif /* __ARCH_ARM64_SRC_H616_H616_SDHCI_H */
