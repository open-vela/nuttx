/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_spi.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SPI_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_SPI_H

#define LS2K0300_SPI_SPCR            0x00
#define LS2K0300_SPI_SPSR            0x01
#define LS2K0300_SPI_FIFO            0x02
#define LS2K0300_SPI_SPER            0x03
#define LS2K0300_SPI_PARA            0x04
#define LS2K0300_SPI_SPCS            0x04
#define LS2K0300_SPI_SFCS            0x05
#define LS2K0300_SPI_TIMI            0x06

#define SPCR_SPE                    0x40
#define SPSR_SPIF                   0x80
#define SPSR_WCOL                   0x40
#define PARA_MEM_EN                 0x01

#endif
