/****************************************************************************
 * arch/arm64/include/h616/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_H616_CHIP_H
#define __ARCH_ARM64_INCLUDE_H616_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of bytes in x kibibytes/mebibytes/gibibytes */

#define KB(x)           ((x) << 10)
#define MB(x)           (KB(x) << 10)
#define GB(x)           (MB(UINT64_C(x)) << 10)

/* Allwinner H616 Generic Interrupt Controller v2: Distributor and CPU
 * From sun50iw9p1.dtsi:
 *   gic: interrupt-controller@03020000
 *     reg = <0x03021000 0x1000>,  GIC Dist
 *           <0x03022000 0x2000>,  GIC CPU
 *           <0x03024000 0x2000>,  GIC VCPU Control
 *           <0x03026000 0x2000>;  GIC VCPU
 * Note: NuttX arm64_gicv2 uses CONFIG_GICD_BASE for distributor and
 *       CONFIG_GICR_BASE for the CPU interface (legacy naming).
 */

#define CONFIG_GICD_BASE          0x03021000
#define CONFIG_GICR_BASE          0x03022000

/* Allwinner H616 Memory Map: RAM and Device I/O */

#define CONFIG_RAMBANK1_ADDR      0x40000000
#define CONFIG_RAMBANK1_SIZE      MB(1024)

#define CONFIG_DEVICEIO_BASEADDR  0x00000000
#define CONFIG_DEVICEIO_SIZE      MB(512)

/* U-Boot loads NuttX at this address (kernel_addr_r) */

#define CONFIG_LOAD_BASE          0x40080000

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

.macro  get_cpu_id xreg0
  mrs    \xreg0, mpidr_el1
  ubfx   \xreg0, \xreg0, #0, #8
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_H616_CHIP_H */
