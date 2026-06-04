/****************************************************************************
 * arch/arm/src/t113/t113_ce.h
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied.  See the License for the specific language governing
 * permissions and limitations under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_T113_T113_CE_H
#define __ARCH_ARM_SRC_T113_T113_CE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: t113_ce_initialize
 *
 * Description:
 *   Initialize the T113 Crypto Engine.  Enables CCU clocks, resets the
 *   CE block, and provides:
 *     - aes_cypher()          for CONFIG_CRYPTO_AES  (weak override)
 *     - t113_ce_sha256()      for CONFIG_T113_CE_SHA256
 *     - /dev/urandom          for CONFIG_T113_CE_TRNG (devurandom_register)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int t113_ce_initialize(void);

/****************************************************************************
 * Name: t113_ce_sha256
 *
 * Description:
 *   Compute SHA-256 digest using the CE hardware.
 *   Available when CONFIG_T113_CE_SHA256 is enabled.
 *
 * Input Parameters:
 *   in      - Input data buffer
 *   insize  - Input data length in bytes
 *   digest  - Output buffer for 32-byte SHA-256 digest
 *
 * Returned Value:
 *   OK on success; negative errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_T113_CE_SHA256
int t113_ce_sha256(FAR const void *in, size_t insize,
                   FAR uint8_t *digest);
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_CE_H */
