/****************************************************************************
 * arch/arm/src/bk7258/bk7258_psram.h
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

#ifndef __ARCH_ARM_SRC_BK7258_BK7258_PSRAM_H
#define __ARCH_ARM_SRC_BK7258_BK7258_PSRAM_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bk7258_psram_init(void);
uint32_t bk7258_psram_get_id(void);
uint32_t bk7258_psram_get_size(void);
int bk7258_psram_probe(void);
int bk7258_psram_test(uint32_t size_bytes);
int bk7258_psram_alias(void);
int bk7258_psram_width(void);

#endif /* __ARCH_ARM_SRC_BK7258_BK7258_PSRAM_H */
