/****************************************************************************
 * include/nuttx/debug.h
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

/* Compatibility shim for the ESP32-P4 port sources: the openvela fork
 * keeps debug macros in <debug.h> and DEBUGASSERT in <assert.h>, while
 * upstream NuttX code includes <nuttx/debug.h>.  Map to the fork headers.
 */

#ifndef __INCLUDE_NUTTX_DEBUG_H
#define __INCLUDE_NUTTX_DEBUG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <debug.h>

#endif /* __INCLUDE_NUTTX_DEBUG_H */
