# ##############################################################################
# libs/libm/libm/xtensa/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
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

if(CONFIG_XTENSA_LIBM)
  set(SRCS)

  if(CONFIG_LIBM_ARCH_FMAF)
    list(APPEND SRCS arch_fmaf.c)
  endif()

  if(CONFIG_LIBM_ARCH_TRUNCF)
    list(APPEND SRCS arch_truncf.c)
  endif()

  if(CONFIG_LIBM_ARCH_SQRTF)
    list(APPEND SRCS arch_sqrtf.c)
  endif()

  if(CONFIG_LIBM_ARCH_FLOORF)
    list(APPEND SRCS arch_floorf.c)
  endif()

  if(CONFIG_LIBM_ARCH_CEILF)
    list(APPEND SRCS arch_ceilf.c)
  endif()

  if(CONFIG_LIBM_ARCH_RINTF)
    list(APPEND SRCS arch_rintf.c)
  endif()

  if(CONFIG_LIBM_ARCH_ROUNDF)
    list(APPEND SRCS arch_roundf.c)
  endif()

  if(CONFIG_LIBM_ARCH_EXPF)
    list(APPEND SRCS arch_expf.c)
  endif()

  if(CONFIG_LIBM_ARCH_LOGF)
    list(APPEND SRCS arch_logf.c)
  endif()

  if(CONFIG_LIBM_ARCH_SINF)
    list(APPEND SRCS arch_sinf.c)
  endif()

  if(CONFIG_LIBM_ARCH_COSF)
    list(APPEND SRCS arch_cosf.c)
  endif()

  if(CONFIG_LIBM_ARCH_POWF)
    list(APPEND SRCS arch_powf.c)
  endif()

  if(CONFIG_LIBM_ARCH_ASINF)
    list(APPEND SRCS arch_asinf.c)
  endif()

  if(CONFIG_LIBM_ARCH_EXP2F)
    list(APPEND SRCS arch_exp2f.c)
  endif()

  if(CONFIG_LIBM_ARCH_CBRTF)
    list(APPEND SRCS arch_cbrtf.c)
  endif()

  if(CONFIG_LIBM_ARCH_HYPOTF)
    list(APPEND SRCS arch_hypotf.c)
  endif()

  if(CONFIG_LIBM_ARCH_ERFF)
    list(APPEND SRCS arch_erff.c)
  endif()

  target_sources(c PRIVATE ${SRCS})
endif()
