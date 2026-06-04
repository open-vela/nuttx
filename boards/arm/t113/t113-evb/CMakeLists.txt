# ##############################################################################
# boards/arm/t113/t113-evb/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
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

add_subdirectory(src)
add_subdirectory(uartring)

# For CONFIG_T113_BOOT0: mksunxi.py patches eGON checksum in nuttx.bin. Since
# board CMakeLists.txt runs before the nuttx target is defined, we inject the
# command into nuttx_generate_outputs via a global property that is consumed by
# the output generation cmake module.
#
# Fallback: run manually after build: python3 tools/mksunxi.py --nand
# build_<config>/nuttx.bin

if(CONFIG_T113_BOOT0)
  set_property(GLOBAL PROPERTY T113_MKSUNXI_CMD
                               "python3;${NUTTX_DIR}/tools/mksunxi.py;--nand")
endif()
