# ##############################################################################
# boards/arm64/h616/yuzuki-h616/src/CMakeLists.txt
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
set(SRCS yuzuki-h616_boardinit.c yuzuki-h616_bringup.c)

if(CONFIG_H616_SDHCI)
  list(APPEND SRCS yuzuki-h616_sdhci.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS yuzuki-h616_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS yuzuki-h616_reset.c)
  endif()
endif()

if(CONFIG_ETC_ROMFS)
  # rcS/rc.sysinit are NSH scripts, NOT C sources: they go into romfs verbatim
  # via RCRAWS (RCSRCS would run them through the C preprocessor, and '#
  # comment' lines then break the build). Keep them NSH-clean.
  set(RCRAWS etc/init.d/rc.sysinit etc/init.d/rcS)

  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCRAWS
    ${RCRAWS}
    PATH
    ${CMAKE_CURRENT_LIST_DIR}/etc)
  target_link_libraries(board PRIVATE romfs_etc)
endif()

target_sources(board PRIVATE ${SRCS})
