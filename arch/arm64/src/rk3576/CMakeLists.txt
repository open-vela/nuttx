# ##############################################################################
# arch/arm64/src/rk3576/CMakeLists.txt
#
# RK3576 chip layer: openvela AMP slave (Cortex-A53) alongside Linux.
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS rk3576_boot.c rk3576_serial.c rk3576_addrenv.c rk3576_soc.c)

if(CONFIG_RK3576_SAI)
  list(APPEND SRCS rk3576_sai.c)
endif()

if(CONFIG_RK3576_PDM)
  list(APPEND SRCS rk3576_pdm.c)
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS rk3576_rptun.c rk3576_rsctable.c)
endif()

target_sources(arch PRIVATE ${SRCS})
