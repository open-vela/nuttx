# SPDX-License-Identifier: Apache-2.0

set(SRCS t113_boardinit.c)

if(CONFIG_T113_INTC_PROBE)
  list(APPEND SRCS t113_intc_probe.c)
endif()

if(CONFIG_LIBCXXTOOLCHAIN)
  # Provide __getreent for the toolchain libsupc++ verbose terminate handler.
  list(APPEND SRCS t113_libc_stubs.c)

  # Eagerly bootstrap the classic locale at C++ static-init so the toolchain
  # libstdc++ _S_classic/_S_global statics are constructed consistently before
  # any task constructs a stream.  Otherwise the first std::ifstream's
  # ios_base::_M_init() NULL-derefs the classic locale _Impl (EXCCAUSE=000d).
  list(APPEND SRCS t113_cxx_locale_init.cc)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ETC_ROMFS)
  # Bake the etc/ dir (startup scripts + the KWS model) into a ROMFS the
  # NuttX core auto-mounts at /etc. model.tflite rides along verbatim as an
  # RCRAW (the same mechanism the AP board uses for hifi4.elf), so
  # `tflm -i /etc/model.tflite` can read it on the DSP.
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCSRCS
    etc/init.d/rcS
    etc/init.d/rc.sysinit
    RCRAWS
    etc/model.tflite
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)

  target_link_libraries(board PRIVATE romfs_etc)
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT
  "${CMAKE_CURRENT_LIST_DIR}/../scripts/t113-evb-dsp.ld")
