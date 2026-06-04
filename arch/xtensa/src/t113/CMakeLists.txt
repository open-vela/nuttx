# SPDX-License-Identifier: Apache-2.0

set(SRCS
  t113_start.c
  t113_irq.c
  t113_serial.c
  t113_timerisr.c
  t113_allocateheap.c
  t113_idle.c
  t113_clock.c
  t113_msgbox.c
)

if(CONFIG_RPTUN)
  list(APPEND SRCS t113_dsp_rptun.c t113_rsctable.c t113_addrenv.c)
endif()

target_sources(arch PRIVATE ${SRCS})

# The IRAM reset stub and the exception/interrupt vectors are referenced
# only by hardware (reset PC, VECBASE) and by linker-script KEEP(), not by
# any C symbol.  Put them in the final nuttx target -- not the on-demand
# arch archive -- so KEEP() retains them without forcing --undefined.
target_sources(nuttx PRIVATE t113_head.S t113_vectors.S)
