/****************************************************************************
 * boards/xtensa/t113/t113-evb-dsp/src/t113_boardinit.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <debug.h>
#include <nuttx/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_clock_init(void);

#ifdef CONFIG_T113_INTC_PROBE
void t113_intc_probe(void);
#endif

#ifdef CONFIG_RPTUN
int t113_dsp_rptun_init(void);
#endif

#ifdef CONFIG_LIBCXXTOOLCHAIN
void t113_cxx_locale_bootstrap(void);
#endif

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* AP-side already configures UART2 pin mux + clock before releasing
   * the DSP from reset; no additional setup is needed here.
   */

  t113_clock_init();

#ifdef CONFIG_LIBCXXTOOLCHAIN
  /* Bring up the toolchain libstdc++ classic locale once, here, before any
   * task constructs a std::ifstream.  Otherwise the lazy classic-locale
   * bootstrap leaves _S_classic/_S_global inconsistent and the first
   * stream's ios_base::_M_init() NULL-derefs the classic locale _Impl
   * (EXCCAUSE=000d).
   */

  t113_cxx_locale_bootstrap();
#endif

#ifdef CONFIG_RPTUN
  /* Register the DSP-side rptun slave (/dev/rptun/ap) and start it; it
   * waits for the AP master's vring kicks over the MSGBOX doorbell.
   */

  int rptret = t113_dsp_rptun_init();

  if (rptret < 0)
    {
      syslog(LOG_ERR, "ERROR: t113_dsp_rptun_init failed: %d\n", rptret);
    }
#endif

#ifdef CONFIG_T113_INTC_PROBE
  /* Boot-time silicon probe to discover DSP_INTC source IDs.  Blocks
   * NSH start-up until the probe times out or a source fires; intended
   * only for one-off bring-up builds.
   */

  t113_intc_probe();
#endif
}
#endif

#ifdef CONFIG_BOARDCTL
int board_app_initialize(uintptr_t arg)
{
  /* Board application initialization.  Peripheral drivers are initialized
   * in board_late_initialize(); nothing additional is needed here.
   */

  (void)arg;
  return 0;
}
#endif
