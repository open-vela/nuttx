/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/src/stm32_pwm.c
 *
 * VelaGuard TIM15 CH2 PWM (/dev/pwm0) registration for the expansion board.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "stm32h750b-dk.h"

#ifdef CONFIG_PWM
#  ifdef CONFIG_STM32H7_TIM15_PWM
#    define HAVE_PWM 1
#  endif
#endif

int stm32_pwm_setup(void)
{
#ifdef HAVE_PWM
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  if (!initialized)
    {
      pwm = stm32_pwminitialize(15);
      if (!pwm)
        {
          tmrerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          tmrerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}
