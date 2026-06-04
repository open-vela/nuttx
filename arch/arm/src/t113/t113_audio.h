/****************************************************************************
 * arch/arm/src/t113/t113_audio.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_T113_T113_AUDIO_H
#define __ARCH_ARM_SRC_T113_T113_AUDIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/audio/audio.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct audio_lowerhalf_s *t113_codec_adc_initialize(void);
struct audio_lowerhalf_s *t113_codec_dac_initialize(void);

#endif /* __ARCH_ARM_SRC_T113_T113_AUDIO_H */
