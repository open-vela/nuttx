/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_board_audio.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * AI-VOX3 carrier board audio bring-up (ES8311 codec on I2C0 + I2S0).
 *
 * Hardware wiring (AI-VOX3 vendor BSP, cross-verified between:
 *   - ai_vox-main/examples/ai-vox3/ai-vox3/main.cpp:58-65
 *   - Arduino-period port build_hb/sketch/hardware_pins.h:93-109
 *   - AI-VOX3_openvela_port.md pin table):
 *
 *   I2S0 MCLK = GPIO11   (codec MCLK)
 *   I2S0 BCLK = GPIO10   (codec SCLK)
 *   I2S0 WS   = GPIO8    (codec LRCK)
 *   I2S0 DOUT = GPIO7    (codec DSDIN, ESP32 -> codec DAC)
 *   I2S0 DIN  = GPIO9    (codec ASDOUT, codec ADC -> ESP32)
 *   I2C0 SDA  = GPIO13   (codec CDATA)
 *   I2C0 SCL  = GPIO12   (codec CCLK)
 *   I2C addr  = 0x18     (7-bit, per working Arduino driver)
 *
 * NOTE: There is NO dedicated PA-enable GPIO on this board.  The speaker
 * power amplifier is driven directly from the codec analog output and is
 * enabled/unmuted purely through ES8311 registers by the codec driver
 * (verified against build_hb/sketch/es8311_audio.cpp).
 *
 * Device registration uses the ALSA-lib-on-NuttX default names:
 *   /dev/audio/pcm0p   playback (apps/audioutils/alsa-lib/pcm.c:59)
 *   /dev/audio/pcm0c   capture
 * which are also the defaults of AGENT_AUDIO_PLAYBACK_DEV /
 * AGENT_AUDIO_CAPTURE_DEV in packages/ai_agent/include/agent_config.h.
 * The traditional NuttX-native audio API names "pcm0" / "pcm_in0" are
 * registered as aliases for generic NuttX audio users.
 *
 * This file intentionally mirrors boards/xtensa/esp32s3/common/src/
 * esp32s3_es8311.c but registers RAW codec lower-halves (no PCM decoder
 * front end): this tree has no drivers/audio/pcm.c, and the ALSA-lib /
 * media-framework chain delivers raw PCM frames, not WAV containers.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <syslog.h>
#include <errno.h>
#include <stdbool.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/es8311.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32s3_i2c.h"
#include "esp32s3_i2s.h"

#if defined(CONFIG_ESP32S3_I2S) && defined(CONFIG_AUDIO_ES8311)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ES8311 control I2C: port 0, 7-bit address 0x18, 400 kHz (vendor BSP) */

#define BOARD_ES8311_I2C_PORT  0
#define BOARD_ES8311_I2C_ADDR  0x18
#define BOARD_ES8311_I2C_FREQ  400000

/* ES8311 data I2S: port 0, TX and RX share one BCLK/WS pair */

#define BOARD_ES8311_I2S_PORT  0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Two independent ES8311 instances (playback / capture) sharing the same
 * I2C bus and I2S port, mirroring common/src/esp32s3_es8311.c.
 */

static struct es8311_lower_s g_es8311_lower[2];
static bool g_audio_initialized = false;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_board_audio_initialize
 *
 * Description:
 *   Initialize the AI-VOX3 ES8311 audio codec and register the audio
 *   devices /dev/audio/pcm0p, /dev/audio/pcm0c (ALSA-lib default names,
 *   used by the ai_agent voice stack) and the aliases /dev/audio/pcm0,
 *   /dev/audio/pcm_in0.
 *
 *   The I2S/I2C pad GPIOs come from the Kconfig options
 *   CONFIG_ESP32S3_I2S0_{MCLK,BCLK,WS,DOUT,DIN}PIN and
 *   CONFIG_ESP32S3_I2C0_{SDA,SCL}PIN, set to the AI-VOX3 wiring in the
 *   ai_agent defconfig.
 *
 * Returned Value:
 *   Zero is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp32s3_board_audio_initialize(void)
{
  struct audio_lowerhalf_s *play;
  struct audio_lowerhalf_s *cap;
  struct i2s_dev_s *i2s;
  struct i2c_master_s *i2c;
  int ret;

  if (g_audio_initialized)
    {
      return OK;
    }

  /* Get the I2S interface shared by codec TX and RX */

  i2s = esp32s3_i2sbus_initialize(BOARD_ES8311_I2S_PORT);
  if (i2s == NULL)
    {
      syslog(LOG_ERR, "audio: I2S%d init failed\n", BOARD_ES8311_I2S_PORT);
      return -ENODEV;
    }

  /* Get the I2C interface used for codec register access */

  i2c = esp32s3_i2cbus_initialize(BOARD_ES8311_I2C_PORT);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "audio: I2C%d init failed\n", BOARD_ES8311_I2C_PORT);
      return -ENODEV;
    }

  /* Playback instance (I2S TX -> codec DAC) */

  g_es8311_lower[0].address   = BOARD_ES8311_I2C_ADDR;
  g_es8311_lower[0].frequency = BOARD_ES8311_I2C_FREQ;

  play = es8311_initialize(i2c, i2s, &g_es8311_lower[0]);
  if (play == NULL)
    {
      syslog(LOG_ERR, "audio: ES8311 playback init failed\n");
      return -ENODEV;
    }

  /* Capture instance (codec ADC -> I2S RX) */

  g_es8311_lower[1].address   = BOARD_ES8311_I2C_ADDR;
  g_es8311_lower[1].frequency = BOARD_ES8311_I2C_FREQ;

  cap = es8311_initialize(i2c, i2s, &g_es8311_lower[1]);
  if (cap == NULL)
    {
      syslog(LOG_ERR, "audio: ES8311 capture init failed\n");
      return -ENODEV;
    }

  /* Register under the ALSA-lib default device names.  The ALSA lib maps
   * its "default" device to "pcm0p" / "pcm0c" (apps/audioutils/alsa-lib/
   * pcm.c:59), and ai_agent's AGENT_AUDIO_*_DEV defaults point at the
   * same paths (packages/ai_agent/include/agent_config.h:352/357).
   */

  ret = audio_register("pcm0p", play);
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio: register pcm0p failed: %d\n", ret);
      return ret;
    }

  ret = audio_register("pcm0c", cap);
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio: register pcm0c failed: %d\n", ret);
      return ret;
    }

  /* Traditional NuttX-native audio API aliases */

  ret = audio_register("pcm0", play);
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio: register pcm0 failed: %d\n", ret);
      return ret;
    }

  ret = audio_register("pcm_in0", cap);
  if (ret < 0)
    {
      syslog(LOG_ERR, "audio: register pcm_in0 failed: %d\n", ret);
      return ret;
    }

  g_audio_initialized = true;
  syslog(LOG_INFO, "audio: ES8311 ready on I2C%d/I2S%d "
         "(pcm0p pcm0c pcm0 pcm_in0)\n",
         BOARD_ES8311_I2C_PORT, BOARD_ES8311_I2S_PORT);
  return OK;
}

#endif /* CONFIG_ESP32S3_I2S && CONFIG_AUDIO_ES8311 */
