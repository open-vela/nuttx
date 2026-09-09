/****************************************************************************
 * arch/arm/src/bk7258/bk7258_random.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <syslog.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include "bk7258_wifi.h"

static mutex_t g_random_lock = NXMUTEX_INITIALIZER;
static uint8_t g_previous[32];
static bool g_have_previous;
static bool g_failed;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t bk7258_random_read(struct file *filep, char *buffer,
                                  size_t length)
{
  uint8_t block[32];
  size_t done = 0;
  size_t count;
  int ret;

  (void)filep;
  if (length == 0)
    {
      return 0;
    }

  ret = nxmutex_lock(&g_random_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (g_failed)
    {
      nxmutex_unlock(&g_random_lock);
      return -EIO;
    }

  /* Limit each read to one bounded mailbox transaction. Callers must handle
   * short reads. Detect stuck output, not statistical entropy quality.
   */

  ret = bk7258_wifi_random_block(block, sizeof(block));
  if (ret == 0)
    {
      bool constant = true;
      for (count = 4; count < sizeof(block); count++)
        {
          constant &= block[count] == block[count % 4];
        }

      if (constant || (g_have_previous &&
                      memcmp(block, g_previous, sizeof(block)) == 0))
        {
          g_failed = true;
          ret = -EIO;
        }
      else
        {
          memcpy(g_previous, block, sizeof(block));
          g_have_previous = true;
          done = length < sizeof(block) ? length : sizeof(block);
          memcpy(buffer, block, done);
        }
    }

  explicit_bzero(block, sizeof(block));
  nxmutex_unlock(&g_random_lock);
  return ret < 0 ? ret : (ssize_t)done;
}

static const struct file_operations g_random_ops =
{
  .read = bk7258_random_read,
};

void devrandom_register(void)
{
  int ret = register_driver("/dev/random", &g_random_ops, 0444, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "CP random registration failed: %d\n", ret);
    }
}
