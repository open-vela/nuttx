/****************************************************************************
 * arch/arm/src/bk7258/bk7258_pm_pwc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * BK7258 PWC worker and CPU1 ready handshake.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/kthread.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>

#include "bk7258_driver.h"
#include "bk7258_psram.h"
#include "hardware/bk7258_mbox.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PM_CPU1_BOOT_READY_CMD 0x5u
#define PM_CTRL_PSRAM_POWER_CMD 0x7u
#define PM_CP1_PSRAM_MALLOC_STATE_CMD 0x8u
#define PM_CP1_DUMP_PSRAM_MALLOC_INFO_CMD 0x9u
#define PM_CP1_RECOVERY_CMD 0xau
#define PM_POWER_PSRAM_MODULE_CPU1 11u
#define PM_POWER_SUBMODULE_SDIO 91u
#define PM_CLOCK_SDIO 22u
#define PM_CLOCK_POWER_UP 1u
#define PM_POWER_MODULE_STATE_ON 0u
#define PM_POWER_MODULE_STATE_OFF 1u
#define PM_RECOVERY_STATE_INIT 0u
#define PM_RECOVERY_STATE_FINISH 1u
#define PM_PSRAM_PROTO_V1 1u
#define PM_QUEUE_DEPTH 8u
#define PM_TRANSPORT_TIMEOUT_MS 500u
#define PM_RESPONSE_TIMEOUT_MS 1000u

struct pwc_message
{
  uint32_t header;
  uint32_t param1;
  uint32_t param2;
  uint32_t param3;
};

static struct pwc_message g_queue[PM_QUEUE_DEPTH];
static volatile unsigned int g_head;
static volatile unsigned int g_tail;
static sem_t g_pwc_sem;
static sem_t g_worker_sem;
static sem_t g_psram_power_sem;
static mutex_t g_pwc_tx_lock = NXMUTEX_INITIALIZER;
#ifdef CONFIG_BK7258_PSRAM
static mutex_t g_psram_power_lock = NXMUTEX_INITIALIZER;
#endif
static volatile bool g_worker_ready;
static volatile int g_worker_result;
static volatile bool g_psram_power_waiting;
static volatile uint32_t g_psram_power_expected_state;
static volatile uint32_t g_psram_power_state;
static volatile int32_t g_psram_power_result;
static volatile uint32_t g_psram_power_version;

static int pwc_send_wait(uint8_t command, uint32_t param1, uint32_t param2,
                         uint32_t param3);

#ifdef CONFIG_BK7258_PSRAM
/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int psram_power_set(uint32_t state)
{
  irqstate_t flags;
  int response_ret;
  int transport_ret;
  int ret;

  ret = nxmutex_lock(&g_psram_power_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxsem_reset(&g_psram_power_sem, 0);
  if (ret < 0)
    {
      nxmutex_unlock(&g_psram_power_lock);
      return ret;
    }

  flags = rspin_lock_irqsave(&g_bk7258_driver_lock);
  g_psram_power_expected_state = state;
  g_psram_power_state = UINT32_MAX;
  g_psram_power_result = -EINPROGRESS;
  g_psram_power_version = 0;
  g_psram_power_waiting = true;
  rspin_unlock_irqrestore(&g_bk7258_driver_lock, flags);

  ret = pwc_send_wait(PM_CTRL_PSRAM_POWER_CMD,
                      PM_POWER_PSRAM_MODULE_CPU1, state,
                      PM_PSRAM_PROTO_V1);
  if (ret == OK)
    {
      transport_ret = OK;
      response_ret = nxsem_tickwait_uninterruptible(
                       &g_psram_power_sem,
                       MSEC2TICK(PM_RESPONSE_TIMEOUT_MS));
      ret = transport_ret != OK ? transport_ret : response_ret;
      if (ret > 0)
        {
          ret = -EIO;
        }
    }
  else if (ret > 0)
    {
      ret = -EIO;
    }

  flags = rspin_lock_irqsave(&g_bk7258_driver_lock);
  g_psram_power_waiting = false;
  rspin_unlock_irqrestore(&g_bk7258_driver_lock, flags);

  if (ret == OK && (g_psram_power_state != state ||
                    g_psram_power_result != OK ||
                    g_psram_power_version != PM_PSRAM_PROTO_V1))
    {
      ret = g_psram_power_result < 0 ? g_psram_power_result : -EPROTO;
    }

  if (ret < 0)
    {
      printf("PWC: PSRAM power %s failed, error=%d result=%ld "
             "state=%lu version=%lu\n",
             state == PM_POWER_MODULE_STATE_ON ? "ON" : "OFF", ret,
             (long)g_psram_power_result,
             (unsigned long)g_psram_power_state,
             (unsigned long)g_psram_power_version);
    }

  nxmutex_unlock(&g_psram_power_lock);
  return ret;
}

int bk7258_sdio_clock_request(bool enable)
{
  int ret;

  if (!enable)
    {
      ret = pwc_send_wait(0x2u, PM_CLOCK_SDIO, 0u, 0u);
      if (ret < 0)
        {
          return ret;
        }

      return pwc_send_wait(0x1u, PM_POWER_SUBMODULE_SDIO, 1u, 0u);
    }

  ret = pwc_send_wait(0x1u, PM_POWER_SUBMODULE_SDIO, 0u, 0u);
  if (ret < 0)
    {
      return ret;
    }

  ret = pwc_send_wait(0x2u, PM_CLOCK_SDIO, PM_CLOCK_POWER_UP, 0u);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(30000);
  return OK;
}
#endif

static int pwc_send_wait(uint8_t command, uint32_t param1, uint32_t param2,
                          uint32_t param3)
{
  int ret;

  ret = nxmutex_lock(&g_pwc_tx_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = bk7258_mailbox_send_pwc(command, param1, param2, param3);
  if (ret == OK)
    {
      ret = bk7258_mailbox_wait_pwc(PM_TRANSPORT_TIMEOUT_MS);
      if (ret > 0)
        {
          ret = -EIO;
        }
    }
  else if (ret > 0)
    {
      ret = -EIO;
    }

  nxmutex_unlock(&g_pwc_tx_lock);
  return ret;
}

static int pwc_rx(const void *raw)
{
  const uint8_t *bytes = raw;
  struct pwc_message message;
  unsigned int next;

  if (bytes == NULL)
    {
      return -EINVAL;
    }

  /* The logical mailbox invokes PWC callbacks with interrupts disabled.
   * Keep the queue update in that critical section.
   */

  next = (g_head + 1u) % PM_QUEUE_DEPTH;
  if (next == g_tail)
    {
      return -EAGAIN;
    }

  message.header = ((const uint32_t *)bytes)[0];
  message.param1 = ((const uint32_t *)bytes)[1];
  message.param2 = ((const uint32_t *)bytes)[2];
  message.param3 = ((const uint32_t *)bytes)[3];
  g_queue[g_head] = message;
  g_head = next;
  nxsem_post(&g_pwc_sem);
  return OK;
}

static int pwc_worker(int argc, char **argv)
{
  int ret = OK;

  (void)argc;
  (void)argv;

  g_worker_result = OK;
  g_worker_ready = true;
  nxsem_post(&g_worker_sem);

  for (; ; )
    {
      struct pwc_message message;
      irqstate_t flags;

      nxsem_wait_uninterruptible(&g_pwc_sem);
      flags = rspin_lock_irqsave(&g_bk7258_driver_lock);
      message = g_queue[g_tail];
      g_tail = (g_tail + 1u) % PM_QUEUE_DEPTH;
      rspin_unlock_irqrestore(&g_bk7258_driver_lock, flags);
      /* A full software queue leaves the physical descriptor deferred with
       * no transport ACK.  Freeing one entry is the progress event that must
       * retry that descriptor even when no new mailbox IRQ arrives.
       */

      bk7258_mbox_kick_rx();

      /* Hardware power/clock actions stay in this thread, never in IRQ. */

      switch (message.header & 0xffu)
        {
          case PM_CTRL_PSRAM_POWER_CMD:
            flags = rspin_lock_irqsave(&g_bk7258_driver_lock);
            if (g_psram_power_waiting &&
                message.param1 == g_psram_power_expected_state)
              {
                g_psram_power_state = message.param1;
                g_psram_power_result = (int32_t)message.param2;
                g_psram_power_version = message.param3;
                g_psram_power_waiting = false;
                nxsem_post(&g_psram_power_sem);
              }

            rspin_unlock_irqrestore(&g_bk7258_driver_lock, flags);
            break;

#ifdef CONFIG_BK7258_PSRAM
          case PM_CP1_PSRAM_MALLOC_STATE_CMD:
            if (message.param1 == 0u)
              {
                ret = pwc_send_wait(PM_CP1_PSRAM_MALLOC_STATE_CMD,
                                    PM_CP1_PSRAM_MALLOC_STATE_CMD,
                                    bk7258_psram_heap_used(), 0);
                if (ret != OK)
                  {
                    printf("PWC: PSRAM malloc state response failed, "
                           "error=%d\n", ret);
                  }
              }
            else if (message.param1 == PM_POWER_MODULE_STATE_OFF)
              {
                /* This CP notification is sent after physical power-down. */

                bk7258_psram_power_lost();
              }
            break;

          case PM_CP1_DUMP_PSRAM_MALLOC_INFO_CMD:
            bk7258_psram_dump();
            break;

          case PM_CP1_RECOVERY_CMD:
            {
              int shutdown_ret = bk7258_psram_shutdown();

              ret = pwc_send_wait(
                      PM_CP1_RECOVERY_CMD, PM_POWER_PSRAM_MODULE_CPU1,
                      shutdown_ret == 0 ? PM_RECOVERY_STATE_FINISH :
                                          PM_RECOVERY_STATE_INIT, 0);
              if (ret != OK)
                {
                  printf("PWC: recovery response failed, error=%d\n", ret);
                }
            }
            break;
#endif

          default:
            break;
        }
    }

  return 0;
}

int bk7258_pwc_start(void)
{
  int pid;
  int ret;

  nxsem_init(&g_pwc_sem, 0, 0);
  nxsem_init(&g_worker_sem, 0, 0);
  nxsem_init(&g_psram_power_sem, 0, 0);
  g_head = 0;
  g_tail = 0;
  g_worker_ready = false;
  g_worker_result = -EINPROGRESS;
  g_psram_power_waiting = false;
  g_psram_power_expected_state = PM_POWER_MODULE_STATE_OFF;
  g_psram_power_state = PM_POWER_MODULE_STATE_OFF;
  g_psram_power_result = -EINPROGRESS;
  g_psram_power_version = 0;
  bk7258_mailbox_set_pwc_rx(pwc_rx);

  pid = kthread_create("pwc", 110, 2048, pwc_worker, NULL);
  if (pid < 0)
    {
      return pid;
    }

  if (nxsem_tickwait_uninterruptible(&g_worker_sem, MSEC2TICK(200)) < 0)
    {
      printf("PWC: worker start timeout\n");
      bk7258_mailbox_dump_stats();
      return -ETIMEDOUT;
    }

  if (g_worker_result < 0 || !g_worker_ready)
    {
      printf("PWC: worker start failed, error=%d\n", g_worker_result);
      return g_worker_result < 0 ? g_worker_result : -EIO;
    }

  printf("PWC: worker ready\n");

  /* This bootstrap frame is allowed before HW_CTRL and UART readiness. */

  ret = pwc_send_wait(PM_CPU1_BOOT_READY_CMD, 1, 0, 0);
  if (ret < 0)
    {
      printf("PWC: CPU1 boot-ready failed, error=%d\n", ret);
      bk7258_mailbox_dump_stats();
      return ret;
    }

  printf("PWC: CPU1 boot-ready acknowledged by CP transport\n");

  return OK;
}

#ifdef CONFIG_BK7258_PSRAM
int bk7258_pwc_psram_start(void)
{
  int ret;

  /* Ordinary PWC commands require READY. Keep them out of the bootstrap
   * routine, which runs before HW_CTRL can make the console ready.
   */

  if (!bk7258_mailbox_link_ready()) return -ENOLINK;

  ret = psram_power_set(PM_POWER_MODULE_STATE_ON);
  if (ret < 0)
    {
      bk7258_mailbox_dump_stats();
      if (psram_power_set(PM_POWER_MODULE_STATE_OFF) < 0)
        {
          printf("PWC: PSRAM rollback response failed\n");
        }

      return ret;
    }

  ret = bk7258_psram_initialize();
  if (ret < 0)
    {
      printf("PWC: PSRAM allocator initialization failed, error=%d\n", ret);
      if (psram_power_set(PM_POWER_MODULE_STATE_OFF) < 0)
        {
          printf("PWC: PSRAM rollback response failed\n");
        }

      return ret;
    }

  printf("PWC: v36 PSRAM ready; dedicated AP heap online\n");
  return OK;
}
#endif
