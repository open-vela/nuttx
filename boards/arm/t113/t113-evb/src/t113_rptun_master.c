/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_rptun_master.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* core0-NuttX as the rptun ELF master.  It registers one rptun device per
 * remote core it loads and starts each by hand with "rptun start
 * /dev/rptun/<core>":
 *
 *   T113_RPTUN_CORE1 -> /dev/rptun/core1 : the core1 NuttX image (same ELF
 *                       Linux remoteproc loads).  GIC-SGI doorbell, released
 *                       from reset at the loader-resolved ELF e_entry.
 *   T113_RPTUN_DSP   -> /dev/rptun/dsp   : the HiFi4 DSP.  MSGBOX doorbell,
 *                       da/pa address-env for the Xtensa view, released at
 *                       the fixed DSP linker entry.
 *
 * Both loaders share the rptun ELF-loader contract (get_firmware returns an
 * ELF path; the framework's remoteproc_load copies the PT_LOAD segments and
 * reads the static .resource_table from the ELF) but differ in the doorbell
 * backend, address-env, entry source, and stop op -- so they keep separate
 * ops tables and register independently.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/serial/uart_rpmsg.h>

#include "arm_internal.h"
#include "gic.h"
#include "t113_boot.h"
#include "t113-evb.h"

#ifdef CONFIG_T113_RPTUN_DSP
#  include "t113_dsp.h"
#  include "t113_msgbox.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* rpmsg-tty RX/TX ring buffer size (bytes). */

#define T113_RPMSG_UART_BUFSIZE 256

#ifdef CONFIG_T113_RPTUN_CORE1
/* GIC SGI doorbell for core1 (matches the t113-evb master convention):
 *   core0 -> core1 : SGI15  (we trigger)
 *   core1 -> core0 : SGI14  (we receive)
 */

#  define CORE1_DOORBELL_TRIGGER  GIC_IRQ_SGI15   /* outbound: we kick core1 */
#  define CORE1_DOORBELL_EVENT    GIC_IRQ_SGI14   /* inbound: core1 kicks us */
#  define CORE1_CPUNAME           "core1"
#endif

#ifdef CONFIG_T113_RPTUN_DSP
/* DSP ELF entry point.  Fixed by the DSP linker script
 * (ENTRY(_ResetVector) at IRAM_STUB = 0x00400000), so we hand it to
 * t113_dsp_release() directly rather than threading rproc->bootaddr
 * through the framework (which does not expose it to ops->start).
 */

#  define T113_DSP_ENTRY      0x00400000
#  define T113_DSP_CPUNAME    "dsp"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_rptun_dev_s
{
  struct rptun_dev_s rptun;
  rptun_callback_t   callback;
  FAR void          *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The core1 and DSP loaders below each keep their ops table and device
 * state next to the static functions that back them.
 */

/****************************************************************************
 * core1 loader (GIC-SGI doorbell, identity address map, ELF e_entry)
 ****************************************************************************/

#ifdef CONFIG_T113_RPTUN_CORE1

static const char *t113_core1_get_cpuname(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return CORE1_CPUNAME;
}

static const char *t113_core1_get_firmware(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return CONFIG_T113_RPTUN_CORE1_FIRMWARE_PATH;
}

static bool t113_core1_is_autostart(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);

  /* Manual: the user runs "rptun start /dev/rptun/core1". */

  return false;
}

static bool t113_core1_is_master(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return true;
}

static int t113_core1_start(FAR struct rptun_dev_s *dev)
{
  FAR struct remoteproc *rproc = rptun_dev_to_rproc(dev);
  uintptr_t entry;

  /* remoteproc_load has copied the ELF PT_LOAD segments into DRAM and
   * resolved rproc->bootaddr = ELF e_entry.  Guard against a NULL
   * back-pointer (a registration-failure path that never reaches start).
   */

  if (rproc == NULL)
    {
      rptunerr("core1 rproc back-pointer is NULL\n");
      return -EINVAL;
    }

  entry = (uintptr_t)rproc->bootaddr;

  /* core1 comes out of reset with cold caches and fetches the image straight
   * from DRAM, so the segments core0 just wrote (cacheable mapping) must be
   * cleaned to the point of coherency first.
   */

  up_clean_dcache_all();

  t113_release_cpu1(entry);
  return 0;
}

static int t113_core1_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
{
  cpu_set_t cpuset;

  UNUSED(dev);
  UNUSED(vqid);

  /* Kick core1 via its inbound SGI; the GIC SGI carries no payload, the peer
   * scans all vrings on wake (RPTUN_NOTIFY_ALL semantics).
   */

  CPU_ZERO(&cpuset);
  CPU_SET(1, &cpuset);
  up_trigger_irq(CORE1_DOORBELL_TRIGGER, cpuset);
  return 0;
}

static int t113_core1_doorbell_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct t113_rptun_dev_s *priv = arg;

  UNUSED(irq);
  UNUSED(context);

  if (priv != NULL && priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  return OK;
}

static int t113_core1_register_callback(FAR struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        FAR void *arg)
{
  FAR struct t113_rptun_dev_s *priv =
    (FAR struct t113_rptun_dev_s *)dev;

  priv->callback = callback;
  priv->arg      = arg;

  if (callback != NULL)
    {
      up_enable_irq(CORE1_DOORBELL_EVENT);
    }
  else
    {
      up_disable_irq(CORE1_DOORBELL_EVENT);
    }

  return 0;
}

static const struct rptun_ops_s g_t113_core1_rptun_ops =
{
  .get_cpuname       = t113_core1_get_cpuname,
  .get_firmware      = t113_core1_get_firmware,
  .get_addrenv       = NULL,   /* core1 ELF linked at physical DRAM address
                                * (VMA==LMA==PA); rptun_da_to_pa returns
                                * pa=da when addrenv is NULL. */
  .get_resource      = NULL,   /* framework reads .resource_table from ELF */
  .is_autostart      = t113_core1_is_autostart,
  .is_master         = t113_core1_is_master,
  .start             = t113_core1_start,
  .notify            = t113_core1_notify,
  .register_callback = t113_core1_register_callback,
};

static struct t113_rptun_dev_s g_t113_core1_rptun_dev;

static int t113_core1_rptun_init(void)
{
  FAR struct t113_rptun_dev_s *dev = &g_t113_core1_rptun_dev;
  int ret;

  ret = irq_attach(CORE1_DOORBELL_EVENT, t113_core1_doorbell_isr, dev);
  if (ret < 0)
    {
      rptunerr("core1 doorbell irq_attach failed: %d\n", ret);
      return ret;
    }

  dev->rptun.ops = &g_t113_core1_rptun_ops;

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      rptunerr("core1 rptun_initialize failed: %d\n", ret);
      irq_detach(CORE1_DOORBELL_EVENT);
    }

  return ret;
}

#endif /* CONFIG_T113_RPTUN_CORE1 */

/****************************************************************************
 * DSP loader (MSGBOX doorbell, da/pa address-env, fixed linker entry)
 ****************************************************************************/

#ifdef CONFIG_T113_RPTUN_DSP

/* DSP-view device address (da) to AP-view physical address (pa) mapping
 * table for the rptun loader.  Each entry translates one DSP address window
 * to the corresponding AP physical address so ELF PT_LOAD segments can be
 * placed correctly.  Terminated by a size == 0 sentinel.
 */

static const struct rptun_addrenv_s g_t113_dsp_addrenv[] =
{
  { .pa = 0x00028000, .da = 0x20028000, .size = 0x00010000 }, /* IRAM cache */
  { .pa = 0x00038000, .da = 0x20038000, .size = 0x00008000 }, /* DRAM0 cache */
  { .pa = 0x00040000, .da = 0x20040000, .size = 0x00008000 }, /* DRAM1 cache */
  { .pa = 0x40000000, .da = 0x30000000, .size = 0x10000000 }, /* DDR cache */
  { .pa = 0x00028000, .da = 0x00400000, .size = 0x00010000 }, /* IRAM ncache */
  { .pa = 0x00038000, .da = 0x00420000, .size = 0x00008000 }, /* DRAM0 ncache */
  { .pa = 0x00040000, .da = 0x00440000, .size = 0x00008000 }, /* DRAM1 ncache */
  { .pa = 0x40000000, .da = 0x40000000, .size = 0x10000000 }, /* DDR ncache */
  { .pa = 0, .da = 0, .size = 0 }
};

static struct t113_rptun_dev_s g_t113_dsp_rptun_dev;

static const char *t113_dsp_get_cpuname(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return T113_DSP_CPUNAME;
}

static const char *t113_dsp_get_firmware(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return CONFIG_T113_RPTUN_DSP_FIRMWARE_PATH;
}

static const struct rptun_addrenv_s *
t113_dsp_get_addrenv(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return g_t113_dsp_addrenv;
}

static bool t113_dsp_is_autostart(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);

  /* Manual: the user runs "rptun start /dev/rptun/dsp". */

  return false;
}

static bool t113_dsp_is_master(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return true;
}

static int t113_dsp_start(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);

  /* The framework's remoteproc_load has already copied the ELF segments
   * into SRAM/DDR via our address-env.  Release the DSP from reset at the
   * fixed linker entry point.
   */

  return t113_dsp_release(T113_DSP_ENTRY);
}

static int t113_dsp_stop(FAR struct rptun_dev_s *dev)
{
  UNUSED(dev);
  t113_dsp_halt();
  return 0;
}

static int t113_dsp_notify(FAR struct rptun_dev_s *dev, uint32_t vqid)
{
  UNUSED(dev);

  /* Kick the DSP: the vring notifyid is the doorbell payload. */

  t113_msgbox_send(vqid);
  return 0;
}

static void t113_dsp_msgbox_rx(FAR void *arg, uint32_t word)
{
  FAR struct t113_rptun_dev_s *priv = (FAR struct t113_rptun_dev_s *)arg;

  if (priv->callback != NULL)
    {
      priv->callback(priv->arg, word);
    }
}

static int t113_dsp_register_callback(FAR struct rptun_dev_s *dev,
                                      rptun_callback_t callback,
                                      FAR void *arg)
{
  FAR struct t113_rptun_dev_s *priv =
    (FAR struct t113_rptun_dev_s *)dev;

  priv->callback = callback;
  priv->arg      = arg;

  /* Route mailbox RX into the rptun callback (or detach). */

  t113_msgbox_attach(callback ? t113_dsp_msgbox_rx : NULL, priv);
  return 0;
}

static const struct rptun_ops_s g_t113_dsp_rptun_ops =
{
  .get_cpuname       = t113_dsp_get_cpuname,
  .get_firmware      = t113_dsp_get_firmware,
  .get_addrenv       = t113_dsp_get_addrenv,
  .get_resource      = NULL,   /* framework reads .resource_table from ELF */
  .is_autostart      = t113_dsp_is_autostart,
  .is_master         = t113_dsp_is_master,
  .start             = t113_dsp_start,
  .stop              = t113_dsp_stop,
  .notify            = t113_dsp_notify,
  .register_callback = t113_dsp_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int t113_dsp_rptun_init(void)
{
  FAR struct t113_rptun_dev_s *dev = &g_t113_dsp_rptun_dev;

  t113_msgbox_init();

  dev->rptun.ops = &g_t113_dsp_rptun_ops;
  return rptun_initialize(&dev->rptun);
}

#endif /* CONFIG_T113_RPTUN_DSP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rptun_master_init
 *
 * Description:
 *   Register an rptun device for each remote core core0 loads.  Loading and
 *   releasing each remote happen later via "rptun start /dev/rptun/<core>".
 *
 ****************************************************************************/

int t113_rptun_master_init(void)
{
  int ret = OK;

#ifdef CONFIG_T113_RPTUN_CORE1
  ret = t113_core1_rptun_init();
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_T113_RPTUN_DSP
  ret = t113_dsp_rptun_init();
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: rpmsg_serialinit
 *
 * Description:
 *   Called by drivers_initialize() under CONFIG_RPMSG_UART.  core0 is the
 *   master, so it exposes a regular tty per remote core; each peer attaches
 *   its channel once it is loaded and started.
 *
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
#ifdef CONFIG_T113_RPTUN_CORE1
  /* /dev/ttyCORE1 reaches the core1 NuttX shell over rpmsg-tty. */

  uart_rpmsg_init(CORE1_CPUNAME, "CORE1", T113_RPMSG_UART_BUFSIZE, false);
#endif

#ifdef CONFIG_T113_RPTUN_DSP
  /* /dev/ttyDSP reaches the HiFi4 DSP shell over rpmsg-tty. */

  uart_rpmsg_init(T113_DSP_CPUNAME, "DSP", 4096, false);
#endif
}
#endif
