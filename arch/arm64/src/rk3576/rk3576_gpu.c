/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_gpu.c
 *
 * Mali-G51 GPU driver for RK3576
 * Supports open-source Lima/Panfrost driver integration
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_gpu.h"
#include "rk3576_gpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPU timeout (microseconds) */

#define GPU_TIMEOUT_US           1000000
#define GPU_RESET_TIMEOUT_US     100000

/* GPU clock frequency */

#define GPU_CLK_FREQ             500000000  /* 500MHz default */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPU state */

static struct
{
  int type;                     /* GPU type */
  bool initialized;             /* Initialized flag */
  bool power_on;                /* Power state */
  volatile uint32_t *regs;     /* Register base (mmapped) */
} g_gpu;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_gpu_read_reg
 *
 * Description:
 *   Read a GPU register.
 *
 ****************************************************************************/

static inline uint32_t rk3576_gpu_read_reg(uint32_t offset)
{
  if (g_gpu.regs)
    {
      return g_gpu.regs[offset / 4];
    }

  return getreg32(RK3576_GPU_ADDR + offset);
}

/****************************************************************************
 * Name: rk3576_gpu_write_reg
 *
 * Description:
 *   Write to a GPU register.
 *
 ****************************************************************************/

static inline void rk3576_gpu_write_reg(uint32_t offset, uint32_t value)
{
  if (g_gpu.regs)
    {
      g_gpu.regs[offset / 4] = value;
    }
  else
    {
      putreg32(value, RK3576_GPU_ADDR + offset);
    }
}

/****************************************************************************
 * Name: rk3576_gpu_wait_reset
 *
 * Description:
 *   Wait for GPU reset to complete.
 *
 ****************************************************************************/

static int rk3576_gpu_wait_reset(void)
{
  int timeout = GPU_RESET_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t status = rk3576_gpu_read_reg(GPU_INT_RAWSTAT);
      if (status & GPU_INT_RESET_COMPLETED)
        {
          rk3576_gpu_write_reg(GPU_INT_CLEAR, GPU_INT_RESET_COMPLETED);
          return OK;
        }

      up_udelay(10);
    }

  gerr("GPU: reset timeout\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_gpu_init
 *
 * Description:
 *   Initialize the GPU. Detect GPU type and prepare for use.
 *
 ****************************************************************************/

int rk3576_gpu_init(void)
{
  /* Detect GPU by reading ID register */

  uint32_t gpu_id = rk3576_gpu_read_reg(GPU_ID);
  uint32_t product_id = (gpu_id >> 16) & 0xffff;

  g_gpu.initialized = false;
  g_gpu.power_on = false;
  g_gpu.type = GPU_TYPE_NONE;

  /* Check for Mali-G51 (product ID 0x7211 or similar) */

  if (product_id >= 0x7200 && product_id <= 0x72ff)
    {
      g_gpu.type = GPU_TYPE_MALI_G51;
      ginfo("GPU: Mali-G51 detected (ID=0x%08x)\n", gpu_id);
    }
  else if (gpu_id != 0 && gpu_id != 0xffffffff)
    {
      ginfo("GPU: Unknown GPU (ID=0x%08x, product=0x%04x)\n",
              gpu_id, product_id);
      g_gpu.type = GPU_TYPE_MALI_G51;
    }
  else
    {
      gwarn("GPU: No GPU detected, using software rendering\n");
      g_gpu.type = GPU_TYPE_SOFTWARE;
    }

  g_gpu.initialized = true;

  ginfo("GPU: initialized, type=%d\n", g_gpu.type);
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpu_deinit
 *
 * Description:
 *   Deinitialize the GPU.
 *
 ****************************************************************************/

void rk3576_gpu_deinit(void)
{
  if (g_gpu.power_on)
    {
      rk3576_gpu_power_off();
    }

  g_gpu.initialized = false;
}

/****************************************************************************
 * Name: rk3576_gpu_power_on
 *
 * Description:
 *   Power on the GPU.
 *
 ****************************************************************************/

int rk3576_gpu_power_on(void)
{
  if (!g_gpu.initialized)
    {
      return -ENODEV;
    }

  if (g_gpu.power_on)
    {
      return OK;
    }

  if (g_gpu.type == GPU_TYPE_SOFTWARE)
    {
      g_gpu.power_on = true;
      return OK;
    }

  /* Enable GPU clock */

  putreg32(1, RK3576_GPU_ADDR + GPU_CLK_GATE);

  /* Power on GPU */

  putreg32(1, RK3576_GPU_ADDR + GPU_PWRON);

  /* Wait for power on */

  {
    int timeout = 1000;
    while (timeout--)
      {
        if (rk3576_gpu_read_reg(GPU_INT_RAWSTAT) & GPU_INT_RESET_COMPLETED)
          {
            break;
          }

        up_udelay(10);
      }
  }

  /* Clear interrupts */

  rk3576_gpu_write_reg(GPU_INT_CLEAR, 0xffffffff);

  /* Enable core */

  rk3576_gpu_write_reg(GPU_CORE_EN, 1);

  g_gpu.power_on = true;

  ginfo("GPU: powered on\n");
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpu_power_off
 *
 * Description:
 *   Power off the GPU.
 *
 ****************************************************************************/

int rk3576_gpu_power_off(void)
{
  if (!g_gpu.initialized)
    {
      return -ENODEV;
    }

  if (!g_gpu.power_on)
    {
      return OK;
    }

  if (g_gpu.type == GPU_TYPE_SOFTWARE)
    {
      g_gpu.power_on = false;
      return OK;
    }

  /* Disable core */

  rk3576_gpu_write_reg(GPU_CORE_EN, 0);

  /* Power off GPU */

  putreg32(1, RK3576_GPU_ADDR + GPU_POWEROFF);

  /* Disable clock */

  putreg32(0, RK3576_GPU_ADDR + GPU_CLK_GATE);

  g_gpu.power_on = false;

  ginfo("GPU: powered off\n");
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpu_reset
 *
 * Description:
 *   Reset the GPU.
 *
 ****************************************************************************/

int rk3576_gpu_reset(void)
{
  if (g_gpu.type == GPU_TYPE_SOFTWARE)
    {
      return OK;
    }

  /* Soft reset */

  rk3576_gpu_write_reg(GPU_INT_CLEAR, 0xffffffff);

  /* Wait for reset */

  return rk3576_gpu_wait_reset();
}

/****************************************************************************
 * Name: rk3576_gpu_get_type
 *
 * Description:
 *   Get the GPU type.
 *
 ****************************************************************************/

int rk3576_gpu_get_type(void)
{
  return g_gpu.type;
}

/****************************************************************************
 * Name: rk3576_gpu_get_id
 *
 * Description:
 *   Get the GPU ID.
 *
 ****************************************************************************/

uint32_t rk3576_gpu_get_id(void)
{
  if (g_gpu.type == GPU_TYPE_SOFTWARE)
    {
      return 0;
    }

  return rk3576_gpu_read_reg(GPU_ID);
}

/****************************************************************************
 * Name: rk3576_gpu_is_available
 *
 * Description:
 *   Check if GPU is available for use.
 *
 ****************************************************************************/

bool rk3576_gpu_is_available(void)
{
  return g_gpu.initialized && (g_gpu.type != GPU_TYPE_NONE);
}

/****************************************************************************
 * Name: rk3576_gpu_map_registers
 *
 * Description:
 *   Map GPU registers into virtual address space.
 *
 ****************************************************************************/

int rk3576_gpu_map_registers(void)
{
  if (g_gpu.type == GPU_TYPE_SOFTWARE)
    {
      return OK;
    }

  /* For NuttX flat mode, registers are already accessible */

  g_gpu.regs = NULL;

  ginfo("GPU: registers accessible via MMIO\n");
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpu_unmap_registers
 *
 * Description:
 *   Unmap GPU registers.
 *
 ****************************************************************************/

void rk3576_gpu_unmap_registers(void)
{
  g_gpu.regs = NULL;
}
