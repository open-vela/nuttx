#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_spi.h"

#if defined(CONFIG_LS2K0300_SPI)

#define LS2K0300_SPI_CLOCK_FREQ 200000000UL

struct ls2k0300_spi_priv_s
{
  struct spi_dev_s dev;
  uintptr_t base;
  uint32_t clock_freq;
  uint32_t frequency;
  enum spi_mode_e mode;
  int nbits;
  bool initialized;
};

static uint8_t spi_read_reg(FAR struct ls2k0300_spi_priv_s *priv,
                            uint32_t offset)
{
  return getreg8(priv->base + offset);
}

static void spi_write_reg(FAR struct ls2k0300_spi_priv_s *priv,
                          uint32_t offset, uint8_t value)
{
  putreg8(value, priv->base + offset);
}

static int ls2k0300_spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  return OK;
}

static void ls2k0300_spi_select(FAR struct spi_dev_s *dev,
                               uint32_t devid, bool selected)
{
}

static uint32_t ls2k0300_spi_setfrequency(FAR struct spi_dev_s *dev,
                                         uint32_t frequency)
{
  FAR struct ls2k0300_spi_priv_s *priv =
      (FAR struct ls2k0300_spi_priv_s *)dev;
  uint8_t sper_val;
  uint8_t spcr_val;
  uint32_t div;

  if (frequency == 0)
    {
      frequency = priv->clock_freq / 2;
    }

  div = priv->clock_freq / (2 * frequency);
  if (div < 1)
    {
      div = 1;
    }

  if (div > 256)
    {
      div = 256;
    }

  sper_val = 0;
  if (div > 1)
    {
      sper_val = (uint8_t)((div - 1) & 0xff);
    }

  spi_write_reg(priv, LS2K0300_SPI_SPER, sper_val);

  spcr_val = spi_read_reg(priv, LS2K0300_SPI_SPCR);
  spcr_val &= ~0x0c;
  if (div > 256)
    {
      spcr_val |= 0x08;
    }

  spi_write_reg(priv, LS2K0300_SPI_SPCR, spcr_val);

  priv->frequency = priv->clock_freq / (2 * div);
  return priv->frequency;
}

static void ls2k0300_spi_setmode(FAR struct spi_dev_s *dev,
                                enum spi_mode_e mode)
{
  FAR struct ls2k0300_spi_priv_s *priv =
      (FAR struct ls2k0300_spi_priv_s *)dev;
  uint8_t spcr_val;

  spcr_val = spi_read_reg(priv, LS2K0300_SPI_SPCR);
  spcr_val &= ~0x0c;

  switch (mode)
    {
      case SPIDEV_MODE0:
        break;
      case SPIDEV_MODE1:
        spcr_val |= 0x04;
        break;
      case SPIDEV_MODE2:
        spcr_val |= 0x08;
        break;
      case SPIDEV_MODE3:
        spcr_val |= 0x0c;
        break;
      default:
        return;
    }

  spi_write_reg(priv, LS2K0300_SPI_SPCR, spcr_val);
  priv->mode = mode;
}

static void ls2k0300_spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct ls2k0300_spi_priv_s *priv =
      (FAR struct ls2k0300_spi_priv_s *)dev;

  if (nbits != 8)
    {
      return;
    }

  priv->nbits = nbits;
}

static uint32_t ls2k0300_spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct ls2k0300_spi_priv_s *priv =
      (FAR struct ls2k0300_spi_priv_s *)dev;
  uint8_t spsr_val;

  spi_write_reg(priv, LS2K0300_SPI_FIFO, (uint8_t)(wd & 0xff));

  do
    {
      spsr_val = spi_read_reg(priv, LS2K0300_SPI_SPSR);
    }
  while ((spsr_val & SPSR_SPIF) == 0);

  spi_read_reg(priv, LS2K0300_SPI_SPSR);

  return (uint32_t)spi_read_reg(priv, LS2K0300_SPI_FIFO);
}

#ifdef CONFIG_SPI_EXCHANGE
static void ls2k0300_spi_exchange(FAR struct spi_dev_s *dev,
                                 FAR const void *txbuffer,
                                 FAR void *rxbuffer,
                                 size_t nwords)
{
  FAR struct ls2k0300_spi_priv_s *priv =
      (FAR struct ls2k0300_spi_priv_s *)dev;
  FAR const uint8_t *txptr = (FAR const uint8_t *)txbuffer;
  FAR uint8_t *rxptr = (FAR uint8_t *)rxbuffer;
  uint8_t spsr_val;
  uint8_t tx_data;
  uint8_t rx_data;
  size_t i;

  for (i = 0; i < nwords; i++)
    {
      if (txptr)
        {
          tx_data = *txptr++;
        }
      else
        {
          tx_data = 0xff;
        }

      spi_write_reg(priv, LS2K0300_SPI_FIFO, tx_data);

      do
        {
          spsr_val = spi_read_reg(priv, LS2K0300_SPI_SPSR);
        }
      while ((spsr_val & SPSR_SPIF) == 0);

      spi_read_reg(priv, LS2K0300_SPI_SPSR);

      rx_data = spi_read_reg(priv, LS2K0300_SPI_FIFO);

      if (rxptr)
        {
          *rxptr++ = rx_data;
        }
    }
}
#endif

static uint8_t ls2k0300_spi_status(FAR struct spi_dev_s *dev,
                                  uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

static const struct spi_ops_s g_spi_ops =
{
  ls2k0300_spi_lock,
  ls2k0300_spi_select,
  ls2k0300_spi_setfrequency,
#ifdef CONFIG_SPI_DELAY_CONTROL
  NULL,
#endif
  ls2k0300_spi_setmode,
  ls2k0300_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  NULL,
#endif
  ls2k0300_spi_status,
#ifdef CONFIG_SPI_CMDDATA
  NULL,
#endif
  ls2k0300_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  ls2k0300_spi_exchange,
#else
  NULL,
  NULL,
#endif
#ifdef CONFIG_SPI_TRIGGER
  NULL,
#endif
  NULL
};

static struct ls2k0300_spi_priv_s g_spi_priv[2] =
{
  {
    .dev = { .ops = &g_spi_ops },
    .base = PHYS_TO_UNCACHED(LS2K0300_SPI0_BASE),
    .clock_freq = LS2K0300_SPI_CLOCK_FREQ,
    .nbits = 8,
  },
  {
    .dev = { .ops = &g_spi_ops },
    .base = PHYS_TO_UNCACHED(LS2K0300_SPI1_BASE),
    .clock_freq = LS2K0300_SPI_CLOCK_FREQ,
    .nbits = 8,
  },
};

FAR struct spi_dev_s *ls2k0300_spiflash_initialize(int port)
{
  FAR struct ls2k0300_spi_priv_s *priv;
  uint8_t spcr_val;

  if (port < 0 || port > 1)
    {
      return NULL;
    }

  priv = &g_spi_priv[port];

  if (!priv->initialized)
    {
      spi_write_reg(priv, LS2K0300_SPI_SPER, 0x07);

      spcr_val = SPCR_SPE;
      spi_write_reg(priv, LS2K0300_SPI_SPCR, spcr_val);

      spi_write_reg(priv, LS2K0300_SPI_SFCS, 0x00);

      priv->initialized = true;
    }

  return &priv->dev;
}

#endif
