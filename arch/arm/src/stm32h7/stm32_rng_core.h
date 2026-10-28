/* SPDX-License-Identifier: Apache-2.0 */
#ifndef __ARCH_ARM_SRC_STM32H7_STM32_RNG_CORE_H
#define __ARCH_ARM_SRC_STM32H7_STM32_RNG_CORE_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* RM0433 Rev 8, RNG section 34; STM32H750XB only. */

enum stm32_rng_reg_e
{
  RNG_REG_RCC_CR,
  RNG_REG_RCC_D2CCIP2R,
  RNG_REG_RCC_AHB2ENR,
  RNG_REG_RCC_AHB2RSTR,
  RNG_REG_CR,
  RNG_REG_SR,
  RNG_REG_DR
};

struct stm32_rng_io_s
{
  uint32_t (*read)(void *ctx, enum stm32_rng_reg_e reg);
  void (*write)(void *ctx, enum stm32_rng_reg_e reg, uint32_t value);
  void (*update)(void *ctx, enum stm32_rng_reg_e reg,
                 uint32_t clear, uint32_t set);
  uint32_t (*now_ms)(void *ctx);
  void (*wait)(void *ctx);
  void *ctx;
};

struct stm32_rng_state_s
{
  bool initialized;
  bool have_previous;
  uint32_t previous;
  int error;
};

static inline int stm32_rng_word(const struct stm32_rng_io_s *io,
                                 uint32_t started, uint32_t *value)
{
  for (;;)
    {
      uint32_t status;

      if (io->now_ms(io->ctx) - started >= 100u)
        {
          return -ETIMEDOUT;
        }

      status = io->read(io->ctx, RNG_REG_SR);

      /* CECS/SECS and their interrupt status are fatal for this boot. */

      if ((status & 0x66u) != 0)
        {
          return -EIO;
        }

      if ((status & 1u) != 0)
        {
          *value = io->read(io->ctx, RNG_REG_DR);
          return (io->read(io->ctx, RNG_REG_SR) & 0x66u) ? -EIO : 0;
        }

      if (io->now_ms(io->ctx) - started >= 100u)
        {
          return -ETIMEDOUT;
        }

      io->wait(io->ctx);
    }
}

static inline int stm32_rng_core_read(const struct stm32_rng_io_s *io,
                                      struct stm32_rng_state_s *state,
                                      uint8_t *out, size_t length)
{
  uint32_t started = io->now_ms(io->ctx);
  uint32_t word;
  size_t offset = 0;
  int ret;

  if (!length)
    {
      return 0;
    }

  if (!out || length > 4096)
    {
      return -EINVAL;
    }

  if (state->error)
    {
      memset(out, 0, length);
      return state->error;
    }

  if (!state->initialized)
    {
      /* HSI48ON/RDY belong to RCC_CR, not the calibration-only CRRCR. */

      io->update(io->ctx, RNG_REG_RCC_CR, 0, 1u << 12);
      while (!(io->read(io->ctx, RNG_REG_RCC_CR) & (1u << 13)))
        {
          if (io->now_ms(io->ctx) - started >= 20u)
            {
              ret = -ETIMEDOUT;
              goto failed;
            }

          io->wait(io->ctx);
        }

      /* RNGSEL=00 (HSI48), preserve all other kernel clock selections. */

      io->update(io->ctx, RNG_REG_RCC_D2CCIP2R, 3u << 8, 0);
      io->update(io->ctx, RNG_REG_RCC_AHB2ENR, 0, 1u << 6);
      (void)io->read(io->ctx, RNG_REG_RCC_AHB2ENR);
      io->update(io->ctx, RNG_REG_RCC_AHB2RSTR, 0, 1u << 6);
      io->update(io->ctx, RNG_REG_RCC_AHB2RSTR, 1u << 6, 0);

      /* RNGEN=1, IE=0, CED=0: clock error detection remains enabled.
       * 48 MHz > HCLK(200 MHz)/32 on the verified H750B-DK preset.
       */

      io->write(io->ctx, RNG_REG_CR, 1u << 2);
      ret = stm32_rng_word(io, started, &word);
      if (ret < 0)
        {
          goto failed;
        }

      /* Discard the first sample and compare consecutive full words. */

      state->previous = word;
      state->have_previous = true;
      state->initialized = true;
    }

  while (offset < length)
    {
      size_t take = length - offset;

      ret = stm32_rng_word(io, started, &word);
      if (ret < 0)
        {
          goto failed;
        }

      if (state->have_previous && word == state->previous)
        {
          ret = -EIO;
          goto failed;
        }

      state->previous = word;
      state->have_previous = true;
      if (take > sizeof(word))
        {
          take = sizeof(word);
        }

      memcpy(out + offset, &word, take);
      offset += take;
    }

  return (int)length;

failed:
  /* No automatic seed-error recovery; a health error needs a real reset. */

  state->error = ret;
  if (io->read(io->ctx, RNG_REG_RCC_AHB2ENR) & (1u << 6))
    {
      io->write(io->ctx, RNG_REG_CR, 0);
    }
  memset(out, 0, length);
  return ret;
}
#endif
