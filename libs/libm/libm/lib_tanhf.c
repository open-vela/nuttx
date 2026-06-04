/****************************************************************************
 * libs/libm/libm/lib_tanhf.c
 *
 * SPDX-License-Identifier: ISC
 * SPDX-FileCopyrightText: Copyright (C) 2012 Gregory Nutt.
 * SPDX-FileContributor: Ported by: Darcy Gong
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatible, MIT-style license:
 *
 * Copyright (C) 2009, 2010 Nick Johnson <nickbjohnson4224 at gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float tanhf(float x)
{
  float x0;
  float x1;

  /* tanh saturates to +-1 well before expf overflows: |x| >= 9 already
   * rounds to +-1.0F in single precision, and at +-inf the e^x/e^-x ratio
   * would otherwise form inf/inf = NaN.  Handle both (and propagate NaN).
   */

  if (isnanf(x))
    {
      return x;
    }

  if (x >= 9.0F)
    {
      return 1.0F;
    }

  if (x <= -9.0F)
    {
      return -1.0F;
    }

  x0 = expf(x);
  x1 = 1.0F / x0;

  return ((x0 - x1) / (x0 + x1));
}
