/****************************************************************************
 * drivers/bch/bchlib_cache.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#include <sys/types.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "bch.h"

#if defined(CONFIG_BCH_ENCRYPTION)
#  include <nuttx/crypto/crypto.h>
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bch_xor
 ****************************************************************************/

#if defined(CONFIG_BCH_ENCRYPTION)
static void bch_xor(uint32_t *R, uint32_t *A, uint32_t *B)
{
  R[0] = A[0] ^ B[0];
  R[1] = A[1] ^ B[1];
  R[2] = A[2] ^ B[2];
  R[3] = A[3] ^ B[3];
}
#endif

/****************************************************************************
 * Name: bch_cypher
 ****************************************************************************/

#if defined(CONFIG_BCH_ENCRYPTION)
static int bch_cypher(FAR struct bchlib_s *bch, int encrypt)
{
  int blocks = bch->sectsize / 16;
  FAR uint32_t *buffer = (FAR uint32_t *)bch->buffer;
  int i;

  for (i = 0; i < blocks; i++, buffer += 16 / sizeof(uint32_t) )
    {
      uint32_t T[4];
      uint32_t X[4] =
      {
        bch->sector, 0, 0, i
      };

      aes_cypher(X, X, 16, NULL, bch->key, CONFIG_BCH_ENCRYPTION_KEY_SIZE,
                 AES_MODE_ECB, CYPHER_ENCRYPT);

      /* Xor-Encrypt-Xor */

      bch_xor(T, X, buffer);
      aes_cypher(T, T, 16, NULL, bch->key, CONFIG_BCH_ENCRYPTION_KEY_SIZE,
                 AES_MODE_ECB, encrypt);
      bch_xor(buffer, X, T);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bchlib_flushsector
 *
 * Description:
 *   Flush the current contents of the sector buffer (if dirty)
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_flushsector(FAR struct bchlib_s *bch, bool discard)
{
  FAR struct inode *inode;
  ssize_t ret = OK;

  /* Check if the sector has been modified and is out of synch with the
   * media.
   */

  if (bch->dirty && bch->buffer != NULL)
    {
      inode = bch->inode;

#if defined(CONFIG_BCH_ENCRYPTION)
      /* Encrypt data as necessary */

      bch_cypher(bch, CYPHER_ENCRYPT);
#endif

      /* Write the sector to the media */

      ret = inode->u.i_bops->write(inode, bch->buffer, bch->sector, 1);
      if (ret < 0)
        {
          ferr("Write failed: %zd\n", ret);
          return (int)ret;
        }

#if defined(CONFIG_BCH_ENCRYPTION)
      /* Computation overhead to save memory for extra sector buffer
       * TODO: Add configuration switch for extra sector buffer
       */

      bch_cypher(bch, CYPHER_DECRYPT);
#endif

      /* The sector is now in sync with the media */

      bch->dirty = false;
    }

  if (discard)
    {
      bch->sector = (size_t)-1;
    }

  return (int)ret;
}

/****************************************************************************
 * Name: bchlib_readsector
 *
 * Description:
 *   Read the current sector contents into buffer
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_readsector(FAR struct bchlib_s *bch, size_t sector)
{
  FAR struct inode *inode;
  ssize_t ret = OK;

  if (bch->buffer == NULL)
    {
#if CONFIG_BCH_BUFFER_ALIGNMENT != 0
      bch->buffer = kmm_memalign(CONFIG_BCH_BUFFER_ALIGNMENT, bch->sectsize);
#else
      bch->buffer = kmm_malloc(bch->sectsize);
#endif
      if (bch->buffer == NULL)
        {
          ferr("Failed to allocate sector buffer\n");
          return -ENOMEM;
        }
    }

  if (bch->sector != sector)
    {
      inode = bch->inode;

      ret = bchlib_flushsector(bch, true);
      if (ret < 0)
        {
          ferr("Flush failed: %zd\n", ret);
          return (int)ret;
        }

#if defined(CONFIG_BCH_READAHEAD) && CONFIG_BCH_READAHEAD > 0
      /* Sequential read-ahead: when the requested sector is not already
       * in the window, refill the window starting at the requested sector
       * with one multiblock read (capped at the end of the device).  The
       * single-sector read below is kept as a fallback for block drivers
       * that cannot satisfy a large transfer.
       */

      if (bch->ran > 0 && sector >= bch->rastart &&
          sector <  bch->rastart + bch->ran)
        {
          /* Window hit: copy the sector out of the window. */

          size_t rasec = sector - bch->rastart;

          memcpy(bch->buffer, &bch->rabuffer[rasec * bch->sectsize],
                 bch->sectsize);
          bch->sector = sector;
#if defined(CONFIG_BCH_ENCRYPTION)
          bch_cypher(bch, CYPHER_DECRYPT);
#endif
          return OK;
        }
      else
        {
          size_t nreq = (size_t)CONFIG_BCH_READAHEAD;

          if (sector + nreq > bch->nsectors)
            {
              nreq = bch->nsectors - sector;
            }

          if (nreq > 1)
            {
              if (bch->rabuffer == NULL)
                {
                  bch->rabuffer = kmm_malloc((size_t)CONFIG_BCH_READAHEAD *
                                             bch->sectsize);
                }

              if (bch->rabuffer != NULL)
                {
                  ret = inode->u.i_bops->read(inode, bch->rabuffer, sector,
                                              nreq);
                  if (ret == (ssize_t)nreq)
                    {
                      bch->rastart = sector;
                      bch->ran     = nreq;

                      memcpy(bch->buffer, bch->rabuffer, bch->sectsize);
                      bch->sector = sector;
#if defined(CONFIG_BCH_ENCRYPTION)
                      bch_cypher(bch, CYPHER_DECRYPT);
#endif
                      return OK;
                    }

                  /* Fall back to the single sector read below (some
                   * block drivers cap the transfer size or reject
                   * large reads).
                   */
                }
            }
        }
#endif

      ret = inode->u.i_bops->read(inode, bch->buffer, sector, 1);
      if (ret < 0)
        {
          ferr("Read failed: %zd\n", ret);
          return (int)ret;
        }

      bch->sector = sector;
#if defined(CONFIG_BCH_ENCRYPTION)
      bch_cypher(bch, CYPHER_DECRYPT);
#endif
    }

  return (int)ret;
}

#if defined(CONFIG_BCH_READAHEAD) && CONFIG_BCH_READAHEAD > 0
/****************************************************************************
 * Name: bchlib_readahead
 *
 * Description:
 *   Ensure the read-ahead window covers [sector, sector + nsectors) and
 *   report whether it does.  Called from bchlib_read()'s full-sector path
 *   so that small aligned reads (e.g. dd with a small bs=) are served from
 *   one multiblock window fill instead of one single-sector read each.
 *
 * Returned Value:
 *   1  the range is covered by the window (copy from bch->rabuffer)
 *   0  the range cannot be covered (caller performs a direct read)
 *  <0 on error
 *
 * Assumptions:
 *   Caller must assume mutual exclusion
 *
 ****************************************************************************/

int bchlib_readahead(FAR struct bchlib_s *bch, size_t sector,
                     size_t nsectors)
{
  FAR struct inode *inode;
  size_t nreq;
  ssize_t ret;

  /* A range that is bigger than the window can never be covered. */

  if (nsectors >= (size_t)CONFIG_BCH_READAHEAD)
    {
      return 0;
    }

  /* Already covered? */

  if (bch->ran > 0 && sector >= bch->rastart &&
      sector + nsectors <= bch->rastart + bch->ran)
    {
      return 1;
    }

  /* Refill the window starting at the requested sector. */

  nreq = (size_t)CONFIG_BCH_READAHEAD;
  if (sector + nreq > bch->nsectors)
    {
      nreq = bch->nsectors - sector;
    }

  if (nreq <= nsectors)
    {
      return 0;  /* Device tail: window cannot cover more than the read */
    }

  if (bch->rabuffer == NULL)
    {
      bch->rabuffer = kmm_malloc((size_t)CONFIG_BCH_READAHEAD *
                                 bch->sectsize);
      if (bch->rabuffer == NULL)
        {
          return 0;  /* Fall back to the direct read */
        }
    }

  inode = bch->inode;
  ret = inode->u.i_bops->read(inode, bch->rabuffer, sector, nreq);
  if (ret != (ssize_t)nreq)
    {
      return 0;  /* Fall back to the direct read */
    }

  bch->rastart = sector;
  bch->ran     = nreq;
  return 1;
}
#endif
