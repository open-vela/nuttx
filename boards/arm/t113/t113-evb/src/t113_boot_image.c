/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_boot_image.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/boardctl.h>
#include <unistd.h>

#include <nuttx/arch.h>

#include "t113-evb.h"

#ifdef CONFIG_BOARDCTL_BOOT_IMAGE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOOT_CFG_BUFSIZE   128                  /* max cfg bytes (excl. NUL) */
#define IMAGE_MAX_SIZE     (16 * 1024 * 1024)   /* per-file read cap */
#define BOOT_MAX_LOADS     4                    /* AMP=2 / Linux=2 / headroom 2 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct boot_load_s
{
  uintptr_t       addr;
  FAR const char *path;
};

struct boot_plan_s
{
  struct boot_load_s loads[BOOT_MAX_LOADS];
  size_t             nloads;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Trim leading/trailing ASCII whitespace in place; returns the pointer
 * to the first non-whitespace char (may be the original NUL).
 */

static FAR char *trim(FAR char *s)
{
  FAR char *end;

  while (*s == ' ' || *s == '\t')
    {
      s++;
    }

  end = s + strlen(s);
  while (end > s && (*(end - 1) == ' ' || *(end - 1) == '\t' ||
                     *(end - 1) == '\r' || *(end - 1) == '\n'))
    {
      *--end = '\0';
    }

  return s;
}

/* Parse the cfg buffer.  Each non-comment, non-blank line is an
 *
 *     <addr_hex> <path>
 *
 * pair.  Hex base is taken from the first whitespace-separated token,
 * the rest of the line (after trim) is the path.  Returns 0 on
 * success, -EINVAL on malformed input or no loads at all.
 */

static int boot_parse_cfg(FAR char *buf, FAR struct boot_plan_s *plan)
{
  FAR char *line;
  FAR char *save;

  plan->nloads = 0;

  for (line = strtok_r(buf, "\n", &save); line != NULL;
       line = strtok_r(NULL, "\n", &save))
    {
      FAR char *sep;
      FAR char *path;
      FAR char *endptr;

      line = trim(line);
      if (line[0] == '\0' || line[0] == '#')
        {
          continue;
        }

      if (plan->nloads >= BOOT_MAX_LOADS)
        {
          return -EINVAL;
        }

      sep = strpbrk(line, " \t");
      if (sep == NULL)
        {
          return -EINVAL;       /* missing path */
        }

      *sep = '\0';
      path = trim(sep + 1);
      if (path[0] == '\0')
        {
          return -EINVAL;       /* path empty after trim */
        }

      /* Reject empty / non-hex addr.  strtoul silently returns 0 when
       * no hex digits are consumed, which would let "xyz /mnt/foo"
       * boot to physical address 0 (where boot0 itself lives).
       */

      plan->loads[plan->nloads].addr = strtoul(line, &endptr, 16);
      if (endptr == line)
        {
          return -EINVAL;       /* no hex digits */
        }

      plan->loads[plan->nloads].path = path;
      plan->nloads++;
    }

  return plan->nloads > 0 ? 0 : -EINVAL;
}

/* Open <path>, read up to IMAGE_MAX_SIZE bytes into <dst>, close.
 * Returns 0 on success, -EFBIG if the file is larger than the cap
 * (silent truncation would corrupt the image), or other negative
 * errno on driver-level error.
 */

static int load_file(FAR const char *path, uintptr_t dst)
{
  FAR uint8_t *buffer = (FAR uint8_t *)dst;
  size_t       total  = 0;
  ssize_t      nread;
  int          fd;
  int          ret = 0;

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      return -errno;
    }

  while (total < IMAGE_MAX_SIZE)
    {
      nread = read(fd, buffer + total, IMAGE_MAX_SIZE - total);
      if (nread <= 0)
        {
          if (nread < 0)
            {
              ret = -errno;
            }

          break;
        }

      total += (size_t)nread;
    }

  /* Hitting the cap means the file is at least IMAGE_MAX_SIZE;
   * refuse it rather than booting a possibly-truncated image.
   */

  if (ret == 0 && total == IMAGE_MAX_SIZE)
    {
      ret = -EFBIG;
    }

  /* A zero-byte file would jump into whatever was previously at the
   * load address; treat as a hard error.
   */

  if (ret == 0 && total == 0)
    {
      ret = -ENODATA;
    }

  close(fd);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_boot_image
 *
 * Description:
 *   Open <path> as a boot config file, parse one (addr, path) pair per
 *   line, load each file into DRAM at its addr via VFS, then jump.
 *
 *   Calling convention always follows ARM Linux DT:
 *     r0 = 0
 *     r1 = 0xFFFFFFFF
 *     r2 = loads[1].addr if a second image was given, else 0
 *   Entry = loads[0].addr.
 *
 *   boot0 stays image-agnostic: Linux reads r2 as DTB phys, NuttX
 *   ignores r2 (legacy raw), AMP master image ignores r2 (it owns CPU1
 *   release internally).  Three boot forms therefore all use the same
 *   cfg shape, just with different load counts and addresses.
 *
 *   On error, prints a syslog line and returns a negative errno.
 *   Each error message names the offending path and the relevant
 *   limit / errno so a user can act on the prompt without diving
 *   into the source.
 *
 ****************************************************************************/

int board_boot_image(FAR const char *path, uint32_t hdr_size)
{
  char buf[BOOT_CFG_BUFSIZE + 1];   /* +1 for NUL terminator */
  struct boot_plan_s plan;
  uintptr_t boot_arg;
  size_t  i;
  int     fd;
  ssize_t nread;
  int     ret;

  /* hdr_size is part of the boardctl(BOARDIOC_BOOT_IMAGE) ABI but the
   * cfg-driven loader doesn't carry an image header - silence -Wunused.
   */

  UNUSED(hdr_size);

  if (path == NULL)
    {
      syslog(LOG_ERR, "boot: empty path\n");
      return -EINVAL;
    }

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      ret = -errno;
      syslog(LOG_ERR, "boot: open cfg '%s' failed: %d\n", path, ret);
      return ret;
    }

  nread = read(fd, buf, BOOT_CFG_BUFSIZE);
  if (nread < 0)
    {
      ret = -errno;
      close(fd);
      syslog(LOG_ERR, "boot: read cfg '%s' failed: %d\n", path, ret);
      return ret;
    }

  /* If we filled the buffer to the cap, treat the cfg as oversize.
   * BOOT_CFG_BUFSIZE already covers BOOT_MAX_LOADS x ~30B with
   * headroom; hitting the cap means malformed input.
   */

  if (nread == BOOT_CFG_BUFSIZE)
    {
      close(fd);
      syslog(LOG_ERR, "boot: cfg '%s' exceeds %d bytes\n",
             path, BOOT_CFG_BUFSIZE);
      return -EFBIG;
    }

  close(fd);
  if (nread == 0)
    {
      syslog(LOG_ERR, "boot: cfg '%s' is empty\n", path);
      return -EIO;
    }

  buf[nread] = '\0';

  ret = boot_parse_cfg(buf, &plan);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "boot: cfg '%s' malformed (need '<addr_hex> <path>')\n",
             path);
      return ret;
    }

  for (i = 0; i < plan.nloads; i++)
    {
      FAR const char *img = plan.loads[i].path;
      uintptr_t       dst = plan.loads[i].addr;

      ret = load_file(img, dst);
      if (ret == -EFBIG)
        {
          syslog(LOG_ERR, "boot: image '%s' exceeds %d MB\n",
                 img, IMAGE_MAX_SIZE / (1024 * 1024));
          return ret;
        }
      else if (ret < 0)
        {
          syslog(LOG_ERR, "boot: load '%s' to 0x%lx failed: %d\n",
                 img, (unsigned long)dst, ret);
          return ret;
        }
    }

  boot_arg = plan.nloads > 1 ? plan.loads[1].addr : 0;
  t113_boot_jump(plan.loads[0].addr, boot_arg);

  /* Unreachable: t113_boot_jump is declared noreturn. */
}

#endif /* CONFIG_BOARDCTL_BOOT_IMAGE */
