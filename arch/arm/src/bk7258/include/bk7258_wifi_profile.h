/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_wifi_profile.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef BK7258_WIFI_PROFILE_H
#define BK7258_WIFI_PROFILE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/* Shared CP/AP wire and EasyFlash blob ABI. No native pointers or secrets
 * in command-line arguments. EasyFlash supplies the on-flash CRC.
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_WIFI_PROFILE_CMD 0x214u
#define BK7258_WIFI_PROFILE_VERSION 1u
#define BK7258_WIFI_PROFILE_GET 0u
#define BK7258_WIFI_PROFILE_SET 1u
#define BK7258_WIFI_PROFILE_CLEAR 2u
#define BK7258_WIFI_PROFILE_OK 0
#define BK7258_WIFI_PROFILE_MISSING 1
#define BK7258_WIFI_PROFILE_INVALID 2
#define BK7258_WIFI_PROFILE_IO 3

struct bk7258_wifi_profile
{
  uint8_t version;
  uint8_t ssid_length;
  uint8_t password_length;
  uint8_t reserved;
  uint8_t ssid[32];
  uint8_t password[64];
};

struct bk7258_wifi_profile_request
{
  uint32_t operation;
  struct bk7258_wifi_profile profile;
};

struct bk7258_wifi_profile_response
{
  int32_t status;
  struct bk7258_wifi_profile profile;
};

static inline int bk7258_wifi_profile_valid(
  const struct bk7258_wifi_profile *p)
{
  size_t i;
  if (p->version != BK7258_WIFI_PROFILE_VERSION || p->reserved != 0 ||
      p->ssid_length == 0 || p->ssid_length > sizeof(p->ssid) ||
      p->password_length < 8 || p->password_length >= sizeof(p->password))
    return 0;
  for (i = 0; i < sizeof(p->ssid); i++)
    if (i < p->ssid_length ? p->ssid[i] == 0 : p->ssid[i] != 0)
      return 0;
  for (i = 0; i < sizeof(p->password); i++)
    {
      unsigned char c = p->password[i];
      if (i >= p->password_length)
        {
          if (c != 0) return 0;
        }
      else if (c < 32 || c > 126)
        return 0;
    }
  return 1;
}

static_assert(sizeof(struct bk7258_wifi_profile) == 100, "profile ABI");
static_assert(sizeof(struct bk7258_wifi_profile_request) == 104,
               "profile request ABI");
static_assert(sizeof(struct bk7258_wifi_profile_response) == 104,
               "profile response ABI");

int bk7258_wifi_profile_command(unsigned int operation,
                               struct bk7258_wifi_profile *profile);
#endif
