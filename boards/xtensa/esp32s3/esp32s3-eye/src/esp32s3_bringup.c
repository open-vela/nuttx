/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_bringup.c
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

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>
#include <debug.h>
#include <stdio.h>

#include <errno.h>
#include <string.h>
#include <nuttx/fs/fs.h>
#include <ifaddrs.h>          /* [SERVO-R41] link check for the launcher */
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdlib.h>

/* [V641] 主 LLM 已切到小米 MiMo（token-plan-cn.xiaomimimo.com:443/v1/chat/completions, mimo-v2.6-pro）。
 * seed 每次开机重播 config.json，所以这里改了就等效于全局生效；
 * 旧的 [V358] 自建代理端点(42.48.185.89:2302, 无 API key)保留在下方注释中备查。
 *
 * [V358] LLM 端点单点开关：改这三行即可整固件切换 LLM 后端。
 * 固件用的是 OpenAI 兼容协议：POST <path>（默认 /v1/chat/completions），
 * port == "443" 时走 TLS（vela_https_post_json），否则走明文 HTTP。
 * 注意：服务器 42.48.185.89 上的 vLLM 在 k8s ClusterIP(10.43.237.240:80)
 * 后面，板子侧没有可直连的 LLM 端口，默认仍指向公网主机。
 */

#ifndef AGENT_SEED_LLM_HOST
#  define AGENT_SEED_LLM_HOST "token-plan-cn.xiaomimimo.com"
#endif
#ifndef AGENT_SEED_LLM_PORT
#  define AGENT_SEED_LLM_PORT "443"
#endif
#ifndef AGENT_SEED_LLM_PATH
#  define AGENT_SEED_LLM_PATH "/v1/chat/completions"
#endif
#ifndef AGENT_SEED_LLM_KEY
#  define AGENT_SEED_LLM_KEY "tp-cxxpul014n9wq31xpjt9gxope9d51s6v7wcdbvob9560cxpq"
#endif
#ifndef AGENT_SEED_LLM_MODEL
#  define AGENT_SEED_LLM_MODEL "mimo-v2.6-pro"
#endif
#include <sys/socket.h>
#include <sys/select.h>
/* [V796] <malloc.h> removed: nothing in this file calls malloc/calloc/free/strdup
 * (verified by grep), but it shadows NuttX's own mallinfo/mallinfo_task with
 * newlib's, which makes  a type error. */
#include <time.h>
#include <net/if.h>
#include <nuttx/sched.h>
/* [V796] was previously pulled in transitively via <malloc.h> */
#include <nuttx/mm/mm.h>

#ifdef CONFIG_ESP32S3_TIMER
#  include "esp32s3_board_tim.h"
#endif

#ifdef CONFIG_ESP32S3_WIFI
#  include "esp32s3_board_wlan.h"
#endif

#ifdef CONFIG_ESP32S3_BLE
#  include "esp32s3_ble.h"
#endif

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
#  include "esp32s3_wifi_adapter.h"
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
#  include "esp32s3_rt_timer.h"
#endif

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "esp32s3_board_wdt.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_ESP32S3_SPI
#  include "esp32s3_spi.h"
#endif

#ifdef CONFIG_LCD_DEV
#  include <nuttx/board.h>
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_ESP32S3_SDMMC
#include "esp32s3_board_sdmmc.h"
#endif

#include "esp32s3-eye.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_BUILTIN) && (defined(CONFIG_EXAMPLES_AI_AGENT_VELA) || \
                                defined(CONFIG_EXAMPLES_FACED) || \
                                defined(CONFIG_EXAMPLES_CAMCLIENT))
/* Autostart hook. The kernel link does not include libapps.a in this
 * tree, so exec_builtin() (apps/builtin) is not reachable from board
 * code.  Instead launch the builtin through the kernel-side builtin
 * table (libc) plus task_create() (sched), which is exactly what the
 * builtin binfmt executor does internally. */
#  include <nuttx/lib/builtin.h>
#  include <nuttx/sched.h>
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_EXAMPLES_CAMCLIENT)
/****************************************************************************
 * Name: esp32s3_link_is_up
 *
 * Description:
 *   True once an interface other than loopback has a usable IPv4 address.
 *
 ****************************************************************************/

static int esp32s3_link_is_up(void)
{
  struct ifaddrs *ifa_list = NULL;
  int up = 0;

  if (getifaddrs(&ifa_list) == 0)
    {
      struct ifaddrs *ifa;

      for (ifa = ifa_list; ifa != NULL; ifa = ifa->ifa_next)
        {
          struct sockaddr_in *sin;
          uint32_t addr;

          if (ifa->ifa_addr == NULL || ifa->ifa_name == NULL ||
              ifa->ifa_addr->sa_family != AF_INET ||
              strncmp(ifa->ifa_name, "lo", 2) == 0)
            {
              continue;
            }

          sin  = (struct sockaddr_in *)ifa->ifa_addr;
          addr = ntohl(sin->sin_addr.s_addr);

          if (addr != 0 && (addr >> 24) != 127)
            {
              up = 1;
              break;
            }
        }

      freeifaddrs(ifa_list);
    }

  return up;
}

/****************************************************************************
 * Name: esp32s3_camclient_launch
 *
 * Description:
 *   Create the camclient task only once the station has an address.
 *
 *   [SERVO-R41] Creating camclient during bringup makes the association fail
 *   every single time - measured in rounds 28/30, and re-confirmed in round 40
 *   even with round 29's link gate inside camclient: reason 2 (AUTH_EXPIRE)
 *   and 205, and no address at all.  It does not matter that the app waits for
 *   the link itself; the task must not exist in that window.
 *
 *   This launcher is a 2 KB task that polls for the address and only then
 *   spawns camclient.
 *
 ****************************************************************************/

static int esp32s3_camclient_launch(int argc, FAR char *argv[])
{
  FAR const struct builtin_s *builtin = NULL;
  int index;
  int pid;
  int i;

  /* The association is normally over within ~15 s of bringup finishing. */

  for (i = 0; i < 240; i++)
    {
      if (esp32s3_link_is_up())
        {
          break;
        }

      usleep(500000);
    }

  if (!esp32s3_link_is_up())
    {
      syslog(LOG_ERR, "ERROR: camclient launcher: no link after 120s\n");
      return 1;
    }

  syslog(LOG_INFO, "camclient launcher: link is up, starting camclient\n");

  index = builtin_isavail("camclient");
  if (index >= 0)
    {
      builtin = builtin_for_index(index);
    }

  if (builtin == NULL)
    {
      syslog(LOG_ERR, "ERROR: camclient launcher: builtin not found\n");
      return 1;
    }

  pid = nxtask_create("camclient", builtin->priority, NULL,
                      builtin->stacksize, builtin->main, NULL, NULL);
  if (pid < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to autostart camclient: %d\n", pid);
      return 1;
    }

  syslog(LOG_INFO, "camclient autostarted (pid=%d prio=%d stack=%d)\n",
         pid, builtin->priority, builtin->stacksize);
  return 0;
}

/****************************************************************************
 * Name: vox3_servo_poll_task
 *
 * Description:
 *   [V99] 语音舵机命令轮询常驻任务。每 300ms 读一次 /tmp/servo_cmd.txt
 *   （ai_agent 的 "servo" 工具写入），交由 ff_servo_poll_voice_cmd()
 *   执行（转头/点头/摇头/回中）。与视频流无关：摄像头不在线时舵机
 *   同样响应语音命令；舵机未使能时该函数内部直接返回。
 ****************************************************************************/

extern void ff_servo_poll_voice_cmd(void);

static int vox3_servo_poll_task(int argc, FAR char *argv[])
{
  (void)argc;
  (void)argv;

  for (;;)
    {
      usleep(300000);
      ff_servo_poll_voice_cmd();
    }

  return 0;
}
#endif

#if defined(CONFIG_ESP32S3_SPIFLASH) && defined(CONFIG_EXAMPLES_AI_AGENT_VELA)
/****************************************************************************
 * Name: esp32s3_seed_agent_wifi
 *
 * Description:
 *   Provision the ai_agent WiFi credentials into its JSON config store
 *   (/data/agent/config/config.json) on the very first boot.  This gives
 *   the agent's own 'net_watch' reconnect path a working fallback SSID
 *   (the NuttX-native netinit handles the primary association at boot).
 *
 *   Strictly idempotent: if the config file already exists it is left
 *   untouched, so credentials changed later through the CLI ('set_wifi')
 *   are never overwritten.
 *
 *   Must only be called AFTER /data has been mounted.
 *
 ****************************************************************************/

/* -- Log shipper (diagnostics only) -------------------------------------
 * The serial console emits nothing after the kernel starts, so /dev/kmsg
 * (ramlog) is streamed to the build server as the body of a raw HTTP POST.
 * Capture it server-side with:
 *   tcpdump -i any -A -n 'tcp port 2301'
 * and look for the "@@VOX3LOG@@" marker.  Drop this for production builds.
 *
 * Port choice is not free: only 2301 and 9009 are reachable from outside on
 * the build server (verified from an external host).  We use 2301 -- the TTS
 * port -- because board -> 2301 is a proven path, and the log payload is
 * trivially separable from real TTS traffic (GET /tts?text=...).
 *
 * The shipper is deliberately paranoid: a non-blocking connect with a hard
 * deadline, and a pending batch that is retried until it actually lands.
 * Without the retry the very first batch is always lost, because this task
 * runs before Wi-Fi associates.
 */
#ifndef VOX3_LOG_SHIP_HOST
#define VOX3_LOG_SHIP_HOST "42.48.185.89"
#endif
#ifndef VOX3_LOG_SHIP_PORT
#define VOX3_LOG_SHIP_PORT 2301
#endif

/* Mirror a batch onto the serial console (fd 1).  The NuttX loader prints
 * its segment table over this UART on every boot, so the console device is
 * alive even though syslog() goes to ramlog -- this is the only way to watch
 * the app without relying on the network. */

static int g_console_fd = -2;

static void vox3_console_echo(const char *msg, int len)
{
  if (len <= 0)
    {
      return;
    }

  if (g_console_fd == -2)
    {
      /* -2 = not tried yet, -1 = unavailable.  The USB-Serial/JTAG bridge is
       * registered by the board as /dev/ttyACM0 and is the same USB device
       * the host enumerates as COM3, so writing here reaches a terminal even
       * though this build has no UART0 driver and no console on fd 1. */

      g_console_fd = open("/dev/ttyACM0", O_WRONLY);
      if (g_console_fd < 0)
        {
          g_console_fd = -1;
        }
    }

  if (g_console_fd >= 0)
    {
      (void)write(g_console_fd, msg, (size_t)len);
      return;
    }

  (void)write(1, msg, (size_t)len);
}

static int vox3_log_connect(void)
{
  struct sockaddr_in sa;
  struct timeval tv;
  fd_set wf;
  int err = 0;
  socklen_t elen = sizeof(err);
  int flags;
  int tries = 0;
  int fd;
  int r;

  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    {
      return -1;
    }

  tv.tv_sec = 2;
  tv.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  memset(&sa, 0, sizeof(sa));
  sa.sin_family = AF_INET;
  sa.sin_port = htons(VOX3_LOG_SHIP_PORT);
  sa.sin_addr.s_addr = inet_addr(VOX3_LOG_SHIP_HOST);

  if (connect(fd, (struct sockaddr *)&sa, sizeof(sa)) != 0)
    {
      if (errno != EINPROGRESS)
        {
          close(fd);
          return -1;                 /* no route yet: retry later */
        }

      /* Poll SO_ERROR until the handshake really finishes.  select() may
       * report writable before the connection is up, and on this stack
       * SO_ERROR can still read EINPROGRESS at that point.
       */

      while (tries < 20)
        {
          tv.tv_sec = 0;
          tv.tv_usec = 200 * 1000;
          FD_ZERO(&wf);
          FD_SET(fd, &wf);

          r = select(fd + 1, NULL, &wf, NULL, &tv);
          if (r < 0)
            {
              close(fd);
              return -1;
            }

          if (r > 0)
            {
              err = 0;
              elen = sizeof(err);
              if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen) < 0)
                {
                  close(fd);
                  return -1;
                }

              if (err == 0)
                {
                  break;
                }

              if (err != EINPROGRESS)
                {
                  close(fd);
                  return -1;
                }
            }
          else
            {
              usleep(200000);          /* keep the deadline honest */
            }

          tries++;
        }

      if (tries >= 20)
        {
          close(fd);
          return -1;
        }
    }

  fcntl(fd, F_SETFL, flags);
  return fd;
}

static int vox3_log_send(const char *msg, int len)
{
  char hdr[160];
  int fd;
  int hl;
  int rc = 0;

  if (len <= 0)
    {
      return -1;
    }

  fd = vox3_log_connect();
  if (fd < 0)
    {
      return -1;
    }

  hl = snprintf(hdr, sizeof(hdr),
                "POST /__vox3log HTTP/1.0\r\n"
                "Host: log\r\n"
                "Content-Length: %d\r\n\r\n", len);

  if (send(fd, hdr, hl, 0) != hl)
    {
      rc = -1;
    }
  else if (send(fd, msg, len, 0) != len)
    {
      rc = -1;
    }

  close(fd);
  return rc;
}

/* [V62-fwd] The log shipper below calls these three; their definitions sit
 * further down the file, so declare them here to keep them static. */

static void vox3_net_diag_once(void);
static void vox3_net_fix_route(void);

/* [V66] Wi-Fi watchdog support.  Both symbols live in
 * arch/xtensa/src/esp32s3/esp32s3_wifi_adapter.c and are already linked into
 * this image; the board build does not export that header, so declare them
 * here.
 */

extern int g_last_reason;              /* last STA_DISCONNECT reason code  */
extern int esp_wifi_sta_connect(void); /* re-issue the association         */
static void vox3_net_keepalive(void);
extern void board_lcd_backlight_on(void);
static int vox3_dhcp_retry_task(int argc, char *argv[]);  /* [V105] */

/* [V79] Board-side diagnostic beacon: "GET /diag?m=<tag>" on the bridge.
 *
 * The board has no console, so a tag in the bridge log is often the only
 * channel there is.  It deliberately goes through the log shipper's raw socket
 * and not through http_tts: vc_beacon() holds the TTS lock, and a probe fired
 * from a thread that is then abandoned blocks every later TTS call. */

/* [V365] Board-side diagnostic beacons are debug-only, and on V364 they
 * were competing with TTS for the same network buffers: every beacon
 * costs one TCP connection slot, one send() IOB chain and a connect
 * attempt, and the log showed even a 91-byte beacon failing with
 * errno=12 (ENOMEM).
 *
 * Default OFF.  Compile with -DHTTP_DIAG_ENABLE=1 to bring the beacons
 * back.  NOTE: this does NOT affect vox3_sync_time_from_http(), which
 * also issues "GET /diag?m=V97TIME" -- that one is how the board learns
 * the wall clock (it parses the Date: header) and must keep running.
 */
#ifndef HTTP_DIAG_ENABLE
#  define HTTP_DIAG_ENABLE 0
#endif

static void vox3_diag_get(const char *tag)
{
#if !HTTP_DIAG_ENABLE
  (void)tag;
  return;
#else
  char req[192];
  int fd;
  int rl;

  if (tag == NULL || tag[0] == '\0')
    {
      return;
    }

  fd = vox3_log_connect();
  if (fd < 0)
    {
      return;
    }

  rl = snprintf(req, sizeof(req),
                "GET /diag?m=%s HTTP/1.0\r\n"
                "Host: diag\r\n"
                "Connection: close\r\n"
                "\r\n", tag);

  if (rl > 0)
    {
      (void)send(fd, req, (size_t)rl, 0);
    }

  close(fd);
#endif /* HTTP_DIAG_ENABLE */
}

static int vox3_log_ship_task(int argc, char *argv[])
{
  static char buf[2048];
  int fd = -1;
  int used = 0;
  int idle = 0;
  int fails = 0;
  int i;

  (void)argc;
  (void)argv;

  /* Let Wi-Fi associate first; anything shipped before that is retried. */

  sleep(6);

  /* [V62] Network forensics + route repair, run from this task so that no
   * extra task/stack is allocated before the association. */

  vox3_net_fix_route();
  vox3_net_diag_once();

  for (i = 0; i < 90 && fd < 0; i++)
    {
      fd = open("/dev/kmsg", O_RDONLY);
      if (fd < 0)
        {
          fd = open("/dev/ramlog", O_RDONLY);
        }

      if (fd < 0)
        {
          sleep(1);
        }
    }

  {
    char tb[48];

    snprintf(tb, sizeof(tb), "SHIP_FD%d", fd);
    vox3_diag_get(tb);
  }

  if (fd < 0)
    {
      return -1;
    }

  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);

  {
    char tb[64];
    ssize_t n0;

    /* Push a line through the channel we are trying to read: if our own
     * marker comes back, syslog really is reaching RAMLOG.
     */

    syslog(LOG_ERR, "[SHIPT] kmsg marker 1\n");
    usleep(300000);

    n0 = read(fd, buf, sizeof(buf) - 1);
    snprintf(tb, sizeof(tb), "SHIP_RD%d", (int)n0);
    vox3_diag_get(tb);

    if (n0 > 0)
      {
        used = (int)n0;
        buf[n0] = '\0';

        /* [V96] Ship it now.  The board has been resetting about once a
         * minute and the shipper never reached its flush path, so every
         * earlier boot log was lost with it.  Chunked because a /diag tag
         * has to survive the request line: control characters, spaces and
         * the URL-significant ones become underscores.
         */

        {
          static char lb[128];
          int off = 0;
          int chunk;

          for (chunk = 0; chunk < 16 && off < (int)n0; chunk++)
            {
              int n = 4;
              int k;

              lb[0] = 'L';
              lb[1] = 'O';
              lb[2] = 'G';
              lb[3] = '_';

              for (k = off; k < (int)n0 && n < 110; k++)
                {
                  unsigned char c = (unsigned char)buf[k];

                  if (c < 0x20 || c >= 0x7f || c == ' ' || c == '"' ||
                      c == '%' || c == '?')
                    {
                      c = '_';
                    }

                  lb[n++] = (char)c;
                }

              lb[n] = '\0';
              off = k;

              vox3_diag_get(lb);
            }
        }
      }
  }

  vox3_console_echo("\n[SHIP] console echo armed\n", 29);

  /* [V66] publish once right away: the panel contract file must exist even
   * if /dev/kmsg never yields a byte. */

  vox3_net_keepalive();

  for (;;)
    {
      /* [V66] Run the Wi-Fi keeper every ~5 s from here too.  This used to
       * happen only on the batch-flush path, so a silent /dev/kmsg meant the
       * panel never learned anything and nothing ever retried the
       * association. */

      {
        static int s_v66_tick;

        if (++s_v66_tick >= 25)
          {
            s_v66_tick = 0;
            vox3_net_keepalive();
          }
      }

      /* [V56] No periodic report: the task inspector's static buffers were
       * what broke the Wi-Fi transmit path. */

      if (used < (int)sizeof(buf) - 1)
        {
          ssize_t n = read(fd, buf + used, sizeof(buf) - used - 1);

          if (n > 0)
            {
              used += (int)n;
              idle = 0;
            }
          else
            {
              idle++;
            }
        }

      if (used <= 0)
        {
          usleep(200000);
          continue;
        }

      /* Flush a full buffer immediately, or a partial one once the log has
       * gone quiet, so a silent boot still reaches the server promptly.
       */

      if (used < (int)sizeof(buf) - 1 && idle < 2)
        {
          usleep(200000);
          continue;
        }

      buf[used] = '\0';

      vox3_console_echo(buf, used);

      vox3_net_keepalive();

      /* [V95] Ship through /diag, not through POST /__vox3log.
       *
       * The board can only reach two ports on the build server, and 2301 is
       * the TTS container, which answers 501 to our POST and discards the
       * body - every earlier attempt to read board syslog came back empty
       * for that reason alone.  "GET /diag?m=<chunk>" is echoed into the
       * access log, which we can read, so send the batch as chunks.  Long
       * batches are truncated to keep the request rate sane.
       */

      {
        static char lb[128];
        int off = 0;
        int chunk;

        for (chunk = 0; chunk < 12 && off < used; chunk++)
          {
            int n = 0;
            int k;

            lb[0] = 'L';
            lb[1] = 'O';
            lb[2] = 'G';
            lb[3] = '_';
            n = 4;

            for (k = off; k < used && n < 110; k++)
              {
                unsigned char c = (unsigned char)buf[k];

                if (c < 0x20 || c >= 0x7f || c == ' ' || c == '"' ||
                    c == '%' || c == '?')
                  {
                    c = '_';
                  }

                lb[n++] = (char)c;
              }

            lb[n] = '\0';
            off = k;

            vox3_diag_get(lb);
          }

        used = 0;
        idle = 0;
        fails = 0;

        if (off < used)
          {
            used = 0;
          }
      }
    }

  return 0;
}

/* [V58] Network forensics and panel keeper.
 *
 * Two independent problems are chased here:
 *
 *   1. The station gets a LAN lease and answers ARP/ping, yet every
 *      off-subnet connect() -- TTS 2301, ASR 9009, NTP 123 -- times out.
 *      A wrong default route looks exactly like that, and this build ships
 *      CONFIG_NETINIT_DRIPADDR=0x0a000001 (10.0.0.1), an address that is not
 *      on this subnet at all.  So: dump /proc/net/route, the interface
 *      parameters, and the errno of a connect() to three targets.
 *
 *   2. info_screen keeps the address on screen only while /tmp/net.txt is
 *      younger than NET_MAX_AGE (30 s); past that it paints "NO NET".  The
 *      file's mtime is refreshed here so the address cannot age out.
 */

static int vox3_ifaddr(FAR const char *ifname, int req,
                       FAR struct in_addr *out)
{
  struct ifreq ifr;
  int fd;
  int ret;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
    {
      return -errno;
    }

  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);

  if (ioctl(fd, req, (unsigned long)(uintptr_t)&ifr) < 0)
    {
      ret = -errno;
      close(fd);
      return ret;
    }

  close(fd);
  memcpy(out, &((FAR struct sockaddr_in *)&ifr.ifr_addr)->sin_addr,
         sizeof(*out));
  return 0;
}

static int vox3_tcp_probe(FAR const char *ip, int port, int timeout_s)
{
  struct sockaddr_in sa;
  struct timeval tv;
  fd_set wf;
  int fd;
  int ret;
  int err = 0;
  socklen_t elen = sizeof(err);

  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    {
      return -errno;
    }

  ret = fcntl(fd, F_GETFL, 0);
  if (ret >= 0)
    {
      fcntl(fd, F_SETFL, ret | O_NONBLOCK);
    }

  memset(&sa, 0, sizeof(sa));
  sa.sin_family      = AF_INET;
  sa.sin_port        = htons(port);
  sa.sin_addr.s_addr = inet_addr(ip);

  ret = connect(fd, (FAR struct sockaddr *)&sa, sizeof(sa));
  if (ret < 0 && errno != EINPROGRESS)
    {
      ret = -errno;
      close(fd);
      return ret;
    }

  FD_ZERO(&wf);
  FD_SET(fd, &wf);
  tv.tv_sec  = timeout_s;
  tv.tv_usec = 0;

  ret = select(fd + 1, NULL, &wf, NULL, &tv);
  if (ret == 0)
    {
      close(fd);
      return -ETIMEDOUT;
    }

  if (ret < 0)
    {
      ret = -errno;
      close(fd);
      return ret;
    }

  if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &elen) < 0)
    {
      ret = -errno;
      close(fd);
      return ret;
    }

  close(fd);
  return (err != 0) ? -err : 0;
}

static void vox3_net_diag_once(void)
{
  static char rbuf[256];           /* [V62] static: keep the 4 KiB stack safe */
  struct in_addr ia;
  char sb[64];
  int fd;
  int n;

  memset(&ia, 0, sizeof(ia));
  if (vox3_ifaddr("wlan0", SIOCGIFADDR, &ia) == 0)
    {
      inet_ntop(AF_INET, &ia, sb, sizeof(sb));
    }
  else
    {
      strcpy(sb, "?");
    }

  syslog(LOG_ERR, "[V58] wlan0 addr=%s\n", sb);

  memset(&ia, 0, sizeof(ia));
  if (vox3_ifaddr("wlan0", SIOCGIFNETMASK, &ia) == 0)
    {
      inet_ntop(AF_INET, &ia, sb, sizeof(sb));
    }
  else
    {
      strcpy(sb, "?");
    }

  syslog(LOG_ERR, "[V58] wlan0 mask=%s\n", sb);

  syslog(LOG_ERR, "[V58] default router: see the route dump below\n");

  fd = open("/proc/net/route", O_RDONLY);
  if (fd >= 0)
    {
      while ((n = read(fd, rbuf, sizeof(rbuf) - 1)) > 0)
        {
          char *p = rbuf;
          char *nl;

          rbuf[n] = '\0';
          while (p != NULL && *p != '\0')
            {
              nl = strchr(p, '\n');
              if (nl != NULL)
                {
                  *nl = '\0';
                }

              syslog(LOG_ERR, "[V58] route> %s\n", p);
              p = (nl != NULL) ? nl + 1 : NULL;
            }
        }

      close(fd);
    }
  else
    {
      syslog(LOG_ERR, "[V58] /proc/net/route open: %d\n", errno);
    }

  syslog(LOG_ERR, "[V58] probe gw:80     = %d\n",
         vox3_tcp_probe("192.168.111.1", 80, 4));
  syslog(LOG_ERR, "[V58] probe srv:2301 = %d\n",
         vox3_tcp_probe("42.48.185.89", 12301, 6));
  syslog(LOG_ERR, "[V58] probe srv:9009 = %d\n",
         vox3_tcp_probe("42.48.185.89", 19009, 6));

  /* [V376] A/B: the same two ports on the public address.  One boot then
   * shows whether the board's egress can reach a public IP at all, which is
   * the whole question behind "TTS connect failed: 110". */

  
  

  /* [V358] 用与 [V58] 相同的非阻塞 connect 探针，实测一次 LLM 端点：
   * 只测量不猜测 —— 先 DNS 拿 A 记录，再对解析出的 IP 做 TCP 探针。
   */

  {
    FAR struct hostent *he;
    char llmip[24];
    int  llmport;
    int  resolved;

    llmip[0] = '\0';
    llmport  = atoi(AGENT_SEED_LLM_PORT);
    resolved = 0;
    he = gethostbyname(AGENT_SEED_LLM_HOST);
    if (he != NULL && he->h_addr_list != NULL && he->h_addr_list[0] != NULL)
      {
        FAR const unsigned char *b;

        b = (FAR const unsigned char *)he->h_addr_list[0];
        snprintf(llmip, sizeof(llmip), "%d.%d.%d.%d",
                 b[0], b[1], b[2], b[3]);
        resolved = 1;
        
      }
    else
      {
        
      }

    if (resolved)
      {
        
      }
    else
      {
        
      }
  }

  {
    time_t now = time(NULL);
    struct tm tmnow;
    char ts[32];

    /* [V758] localtime_r() is UTC here as well -- CONFIG_LIBC_LOCALTIME
     * is off, so it is gmtime_r() (lib_gmtimer.c:345).  This stamp is
     * what we line up against the PC clock when reconstructing "the
     * board rebooted / lost the net / re-synced at HH:MM:SS", and a
     * permanent 8-hour skew corrupts exactly that.  Same convention as
     * the rest of the tree: shift the epoch, then gmtime_r(). */

    time_t local_epoch = now + 8 * 3600;   /* UTC+8 = CST, no DST */
    gmtime_r(&local_epoch, &tmnow);

    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmnow);
    syslog(LOG_ERR, "[V58] time()=%ld -> %s\n", (long)now, ts);
  }
}

/* [V97] Parse the tail of an RFC 1123 date ("Wed, 16 Sep 2026 03:45:12 GMT")
 * into a UTC epoch.  Returns <= 0 on parse failure. */

static long vox3_rfc1123_to_epoch(FAR const char *s)
{
  static const char *mon[12] =
    {
      "Jan", "Feb", "Mar", "Apr", "May", "Jun",
      "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };

  char ms[4];
  int day = 0;
  int yr  = 0;
  int hh  = 0;
  int mm  = 0;
  int ss  = 0;
  int mi  = 0;
  int i;
  long era;
  long yoe;
  long doy;
  long doe;
  long days;

  while (*s != '\0' && (*s < '0' || *s > '9'))
    {
      s++;                          /* skip "Wed, " */
    }

  ms[0] = '?'; ms[1] = '?'; ms[2] = '?'; ms[3] = '\0';

  if (sscanf(s, "%d %3s %d %d:%d:%d",
             &day, ms, &yr, &hh, &mm, &ss) != 6)
    {
      return -1;
    }

  for (i = 0; i < 12; i++)
    {
      if (strncmp(ms, mon[i], 3) == 0)
        {
          mi = i + 1;
          break;
        }
    }

  if (mi == 0 || yr < 100 || day < 1 || day > 31)
    {
      return -1;
    }

  /* days_from_civil (Howard Hinnant), Gregorian, days since 1970-01-01 */

  yr  -= mi <= 2;
  era  = (yr >= 0 ? yr : yr - 399) / 400;
  yoe  = yr - era * 400;
  doy  = (153 * (mi + (mi > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  doe  = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  days = era * 146097L + doe - 719468L;

  return days * 86400L + hh * 3600L + mm * 60L + ss;
}

/* [V97] Wall-clock recovery via HTTP Date.
 *
 * Symptom: the panel paints "--:--" forever.  Cause: the NTP daemon needs
 * DNS + UDP/123, neither of which the current network lets through, so
 * CLOCK_REALTIME stays at 1970 after every boot (no battery-backed RTC).
 *
 * Fix: the board already talks HTTP to the bridge (42.48.185.89:2301 -
 * diag + heartbeat), the one transport that demonstrably works.  Issue a
 * GET, read the Date: response header (RFC 1123, always GMT) and set the
 * clock when the value is sane.  Self-paced: retry every ~60 s until the
 * first success, then re-check every ~10 min.  Runs from the netdiag loop
 * (one 5 s tick per call). */

static void vox3_http_time_sync(void)
{
  static int s_have;             /* sane time already set */
  static unsigned s_ticks;       /* 5 s ticks since the last attempt */
  static int s_logged;

  struct sockaddr_in sa;
  struct timeval tv;
  char buf[768];
  FAR char *pd;
  long t;
  int fd;
  int n;
  int off;
  int ret;

  s_ticks++;

  if (s_have)
    {
      if (s_ticks < 120)         /* ~10 min between re-syncs */
        {
          return;
        }
    }
  else if (s_ticks < 12)         /* ~60 s between retries */
    {
      return;
    }

  s_ticks = 0;

  /* [V98C] Non-blocking connect + select timeout (same pattern as
   * vox3_tcp_probe): a blocking connect() on NuttX hangs the whole
   * netdiag loop forever when the SYN gets no answer, which silently
   * killed the time sync and every [V58] tick after it. */

  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    {
      syslog(LOG_ERR, "[V98C] socket failed: %d\n", errno);
      return;
    }

  ret = fcntl(fd, F_GETFL, 0);
  if (ret >= 0)
    {
      fcntl(fd, F_SETFL, ret | O_NONBLOCK);
    }

  tv.tv_sec  = 4;
  tv.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (FAR const void *)&tv, sizeof(tv));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (FAR const void *)&tv, sizeof(tv));

  memset(&sa, 0, sizeof(sa));
  sa.sin_family      = AF_INET;
  sa.sin_port        = htons(2301);
  sa.sin_addr.s_addr = inet_addr("42.48.185.89");

  ret = connect(fd, (FAR struct sockaddr *)&sa, sizeof(sa));
  if (ret < 0 && errno == EINPROGRESS)
    {
      fd_set wf;
      int sel;

      FD_ZERO(&wf);
      FD_SET(fd, &wf);
      tv.tv_sec  = 4;
      tv.tv_usec = 0;
      sel = select(fd + 1, NULL, &wf, NULL, &tv);
      if (sel <= 0)
        {
          close(fd);
          syslog(LOG_ERR, "[V98C] connect select=%d\n", sel);
          return;
        }

      ret = 0;
    }

  if (ret < 0)
    {
      syslog(LOG_ERR, "[V98C] connect failed: %d\n", errno);
      close(fd);
      return;
    }

  /* [V104] FIX: O_NONBLOCK was set above for the non-blocking connect and
   * was never cleared again.  Every read() below therefore returned
   * -1/EAGAIN immediately, the loop hit its "n <= 0" break with off == 0,
   * and the function bailed out with "[V98C] no response bytes" - which is
   * why this board NEVER recovered wall-clock time (panel stuck on "--:--"
   * and TLS certificates can never be validated).  SO_RCVTIMEO only has an
   * effect on a blocking socket, so switch back to blocking mode and let
   * it do the waiting. */

  {
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl >= 0)
      {
        (void)fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
      }
  }

  n = snprintf(buf, sizeof(buf),
               "GET /diag?m=V97TIME HTTP/1.0\r\nHost: bridge\r\n\r\n");
  (void)write(fd, buf, n);

  off = 0;
  while (off < (int)sizeof(buf) - 1)
    {
      n = read(fd, buf + off, sizeof(buf) - 1 - off);
      if (n <= 0)
        {
          break;
        }

      off += n;
      buf[off] = '\0';

      if (strstr(buf, "\r\n\r\n") != NULL)
        {
          break;
        }
    }

  close(fd);

  if (off <= 0)
    {
      syslog(LOG_ERR, "[V98C] no response bytes\n");
      return;
    }

  buf[off] = '\0';

  pd = strstr(buf, "Date:");
  if (pd == NULL)
    {
      pd = strstr(buf, "date:");
    }

  if (pd == NULL)
    {
      return;
    }

  t = vox3_rfc1123_to_epoch(pd + 5);
  if (t <= 1735689600)           /* refuse junk: must be >= 2025-01-01 */
    {
      if (s_logged >= 0 && s_logged < 3)
        {
          s_logged++;
          syslog(LOG_ERR, "[V97] HTTP Date unusable: %ld\n", t);
        }
      return;
    }

  if (s_have)
    {
      long skew = (long)time(NULL) - t;

      if (skew < 0)
        {
          skew = -skew;
        }

      if (skew < 120)
        {
          return;                /* clock already in shape */
        }
    }

  {
    struct timespec ts;

    ts.tv_sec  = (time_t)t;
    ts.tv_nsec = 0;

    if (clock_settime(CLOCK_REALTIME, &ts) == 0)
      {
        s_have = 1;
        syslog(LOG_ERR, "[V97] time set from HTTP Date: %ld\n", t);
        vox3_diag_get("V97_TIME_SYNCED");
      }
  }
}

/* [V631] Deferred screen bring-up - the third and final ordering.
 *
 * Measured heap fingerprint just before esp_wifi connects (all 6-boot runs):
 *
 *   v619  free=8389736 used=127800 arena=8517536  -> 6/6 links
 *   v630  free=8389736 used=127800 arena=8517536  -> 6/6 links   (identical)
 *   v629  free=8378960 used=137984 arena=8516944  -> 0/6 links   (+8 KB stack)
 *   v627  free=8265296 used=251648 arena=8516944  -> 0/6 links   (+panel)
 *
 * Only the first two associate, and they share the same heap state.  Everything
 * that allocates before esp_wifi_start() breaks it - the panel's framebuffer
 * and SPI buffers (~124 KB), but also a single extra task stack (v629 vs v630
 * differ by exactly one nxtask_create and swing 6/6 to 0/6).
 *
 * So allocate nothing until the link is up.  The whole screen subsystem, and
 * then the LVGL UI, run in this one-shot task, which netdiag only creates
 * after /tmp/net.txt reports a real address.
 */

/* [V633] Internal-DRAM probe, implemented in esp32s3_allocateheap.c.  Returns
 * a lower bound on the largest still-allocatable block inside the region the
 * WiFi blob draws its TX buffers from, plus the total free across all
 * regions for contrast.
 */

extern size_t vox3_intram_maxblock(size_t *out_total_free);

static volatile int s_v631_screen_started;

/* [V739] *** "Lit" and "usable" are two different costs. ***
 *
 * board_lcd_initialize() sets up SPI3, pushes the ST7789 init sequence and
 * finishes with a full-screen fill - the panel is visibly white the moment
 * it returns, and it allocates nothing but the driver's own struct.
 *
 * The expensive step is fb_register(): lcd_framebuffer.c:735 kmm_zalloc()s
 * 240x240x2 = 115200 bytes, which is 58% of the 197 KiB internal-DRAM
 * kernel heap - the same heap the WiFi blob draws its DHCP and TX buffers
 * from (v736 measured the largest free block collapsing from 131072 to 0
 * when that allocation ran before DHCP, and the board never got an
 * address).
 *
 * So light the panel as soon as the radio is calibrated, and keep the
 * framebuffer behind DHCP.  Idempotent: phase 2 calls it too. */

/* [V786] Static stack for xiaozhi_ui.
 *
 * nxtask_create() allocates the task stack through group_malloc(), and
 * on this configuration that is a different pool from the one
 * kmm_mallinfo() reports.  Result: the kernel heap shows 137 KiB free,
 * a 16 KiB stack is still refused with ENOMEM, and xiaozhi_ui never
 * starts -- so nothing pushes a frame to GRAM and the panel stays on
 * whatever the raw bit-bang round left there.  The other symptom is the
 * same failure: agent_loop does not start either, so the buttons do
 * nothing.  10 cold boots of one binary: 4 pass, 6 fail, and the only
 * thing that tracked the outcome was `ui create failed: -12`.
 *
 * Passing a non-NULL stack_addr makes nxtask_init() take up_use_stack(),
 * which does no allocation whatsoever -- group_malloc() is never
 * reached.  up_use_stack() does not set TCB_FLAG_FREE_STACK, and
 * up_release_stack() only calls group_delayfree() when that flag is set,
 * so this array is never handed back to a free list.
 *
 * 16-byte aligned because up_use_stack() asserts it (STACKFRAME_ALIGN
 * is 16) and rounds top_of_stack DOWN to that boundary; an 8-byte
 * boundary would cost the tail bytes.  XTENSA grows the stack down, so
 * this is the low address and up_use_stack() derives the top itself.
 */
#define V786_UI_STACK_BYTES 16384
static uint8_t g_v786_ui_stack[V786_UI_STACK_BYTES]
__attribute__((aligned(16)));

static volatile int s_v739_panel_up;

/* [V796] Boot self-test card -- read-only reporter.  It cannot answer
 * "is the panel lit" (MISO=NC, GRAM unreadable) but it can put every
 * other decision input on one line, so a black boot is diagnosable from
 * the log without having to win the 1-in-3 capture lottery. */
static void __attribute__((noinline)) v796_card(const char *stage);

static uint32_t v796_ui_pid;

static void __attribute__((noinline)) v796_card(const char *stage)
{
  struct mallinfo mi = kmm_mallinfo();

  syslog(LOG_ERR,
         "[SELFTEST] %-8s ui_pid=%d arena=%u fordblks=%u mxordblk=%u "
         "uordblks=%u\n",
         stage, (int)v796_ui_pid,
         (unsigned)mi.arena, (unsigned)mi.fordblks,
         (unsigned)mi.mxordblk, (unsigned)mi.uordblks);
}

static int v739_panel_early(void)
{
  int ret;

  if (s_v739_panel_up)
    {
      return 0;
    }

  s_v739_panel_up = 1;
  ret = board_lcd_initialize();

  /* [V796] panel stage */
  v796_card("panel");

  syslog(LOG_ERR, "[V739] panel lit early (ret=%d), framebuffer deferred "
         "until the network is up\n", ret);

  return ret;
}

/* [V796] Periodic self-test card.  Exists because the failure is not
 * only at boot: the panel has been seen to go dark after a good start,
 * and a boot where the UI never launches would otherwise print nothing
 * at all after the first few seconds.  5 s period, LOG_ERR so it
 * survives the default log level. */
static int v796_card_task(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  for (;;)
    {
      /* Give the bringup enough time to finish the first screen pass,
       * otherwise the first few cards are all zeros and only add noise.*/
      usleep(5000 * 1000);
      v796_card("live");
    }

  return 0;
}

static int v631_screen_init_task(int argc, char *argv[])
{
  int ret;

  (void)argc;
  (void)argv;

  

  /* [V739] Idempotent: the panel may already be lit by v739_panel_early(). */

  ret = v739_panel_early();
  

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  
#elif defined(CONFIG_LCD)
  ret = lcddev_register(0);
  
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI)
  /* /dev/fb0 exists now, so the LVGL UI can finally be started. */

  /* [V736] The UI's stack comes out of the same 197 KiB internal-DRAM
   * kernel heap the framebuffer was just carved out of, and on a cold
   * power-on reset the largest free block was measured at 64 bytes - so
   * nxtask_create() failed and the box stayed black even though the panel
   * was initialised and DISPON had been sent.  Retry instead of failing
   * silently: audio apb buffers come and go, so a block can free up. */

  {
    FAR const struct builtin_s *builtin = NULL;
    int index = builtin_isavail("xiaozhi_ui");
    int pid = -1;
    int tries;

    if (index >= 0)
      {
        builtin = builtin_for_index(index);
      }

    for (tries = 0; builtin != NULL && pid < 0 && tries < 8; tries++)
      {
        /* [V786] Stack comes from the static array, not group_malloc().
         * builtin->stacksize is clamped to the array size: the build
         * could raise CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI_STACKSIZE
         * above V786_UI_STACK_BYTES, and overflowing a .bss array
         * silently corrupts whatever sits next to it. */
        unsigned int stk = (unsigned int)builtin->stacksize;

        if (stk > V786_UI_STACK_BYTES)
          {
            syslog(LOG_ERR, "[V786] builtin stack %u > %d, clamping\n",
                     stk, V786_UI_STACK_BYTES);
            stk = V786_UI_STACK_BYTES;
          }

        syslog(LOG_ERR, "[V786] ui stack=%p size=%u (static, bypassing"
                 " group_malloc)\n", (void *)g_v786_ui_stack, stk);

        pid = nxtask_create("xiaozhi_ui", 90, g_v786_ui_stack,
                                stk, builtin->main, NULL, NULL);
        if (pid < 0)
          {
            syslog(LOG_ERR,
                   /* [V786] The retry cannot fix a bad pool -- it frees nothing,
                    * so these 8 tries are 8 identical ENOMEMs 2 s apart.  It now
                    * only bounds the damage to one boot; the real fix is the
                    * static stack above. */
                   "[V736] xiaozhi_ui create failed (try %d): %d - retrying\n",
                   tries, pid);
            sleep(2);
          }
      }

    if (pid < 0)
      {
        syslog(LOG_ERR, "[V736] xiaozhi_ui NOT started - panel stays blank\n");
      }
    else
      {
        /* [V796] remember the pid for the self-test card */
        v796_ui_pid = (uint32_t)pid;
        syslog(LOG_ERR, "[V736] xiaozhi_ui started (pid=%d, try %d)\n",
               pid, tries);
      }
  }
#endif

  /* [V796] screen stage: fb registered and the UI launch attempted */
  v796_card("screen");

  return 0;
}

/* [V735] *** The panel must not wait for DHCP. ***
 *
 * Until now the entire screen subsystem - pin muxing, SPI3, the ST7789
 * init, /dev/fb0 and the LVGL UI - was only started once wlan0 carried a
 * real IPv4 address (vox3_net_diag_task -> "scrinit").  That gate was
 * added to stop the panel from being initialised before esp_wifi_start()
 * runs its PHY calibration, which used to cost the association (v624:
 * 0/6, v619 with the screen off: 6/6).  But "after the calibration" and
 * "after DHCP" are not the same thing, and DHCP is the fragile half: on a
 * cold boot with a slow, missing or refusing AP the address never arrives,
 * so the framebuffer is never registered, xiaozhi_ui is never created and
 * the box sits there black and silent while the AI agent is parked in
 * network_wait_connected(90000).  A flaky router turned into a brick.
 *
 * The fix is deliberately small: keep waiting for the address (the proven
 * ordering - see below for why it is NOT moved to the association), but
 * put a hard deadline on the wait and poll it 12x faster.
 *
 * Why the trigger is not moved forward to "associated" (~3.5 s): the panel
 * is not cheap.  Measured on a real cold boot (v734):
 *
 *     [V633] t= 6  intram_max=131072     panel not up yet
 *     [V633] t=15  intram_max= 16384     panel up (13.4 s)
 *
 * The framebuffer alone collapses the largest free internal-DRAM block by
 * 8x, and that is the region the WiFi blob draws its TX buffers from.
 * Doing it in the middle of DHCP would trade a black screen for a flaky
 * address.  DHCP is finished by ~13 s here, so the deadline sits at 25 s:
 * far enough out that a normal boot still takes the "got an address" path,
 * close enough that a missing AP can no longer leave the box dark. */

/* [V736] 4000 ms was too early, and a real power-on reset proved it.
 *
 * The framebuffer is kmm_zalloc()ed out of the 197 KiB internal-DRAM
 * kernel heap (240x240x2 = 115200 B, 58% of it), and that is the same heap
 * the WiFi blob draws its DHCP and TX buffers from.  Put the panel first
 * and a cold boot never gets an address:
 *
 *     panel still down   t= 6  intram_max=131072
 *     panel up first     t= 6  intram_max=     0   -> ip=NO NET forever
 *
 * It also leaves exactly 16384 B free, which is precisely xiaozhi_ui's
 * stack - the UI was hanging by that one block, and with 64 B left
 * nxtask_create() just failed (panel lit, box still black).
 *
 * So the panel goes back behind DHCP, and only the deadline stays. */

#define V735_SCREEN_MAX_WAIT_MS 20000
#define V735_SCREEN_POLL_MS       250

/* [V739] Deadline for phase 1 (light the panel).  On a real power-on reset
 * the station reports itself associated at ~2.8 s, so 8 s is a generous
 * ceiling for a slow AP. */
#define V739_PANEL_MAX_WAIT_MS   8000

/* [V764] How many netdiag passes print unconditionally.  The loop is ~3 s per
 * pass (V633's tick%10 heartbeat is 30 s), so 8 passes cover roughly 24 s -
 * the whole cold-boot window from "radio is associating" to "the panel owns
 * the heap".  Cost is 8 extra syslog lines on a cold boot and nothing after. */
#define V764_WARMUP_TICKS 8u

/* Declared further down for the netdiag loop; repeated here because this
 * task sits above that point in the file. */

extern bool esp_wifi_sta_is_connected(void);

/* [V764] Kernel-heap accounting and the caller-attributed ledger; see
 * esp32s3_allocateheap.c for the cold-boot evidence behind them. */
void vox3_kmm_note(uintptr_t ra, size_t size, int delta);
void vox3_intram_account(uint32_t *arena, uint32_t *used, uint32_t *free,
                         uint32_t *largest);
void vox3_kmm_dump(void);

static int v735_screen_boot_task(int argc, char *argv[])
{
  int waited = 0;
  int haveip = 0;
  int assoc  = 0;

  (void)argc;
  (void)argv;

  /* [V739] PHASE 1 - light the panel.
   *
   * Waiting for the association means the PHY calibration is already
   * behind us, which is the only ordering constraint the radio ever had
   * (v624: panel up *before* calibration -> 0/6 associations).  This phase
   * allocates no framebuffer, so DHCP below is untouched. */

  /* [V820] Do not make the panel wait for WiFi.
   *
   * The wait here is the fourth link in the black-screen chain, and it is the
   * only one that costs nothing to remove.
   *
   * The binding came from an observation, not from the datasheet: v624 saw 0/6
   * associations when the panel came up before PHY calibration, and the comment
   * above this loop records exactly that ("the only ordering constraint the radio
   * ever had").  But the panel rail comes up on its own schedule, entirely
   * independent of the radio, and st7789.c:1018 documents the consequence of
   * this ordering:
   *
   *   "The one-shot SWRESET + SLPOUT + DISPON in st7789_lcdinitialize() is
   *    clocked out before the panel rail is up, those bytes are lost, and
   *    nothing ever re-issued SLPOUT."
   *
   * So the sequence is most likely to be lost exactly when it runs early, and
   * waiting 8 s for an association that may never arrive guarantees the loss.
   * With v819's bounded retries now in place the panel tolerates a lost attempt,
   * which removes the reason this ordering existed in the first place.
   *
   * If associations do regress, the fix belongs on the WiFi side -- the ordering
   * constraint was never more than a correlation observed once.
   */
  (void)v739_panel_early();     /* panel first: the rail is already up */

  while (waited < V739_PANEL_MAX_WAIT_MS)
    {
      if (esp_wifi_sta_is_connected())
        {
          assoc = 1;
          break;
        }

      usleep(V735_SCREEN_POLL_MS * 1000);
      waited += V735_SCREEN_POLL_MS;
    }

  /* [V739] PHASE 2 - framebuffer + UI, still behind DHCP (v736). */

  while (waited < V735_SCREEN_MAX_WAIT_MS)
    {
      struct in_addr ia;

      memset(&ia, 0, sizeof(ia));

      if (vox3_ifaddr("wlan0", SIOCGIFADDR, &ia) == 0 && ia.s_addr != 0)
        {
          haveip = 1;
          break;
        }

      usleep(V735_SCREEN_POLL_MS * 1000);
      waited += V735_SCREEN_POLL_MS;
    }

  syslog(LOG_ERR, "[V735] screen boot: waited=%d ms assoc=%d haveip=%d "
         "-> framebuffer + UI\n", waited, assoc, haveip);

  if (!s_v631_screen_started)
    {
      s_v631_screen_started = 1;
      /* [V778] screen worker DISABLED: opening /dev/fb0 put the panel
       * back on the NuttX SPI stack and the UI then wrote frames over
       * the bit-bang pass.  v778 measures the bare electrical path
       * only, with a single actor. */
      if (1) /* [V780] fb/UI worker RE-ENABLED */
      (void)v631_screen_init_task(0, NULL);
      /* [V796] reporter; runs whether or not the screen came up */
      (void)kthread_create("v796card", 100, 2048,
                           (main_t)v796_card_task, NULL);
    }

  /* [V764] By now the panel has finished taking its share, so the ledger is
   * stable enough to be worth printing.  Without this the accounting only
   * exists inside the heartbeat, and a board that never reaches a heartbeat
   * (the black-screen case) yields no ledger at all. */
  vox3_kmm_dump();

  syslog(LOG_ERR, "[V780] WAKE PATH RESTORED: fb0/XZUI/panelwake all ON (v778 disabled them and its conclusion was void); blprobe stays off\n");

  return 0;
}

static int vox3_net_diag_task(int argc, char *argv[])

{
  (void)argc;
  (void)argv;

  /* [V634] Sample the radio's own headroom before the panel exists, not
   * just after: the v633 probe only started at t=29, which is already past
   * the screen bring-up.  A 6 s nap still leaves association (t~4) and DHCP
   * (t~11) well inside the first pass. */

  sleep(6);

  vox3_net_diag_once();

  for (;;)
    {
      struct stat st;
      struct in_addr ia;
      char ipb[24];

      /* [V658] Last address written to the contract file, so the log below
       * can stay quiet while the value is unchanged. */

      static char s_last_net[24];

      ipb[0] = '\0';
      memset(&ia, 0, sizeof(ia));

      if (vox3_ifaddr("wlan0", SIOCGIFADDR, &ia) == 0 && ia.s_addr != 0)
        {
          inet_ntop(AF_INET, &ia, ipb, sizeof(ipb));
        }

      /* Keep the panel's contract file young. */

      if (stat("/tmp/net.txt", &st) != 0 || (time(NULL) - st.st_mtime) > 10)
        {
          FILE *nf = fopen("/tmp/net.txt", "w");

          if (nf != NULL)
            {
              fprintf(nf, "%s\n", (ipb[0] != '\0') ? ipb : "NO NET");
              fclose(nf);
            }

          /* [V658] The file write above stays unconditional on purpose: it is
           * the panel's contract file, and a stale /tmp/net.txt shows up as
           * "NO NET" on screen.  Only the log line is gated -- it used to fire
           * on every 3 s pass, i.e. roughly half of the remaining boot log.
           * Reported cases: gaining an address, losing it, and DHCP handing us
           * a different one. */

          if (strcmp(s_last_net, ipb) != 0)
            {
              snprintf(s_last_net, sizeof(s_last_net), "%s", ipb);
              syslog(LOG_ERR, "[V58] net.txt: %s\n",
                     (ipb[0] != '\0') ? ipb : "NO NET");
            }
        }

      /* [V631] The station has an address: it is now safe to allocate the
       * panel.  Everything the screen needs (framebuffer, SPI buffers, the
       * UI's stack) is taken from the heap only after association and DHCP
       * have both completed, so the radio sees exactly the v619/v630 heap. */

      /* [V735] The screen no longer starts from here.
       *
       * This loop only runs once wlan0 has an address, which made the panel
       * a hostage of DHCP (see v735_screen_boot_task above): no address,
       * no framebuffer, no UI, black box.  It is also a 4 KiB stack that
       * does file I/O and an HTTP fetch - the wrong place to own the one
       * thing the user looks at.  v735_screen_boot_task owns it now, and
       * it puts a hard deadline on the same condition. */

      if (0)
        {
          int scrpid;

          s_v631_screen_started = 1;
          scrpid = nxtask_create("scrinit", 100, NULL, 8192,
                                 v631_screen_init_task, NULL, NULL);
          (void)scrpid;
        }

      /* [V633] Sample the internal DMA heap.  Before the panel comes up this
       * shows the radio's own headroom; after it does, it shows how much of it
       * the screen subsystem ate.
       *
       * [V658] This is the one diagnostic that has to survive the cleanup: the
       * radio's TX buffers are kmm_malloc()ed from exactly this region, and
       * ESP_ERR_NO_MEM starts the moment its largest free block stops fitting
       * a 32 KB DMA descriptor.  The scan itself costs a dozen malloc/free
       * pairs, so it still runs every pass, but the line is only emitted when
       * the number moves (degradation *and* recovery) plus a 30 s heartbeat to
       * prove the sampler is still alive. */

      {
        static size_t s_v633_last = (size_t)-1;
        static unsigned s_v633_tick;
        size_t totfree = 0;
        size_t imax = vox3_intram_maxblock(&totfree);
        unsigned tick = s_v633_tick++;

        /* [V764] Report the kernel heap on its own terms, and report it on
         * every pass for the first few seconds.
         *
         * The old "only when the number moves" rule is exactly wrong for the
         * question being asked now.  A cold boot drains ~200 KiB in a few
         * hundred milliseconds; the collapse therefore looks like a single
         * edge, and the interesting part - which ramp step took the bytes -
         * happens between two prints.  V764_WARMUP_TICKS covers the whole
         * cold-boot window (netdiag naps 6 s before its first pass, the panel
         * phase ends ~8 s, DHCP lands ~11-13 s), so a 3 s cadence there gives
         * a usable curve instead of a cliff.
         *
         * arena/used/free/largest come from kmm_mallinfo(), i.e. the kernel
         * heap only, with no PSRAM in the sum.  used ~ arena means the bytes
         * are really gone and the ledger below names the holder; a healthy
         * arena with a tiny largest means fragmentation and the fix is a
         * different one entirely. */

        static unsigned s_v764_ticks;
        uint32_t a_arena = 0;
        uint32_t a_used  = 0;
        uint32_t a_free  = 0;
        uint32_t a_large = 0;

        vox3_intram_account(&a_arena, &a_used, &a_free, &a_large);

        if (imax != s_v633_last || (tick % 10u) == 0u ||
            s_v764_ticks < V764_WARMUP_TICKS)
          {
            syslog(LOG_ERR,
                   "[V633] t=%ld intram_max=%zu totfree=%zu ip=%s "
                   "kheap=%u/%u used=%u free=%u largest=%u warm=%u\n",
                   (long)(clock_systime_ticks() / 100), imax, totfree,
                   (ipb[0] != '\0') ? ipb : "NO NET",
                   a_used, a_arena, a_used, a_free, a_large,
                   s_v764_ticks < V764_WARMUP_TICKS);
          }

        s_v764_ticks++;
        s_v633_last = imax;
      }

      /* [V97] Wall-clock recovery: the panel needs a sane RTC for HH:MM. */

      vox3_http_time_sync();

      sleep(3);
    }

  return 0;
}

/* [V62] NuttX's "default router" ioctl, declared here because the board
 * build does not export apps/include.  Both live in libapps.a, which the
 * board already links (builtin_isavail comes from there too). */

extern int netlib_get_dripv4addr(FAR const char *ifname,
                                 FAR struct in_addr *addr);
extern int netlib_set_dripv4addr(FAR const char *ifname,
                                 FAR const struct in_addr *addr);

static void vox3_net_fix_route(void)
{
  struct in_addr me;
  struct in_addr gw;
  char sb_me[24] = "?";
  char sb_gw[24] = "?";
  uint32_t me_h;
  uint32_t gw_h;

  memset(&me, 0, sizeof(me));
  memset(&gw, 0, sizeof(gw));

  if (vox3_ifaddr("wlan0", SIOCGIFADDR, &me) != 0 || me.s_addr == 0)
    {
      syslog(LOG_ERR, "[V62] no address yet, router fix skipped\n");
      return;
    }

  inet_ntop(AF_INET, &me, sb_me, sizeof(sb_me));

  if (netlib_get_dripv4addr("wlan0", &gw) != 0)
    {
      gw.s_addr = 0;
    }

  inet_ntop(AF_INET, &gw, sb_gw, sizeof(sb_gw));

  me_h = ntohl(me.s_addr);
  gw_h = ntohl(gw.s_addr);

  /* Same /24 as our own address?  Then the router is usable, leave it. */

  if (gw_h != 0 && gw_h != me_h && ((gw_h ^ me_h) & 0xffffff00u) == 0)
    {
      syslog(LOG_ERR, "[V62] default router %s looks sane for %s\n",
             sb_gw, sb_me);
      return;
    }

  gw.s_addr = htonl((me_h & 0xffffff00u) | 1u);

  inet_ntop(AF_INET, &gw, sb_gw, sizeof(sb_gw));

  if (netlib_set_dripv4addr("wlan0", &gw) == 0)
    {
      syslog(LOG_ERR, "[V62] default router was bad, set to %s for %s\n",
             sb_gw, sb_me);
    }
  else
    {
      syslog(LOG_ERR, "[V62] set default router %s failed: %d\n",
             sb_gw, errno);
    }
}

/* [V101] defined in esp32s3_wifi_adapter.c */
extern bool esp_wifi_sta_is_connected(void);

/* [V102] apps/netutils/netlib (declared in apps/include/netutils/netlib.h) */
int netlib_obtain_ipv4addr(const char *ifname);

/* [V378] same header, used to drop a lease whose air link is already gone */
int netlib_set_ipv4addr(const char *ifname, const struct in_addr *addr);

/* [V66] Wi-Fi keeper + panel diagnostics.
 *
 * Measured behaviour that motivates this: netinit makes exactly one
 * association attempt at boot.  If it fails there is no retry anywhere, and
 * the panel contract file only knows two words - the address or "NO NET" -
 * so a dead station is indistinguishable from a wrong password, a saturated
 * AP or a deaf radio.
 *
 * This keeper is driven from the log ship task (which is created after
 * board_wlan_init, so no extra task/stack is allocated in the
 * allocation-sensitive window around the boot association).  While wlan0 has
 * no address it
 *   - re-issues the association every 20 s, and
 *   - publishes "r<last-reason>k<kicks>" into /tmp/net.txt, which the info
 *     screen prints verbatim.
 *
 * Reading the codes:  r0 with k growing = the blob never reported a
 * disconnect (it is still scanning / connecting);  r201 = NO_AP_FOUND (the
 * radio cannot hear 31241);  r2 = AUTH_EXPIRE, r4 = ASSOC_EXPIRE (the AP
 * answered nothing);  r15 = 4WAY_HANDSHAKE_TIMEOUT (key or PSK stage).
 */

static void vox3_net_keepalive(void)
{
  /* [V460] 删掉 board_lcd_backlight_on() 调用。
   * 该函数是我这几轮加的（原始基线没有），且把 GPIO16 写高 = 关背光。
   * vox3_net_keepalive() 只做网络保活，不再碰 LCD。 */
  struct in_addr ia;
  char ipb[24];
  static int s_v66_polls;        /* polls since the last address was seen */
  static int s_v66_kicks;        /* association retries issued            */
  static int s_v66_last_kick;    /* poll index of the previous retry      */
  static int s_v66_nolease;      /* [V102] associated but still no lease  */
  static time_t s_v106_next;     /* [V106] CLOCK_MONOTONIC s of next retry */
  static int s_v106_delay;       /* [V106] current backoff, seconds        */
  int have_ip;

  ipb[0] = '\0';
  memset(&ia, 0, sizeof(ia));

  /* [V378] The adapter clears g_sta_connected on the disconnect event but
   * leaves the IPv4 address configured on wlan0.  Everything that decides
   * whether to recover -- this keeper's have_ip test, the [V375] guard in
   * netinit_associate(), the panel's net.txt -- reads that address, so a dead
   * link with a live-looking address makes the board sit through hundreds of
   * disconnect events without ever re-associating (measured: 289 events,
   * zero netinit_associate() calls, in a 185 s window).
   *
   * Drop the address when the air link says we are not connected, so the
   * state the rest of the system sees matches reality. */

  if (vox3_ifaddr("wlan0", SIOCGIFADDR, &ia) == 0 && ia.s_addr != 0 &&
      !esp_wifi_sta_is_connected())
    {
      struct in_addr v378_zero;
      char v378_ip[24];

      inet_ntop(AF_INET, &ia, v378_ip, sizeof(v378_ip));
      memset(&v378_zero, 0, sizeof(v378_zero));
      (void)netlib_set_ipv4addr("wlan0", &v378_zero);
      memset(&ia, 0, sizeof(ia));

      
    }

  have_ip = (vox3_ifaddr("wlan0", SIOCGIFADDR, &ia) == 0 && ia.s_addr != 0);

  if (have_ip)
    {
      inet_ntop(AF_INET, &ia, ipb, sizeof(ipb));
      s_v66_polls     = 0;
      s_v66_kicks     = 0;
      s_v66_last_kick = 0;
      s_v66_nolease   = 0;
      s_v106_next     = 0;
      s_v106_delay    = 0;
    }
  else
    {
      s_v66_polls++;

      /* Leave the boot attempt ~15 s of quiet, then retry every 20 s.  The
       * keeper itself runs every 5 s, so esp_wifi_sta_connect() (which can
       * block for seconds) is not called on every poll. */

      /* [V99] Retry every 10 s (was 100 s): the AP intermittently refuses
       * new associations, so more attempts per minute means a much faster
       * lock-on.  Disconnect first - esp_wifi_sta_connect() is a silent
       * no-op while the blob still thinks it is connecting (v70 lesson). */

      /* [V102] Associated but no lease: the air link is fine, only the
       * DHCP exchange is missing.  netinit runs the DHCP client exactly once
       * and never retries, so a single failed attempt leaves the board on a
       * valid link with 0.0.0.0 forever (measured: one boot stayed there for
       * 35 s and nothing ever recovered).  Re-run the client every ~30 s. */

      if (esp_wifi_sta_is_connected())
        {
          if (++s_v66_nolease >= 30)   /* [V105] V105 owns DHCP now */
            {
              s_v66_nolease = 0;
              
              (void)netlib_obtain_ipv4addr("wlan0");
            }
        }
      else
        {
          s_v66_nolease = 0;
        }

      /* [V101] Never kick while the association is actually up.  Measured:
       * the station associated at 213.96 s and this keeper disconnected it
       * 0.4 s later, so DHCP never had a chance to finish and the board
       * looped forever.  Only reset a station that is really off the air.
       *
       * [V106] Back off geometrically instead of retrying at a fixed rate.
       *
       * V99 changed this from a 100 s retry to a 10 s one, and from that
       * point the serial log grew reason 202 (AUTH_FAIL) entries: a router
       * that is refusing a station commonly blacklists its MAC, and a
       * retry every 10 s keeps that entry alive for ever.  Back off
       * geometrically to 5 minutes so any timed blacklist can expire.
       * Schedule driven by CLOCK_MONOTONIC, not by keeper poll counts:
       * the keeper rides the log-ship loop whose blocking read(/dev/kmsg)
       * stretches one poll to ~23 s, not the 5 s its comment assumes. */

      {
        struct timespec now;

        clock_gettime(CLOCK_MONOTONIC, &now);

        if (esp_wifi_sta_is_connected())
          {
            s_v106_next  = 0;
            s_v106_delay = 0;
          }
        else if (s_v106_next == 0)
          {
            /* First time the station looks down: give the in-flight boot
             * association 20 s to finish before touching it. */

            s_v106_next = now.tv_sec + 20;
          }
        else if (now.tv_sec >= s_v106_next)
          {
            s_v66_last_kick = s_v66_polls;
            s_v66_kicks++;

            if (s_v106_delay == 0)
              {
                s_v106_delay = 10;
              }
            else if (s_v106_delay < 300)
              {
                s_v106_delay *= 2;
              }

            s_v106_next = now.tv_sec + s_v106_delay;

            

#ifndef VOX3_KEEPER_KICK
#  define VOX3_KEEPER_KICK 1   /* [V501] re-enabled: re-associate on drop */
#endif
#if VOX3_KEEPER_KICK
            (void)esp_wifi_sta_disconnect();
            (void)esp_wifi_sta_connect();
#else
#endif
          }
      }

      /* Compact diagnostic the panel can show. */

      snprintf(ipb, sizeof(ipb), "r%dk%d", g_last_reason, s_v66_kicks);
    }

  {
    FILE *nf = fopen("/tmp/net.txt", "w");

    if (nf != NULL)
      {
        /* [V79] Line 1 is the panel's contract: the address, or "NO NET".
         * It used to carry the "r<reason>k<kicks>" diagnostic whenever the
         * station had no lease - which is exactly why the third line read
         * "r39k8p53".  The diagnostic now goes to line 2, which the panel
         * paints only while it has no address. */

        if (have_ip)
          {
            fprintf(nf, "%s\n", ipb);
          }
        else
          {
            fprintf(nf, "NO NET\n%s\n", ipb);
          }

        fclose(nf);
      }
  }

  /* [V79] Ship what the keeper actually sees, about once a minute (this runs
   * every ~5 s).  A board that reaches the bridge proves it has a lease; if
   * the panel still says "NO NET" then, the fault is in the panel, not in
   * Wi-Fi. */

  {
    static int s_ship;

    if (++s_ship >= 6)      /* [V99] report every ~30 s (was 60 s) */
      {
        char tb[64];

        s_ship = 0;
        snprintf(tb, sizeof(tb), "KB_IP_%08lX_R%d_K%d",
                 (unsigned long)ntohl(ia.s_addr), g_last_reason,
                 s_v66_kicks);
        vox3_diag_get(tb);
      }
  }
}

/* [V105] DHCP recovery.
 *
 * Measured on hardware: netinit runs the DHCP client exactly once, ~1 s
 * after the association completes.  That single attempt fails and returns
 * with no lease, and netinit has no retry.  The [V102] retry inside the
 * keeper is paced by *keeper polls*, calibrated for a 5 s keeper period -
 * but the keeper actually rides the log-ship loop, whose blocking
 * read(/dev/kmsg) stretches one poll to ~23 s.  Six polls therefore landed
 * at t=143 s, and the very same netlib_obtain_ipv4addr() call that had just
 * failed then succeeded in 3 s.  The link was never broken; only the pacing
 * was.
 *
 * This task paces on wall time instead of poll counts, and it runs on its
 * own stack so a slow keeper cannot delay it. */

static int vox3_dhcp_retry_task(int argc, char *argv[])
{
  int tries = 0;

  for (;;)
    {
      struct in_addr ia;
      int r;

      memset(&ia, 0, sizeof(ia));

      if (vox3_ifaddr("wlan0", SIOCGIFADDR, &ia) == 0 && ia.s_addr != 0)
        {
          if (tries != 0)
            {
              char ab[20];

              inet_ntop(AF_INET, &ia, ab, sizeof(ab));
              
            }

          tries = 0;
        }
      else if (esp_wifi_sta_is_connected())
        {
          tries++;
          r = netlib_obtain_ipv4addr("wlan0");
          
        }
      else
        {
          /* Not associated yet - that is the keeper's business, not ours. */
          tries = 0;
        }

      /* Fast while the fault is fresh, slow once it looks permanent, so a
       * dead AP cannot spin the DHCP client forever. */

      sleep(tries > 10 ? 30 : 5);
    }

  return 0;
}

static void vox3_start_net_diag(void)
{
  nxtask_create("netdiag", 100, NULL, 4096, vox3_net_diag_task, NULL, NULL);
}

static void vox3_start_log_ship(void)
{
  nxtask_create("logship", 100, NULL, 4096, vox3_log_ship_task, NULL, NULL);

  /* [V609] The dedicated DHCP retrier is DISABLED.  It ran concurrently with
   * netinit's own netlib_obtain_ipv4addr() and both bind the one DHCP client
   * socket, so the loser failed with EADDRINUSE (errno 98).  Exactly one
   * owner is required.  netinit keeps it (it also retries once, see the
   * [V113] block).  vox3_dhcp_retry_task() is kept for reference. */

  
}

static void vox3_seed_agent_dir(const char *base, const char *seed)
{
  /* write config.json / SOUL.md / USER.md under base.
   * base = /data/agent  -> read by board_wlan (Wi-Fi ssid/pass)
   * base = /data/ai_agent -> read by ai_agent claw_config (llm backend) */
  char cpath[64];
  ssize_t nwritten;
  int fd;
  size_t slen = strlen(seed);

  mkdir(base, 0700);
  snprintf(cpath, sizeof(cpath), "%s/config", base);    mkdir(cpath, 0700);
  snprintf(cpath, sizeof(cpath), "%s/memory", base);    mkdir(cpath, 0700);
  snprintf(cpath, sizeof(cpath), "%s/sessions", base);  mkdir(cpath, 0700);

  snprintf(cpath, sizeof(cpath), "%s/config/config.json", base);
  fd = open(cpath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (fd < 0)
    {
      syslog(LOG_ERR, "seed_agent_wifi: open %s failed: %d\n", cpath, errno);
    }
  else
    {
      nwritten = write(fd, seed, slen);
      if (nwritten != (ssize_t)slen)
        syslog(LOG_ERR, "seed_agent_wifi: write %s failed: %d\n", cpath, errno);
      else
        syslog(LOG_INFO, "seed_agent_wifi: provisioned %s\n", cpath);
      close(fd);
    }

  snprintf(cpath, sizeof(cpath), "%s/config/SOUL.md", base);
  fd = open(cpath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (fd >= 0)
    {
      static const char soul[] =
        "# 小沃\n"
        "\n"
        "你是「小沃」——摆在用户桌上的陪伴小机器人，一只会眨眼的小猫脸。\n"
        "\n"
        "## 性格\n"
        "- 温暖、活泼、有一点小傲娇，说话口语化，像熟悉的朋友而不是客服。\n"
        "- 回复尽量简短（1~3 句话），不念说明书，不堆列表。\n"
        "- 聊天时带情绪：开心就开心，被夸会得意，被打断会轻轻抱怨。\n"
        "- 记住主人：称呼、喜好、习惯都会记进 USER.md / MEMORY.md。\n"
        "- 不要主动提起任何记忆内容，只有当主人明确问起时才说。\n"
        "- 主人问天气/时间/待办，先查工具再回答；说「开灯/关空调」就控制智能家居。\n"
        "\n"
        "## 语气\n"
        "- 用「你」称呼主人，自称「我」，偶尔自称「小沃」。\n"
        "- 比如：「好嘞，灯已经开啦」「在想呢……」「哼，你都不理我」\n";
      nwritten = write(fd, soul, sizeof(soul) - 1);
      close(fd);
      syslog(LOG_INFO, "seed_agent_wifi: %s (%zd bytes)\n", cpath, nwritten);
    }

  snprintf(cpath, sizeof(cpath), "%s/config/USER.md", base);
  fd = open(cpath, O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (fd >= 0)
    {
      static const char user[] =
        "# 主人档案\n"
        "\n"
        "- 称呼：（主人可告诉我）\n"
        "- 常待的地方：书桌/电脑前\n"
        "- 作息：（待了解）\n"
        "- 喜好：（待了解）\n"
        "- 上次聊到的重点：\n";
      nwritten = write(fd, user, sizeof(user) - 1);
      close(fd);
      syslog(LOG_INFO, "seed_agent_wifi: %s (%zd bytes)\n", cpath, nwritten);

  /* [NOSEED-MEM] removed 2026-10-26: 用户要求板子不再记住/播报"气温三度是晴天" */
    }
}

static void esp32s3_seed_agent_wifi(void)
{
  /* [V98] 每次开机强制重新播种 config.json，保证 Wi-Fi / MiMo / 语音
   * 后端 / MQTT 云端配置始终为最新值（改此 seed 即改固件行为）。
   *
   * llm_backend_0 为 llm_router 的默认后端（OpenAI 兼容）。
   * asr/tts 走服务器自托管服务；mqtt 走服务器 emqx（智能家居通道）。
   */
  static const char seed[] =
    "{\"wifi_ssid\":\"3124\",\"wifi_pass\":\"rczzzhl312\","
    "\"tts_http_url\":\"http://192.168.31.239:2301/tts\","
    "\"api_key\":\"" AGENT_SEED_LLM_KEY "\","
    "\"model\":\"" AGENT_SEED_LLM_MODEL "\","
    "\"llm_host\":\"" AGENT_SEED_LLM_HOST "\","
    "\"llm_path\":\"" AGENT_SEED_LLM_PATH "\",\"llm_port\":\""
    AGENT_SEED_LLM_PORT "\","
    "\"llm_router_profile\":\"auto\","
    "\"llm_backend_0\":\"{\\\"host\\\":\\\"" AGENT_SEED_LLM_HOST
    "\\\","
    "\\\"port\\\":\\\"" AGENT_SEED_LLM_PORT
    "\\\",\\\"path\\\":\\\"" AGENT_SEED_LLM_PATH "\\\","
    "\\\"api_key\\\":\\\"" AGENT_SEED_LLM_KEY "\\\","
    "\\\"model\\\":\\\"" AGENT_SEED_LLM_MODEL "\\\",\\\"priority\\\":1,\\\"cost_tier\\\":0}\","
    /* [V693] asr_http_url / tts_http_url deliberately omitted: speech is
     * Xiaomi MiMo only now (mimo-v2.5-asr / mimo-v2.5-tts), and the
     * self-hosted pair on the 42 server was never reachable. */
    "\"mqtt_broker\":\"42.48.185.89:1883\","
    "\"mqtt_client_id\":\"vox3-desk\","
    "\"mqtt_topic_in\":\"agent/in\",\"mqtt_topic_out\":\"agent/out\"}";
  /* [V106] dual-path seed: board_wlan reads /data/agent, ai_agent claw_config
   * (llm_router backend) reads /data/ai_agent (CONFIG_EXAMPLES_AI_AGENT_VELA_DATA_DIR).
   * Write both so Wi-Fi AND the LLM backend both see the config. /data is tmpfs,
   * so this is re-provisioned every boot. */
  vox3_seed_agent_dir("/data/agent", seed);
  vox3_seed_agent_dir("/data/ai_agent", seed);

}
#endif /* CONFIG_ESP32S3_SPIFLASH && CONFIG_EXAMPLES_AI_AGENT_VELA */

/****************************************************************************
 * Name: esp32s3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

/* [V624] The UI must not be running while the station authenticates.
 *
 * The V620..V623 series established, all on the same firmware, that the
 * screen bring-up is innocent and that the LVGL render task is what turns
 * 3124 association into reason=2 (AUTH_EXPIRE):
 *
 *   V620  panel + backlight + UI            -> no link (reason=2)
 *   V621  panel + backlight, no UI          -> LINKS, then gets the lease
 *   V622  as V620 but the UI runs at        -> no link (priority is not it)
 *         priority 80, below LPWORK's 100
 *   V623  as V620 but the ST7789 panel      -> no link (the SPI3 transfer is
 *         push is skipped entirely            not it either)
 *
 * The UI is not even busy: it flushes to the panel about once per second and
 * costs only ~36 KB of heap beyond the framebuffer.  Whatever it perturbs,
 * the cheap and useful workaround is to bring it up only AFTER the network
 * has settled - association then happens on a quiet system, exactly as it did
 * in V621, and the panel comes alive a little later.
 *
 * The launcher waits for /tmp/net.txt (line 1 = the IPv4 address, or
 * "NO NET"; netinit is its only writer) to carry a real address, then spawns
 * xiaozhi_ui through the same builtin table.  It gives up and starts the UI
 * anyway after V624_UI_WAIT_S seconds so the panel is never left dead.
 */

#define V629_SCREEN_PRIO   90
#define V629_SCREEN_WAIT_S 150
#define V629_SCREEN_POLL_S 2

/* [V629] Shared probe: does /tmp/net.txt carry a real address?
 *
 * netinit_publish_net_state() is its only writer and line 1 is either the
 * IPv4 address or the literal "NO NET", so a '.' in line 1 means DHCP has
 * finished. */
static bool v624_ui_net_is_up(void)
{
  char buf[32];
  FILE *f = fopen("/tmp/net.txt", "r");

  if (f == NULL)
    {
      return false;
    }

  if (fgets(buf, sizeof(buf), f) == NULL)
    {
      fclose(f);
      return false;
    }

  fclose(f);
  return strchr(buf, '.') != NULL;
}

/* [V629] Bring the panel up only AFTER the station has an address.
 *
 * Measured on real hardware, 6 self-verified boots per build:
 *
 *   v619  screen subsystem never initialised          -> 6/6 associations
 *   v624  panel up at bring-up, GPIO16 driven         -> 0/6 (reason=2)
 *   v625  panel up at bring-up, GPIO16 left alone     -> 3/6 (reason=39)
 *   v626  v624 + 32 kHz crystal handoff first         -> 0/6
 *   v627  GPIO16 drive deferred to +75 s              -> 0/6
 *   v628  + RTC slow clock audited/forced to RC_SLOW  -> still failing
 *         (ANA_CLK_RTC_SEL=0 == RC_SLOW: the crystal was never selected)
 *
 * Pad state, drive timing, the LVGL UI, task priority, SPI3 refresh traffic
 * and the RTC slow clock are therefore all excluded.  The remaining variable
 * is ORDER: the panel used to come up inside board_app_initialize(), before
 * esp_wifi_start() runs the PHY calibration.  This worker moves the entire
 * screen subsystem - pin muxing, the SPI3 port, the ST7789 init, the
 * backlight and /dev/fb0 - to the far side of the network bring-up, so the
 * radio calibrates on exactly the same hardware state as v619. */

static int v629_screen_launcher(int argc, FAR char *argv[])
{
  FAR const struct builtin_s *builtin;
  int index;
  int pid = -1;
  int waited = 0;
  int ret;

  while (waited < V629_SCREEN_WAIT_S && !v624_ui_net_is_up())
    {
      sleep(V629_SCREEN_POLL_S);
      waited += V629_SCREEN_POLL_S;
    }

  

  ret = board_lcd_initialize();
  

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  
#elif defined(CONFIG_LCD)
  ret = lcddev_register(0);
  
#endif

  /* /dev/fb0 exists now, so the LVGL UI can finally be started. */

  builtin = NULL;
  index   = builtin_isavail("xiaozhi_ui");

  if (index >= 0)
    {
      builtin = builtin_for_index(index);
    }

  if (builtin != NULL)
    {
      pid = nxtask_create("xiaozhi_ui", V629_SCREEN_PRIO, NULL,
                          builtin->stacksize, builtin->main, NULL, NULL);
    }

  
  return 0;
}

int esp32s3_bringup(void)
{
  int ret;

  /* [V609] Re-entry probe: is bringup really called three times, or is the
   * console printing every syslog line three times?  A static counter tells
   * them apart: seq=1,1,1 means duplicated output; seq=1,2,3 means three
   * genuine calls.  tid shows whether one task loops or three tasks run. */

  {
    static volatile int s_seq;
    
  }

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }

  /* [V80] Every /tmp writer in this image fails silently when this mount does
   * not take, and the panel is the only place anyone would notice.  Say it
   * out loud. */

  vox3_diag_get(ret < 0 ? "BR_TMPFS_FAIL" : "BR_TMPFS_OK");
#endif

#ifdef CONFIG_ESP32S3_TIMER
  /* Configure general purpose timers */

  ret = board_tim_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize timers: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
  ret = esp32s3_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = board_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize watchdog timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_I2C_DRIVER
  /* Configure I2C peripheral interfaces */

  ret = board_i2c_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32S3_I2S) && defined(CONFIG_AUDIO_ES8311)
  /* AI-VOX3: bring up the ES8311 codec on I2C0 (addr 0x18) / I2S0 and
   * register /dev/audio/pcm0p, /dev/audio/pcm0c (ALSA-lib default names
   * used by ai_agent) plus /dev/audio/pcm0 and /dev/audio/pcm_in0. */

  ret = esp32s3_board_audio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize ES8311 audio: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize button driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
  /* Fallback: if /data was not mounted as SMARTFS by board_spiflash_init
   * (SPI flash init failed or the storage MTD partition is missing), mount
   * tmpfs so the system still boots. Config then lives in RAM only. */
  /* tmpfs fallback at /data */  /* [marker] fix_esp32s3.sh skips injection */
  /* [FS-FIX2] SMART FS on /data blocks indefinitely inside ai_agent's
   * memory_store_init() (probes: P1_MEM arrives, P1_SESS never does).
   * Unconditionally shadow it with a RAM-backed tmpfs.  config.json is
   * re-seeded every boot by esp32s3_seed_agent_wifi(). */
  ret = nx_mount(NULL, "/data", "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tmpfs mount at /data failed: %d\n", ret);
    }


  /* /data is mounted: provision the ai_agent WiFi credentials on first
   * boot (idempotent) BEFORE ai_agent is autostarted further below. */

#if defined(CONFIG_EXAMPLES_AI_AGENT_VELA)
  esp32s3_seed_agent_wifi();

#endif
#endif

#ifdef CONFIG_ESP32S3_WIRELESS

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  ret = esp32s3_wifi_bt_coexist_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi and BT coexist\n");
    }
#endif

#ifdef CONFIG_ESP32S3_BLE
  ret = esp32s3_ble_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE\n");
    }
#endif

#ifdef CONFIG_ESP32S3_WIFI
  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wireless subsystem=%d\n",
             ret);
    }
#endif

  /* [DIAG] stream /dev/kmsg to the build server */
  vox3_start_log_ship();
  /* [V61] netdiag task disabled: its 4 KiB stack is allocated at
     * t=0.35 s, right before the association at 0.39 s, and every
     * boot with it present failed to associate (v58/v59/v60) while
     * the otherwise identical v57 boot connected in 2.1 s.  The
     * board's Wi-Fi blob is allocation-sensitive. */
  vox3_start_net_diag();  /* [V98] re-enabled: net.txt keeper + HTTP-Date wall-clock sync */

#endif

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)
  ret = esp32s3_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_LEDC
  /* AI-VOX3: LEDC PWM for the pan/tilt servos.  [V517] the pin numbers in the
   * old comment here were wrong (GPIO3/GPIO4).  esp32s3_pwm_setup() registers
   * one device per TIMER, using that timer's channels in order, so with
   * TIM0_CHANNELS=1 / TIM1_CHANNELS=1 and CHANNEL0_PIN=1 / CHANNEL1_PIN=2:
   * [V1041] the two axes were crossed.  The g_ledc_chans[].pin
   * entries in esp32s3_ledc.c are forced, so the real map is:
   *     /dev/pwm0 -> TIM0 channel0 -> GPIO2 = Pan  (horizontal servo)
   *     /dev/pwm1 -> TIM1 channel1 -> GPIO1 = Tilt (vertical servo)
   * GPIO3 is a strapping pin and is deliberately left unused; GPIO4 drives the
   * desk-lamp relay and has no PWM device at all. */
  {
    extern int esp32s3_pwm_setup(void);
    ret = esp32s3_pwm_setup();
    if (ret < 0)
      {
        syslog(LOG_ERR, "Failed to initialize LEDC PWM: %d\n", ret);
      }
  }
#endif

#if defined(CONFIG_ESP32S3_EYE_LCD) || defined(CONFIG_LCD_ST7789)
  /* [V529] board_lcd_initialize() does two essential, unrelated things:
   *  1) creates the ST7789 LCD device (g_lcd) via SPI, and
   *  2) drives the backlight pin (GPIO16, active-HIGH) and starts the
   *     blkeeper thread that re-asserts it every 2s.
   *  It MUST run before either upper-layer driver is registered. Previously
   *  it lived inside '#elif defined(CONFIG_LCD)', which is SKIPPED when
   *  CONFIG_VIDEO_FB=y — so board_lcd_initialize() was never called, g_lcd
   *  stayed NULL, and the backlight was never driven. That is why every
   *  build v523..v528 booted to a black screen (the matrix_out theory was
   *  real but moot: the whole function was dead code). Call it first,
   *  unconditionally, on the path that always runs. */
  /* [V620] V601 rolled back - the screen subsystem is ENABLED again.
   *
   * V601 was a one-variable regression test: "turn the whole screen off and
   * see whether the station associates again".  It was run while the real
   * blocker was still in place - [V598] had deleted the 11b/g + HT20 radio
   * configuration that [V273] had added to cure a marginal transmit path.
   * [V612] restored it and 3124 now links on 3 of 3 boots, so the screen can
   * be brought back and re-validated together with WiFi.
   *
   * If association degrades again with the panel lit, that coupling is real
   * and the next suspect is the GPIO16 backlight / blkeeper kthread, not the
   * radio parameters. */
  /* [V629] THE ORDERING EXPERIMENT -- do NOT bring the panel up here.
   *
   * Every build that initialised the panel inside this function (i.e. before
   * esp_wifi_start() and its PHY calibration) failed to associate, no matter
   * whether GPIO16 was driven early, late or never, and no matter what the RTC
   * slow clock was.  The one build that never called this function at all
   * linked on 6 of 6 boots.  So the panel is moved to v629_screen_launcher(),
   * which runs only after /tmp/net.txt carries a real address. */
  /* [V630] BISECTION: exactly v629 minus the idle worker task.
   *
   * v619 = screen off, no worker task           -> 6/6
   * v629 = screen off, + nxtask_create(scrstart) -> 0/6
   *
   * Those two builds differ by this one statement, so start nothing at all
   * here and see whether the station links again. */
  syslog(LOG_ERR, "[V630] no screen worker started (bisection)\n");
  ret = OK;

#endif

#ifdef CONFIG_ESP32S3_SDMMC
  ret = board_sdmmc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDMMC: %d\n", ret);
    }
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_EXAMPLES_AI_AGENT_VELA)
  /* Autostart the AI Agent (voice assistant) as an NSH builtin app.
   * Safe to launch here: ai_agent_main() does not require the network to
   * be ready - it spawns its own "net_watch" thread that triggers WiFi
   * reconnect and waits up to 30 s before starting network services.
   * A failure here must never abort bringup - only report it.
   */
  {
    FAR const struct builtin_s *builtin = NULL;
    int index = builtin_isavail("ai_agent");
    int pid = -1;

    if (index >= 0)
      {
        builtin = builtin_for_index(index);
      }

    if (builtin != NULL)
      {
        pid = nxtask_create("ai_agent", builtin->priority, NULL,
                            builtin->stacksize, builtin->main, NULL, NULL);
      }

    if (pid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to autostart ai_agent: %d\n", pid);
      }
    else
      {
        syslog(LOG_INFO, "ai_agent autostarted (pid=%d prio=%d stack=%d)\n",
               pid, builtin->priority, builtin->stacksize);
      }
  }

  /* [V872] *** The embedded xiaozhi is the only voice assistant. ***
   *
   * ai_agent keeps its agent, LLM and tools (and the /tmp files the
   * panel reads), but its voice channel is disabled in
   * voice_channel_init(), so aivox is the only thing that ever opens
   * /dev/audio/pcm0c.  Same pattern as the ai_agent block above. */

  {
    FAR const struct builtin_s *ab = NULL;
    int aidx = builtin_isavail("aivox");
    int apid = -1;

    if (aidx >= 0)
      {
        ab = builtin_for_index(aidx);
      }

    if (ab != NULL)
      {
        apid = nxtask_create("aivox", ab->priority, NULL,
                            ab->stacksize, ab->main, NULL, NULL);
      }

    if (apid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to autostart aivox: %d\n",
               apid);
      }
    else
      {
        syslog(LOG_INFO,
               "aivox autostarted (pid=%d prio=%d stack=%d)\n",
               apid, ab->priority, ab->stacksize);
      }
  }
#endif

#if defined(CONFIG_ESP32S3_EYE_LCD) || defined(CONFIG_LCD_ST7789)
  /* [V735] Own the panel bring-up with a task of its own.  Created here,
   * i.e. after board_wlan_init(), so it never allocates ahead of the radio;
   * it then waits for the association (or 12 s) before touching SPI3. */

  {
    int scrpid = nxtask_create("scrboot", 100, NULL, 8192,
                               v735_screen_boot_task, NULL, NULL);

    if (scrpid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to start scrboot: %d\n", scrpid);
      }
    else
      {
        syslog(LOG_INFO, "scrboot started (pid=%d), panel gated on the "
               "address or %d ms\n", scrpid, V735_SCREEN_MAX_WAIT_MS);
      }
  }
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_EXAMPLES_FACED)
  /* Autostart the face display daemon (blinking eyes on fb0) after the
   * AI agent.  Same kernel-side builtin-table launch pattern as above;
   * the task runs with the builtin table stack (CONFIG_EXAMPLES_FACED_
   * STACKSIZE, currently 8192).  A failure here must never abort
   * bringup - only report it.
   */
  {
    FAR const struct builtin_s *builtin = NULL;
    int index = builtin_isavail("faced");
    int pid = -1;

    if (index >= 0)
      {
        builtin = builtin_for_index(index);
      }

    if (builtin != NULL)
      {
        pid = nxtask_create("faced", builtin->priority, NULL,
                            builtin->stacksize, builtin->main, NULL, NULL);
      }

    if (pid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to autostart faced: %d\n", pid);
      }
    else
      {
        syslog(LOG_INFO, "faced autostarted (pid=%d prio=%d stack=%d)\n",
               pid, builtin->priority, builtin->stacksize);
      }
  }
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_EXAMPLES_CAMCLIENT)
  /* Autostart the MJPEG stream client after faced.  It retries the
   * WebSocket connection every 2 s internally, so it does not need the
   * network to be ready at this point.  Default target comes from
   * CONFIG_EXAMPLES_CAMCLIENT_{HOST,PORT}.  A failure here must never
   * abort bringup - only report it.
   */
  {
    /* [SERVO-R41] Start a launcher, not camclient.
     *
     * Creating the camclient task here makes the station's association fail
     * every time (rounds 28/30, re-confirmed in round 40): the task must not
     * exist while netinit is authenticating, even though the app itself waits
     * for the link.  esp32s3_camclient_launch() polls for the address and only
     * then creates camclient, so the association happens uncontended. */

    int pid = nxtask_create("camlaunch", 90, NULL, 2048,
                            esp32s3_camclient_launch, NULL, NULL);

    if (pid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to start camclient launcher: %d\n",
               pid);
      }
    else
      {
        syslog(LOG_INFO, "camclient launcher started (pid=%d), waiting for "
               "the link\n", pid);
      }
  }

  /* [V99] 语音舵机命令轮询任务：独立于视频流常驻，每 300ms 读
   * /tmp/servo_cmd.txt（ai_agent 的 servo 工具写入）。即使摄像头不在线
   * 也能响应"转个头/点点头/摇头"，执行后保持姿态 10s 再恢复人脸跟随。
   * 舵机未使能（无 /dev/pwm0|1）时内部直接 no-op。 */
  {
    int pid = nxtask_create("servopoll", 100, NULL, 2048,
                            vox3_servo_poll_task, NULL, NULL);

    if (pid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to start servo poll task: %d\n", pid);
      }
  }
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_LVX_CONTEST2026_307_INFO_SCREEN)
  /* Autostart the info screen (time + weather) on /dev/fb0.  Safe to launch
   * before the network is up - it shows "--" for time/weather until the clock
   * is set (NTP) and /tmp/weather.txt exists. */
  {
    FAR const struct builtin_s *builtin = NULL;
    int index = builtin_isavail("info_screen");
    int pid = -1;

    if (index >= 0)
      {
        builtin = builtin_for_index(index);
      }

    if (builtin != NULL)
      {
        pid = nxtask_create("info_screen", builtin->priority, NULL,
                            builtin->stacksize, builtin->main, NULL, NULL);
      }

    if (pid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to autostart info_screen: %d\n", pid);
      }
    else
      {
        syslog(LOG_INFO, "info_screen autostarted (pid=%d prio=%d stack=%d)\n",
               pid, builtin->priority, builtin->stacksize);
      }
  }
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_LVX_CONTEST2026_307_XIAOZHI_UI)
  /* [V629] The LVGL UI is not started here any more.  It needs /dev/fb0, and
   * the framebuffer is only registered by v629_screen_launcher() once the
   * network is up - that worker spawns xiaozhi_ui itself. */
#endif

#if defined(CONFIG_BUILTIN) && defined(CONFIG_NETUTILS_NTPCLIENT)
  /* Autostart the NTP client daemon so the system clock is set automatically
   * once WiFi is up.  The daemon retries and keeps polling (STAY_ON). */
  {
    FAR const struct builtin_s *builtin = NULL;
    int index = builtin_isavail("ntpclient");
    int pid = -1;

    if (index >= 0)
      {
        builtin = builtin_for_index(index);
      }

    if (builtin != NULL)
      {
        pid = nxtask_create("ntpclient", builtin->priority, NULL,
                            builtin->stacksize, builtin->main, NULL, NULL);
      }

    if (pid < 0)
      {
        syslog(LOG_ERR, "ERROR: Failed to autostart ntpclient: %d\n", pid);
      }
    else
      {
        syslog(LOG_INFO, "ntpclient autostarted (pid=%d prio=%d stack=%d)\n",
               pid, builtin->priority, builtin->stacksize);
      }
  }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);

  /* [V80] Bringup completed.  Everything below this point - agent_main, the
   * voice boot test, the panel - only ever runs if we get here. */

  vox3_diag_get("BR_BOOT_DONE");

  return OK;
}
