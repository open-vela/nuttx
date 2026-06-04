/****************************************************************************
 * boards/arm/t113/t113-evb/uartring/uartring_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART Loopback Test - TX->RX self-loopback on each UART.
 * Verifies data integrity and measures throughput.
 *
 * Usage:
 *   uartring [-b baud] [-s size] [-m both|tx|rx] [-p] [dev1 ... devN]
 *
 * Modes:
 *   both (default) - self-loopback: TX thread writes, main reads back
 *   tx             - TX only: write pattern, do not read (peer verifies)
 *   rx             - RX only: read pattern, do not write (peer sends)
 *
 * Wiring: connect TX pin to RX pin on each UART under test
 *         (or wire to a peer that echoes / sources data for unidirectional).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <termios.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_DEVS      8
#define BUF_SIZE      2048
#define DEFAULT_BAUD  CONFIG_EXAMPLES_UARTRING_BAUD
#define DEFAULT_SIZE  CONFIG_EXAMPLES_UARTRING_SIZE
#define STALL_LIMIT   20

#define MODE_BOTH     0
#define MODE_TX       1
#define MODE_RX       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct loopback_ctx_s
{
  int         fd;
  const char *name;
  int         baud;
  int         mode;
  size_t      total_size;
  size_t      sent;
  size_t      rcvd;
  size_t      errors;
  double      elapsed;
  int         result;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_default_devs[] =
{
  "/dev/ttyS1",
  "/dev/ttyS2",
  "/dev/ttyS3",
  "/dev/ttyS4",
  "/dev/ttyS5",
};

#define DEFAULT_NDEVS \
  (sizeof(g_default_devs) / sizeof(g_default_devs[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int open_uart_raw(const char *path, int baud)
{
  struct termios tio;
  int fd;

  fd = open(path, O_RDWR | O_NOCTTY);
  if (fd < 0)
    {
      fprintf(stderr, "ERROR: open %s: %s\n",
              path, strerror(errno));
      return -1;
    }

  tcgetattr(fd, &tio);
  cfmakeraw(&tio);
  cfsetispeed(&tio, baud);
  cfsetospeed(&tio, baud);
  tio.c_cc[VMIN]  = 1;
  tio.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
      fprintf(stderr, "ERROR: tcsetattr %s: %s\n",
              path, strerror(errno));
      close(fd);
      return -1;
    }

  tcflush(fd, TCIOFLUSH);
  return fd;
}

/****************************************************************************
 * Name: tx_thread
 *
 * Description:
 *   Blast pattern data. Runs in a separate thread so main thread
 *   can read back concurrently (self-loopback: same fd).
 *
 ****************************************************************************/

static void *tx_thread(void *arg)
{
  struct loopback_ctx_s *ctx = (struct loopback_ctx_s *)arg;
  uint8_t buf[BUF_SIZE];
  size_t i;
  ssize_t n;

  while (ctx->sent < ctx->total_size)
    {
      size_t chunk = ctx->total_size - ctx->sent;
      if (chunk > BUF_SIZE)
        {
          chunk = BUF_SIZE;
        }

      for (i = 0; i < chunk; i++)
        {
          buf[i] = (uint8_t)((ctx->sent + i) & 0xff);
        }

      n = write(ctx->fd, buf, chunk);
      if (n <= 0)
        {
          break;
        }

      ctx->sent += n;
    }

  return NULL;
}

/****************************************************************************
 * Name: test_loopback
 *
 * Description:
 *   Run loopback test on a single UART. TX thread writes pattern,
 *   main function reads back and verifies.
 *
 ****************************************************************************/

static int test_loopback(struct loopback_ctx_s *ctx)
{
  pthread_t tid;
  uint8_t rxbuf[BUF_SIZE];
  ssize_t n;
  struct timespec ts_start;
  struct timespec ts_end;
  struct timespec deadline;
  struct timespec now;
  double tmo;
  size_t last_rcvd = 0;
  int stall_count = 0;
  int i;
  int ret;
  int need_tx = (ctx->mode != MODE_RX);
  int need_rx = (ctx->mode != MODE_TX);

  ctx->sent   = 0;
  ctx->rcvd   = 0;
  ctx->errors = 0;
  ctx->result = 0;

  tcflush(ctx->fd, TCIOFLUSH);

  if (need_tx)
    {
      ret = pthread_create(&tid, NULL, tx_thread, ctx);
      if (ret != 0)
        {
          fprintf(stderr, "  ERROR: pthread_create: %d\n", ret);
          return -1;
        }
    }

  clock_gettime(CLOCK_MONOTONIC, &ts_start);

  /* MARK_START: shared CLOCK_MONOTONIC ns so host can compute the
   * overlap envelope of N concurrent uartring instances.  All tasks
   * share g_monotonic_basetime (sched/clock/clock_gettime.c), so these
   * timestamps are directly comparable across tasks.
   */

  printf("\nMARK_START port=%s ns=%lld\n",
         ctx->name,
         (long long)ts_start.tv_sec * 1000000000LL +
         (long long)ts_start.tv_nsec);
  fflush(stdout);

  /* Upper-bound time budget.  Factor picks the worst case:
   *   -m tx / -m rx : ~100% of theoretical (one-way) -> 2x is plenty
   *   -m both (self-loopback via peer echo) : as low as ~14% in duplex
   *                                           -> 10x covers the roundtrip
   * STALL_LIMIT / VTIME still catches real hangs inside the budget.
   */

  tmo = (double)ctx->total_size * 10.0 / (double)ctx->baud *
        (ctx->mode == MODE_BOTH ? 10.0 : 2.0);
  if (tmo < 3.0)
    {
      tmo = 3.0;
    }

  clock_gettime(CLOCK_MONOTONIC, &deadline);
  deadline.tv_sec += (time_t)(tmo + 2.0);

  while (need_rx && ctx->rcvd < ctx->total_size)
    {
      clock_gettime(CLOCK_MONOTONIC, &now);
      if (now.tv_sec > deadline.tv_sec)
        {
          fprintf(stderr, "  ERROR: timeout %zu/%zu\n",
                  ctx->rcvd, ctx->total_size);
          ctx->result = -1;
          break;
        }

      n = read(ctx->fd, rxbuf, sizeof(rxbuf));
      if (n <= 0)
        {
          if (errno == EAGAIN || errno == ETIMEDOUT)
            {
              if (ctx->rcvd == last_rcvd)
                {
                  if (++stall_count > STALL_LIMIT)
                    {
                      fprintf(stderr, "  ERROR: stall %zu/%zu\n",
                              ctx->rcvd, ctx->total_size);
                      ctx->result = -1;
                      break;
                    }
                }
              else
                {
                  last_rcvd = ctx->rcvd;
                  stall_count = 0;
                }

              continue;
            }

          fprintf(stderr, "  ERROR: read: %s\n",
                  strerror(errno));
          ctx->result = -1;
          break;
        }

      stall_count = 0;
      last_rcvd = ctx->rcvd;

      for (i = 0; i < n; i++)
        {
          uint8_t expected =
            (uint8_t)((ctx->rcvd + i) & 0xff);
          if (rxbuf[i] != expected)
            {
              if (ctx->errors < 8)
                {
                  fprintf(stderr,
                          "  MISMATCH @%zu: 0x%02x != 0x%02x\n",
                          ctx->rcvd + i, rxbuf[i], expected);
                }

              ctx->errors++;
            }
        }

      ctx->rcvd += n;
    }

  if (need_tx)
    {
      pthread_join(tid, NULL);
    }

  /* For TX-only mode, wait for all bytes to drain to the wire before
   * stopping the clock so throughput is measured correctly.
   */

  if (need_tx && !need_rx)
    {
      tcdrain(ctx->fd);
    }

  clock_gettime(CLOCK_MONOTONIC, &ts_end);
  ctx->elapsed = (double)(ts_end.tv_sec - ts_start.tv_sec) +
                 (double)(ts_end.tv_nsec - ts_start.tv_nsec)
                 / 1e9;

  /* MARK_END pairs with MARK_START above.  Host derives the true
   * parallel overlap window as [max(MARK_START_i), min(MARK_END_i)].
   */

  printf("\nMARK_END port=%s ns=%lld rcvd=%zu errors=%zu\n",
         ctx->name,
         (long long)ts_end.tv_sec * 1000000000LL +
         (long long)ts_end.tv_nsec,
         ctx->rcvd, ctx->errors);
  fflush(stdout);

  if (need_tx && ctx->sent < ctx->total_size)
    {
      fprintf(stderr, "  ERROR: tx incomplete %zu/%zu\n",
              ctx->sent, ctx->total_size);
      ctx->result = -1;
    }

  if (ctx->errors > 0)
    {
      ctx->result = -1;
    }

  return ctx->result;
}

static void usage(const char *progname)
{
  fprintf(stderr,
          "Usage: %s [-b baud] [-s size] [-m both|tx|rx] [-p] "
          "[dev1 ... devN]\n", progname);
}

static int parse_mode(const char *s)
{
  if (strcmp(s, "both") == 0)
    {
      return MODE_BOTH;
    }

  if (strcmp(s, "tx") == 0)
    {
      return MODE_TX;
    }

  if (strcmp(s, "rx") == 0)
    {
      return MODE_RX;
    }

  return -1;
}

struct worker_args_s
{
  struct loopback_ctx_s ctx;
  int                   result;
};

static void *worker_entry(void *arg)
{
  struct worker_args_s *w = (struct worker_args_s *)arg;
  w->result = test_loopback(&w->ctx);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  const char *devs[MAX_DEVS];
  int ndevs = 0;
  int baud = DEFAULT_BAUD;
  size_t total_size = DEFAULT_SIZE;
  int mode = MODE_BOTH;
  int parallel = 0;
  int opt;
  int i;
  int fail = 0;

  while ((opt = getopt(argc, argv, "b:s:m:ph")) != -1)
    {
      switch (opt)
        {
          case 'b':
            baud = atoi(optarg);
            break;
          case 's':
            total_size = (size_t)atoi(optarg);
            break;
          case 'm':
            mode = parse_mode(optarg);
            if (mode < 0)
              {
                fprintf(stderr, "ERROR: invalid mode '%s'\n", optarg);
                usage(argv[0]);
                return 1;
              }
            break;
          case 'p':
            parallel = 1;
            break;
          default:
            usage(argv[0]);
            return opt == 'h' ? 0 : 1;
        }
    }

  if (optind < argc)
    {
      for (i = optind; i < argc && ndevs < MAX_DEVS; i++)
        {
          devs[ndevs++] = argv[i];
        }
    }
  else
    {
      ndevs = DEFAULT_NDEVS;
      for (i = 0; i < ndevs; i++)
        {
          devs[i] = g_default_devs[i];
        }
    }

  printf("Loopback: %d ports, baud=%d, size=%zu, mode=%s%s\n",
         ndevs, baud, total_size,
         mode == MODE_TX ? "tx" : mode == MODE_RX ? "rx" : "both",
         parallel ? " [PARALLEL]" : "");

  if (parallel)
    {
      pthread_t              tids[MAX_DEVS];
      struct worker_args_s  *ws;

      ws = calloc(ndevs, sizeof(*ws));
      if (ws == NULL)
        {
          fprintf(stderr, "ERROR: calloc\n");
          return 1;
        }

      for (i = 0; i < ndevs; i++)
        {
          ws[i].ctx.fd = open_uart_raw(devs[i], baud);
          if (ws[i].ctx.fd < 0)
            {
              fail++;
              continue;
            }

          ws[i].ctx.name       = devs[i];
          ws[i].ctx.baud       = baud;
          ws[i].ctx.mode       = mode;
          ws[i].ctx.total_size = total_size;

          if (pthread_create(&tids[i], NULL, worker_entry,
                             &ws[i]) != 0)
            {
              close(ws[i].ctx.fd);
              ws[i].ctx.fd = -1;
              fail++;
            }
        }

      for (i = 0; i < ndevs; i++)
        {
          if (ws[i].ctx.fd < 0)
            {
              continue;
            }

          pthread_join(tids[i], NULL);

          if (ws[i].result < 0)
            {
              size_t metric = (mode == MODE_TX) ?
                              ws[i].ctx.sent : ws[i].ctx.rcvd;
              printf("%-12s: FAIL (%zu/%zu, %zu errors)\n",
                     devs[i], metric, total_size, ws[i].ctx.errors);
              fail++;
            }
          else
            {
              size_t metric = (mode == MODE_TX) ?
                              ws[i].ctx.sent : ws[i].ctx.rcvd;
              double tput   = ws[i].ctx.elapsed > 0.001 ?
                              (double)metric / ws[i].ctx.elapsed : 0;
              double theo   = (double)baud / 10.0;

              printf("%-12s: OK %zu/%zu  %.2fs  %.0f B/s  %.1f%%\n",
                     devs[i], metric, total_size,
                     ws[i].ctx.elapsed, tput,
                     theo > 0 ? tput / theo * 100.0 : 0);
            }

          close(ws[i].ctx.fd);
        }

      free(ws);
    }
  else
    {
      for (i = 0; i < ndevs; i++)
        {
          struct loopback_ctx_s ctx;
          double tput;
          double theoretical;

          ctx.fd = open_uart_raw(devs[i], baud);
          if (ctx.fd < 0)
            {
              fail++;
              continue;
            }

          ctx.name       = devs[i];
          ctx.baud       = baud;
          ctx.mode       = mode;
          ctx.total_size = total_size;

          printf("%-12s: ", devs[i]);
          fflush(stdout);

          if (test_loopback(&ctx) < 0)
            {
              size_t metric = (mode == MODE_TX) ? ctx.sent : ctx.rcvd;
              printf("FAIL (%zu/%zu, %zu errors)\n",
                     metric, total_size, ctx.errors);
              fail++;
            }
          else
            {
              size_t metric = (mode == MODE_TX) ? ctx.sent : ctx.rcvd;

              tput = ctx.elapsed > 0.001 ?
                     (double)metric / ctx.elapsed : 0;
              theoretical = (double)baud / 10.0;

              printf("OK %zu/%zu  %.2fs",
                     metric, total_size, ctx.elapsed);

              if (tput > 0)
                {
                  printf("  %.0f B/s  %.1f%%",
                         tput, tput / theoretical * 100.0);
                }

              printf("\n");
            }

          close(ctx.fd);
        }
    }

  printf("\n%s: %d/%d passed\n",
         fail ? "FAIL" : "PASS", ndevs - fail, ndevs);

  return fail ? 1 : 0;
}
