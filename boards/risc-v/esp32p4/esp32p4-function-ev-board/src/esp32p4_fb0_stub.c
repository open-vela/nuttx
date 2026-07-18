/****************************************************************************
 * esp32p4_fb0_stub.c - Minimal /dev/fb0 character device stub
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <syslog.h>

static uint8_t g_fb0_dummy[64];

static ssize_t fb0_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  size_t copylen = buflen < sizeof(g_fb0_dummy) ?
                   buflen : sizeof(g_fb0_dummy);
  memcpy(buffer, g_fb0_dummy, copylen);
  return (ssize_t)copylen;
}

static ssize_t fb0_write(FAR struct file *filep,
                         FAR const char *buffer, size_t buflen)
{
  size_t copylen = buflen < sizeof(g_fb0_dummy) ?
                   buflen : sizeof(g_fb0_dummy);
  memcpy(g_fb0_dummy, buffer, copylen);
  return (ssize_t)copylen;
}

static const struct file_operations g_fb0_fops =
{
  .open    = NULL,
  .close   = NULL,
  .read    = fb0_read,
  .write   = fb0_write,
  .seek    = NULL,
  .ioctl   = NULL,
  .mmap    = NULL,
  .truncate = NULL,
  .poll    = NULL,
};

int esp_fb0_stub_register(void)
{
  return register_driver("/dev/fb0", &g_fb0_fops, 0666, NULL);
}

/* Alias for bringup code compatibility */
int board_fb_initialize(void)
{
  return esp_fb0_stub_register();
}
