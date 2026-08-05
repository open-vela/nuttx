/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_jpeg_enc.c
 * ESP32-P4 Hardware JPEG Encoder in V4L2 M2M framework
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/v4l2_m2m.h>

/* TODO: Include ESP-HAL JPEG headers when HAL is compiled:
 * #include "driver/jpeg_encode.h"
 * #include "driver/jpeg_types.h"
 */

typedef void *jpeg_encoder_handle_t;

#define JPEG_DEF_WIDTH    1024
#define JPEG_DEF_HEIGHT   600
#define JPEG_MAX_BUF      6

struct esp_jpeg_dev_s
{
  struct v4l2_format  out_fmt;
  struct v4l2_format  cap_fmt;
  jpeg_encoder_handle_t engine;
  size_t  encoded;
};

static int jpeg_cap_enum_fmt(FAR void *p, FAR struct v4l2_fmtdesc *f)
{ if (f->index > 0) return -EINVAL; f->pixelformat = V4L2_PIX_FMT_JPEG;
  f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE; return OK; }

static int jpeg_out_enum_fmt(FAR void *p, FAR struct v4l2_fmtdesc *f)
{ if (f->index > 0) return -EINVAL; f->pixelformat = V4L2_PIX_FMT_RGB565;
  f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT; return OK; }

static int jpeg_cap_g_fmt(FAR void *p, FAR struct v4l2_format *f)
{ memcpy(f, &((struct esp_jpeg_dev_s *)p)->cap_fmt, sizeof(*f)); return OK; }

static int jpeg_out_g_fmt(FAR void *p, FAR struct v4l2_format *f)
{ memcpy(f, &((struct esp_jpeg_dev_s *)p)->out_fmt, sizeof(*f)); return OK; }

static int jpeg_cap_s_fmt(FAR void *p, FAR struct v4l2_format *f)
{ struct esp_jpeg_dev_s *d = p;
  if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_JPEG) return -EINVAL;
  f->fmt.pix.width  = d->out_fmt.fmt.pix.width;
  f->fmt.pix.height = d->out_fmt.fmt.pix.height;
  f->fmt.pix.sizeimage = d->out_fmt.fmt.pix.sizeimage;
  memcpy(&d->cap_fmt, f, sizeof(*f)); return OK; }

static int jpeg_out_s_fmt(FAR void *p, FAR struct v4l2_format *f)
{ struct esp_jpeg_dev_s *d = p;
  if (f->fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) return -EINVAL;
  f->fmt.pix.width  = CLAMP(f->fmt.pix.width,  16, 1024);
  f->fmt.pix.height = CLAMP(f->fmt.pix.height, 16, 768);
  f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height * 2;
  memcpy(&d->out_fmt, f, sizeof(*f)); return OK; }

static size_t jpeg_cap_bufsize(FAR void *p)
{ return ((struct esp_jpeg_dev_s *)p)->out_fmt.fmt.pix.sizeimage; }

static size_t jpeg_out_bufsize(FAR void *p)
{ return ((struct esp_jpeg_dev_s *)p)->out_fmt.fmt.pix.sizeimage; }

static size_t jpeg_cap_bufcnt(FAR void *p) { return 4; }
static size_t jpeg_out_bufcnt(FAR void *p) { return 4; }

static FAR void *jpeg_alloc(FAR void *p, size_t sz) { return kmm_malloc(sz); }
static void jpeg_free(FAR void *p, FAR void *a) { kmm_free(a); }

static int jpeg_open(FAR void *cookie, FAR void **priv)
{
  struct esp_jpeg_dev_s *d = kmm_zalloc(sizeof(*d));
  if (!d) return -ENOMEM;
  d->out_fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
  d->out_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  d->out_fmt.fmt.pix.width  = JPEG_DEF_WIDTH;
  d->out_fmt.fmt.pix.height = JPEG_DEF_HEIGHT;
  d->out_fmt.fmt.pix.sizeimage = JPEG_DEF_WIDTH * JPEG_DEF_HEIGHT * 2;
  d->cap_fmt = d->out_fmt;
  d->cap_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
  /* TODO: Allocate ESP-HAL JPEG encoder engine once HAL is compiled in */
  d->engine = NULL;
  *priv = d; return OK;
}

static int jpeg_close(FAR void *priv) { kmm_free(priv); return OK; }

static int jpeg_cap_streamon(FAR void *priv) { ((struct esp_jpeg_dev_s *)priv)->encoded = 0; return OK; }
static int jpeg_out_streamon(FAR void *priv) { return OK; }
static int jpeg_cap_streamoff(FAR void *priv) { return OK; }
static int jpeg_out_streamoff(FAR void *priv) { return OK; }
static int jpeg_cap_avail(FAR void *priv) { return 0; }
static int jpeg_out_avail(FAR void *priv) { return 0; }

static int jpeg_querycap(FAR void *priv, FAR struct v4l2_capability *cap)
{
  strlcpy((FAR char *)cap->driver, "esp32p4-jpeg", sizeof(cap->driver));
  strlcpy((FAR char *)cap->card,   "ESP32-P4 JPEG Enc", sizeof(cap->card));
  cap->capabilities = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
  return OK;
}

static const struct codec_ops_s g_jpeg_m2m_ops =
{
  .open               = jpeg_open,
  .close              = jpeg_close,
  .capture_streamon   = jpeg_cap_streamon,
  .output_streamon    = jpeg_out_streamon,
  .capture_streamoff  = jpeg_cap_streamoff,
  .output_streamoff   = jpeg_out_streamoff,
  .capture_available  = jpeg_cap_avail,
  .output_available   = jpeg_out_avail,
  .capture_enum_fmt   = jpeg_cap_enum_fmt,
  .output_enum_fmt    = jpeg_out_enum_fmt,
  .capture_g_fmt      = jpeg_cap_g_fmt,
  .output_g_fmt       = jpeg_out_g_fmt,
  .capture_s_fmt      = jpeg_cap_s_fmt,
  .output_s_fmt       = jpeg_out_s_fmt,
  .capture_g_bufsize  = jpeg_cap_bufsize,
  .output_g_bufsize   = jpeg_out_bufsize,
  .capture_g_bufcnt   = jpeg_cap_bufcnt,
  .output_g_bufcnt    = jpeg_out_bufcnt,
  .alloc_buf          = jpeg_alloc,
  .free_buf           = jpeg_free,
  .querycap           = jpeg_querycap,
};

int esp_jpeg_encoder_register(FAR const char *devpath)
{
  struct codec_s *c = kmm_zalloc(sizeof(*c));
  if (!c) return -ENOMEM;
  c->ops = &g_jpeg_m2m_ops;
  int ret = codec_register(devpath, c);
  if (ret < 0) kmm_free(c);
  else vinfo("JPEG M2M encoder at %s\n", devpath);
  return ret;
}
