/****************************************************************************
 * drivers/usbhost/usbhost_uvc.c
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>
#include <nuttx/signal.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/uvc.h>
#include <nuttx/usb/usbhost_uvc.h>

#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/video.h>
#include <nuttx/video/v4l2_cap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  warning "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

/* MIN/MAX macros */

#ifndef MIN
#  define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef CONFIG_USBHOST_UVC_BUFSIZE
#  define CONFIG_USBHOST_UVC_BUFSIZE 32768
#endif

#ifndef CONFIG_USBHOST_UVC_NFRAMES
#  define CONFIG_USBHOST_UVC_NFRAMES 3
#endif

#ifndef CONFIG_USBHOST_UVC_DEFPRIO
#  define CONFIG_USBHOST_UVC_DEFPRIO 150
#endif

#ifndef CONFIG_USBHOST_UVC_STACKSIZE
#  define CONFIG_USBHOST_UVC_STACKSIZE 4096
#endif

/* UVC default frame parameters */

#define UVC_DEFAULT_WIDTH           640
#define UVC_DEFAULT_HEIGHT          480
#define UVC_DEFAULT_FPS             30

/* Frame interval in 100ns units */

#define UVC_INTERVAL_30FPS          333333   /* 100ns units */

/* Transfer buffer alignment.
 * Must match the USB host controller's D-cache line size (64 bytes on
 * Allwinner R528). A smaller alignment causes cache flush/invalidate to
 * corrupt adjacent data during DMA, producing XACTERR on isoc transfers.
 */

#define UVC_BUFFER_ALIGN            64

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* UVC device states */

enum uvc_state_e
{
  UVC_STATE_DISCONNECTED = 0,  /* Device disconnected */
  UVC_STATE_CONNECTED,         /* Device connected, not streaming */
  UVC_STATE_STREAMING          /* Device streaming */
};

/* UVC format information */

struct uvc_format_info_s
{
  uint32_t fourcc;             /* V4L2 pixel format */
  uint16_t width;
  uint16_t height;
  uint32_t frame_interval;     /* 100ns units */
  uint32_t frame_size;         /* Max frame size in bytes */
};

/* UVC device private state */

struct usbhost_uvc_s
{
  /* USB Host Class interface (MUST be the first member) */

  struct usbhost_class_s    usbclass;

  /* Device state */

  volatile enum uvc_state_e state;
  volatile bool             disconnected;
  int16_t                   crefs;
  mutex_t                   lock;
  sem_t                     stream_sem;

  /* USB interface information */

  uint8_t                   vc_ifno;        /* VideoControl interface */
  uint8_t                   vs_ifno;        /* VideoStreaming interface */
  uint8_t                   vs_alt;         /* Streaming alt setting */

  /* Endpoints */

  usbhost_ep_t              ep_stream;      /* Isochronous IN endpoint */
  uint8_t                   ep_addr;        /* Endpoint address */
  uint16_t                  ep_maxpacketsize;
  uint8_t                   ep_interval;    /* bInterval of chosen alt */
  uint8_t                   ep_mult;        /* Transactions/microframe (1..3) */

  /* VideoStreaming alternate-setting bandwidth table.  Each entry records,
   * for one alt setting of the VS interface, its isoc IN endpoint address,
   * per-packet size, mult (transactions per microframe) and resulting
   * per-microframe bandwidth.  Used to pick the smallest alt whose bandwidth
   * covers the negotiated dwMaxPayloadTransferSize.
   */

#define UVC_MAX_ALTS 8
  struct
  {
    uint8_t  alt;                            /* Alternate setting number */
    uint8_t  ep_addr;                        /* Endpoint address */
    uint8_t  interval;                       /* bInterval */
    uint8_t  mult;                           /* 1..3 transactions/uframe */
    uint16_t maxpacket;                      /* Bytes per transaction */
    uint32_t bandwidth;                      /* maxpacket * mult */
  } vs_alts[UVC_MAX_ALTS];
  uint8_t                   vs_nalts;        /* Number of entries in vs_alts */

  /* UVC descriptor parsing results */

  uint8_t                   format_idx;     /* Current format index */
  uint8_t                   frame_idx;      /* Current frame index */
  uint32_t                  max_frame_size;
  uint32_t                  max_payload_size;

  /* Supported formats */

  struct uvc_format_info_s  cur_format;

  /* Transfer buffers */

  FAR uint8_t              *xfer_buf;       /* USB transfer buffer */
  size_t                    xfer_buflen;

  /* Frame buffer */

  FAR uint8_t              *frame_buf;      /* Frame assembly buffer */
  size_t                    frame_buflen;
  size_t                    frame_offset;
  uint8_t                   frame_id;       /* FID toggle bit */
  uint8_t                   asm_lastfid;    /* Last FID seen while assembling */

  /* Video subsystem integration */

  struct imgdata_s          imgdata;
  struct imgsensor_s        imgsensor;
  imgdata_capture_t         capture_cb;
  FAR void                 *capture_arg;
  FAR uint8_t              *user_buf;       /* User buffer for capture */
  uint32_t                  user_buflen;

  /* Streaming thread */

  pid_t                     stream_tid;
  bool                      streaming;

  /* Worker queue for deferred operations */

  struct work_s             work;

  /* Device number for /dev/videoN */

  uint8_t                   devno;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* USB Host Class interface methods */

static FAR struct usbhost_class_s *
  usbhost_uvc_create(FAR struct usbhost_hubport_s *hport,
                     FAR const struct usbhost_id_s *id);
static int usbhost_uvc_connect(FAR struct usbhost_class_s *usbclass,
                               FAR const uint8_t *configdesc,
                               int desclen);
static int usbhost_uvc_disconnected(FAR struct usbhost_class_s *usbclass);

/* Helper functions */

static int usbhost_uvc_parse_config(FAR struct usbhost_uvc_s *priv,
                                    FAR const uint8_t *configdesc,
                                    int desclen);
static int usbhost_uvc_parse_vc_interface(FAR struct usbhost_uvc_s *priv,
                                          FAR const uint8_t *desc);
static int usbhost_uvc_parse_vs_interface(FAR struct usbhost_uvc_s *priv,
                                          FAR const uint8_t *desc);
static int usbhost_uvc_probe_commit(FAR struct usbhost_uvc_s *priv,
                                    uint8_t req,
                                    uint8_t selector,
                                    FAR struct uvc_probe_commit_s *pc);
static int usbhost_uvc_set_interface(FAR struct usbhost_uvc_s *priv,
                                     uint8_t iface, uint8_t alt);
static int usbhost_uvc_start_streaming(FAR struct usbhost_uvc_s *priv);
static int usbhost_uvc_stop_streaming(FAR struct usbhost_uvc_s *priv);

/* Streaming thread */

static int usbhost_uvc_stream_thread(int argc, FAR char *argv[]);
static void usbhost_uvc_process_payload(FAR struct usbhost_uvc_s *priv,
                                        FAR const uint8_t *payload,
                                        size_t len);

/* Video subsystem callbacks - imgsensor_ops_s */

static bool uvc_sensor_is_available(FAR struct imgsensor_s *sensor);
static int uvc_sensor_init(FAR struct imgsensor_s *sensor);
static int uvc_sensor_uninit(FAR struct imgsensor_s *sensor);
static FAR const char *
  uvc_sensor_get_driver_name(FAR struct imgsensor_s *sensor);
static int uvc_sensor_validate_frame_setting(
  FAR struct imgsensor_s *sensor,
  imgsensor_stream_type_t type,
  uint8_t nr_datafmt,
  FAR imgsensor_format_t *datafmts,
  FAR imgsensor_interval_t *interval);
static int uvc_sensor_start_capture(FAR struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_datafmt,
                                    FAR imgsensor_format_t *datafmts,
                                    FAR imgsensor_interval_t *interval);
static int uvc_sensor_stop_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type);

/* Video subsystem callbacks - imgdata_ops_s */

static int uvc_data_init(FAR struct imgdata_s *data);
static int uvc_data_uninit(FAR struct imgdata_s *data);
static int uvc_data_validate_frame_setting(
  FAR struct imgdata_s *data,
  uint8_t nr_datafmt,
  FAR imgdata_format_t *datafmt,
  FAR imgdata_interval_t *interval);
static int uvc_data_start_capture(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmt,
                                  FAR imgdata_format_t *datafmt,
                                  FAR imgdata_interval_t *interval,
                                  imgdata_capture_t callback,
                                  FAR void *arg);
static int uvc_data_stop_capture(FAR struct imgdata_s *data);
static int uvc_data_set_buf(FAR struct imgdata_s *data,
                            uint8_t nr_datafmts,
                            FAR imgdata_format_t *datafmts,
                            FAR uint8_t *addr,
                            uint32_t size);

/* Memory helpers */

static inline FAR struct usbhost_uvc_s *usbhost_uvc_allocclass(void);
static inline void usbhost_uvc_freeclass(FAR struct usbhost_uvc_s *priv);
static inline uint16_t usbhost_uvc_getle16(FAR const uint8_t *val);
static inline uint32_t usbhost_uvc_getle32(FAR const uint8_t *val);

/* Device number management */

static int usbhost_uvc_allocdevno(FAR struct usbhost_uvc_s *priv);
static void usbhost_uvc_freedevno(FAR struct usbhost_uvc_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB Host Class ID - match Video Control and Video Streaming interfaces */

static const struct usbhost_id_s g_uvc_id[] =
{
  {
    USB_CLASS_VIDEO,              /* base - Video Interface Class */
    UVC_SUBCLASS_CONTROL,         /* subclass - Video Control */
    0,                            /* proto - any */
    0,                            /* vid */
    0                             /* pid */
  },
  {
    USB_CLASS_VIDEO,              /* base - Video Interface Class */
    UVC_SUBCLASS_STREAMING,       /* subclass - Video Streaming */
    0,                            /* proto - any */
    0,                            /* vid */
    0                             /* pid */
  },
  {
    USB_CLASS_MISC,               /* base - Miscellaneous Class (0xEF) */
    2,                            /* subclass - Common Class */
    1,                            /* proto - IAD */
    0,                            /* vid */
    0                             /* pid */
  },
};

/* USB Host Class registry entry */

static struct usbhost_registry_s g_uvc_registry =
{
  NULL,                           /* flink */
  usbhost_uvc_create,            /* create */
  3,                             /* nids - three IDs to match */
  g_uvc_id                       /* id[] */
};

/* Device number bitmap */

static uint32_t g_uvc_devinuse;

/* Image sensor operations */

static const struct imgsensor_ops_s g_uvc_sensor_ops =
{
  .is_available            = uvc_sensor_is_available,
  .init                    = uvc_sensor_init,
  .uninit                  = uvc_sensor_uninit,
  .get_driver_name         = uvc_sensor_get_driver_name,
  .validate_frame_setting  = uvc_sensor_validate_frame_setting,
  .start_capture           = uvc_sensor_start_capture,
  .stop_capture            = uvc_sensor_stop_capture,
};

/* Image data operations */

static const struct imgdata_ops_s g_uvc_data_ops =
{
  .init                    = uvc_data_init,
  .uninit                  = uvc_data_uninit,
  .validate_frame_setting  = uvc_data_validate_frame_setting,
  .start_capture           = uvc_data_start_capture,
  .stop_capture            = uvc_data_stop_capture,
  .set_buf                 = uvc_data_set_buf,
};

/* Default supported formats */

static const struct v4l2_fmtdesc g_uvc_fmtdescs[] =
{
  {
    .pixelformat = V4L2_PIX_FMT_MJPEG,
    .description = "MJPEG",
  },
};

static const struct v4l2_frmsizeenum g_uvc_frmsizes[] =
{
  {
    .discrete = {
      .width = 640,
      .height = 480,
    },
  },
  {
    .discrete = {
      .width = 320,
      .height = 240,
    },
  },
};

static const struct v4l2_frmivalenum g_uvc_frmintervals[] =
{
  {
    .discrete = {
      .numerator = 1,
      .denominator = 30,
    },
  },
};

/****************************************************************************
 * Private Functions - Memory Helpers
 ****************************************************************************/

static inline FAR struct usbhost_uvc_s *usbhost_uvc_allocclass(void)
{
  FAR struct usbhost_uvc_s *priv;

  DEBUGASSERT(!up_interrupt_context());

  priv = (FAR struct usbhost_uvc_s *)
    kmm_zalloc(sizeof(struct usbhost_uvc_s));

  uinfo("Allocated: %p\n", priv);
  return priv;
}

static inline void usbhost_uvc_freeclass(FAR struct usbhost_uvc_s *priv)
{
  DEBUGASSERT(priv != NULL);
  uinfo("Freeing: %p\n", priv);
  kmm_free(priv);
}

static inline uint16_t usbhost_uvc_getle16(FAR const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

static inline uint32_t usbhost_uvc_getle32(FAR const uint8_t *val)
{
  return (uint32_t)usbhost_uvc_getle16(&val[2]) << 16 |
         (uint32_t)usbhost_uvc_getle16(val);
}

/****************************************************************************
 * Private Functions - Device Number Management
 ****************************************************************************/

static int usbhost_uvc_allocdevno(FAR struct usbhost_uvc_s *priv)
{
  irqstate_t flags;
  int devno;

  flags = enter_critical_section();
  for (devno = 0; devno < 32; devno++)
    {
      uint32_t bitno = 1 << devno;
      if ((g_uvc_devinuse & bitno) == 0)
        {
          g_uvc_devinuse |= bitno;
          priv->devno = devno;
          leave_critical_section(flags);
          return OK;
        }
    }

  leave_critical_section(flags);
  return -EMFILE;
}

static void usbhost_uvc_freedevno(FAR struct usbhost_uvc_s *priv)
{
  if (priv->devno < 32)
    {
      irqstate_t flags = enter_critical_section();
      g_uvc_devinuse &= ~(1 << priv->devno);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Private Functions - UVC Protocol
 ****************************************************************************/

static int usbhost_uvc_probe_commit(FAR struct usbhost_uvc_s *priv,
                                    uint8_t req,
                                    uint8_t selector,
                                    FAR struct uvc_probe_commit_s *pc)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  FAR struct usbhost_driver_s *drvr = hport->drvr;
  struct usb_ctrlreq_s ctrl;
  FAR uint8_t *buf;
  int ret;

  /* Allocate buffer for the control request */

  size_t buflen = 0;
  ret = DRVR_ALLOC(drvr, &buf, &buflen);
  if (ret < 0)
    {
      uerr("ERROR: Failed to allocate control buffer: %d\n", ret);
      return ret;
    }

  /* Copy probe/commit structure to buffer */

  memcpy(buf, pc, UVC_PROBE_COMMIT_SIZE);

  /* Build the control request */

  ctrl.req     = req;  /* SET_CUR or GET_CUR */

  /* Set request type based on direction */
  if (req == UVC_SET_CUR)
    {
      ctrl.type = 0x21;  /* Host-to-device, Class, Interface */
    }
  else
    {
      ctrl.type = 0xA1;  /* Device-to-host, Class, Interface */
    }

  ctrl.value[0] = 0;                      /* wValue low byte: zero */
  ctrl.value[1] = selector;                /* wValue high byte: control selector */
  ctrl.index[0] = priv->vs_ifno;          /* wIndex low byte: interface number */
  ctrl.index[1] = 0;                      /* wIndex high byte */
  ctrl.len[0]  = UVC_PROBE_COMMIT_SIZE & 0xff;
  ctrl.len[1]  = (UVC_PROBE_COMMIT_SIZE >> 8) & 0xff;

  syslog(LOG_ERR, "probe_commit: req=%02x type=%02x wValue=%02x%02x "
         "wIndex=%02x%02x wLen=%02x%02x vs_ifno=%d\n",
         ctrl.req, ctrl.type, ctrl.value[1], ctrl.value[0],
         ctrl.index[1], ctrl.index[0], ctrl.len[1], ctrl.len[0],
         priv->vs_ifno);

  if (req == UVC_SET_CUR)
    {
      ret = DRVR_CTRLOUT(drvr, hport->ep0, &ctrl, buf);
    }
  else
    {
      ret = DRVR_CTRLIN(drvr, hport->ep0, &ctrl, buf);
      if (ret >= 0)
        {
          memcpy(pc, buf, UVC_PROBE_COMMIT_SIZE);
        }
    }

  syslog(LOG_ERR, "probe_commit: req=%02x ret=%d\n", req, ret);

  /* DIAG: dump the full 26-byte probe/commit buffer so we can see exactly
   * what the camera negotiates (bmHint, format/frame idx, dwFrameInterval,
   * dwMaxVideoFrameSize, dwMaxPayloadTransferSize).
   */

  if (ret >= 0)
    {
      syslog(LOG_ERR, "probe_commit BUF: %02x %02x %02x %02x %02x %02x "
             "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x "
             "%02x %02x %02x %02x %02x %02x %02x %02x\n",
             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
             buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
             buf[15], buf[16], buf[17], buf[18], buf[19], buf[20], buf[21],
             buf[22], buf[23], buf[24], buf[25]);
    }

  DRVR_FREE(drvr, buf);

  if (ret < 0)
    {
      uerr("ERROR: UVC probe/commit failed: %d (req=%02x)\n", ret, req);
    }

  return ret;
}

static int usbhost_uvc_set_interface(FAR struct usbhost_uvc_s *priv,
                                     uint8_t iface, uint8_t alt)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  FAR struct usbhost_driver_s *drvr = hport->drvr;
  struct usb_ctrlreq_s ctrl;
  int ret;

  /* SET_INTERFACE request: Host-to-device, Standard, Interface */

  ctrl.type    = 0x01;
  ctrl.req     = USB_REQ_SETINTERFACE;
  ctrl.value[0] = alt;
  ctrl.value[1] = 0;
  ctrl.index[0] = iface;
  ctrl.index[1] = 0;
  ctrl.len[0]  = 0;
  ctrl.len[1]  = 0;

  ret = DRVR_CTRLOUT(drvr, hport->ep0, &ctrl, NULL);
  syslog(LOG_ERR, "SET_INTERFACE: if=%d alt=%d ret=%d\n", iface, alt, ret);

  return ret;
}

/****************************************************************************
 * Private Functions - Configuration Descriptor Parsing
 ****************************************************************************/

static int usbhost_uvc_parse_config(FAR struct usbhost_uvc_s *priv,
                                    FAR const uint8_t *configdesc,
                                    int desclen)
{
  FAR struct usb_cfgdesc_s *cfgdesc;
  FAR struct usb_desc_s *desc;
  FAR struct usb_ifdesc_s *ifdesc;
  int remaining;
  bool found_vs = false;
  uint8_t cur_alt = 0;          /* Alt setting of the descriptor we're in */
  bool cur_is_vs = false;       /* True while inside a VS interface descriptor */

  DEBUGASSERT(priv != NULL && configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  cfgdesc = (FAR struct usb_cfgdesc_s *)configdesc;
  if (cfgdesc->type != USB_DESC_TYPE_CONFIG)
    {
      return -EINVAL;
    }

  priv->vs_nalts = 0;

  remaining = (int)usbhost_uvc_getle16(cfgdesc->totallen);
  configdesc += cfgdesc->len;
  remaining  -= cfgdesc->len;

  /* Parse all descriptors in the configuration */

  while (remaining >= sizeof(struct usb_desc_s))
    {
      desc = (FAR struct usb_desc_s *)configdesc;

      switch (desc->type)
        {
          case USB_DESC_TYPE_INTERFACE:
            {
              ifdesc = (FAR struct usb_ifdesc_s *)configdesc;

              uinfo("Interface: class=%02x sub=%02x proto=%02x ifno=%d\n",
                    ifdesc->classid, ifdesc->subclass,
                    ifdesc->protocol, ifdesc->ifno);

              /* Track which interface/alt the following endpoint descriptors
               * belong to, and whether it is the VideoStreaming interface.
               */

              cur_alt    = ifdesc->alt;
              cur_is_vs  = (ifdesc->classid == USB_CLASS_VIDEO &&
                            ifdesc->subclass == UVC_SUBCLASS_STREAMING);

              /* Check for Video Streaming interface */

              if (cur_is_vs)
                {
                  syslog(LOG_ERR, "VS interface: ifno=%d alt=%d\n",
                         ifdesc->ifno, ifdesc->alt);
                  priv->vs_ifno = ifdesc->ifno;
                  found_vs = true;
                }
              /* Check for Video Control interface */

              else if (ifdesc->classid == USB_CLASS_VIDEO &&
                       ifdesc->subclass == UVC_SUBCLASS_CONTROL)
                {
                  priv->vc_ifno = ifdesc->ifno;
                }
            }
            break;

          case USB_DESC_TYPE_ENDPOINT:
            {
              FAR struct usb_epdesc_s *epd =
                (FAR struct usb_epdesc_s *)configdesc;
              uint16_t wmps = usbhost_uvc_getle16(epd->mxpacketsize);
              uint16_t sz   = wmps & 0x7ff;
              uint8_t  mult = ((wmps >> 11) & 0x3) + 1;

              uinfo("Endpoint: addr=%02x attr=%02x maxpkt=%d\n",
                    epd->addr, epd->attr, wmps);

              /* Only record isochronous IN endpoints that belong to the
               * VideoStreaming interface.  Ignore audio/other interfaces'
               * isoc endpoints (e.g. a microphone), which previously got
               * mis-selected as the video stream endpoint.
               */

              if (cur_is_vs && cur_alt != 0 &&
                  (epd->attr & USB_EP_ATTR_XFERTYPE_MASK) ==
                  USB_EP_ATTR_XFER_ISOC &&
                  USB_ISEPIN(epd->addr))
                {
                  if (priv->vs_nalts < UVC_MAX_ALTS)
                    {
                      FAR typeof(priv->vs_alts[0]) *e =
                        &priv->vs_alts[priv->vs_nalts];
                      e->alt       = cur_alt;
                      e->ep_addr   = epd->addr;
                      e->interval  = epd->interval;
                      e->mult      = mult;
                      e->maxpacket = wmps;  /* Full wMaxPacketSize with mult bits for EHCI */
                      e->bandwidth = (uint32_t)sz * mult;

                      syslog(LOG_ERR, "UVC alt table[%d]: alt=%d ep=%02x "
                             "size=%u mult=%u bw=%u interval=%u\n",
                             priv->vs_nalts, cur_alt, epd->addr, sz, mult,
                             e->bandwidth, epd->interval);

                      priv->vs_nalts++;
                    }
                }
            }
            break;

          default:
            {
              /* DIAG: dump class-specific VS interface descriptors
               * (bDescriptorType 0x24 = CS_INTERFACE) so we can see the
               * actual formats/frames the camera offers and their indices,
               * widths, heights and frame intervals.  bDescriptorSubtype:
               * 0x04=FORMAT_UNCOMPRESSED 0x05=FRAME_UNCOMPRESSED
               * 0x06=FORMAT_MJPEG 0x07=FRAME_MJPEG 0x01=VS_INPUT_HEADER.
               */

              if (desc->type == 0x24 && desc->len >= 3)
                {
                  FAR const uint8_t *d = (FAR const uint8_t *)configdesc;

                  /* VS_INPUT_HEADER (subtype 0x01): b3=bNumFormats */

                  if (d[2] == 0x01 && desc->len >= 4)
                    {
                      syslog(LOG_ERR,
                        "VS_INPUT_HEADER on ifno=%d: bNumFormats=%u len=%u\n",
                        priv->vs_ifno, d[3], d[0]);
                    }

                  syslog(LOG_ERR, "DIAG cs-vs: len=%u subtype=0x%02x "
                         "b3=%u b4=%u b5=%u b6=%u b7=%u b8=%u b9=%u b10=%u\n",
                         d[0], d[2],
                         desc->len > 3 ? d[3] : 0, desc->len > 4 ? d[4] : 0,
                         desc->len > 5 ? d[5] : 0, desc->len > 6 ? d[6] : 0,
                         desc->len > 7 ? d[7] : 0, desc->len > 8 ? d[8] : 0,
                         desc->len > 9 ? d[9] : 0, desc->len > 10 ? d[10] : 0);
                }
            }
            break;
        }

      configdesc += desc->len;
      remaining  -= desc->len;
    }

  if (!found_vs || priv->vs_nalts == 0)
    {
      uerr("ERROR: Missing VS interface or isoc endpoint\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Private Functions - Streaming Control
 ****************************************************************************/

static void usbhost_uvc_clear_halt(FAR struct usbhost_uvc_s *priv)
{
  FAR struct usbhost_hubport_s *hport = priv->usbclass.hport;
  FAR struct usbhost_driver_s *drvr = hport->drvr;
  struct usb_ctrlreq_s ctrl;
  FAR uint8_t *buf;
  size_t buflen = 0;
  int ret;

  /* CLEAR_FEATURE(ENDPOINT_HALT) on EP0 */

  ret = DRVR_ALLOC(drvr, &buf, &buflen);
  if (ret < 0)
    {
      return;
    }

  ctrl.type   = 0x02;  /* Host-to-device, Standard, Endpoint */
  ctrl.req    = USB_REQ_CLEARFEATURE;
  ctrl.value[0] = USB_FEATURE_ENDPOINTHALT;
  ctrl.value[1] = 0;
  ctrl.index[0] = 0;  /* EP0 IN */
  ctrl.index[1] = 0;
  ctrl.len[0]  = 0;
  ctrl.len[1]  = 0;

  DRVR_CTRLOUT(drvr, hport->ep0, &ctrl, buf);
  DRVR_FREE(drvr, buf);
}

static int usbhost_uvc_start_streaming(FAR struct usbhost_uvc_s *priv)
{
  struct uvc_probe_commit_s probe;
  int ret;
  bool probe_ok = false;

  syslog(LOG_ERR, "uvc_start_streaming: format_idx=%d frame_idx=%d\n",
         priv->format_idx, priv->frame_idx);

  /* UVC Probe/Commit negotiation */

  memset(&probe, 0, sizeof(probe));
  ret = usbhost_uvc_probe_commit(priv, UVC_GET_CUR,
               UVC_VS_PROBE_CONTROL, &probe);
  if (ret < 0)
    {
      syslog(LOG_ERR, "uvc_start_streaming: GET_CUR failed (%d)\n", ret);
    }
  else
    {
      syslog(LOG_ERR, "uvc_start_streaming: GET_CUR ok: "
             "frame_size=%u payload=%u\n",
             (unsigned int)probe.dwMaxVideoFrameSize,
             (unsigned int)probe.dwMaxPayloadTransferSize);
      probe_ok = true;
    }

  probe.bFormatIndex    = priv->format_idx;
  probe.bFrameIndex     = priv->frame_idx;
  probe.dwFrameInterval = priv->cur_format.frame_interval;
  if (!probe_ok)
    {
      probe.bmHint = 1;
    }

  ret = usbhost_uvc_probe_commit(priv, UVC_SET_CUR,
               UVC_VS_PROBE_CONTROL, &probe);
  if (ret < 0)
    {
      syslog(LOG_ERR, "uvc_start_streaming: SET_CUR failed (%d)\n", ret);
    }
  else
    {
      syslog(LOG_ERR, "uvc_start_streaming: SET_CUR ok\n");
    }

  ret = usbhost_uvc_probe_commit(priv, UVC_GET_CUR,
               UVC_VS_PROBE_CONTROL, &probe);
  if (ret < 0)
    {
      syslog(LOG_ERR, "uvc_start_streaming: GET_CUR(negotiated) failed\n");
    }
  else
    {
      syslog(LOG_ERR, "uvc_start_streaming: GET_CUR(negotiated) ok: "
             "frame_size=%u payload=%u\n",
             (unsigned int)probe.dwMaxVideoFrameSize,
             (unsigned int)probe.dwMaxPayloadTransferSize);
    }

  probe.bFormatIndex    = priv->format_idx;
  probe.bFrameIndex     = priv->frame_idx;
  probe.dwFrameInterval = priv->cur_format.frame_interval;

  ret = usbhost_uvc_probe_commit(priv, UVC_SET_CUR,
               UVC_VS_COMMIT_CONTROL, &probe);
  if (ret < 0)
    {
      syslog(LOG_ERR, "uvc_start_streaming: COMMIT failed (%d)\n", ret);
    }
  else
    {
      syslog(LOG_ERR, "uvc_start_streaming: COMMIT ok\n");
    }

  priv->max_frame_size = probe.dwMaxVideoFrameSize;
  priv->max_payload_size = probe.dwMaxPayloadTransferSize;

  if (priv->max_frame_size == 0)
    {
      priv->max_frame_size = priv->cur_format.width *
                             priv->cur_format.height * 2;
    }

  if (priv->max_payload_size == 0)
    {
      priv->max_payload_size = priv->ep_maxpacketsize;
    }

  syslog(LOG_ERR, "uvc_start_streaming: frame_size=%u payload_size=%u\n",
         (unsigned int)priv->max_frame_size,
         (unsigned int)priv->max_payload_size);

  /* Select the highest-bandwidth alt that still uses mult==1.
   *
   * High-bandwidth alts (mult>1) issue up to `mult` transactions per micro-
   * frame; the device may split a UVC payload across them or pack several
   * payloads together, so a micro-frame's data is no longer exactly one
   * payload with a header at offset 0.  Our ring stages one micro-frame as
   * one FIFO entry and the UVC parser assumes one header per entry, so mult>1
   * corrupts the framing (scrambled image).  Restrict to mult==1 and, among
   * those, pick the widest (here alt=3, 800 B/uframe) for the most headroom.
   */

  {
    int i;
    int best = -1;

    for (i = 0; i < priv->vs_nalts; i++)
      {
        if (priv->vs_alts[i].mult > 1)
          {
            continue;  /* Skip high-bandwidth alts (framing hazard) */
          }

        if (best < 0 ||
            priv->vs_alts[i].bandwidth > priv->vs_alts[best].bandwidth)
          {
            best = i;
          }
      }

    /* Fallback to first alt if nothing matches */

    if (best < 0)
      {
        best = 0;
      }

    if (best < 0)
      {
        syslog(LOG_ERR, "uvc_start_streaming: no VS alt endpoint\n");
        return -ENODEV;
      }

    priv->vs_alt          = priv->vs_alts[best].alt;
    priv->ep_addr         = priv->vs_alts[best].ep_addr;
    priv->ep_maxpacketsize = priv->vs_alts[best].maxpacket;
    priv->ep_mult         = priv->vs_alts[best].mult;
    priv->ep_interval     = priv->vs_alts[best].interval;

    syslog(LOG_ERR, "uvc_start_streaming: selected alt=%d ep=%02x "
           "maxpkt=%u mult=%u bw=%u for payload=%u\n",
           priv->vs_alt, priv->ep_addr,
           priv->ep_maxpacketsize & 0x7ff,
           priv->ep_mult, priv->vs_alts[best].bandwidth,
           (unsigned int)priv->max_payload_size);

    /* Allocate the isoc IN endpoint for the chosen alt setting. */

    if (priv->ep_stream != NULL)
      {
        DRVR_EPFREE(priv->usbclass.hport->drvr, priv->ep_stream);
        priv->ep_stream = NULL;
      }

    {
      struct usbhost_epdesc_s epdesc;

      epdesc.hport        = priv->usbclass.hport;
      epdesc.addr         = priv->ep_addr & USB_EP_ADDR_NUMBER_MASK;
      epdesc.in           = true;
      epdesc.xfrtype      = USB_EP_ATTR_XFER_ISOC;
      epdesc.interval     = priv->ep_interval;
      epdesc.mxpacketsize = priv->ep_maxpacketsize;

      ret = DRVR_EPALLOC(priv->usbclass.hport->drvr, &epdesc,
                         &priv->ep_stream);
      if (ret < 0)
        {
          syslog(LOG_ERR, "uvc_start_streaming: EPALLOC failed %d\n", ret);
          return ret;
        }
    }
  }

  /* Clear any STALL condition on EP0 left by failed probe/commit */

  syslog(LOG_ERR, "uvc_start_streaming: calling clear_halt\n");
  usbhost_uvc_clear_halt(priv);

  /* Select the streaming alt setting to enable the endpoint */

  syslog(LOG_ERR, "uvc_start_streaming: calling SET_INTERFACE if=%d alt=%d\n",
         priv->vs_ifno, priv->vs_alt);
  ret = usbhost_uvc_set_interface(priv, priv->vs_ifno, priv->vs_alt);
  syslog(LOG_ERR, "uvc_start_streaming: SET_INTERFACE returned %d\n", ret);

  if (ret < 0)
    {
      syslog(LOG_ERR, "uvc_start_streaming: SET_INTERFACE failed (%d), "
             "retrying\n", ret);
      usbhost_uvc_clear_halt(priv);
      ret = usbhost_uvc_set_interface(priv, priv->vs_ifno, priv->vs_alt);
      syslog(LOG_ERR, "uvc_start_streaming: SET_INTERFACE retry "
             "returned %d\n", ret);
    }

  priv->state = UVC_STATE_STREAMING;
  syslog(LOG_ERR, "uvc_start_streaming: streaming state set, "
         "signaling stream thread\n");

  /* Give the device time to initialize sensor and start streaming */

  nxsig_usleep(50000);  /* 50ms */

  /* Signal the stream thread that the endpoint is ready */

  nxsem_post(&priv->stream_sem);

  return OK;
}

static int usbhost_uvc_stop_streaming(FAR struct usbhost_uvc_s *priv)
{
  int ret;

  uinfo("Stopping UVC streaming...\n");

  /* Disable streaming endpoint */

  ret = usbhost_uvc_set_interface(priv, priv->vs_ifno, 0);
  if (ret < 0)
    {
      uwarn("WARNING: Failed to reset streaming interface: %d\n", ret);
    }

  priv->state = UVC_STATE_CONNECTED;
  priv->streaming = false;

  uinfo("UVC streaming stopped\n");
  return OK;
}

/****************************************************************************
 * Private Functions - Streaming Thread
 ****************************************************************************/

static void usbhost_uvc_process_payload(FAR struct usbhost_uvc_s *priv,
                                        FAR const uint8_t *payload,
                                        size_t len)
{
  FAR const struct uvc_payload_header_s *header;
  uint8_t header_len;
  size_t data_len;
  uint8_t fid;

  if (len < 2)
    {
      return;
    }

  header = (FAR const struct uvc_payload_header_s *)payload;
  header_len = header->bHeaderLength;
  fid = header->bmHeaderInfo & UVC_HEADER_FID;

  if (header_len > len)
    {
      return;
    }

  data_len = len - header_len;

  /* Frame delimiting — EOF only (MJPEG mode).
   * FID is ignored (this camera flips FID mid-frame).
   * Deliver on EOF with minimal size gate (1KB) to filter empty frames.
   */

  UNUSED(fid);

  if (priv->frame_offset + data_len <= priv->frame_buflen)
    {
      memcpy(priv->frame_buf + priv->frame_offset,
             payload + header_len, data_len);
      priv->frame_offset += data_len;
    }

  if (header->bmHeaderInfo & UVC_HEADER_EOF)
    {
      if (priv->frame_offset >= 1024 &&
          priv->capture_cb != NULL && priv->user_buf != NULL)
        {
          size_t copy_len = priv->frame_offset;
          if (copy_len > priv->user_buflen)
            copy_len = priv->user_buflen;
          memcpy(priv->user_buf, priv->frame_buf, copy_len);
          priv->capture_cb(0, copy_len, 0, priv->capture_arg);
        }
      priv->frame_offset = 0;
    }

  /* (4) Max-size safety: if we somehow reach the full frame size without a
   * boundary marker, deliver to prevent overrunning into the next frame.
   */

  if (priv->max_frame_size > 0 &&
      priv->frame_offset >= priv->max_frame_size)
    {
      if (priv->capture_cb != NULL && priv->user_buf != NULL)
        {
          size_t copy_len = priv->max_frame_size;
          if (copy_len > priv->user_buflen)
            {
              copy_len = priv->user_buflen;
            }

          memcpy(priv->user_buf, priv->frame_buf, copy_len);
          priv->capture_cb(0, copy_len, 0, priv->capture_arg);
        }

      priv->frame_offset = 0;
    }
}

static int usbhost_uvc_stream_thread(int argc, FAR char *argv[])
{
  FAR struct usbhost_uvc_s *priv;
  FAR struct usbhost_hubport_s *hport;
  FAR struct usbhost_driver_s *drvr;
  ssize_t nbytes;

  priv = (FAR struct usbhost_uvc_s *)
    ((uintptr_t)strtoul(argv[1], NULL, 16));

  DEBUGASSERT(priv != NULL);
  hport = priv->usbclass.hport;
  DEBUGASSERT(hport != NULL && hport->drvr != NULL);
  drvr = hport->drvr;

  syslog(LOG_ERR, "UVC stream thread started, waiting for endpoint\n");

  /* Wait for SET_INTERFACE to enable the streaming endpoint */

  nxsem_wait_uninterruptible(&priv->stream_sem);

  syslog(LOG_ERR, "UVC stream thread: endpoint ready, starting transfers\n");

  while (!priv->disconnected && priv->streaming)
    {
      /* Receive isochronous data */

      nbytes = DRVR_TRANSFER(drvr, priv->ep_stream,
                             priv->xfer_buf, priv->xfer_buflen);

      if (nbytes > 0)
        {
          /* The EHCI ISOC driver returns a batch of payloads framed as
           * [u16 LE length][payload bytes] repeated.  Split them back apart
           * and process each individually — process_payload expects one
           * payload (with its own 12-byte UVC header) at a time.  Batching
           * is what lets the consumer keep up with the controller's fill
           * rate; see the serve logic in ehci-driver.c sunxi_ioc_wait().
           */

          size_t off = 0;

          while (off + 2 <= (size_t)nbytes)
            {
              uint16_t plen = (uint16_t)priv->xfer_buf[off] |
                              ((uint16_t)priv->xfer_buf[off + 1] << 8);
              off += 2;

              if (plen == 0 || plen > 800 || off + plen > (size_t)nbytes)
                {
                  /* Truncated/garbage framing, or a length above the ISOC
                   * payload cap (ISOC_FIFO_SLOTSZ=800 in ehci-driver.c) —
                   * stop to avoid misreading the rest of the batch.
                   */

                  break;
                }

              usbhost_uvc_process_payload(priv, priv->xfer_buf + off, plen);
              off += plen;
            }
        }
      else if (nbytes < 0)
        {
          if (nbytes == -EAGAIN || nbytes == -ETIMEDOUT)
            {
              nxsig_usleep(1000);
              continue;
            }

          syslog(LOG_ERR, "UVC stream: transfer error %zd\n", nbytes);

          if (nbytes == -ENODEV || nbytes == -EPIPE)
            {
              break;
            }

          /* On transient errors (EIO/XACTERR), wait before retrying
           * to avoid overwhelming the device.
           */

          nxsig_usleep(5000);  /* 5ms */
        }
    }

  syslog(LOG_ERR, "UVC stream thread exiting\n");
  priv->stream_tid = 0;
  return OK;
}

/****************************************************************************
 * Private Functions - Video Subsystem Callbacks (imgsensor)
 ****************************************************************************/

static bool uvc_sensor_is_available(FAR struct imgsensor_s *sensor)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(sensor, struct usbhost_uvc_s, imgsensor);

  return priv->state == UVC_STATE_CONNECTED ||
         priv->state == UVC_STATE_STREAMING;
}

static int uvc_sensor_init(FAR struct imgsensor_s *sensor)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(sensor, struct usbhost_uvc_s, imgsensor);

  uinfo("UVC sensor init\n");

  /* Allocate frame assembly buffer */

  if (priv->frame_buf == NULL)
    {
      priv->frame_buflen = priv->max_frame_size > 0 ?
        priv->max_frame_size : 256 * 1024;  /* Default 256KB */

      priv->frame_buf = kmm_memalign(UVC_BUFFER_ALIGN,
                                     priv->frame_buflen);
      if (priv->frame_buf == NULL)
        {
          uerr("ERROR: Failed to allocate frame buffer\n");
          return -ENOMEM;
        }
    }

  return OK;
}

static int uvc_sensor_uninit(FAR struct imgsensor_s *sensor)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(sensor, struct usbhost_uvc_s, imgsensor);

  uinfo("UVC sensor uninit\n");

  if (priv->frame_buf != NULL)
    {
      kmm_free(priv->frame_buf);
      priv->frame_buf = NULL;
    }

  return OK;
}

static FAR const char *
  uvc_sensor_get_driver_name(FAR struct imgsensor_s *sensor)
{
  return "UVC Camera";
}

static int uvc_sensor_validate_frame_setting(
  FAR struct imgsensor_s *sensor,
  imgsensor_stream_type_t type,
  uint8_t nr_datafmt,
  FAR imgsensor_format_t *datafmts,
  FAR imgsensor_interval_t *interval)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(sensor, struct usbhost_uvc_s, imgsensor);

  uinfo("Validate frame setting: type=%d nr=%d\n", type, nr_datafmt);

  /* Check if device is connected */

  if (priv->state == UVC_STATE_DISCONNECTED)
    {
      return -ENODEV;
    }

  /* Log the requested format for debugging */

  if (datafmts != NULL && nr_datafmt > 0)
    {
      syslog(LOG_INFO, "uvc_sensor_validate: width=%d height=%d format=%d\n",
             datafmts[0].width, datafmts[0].height, datafmts[0].pixelformat);
    }

  /* Accept the format - UVC camera supports it */

  return OK;
}

static int uvc_sensor_start_capture(FAR struct imgsensor_s *sensor,
                                    imgsensor_stream_type_t type,
                                    uint8_t nr_datafmt,
                                    FAR imgsensor_format_t *datafmts,
                                    FAR imgsensor_interval_t *interval)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(sensor, struct usbhost_uvc_s, imgsensor);
  int ret;

  syslog(LOG_INFO, "uvc_sensor_start_capture: type=%d state=%d\n",
         type, priv->state);

  if (priv->state != UVC_STATE_CONNECTED)
    {
      syslog(LOG_ERR, "uvc_sensor_start_capture: invalid state=%d\n",
             priv->state);
      return -EINVAL;
    }

  /* Update format settings from request */

  if (datafmts != NULL && nr_datafmt > 0)
    {
      priv->cur_format.width = datafmts[0].width;
      priv->cur_format.height = datafmts[0].height;
    }

  if (interval != NULL)
    {
      priv->cur_format.frame_interval =
        (uint32_t)(interval->numerator * 10000000 /
                   interval->denominator);
    }

  /* Start USB streaming */

  syslog(LOG_INFO, "uvc_sensor_start_capture: starting UVC streaming\n");
  ret = usbhost_uvc_start_streaming(priv);
  if (ret < 0)
    {
      syslog(LOG_ERR, "uvc_sensor_start_capture: Failed to start streaming: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "uvc_sensor_start_capture: UVC streaming started\n");
  return OK;
}

static int uvc_sensor_stop_capture(FAR struct imgsensor_s *sensor,
                                   imgsensor_stream_type_t type)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(sensor, struct usbhost_uvc_s, imgsensor);

  uinfo("Stop capture: type=%d\n", type);

  if (priv->state == UVC_STATE_STREAMING)
    {
      return usbhost_uvc_stop_streaming(priv);
    }

  return OK;
}

/****************************************************************************
 * Private Functions - Video Subsystem Callbacks (imgdata)
 ****************************************************************************/

static int uvc_data_init(FAR struct imgdata_s *data)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(data, struct usbhost_uvc_s, imgdata);

  uinfo("UVC data init\n");

  /* Allocate USB transfer buffer */

  if (priv->xfer_buf == NULL)
    {
      priv->xfer_buflen = CONFIG_USBHOST_UVC_BUFSIZE;
      priv->xfer_buf = kmm_memalign(UVC_BUFFER_ALIGN,
                                    priv->xfer_buflen);
      if (priv->xfer_buf == NULL)
        {
          uerr("ERROR: Failed to allocate transfer buffer\n");
          return -ENOMEM;
        }
    }

  return OK;
}

static int uvc_data_uninit(FAR struct imgdata_s *data)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(data, struct usbhost_uvc_s, imgdata);

  uinfo("UVC data uninit\n");

  if (priv->xfer_buf != NULL)
    {
      kmm_free(priv->xfer_buf);
      priv->xfer_buf = NULL;
    }

  return OK;
}

static int uvc_data_validate_frame_setting(
  FAR struct imgdata_s *data,
  uint8_t nr_datafmt,
  FAR imgdata_format_t *datafmt,
  FAR imgdata_interval_t *interval)
{
  uinfo("UVC data validate frame setting\n");

  /* Log the requested format for debugging */

  if (datafmt != NULL && nr_datafmt > 0)
    {
      syslog(LOG_INFO, "uvc_data_validate: width=%d height=%d format=%d\n",
             datafmt[0].width, datafmt[0].height, datafmt[0].pixelformat);
    }

  return OK;
}

static int uvc_data_start_capture(FAR struct imgdata_s *data,
                                  uint8_t nr_datafmt,
                                  FAR imgdata_format_t *datafmt,
                                  FAR imgdata_interval_t *interval,
                                  imgdata_capture_t callback,
                                  FAR void *arg)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(data, struct usbhost_uvc_s, imgdata);
  FAR char *argv[2];
  char arg_str[16];
  int ret;

  syslog(LOG_INFO, "uvc_data_start_capture: Starting data capture\n");

  /* Save callback */

  priv->capture_cb = callback;
  priv->capture_arg = arg;
  priv->frame_offset = 0;
  priv->frame_id = 0;
  priv->asm_lastfid = 0xff;

  /* Start streaming thread */

  priv->streaming = true;

  snprintf(arg_str, sizeof(arg_str), "%p", priv);
  argv[0] = arg_str;
  argv[1] = NULL;

  ret = kthread_create("uvc_stream",
                       CONFIG_USBHOST_UVC_DEFPRIO,
                       CONFIG_USBHOST_UVC_STACKSIZE,
                       usbhost_uvc_stream_thread,
                       argv);
  if (ret < 0)
    {
      uerr("ERROR: Failed to create stream thread: %d\n", ret);
      priv->streaming = false;
      return ret;
    }

  priv->stream_tid = ret;
  uinfo("Stream thread created: pid=%d\n", ret);

  return OK;
}

static int uvc_data_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(data, struct usbhost_uvc_s, imgdata);

  uinfo("UVC data stop capture\n");

  /* Signal thread to stop */

  priv->streaming = false;
  priv->capture_cb = NULL;
  priv->capture_arg = NULL;

  /* Wait for thread to exit */

  if (priv->stream_tid > 0)
    {
      /* Give thread time to exit */

      nxsig_usleep(100000);  /* 100ms */
    }

  return OK;
}

static int uvc_data_set_buf(FAR struct imgdata_s *data,
                            uint8_t nr_datafmts,
                            FAR imgdata_format_t *datafmts,
                            FAR uint8_t *addr,
                            uint32_t size)
{
  FAR struct usbhost_uvc_s *priv =
    container_of(data, struct usbhost_uvc_s, imgdata);

  priv->user_buf = addr;
  priv->user_buflen = size;

  return OK;
}

/****************************************************************************
 * Private Functions - USB Host Class Methods
 ****************************************************************************/

static FAR struct usbhost_class_s *
  usbhost_uvc_create(FAR struct usbhost_hubport_s *hport,
                     FAR const struct usbhost_id_s *id)
{
  FAR struct usbhost_uvc_s *priv;

  syslog(LOG_INFO, "usbhost_uvc_create: Creating UVC instance\n");
  syslog(LOG_INFO, "usbhost_uvc_create: Device class=%d, subclass=%d, proto=%d\n",
         id->base, id->subclass, id->proto);

  /* Allocate a new UVC class instance */

  priv = usbhost_uvc_allocclass();
  if (priv == NULL)
    {
      syslog(LOG_ERR, "usbhost_uvc_create: Failed to allocate class instance\n");
      return NULL;
    }

  syslog(LOG_INFO, "usbhost_uvc_create: Allocated instance: %p\n", priv);

  /* Initialize the instance */

  memset(priv, 0, sizeof(struct usbhost_uvc_s));
  priv->state = UVC_STATE_DISCONNECTED;

  /* Assign a device number */

  if (usbhost_uvc_allocdevno(priv) != OK)
    {
      usbhost_uvc_freeclass(priv);
      return NULL;
    }

  /* Set USB class interface methods */

  priv->usbclass.hport = hport;
  priv->usbclass.connect = usbhost_uvc_connect;
  priv->usbclass.disconnected = usbhost_uvc_disconnected;

  /* Initial reference count */

  priv->crefs = 1;

  /* Initialize synchronization */

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->stream_sem, 0, 0);

  /* Initialize video subsystem interfaces */

  priv->imgsensor.ops = &g_uvc_sensor_ops;
  priv->imgsensor.fmtdescs = g_uvc_fmtdescs;
  priv->imgsensor.fmtdescs_num = 1;
  priv->imgsensor.frmsizes = g_uvc_frmsizes;
  priv->imgsensor.frmsizes_num = 2;
  priv->imgsensor.frmintervals = g_uvc_frmintervals;
  priv->imgsensor.frmintervals_num = 1;

  priv->imgdata.ops = &g_uvc_data_ops;

  /* Set default format.
   *
   * NOTE (root-cause verification): this camera's format 1 is MJPEG and its
   * frame index 1 is 2560x1440 — far too much bandwidth for the 128-byte
   * isoc endpoint, so committing frame 1 made the camera refuse to stream
   * (every isoc IN returned XACTERR).  Frame index 7 is 320x240.  This is a
   * temporary hardcode to confirm the diagnosis; the real fix is to parse
   * the VS frame descriptors and select the index matching the requested
   * width/height.
   */

  priv->format_idx = 1;   /* MJPEG (bFormatIndex=1, 320x240=frame 7) */
  priv->frame_idx = 7;   /* 320x240 */
  priv->cur_format.fourcc = V4L2_PIX_FMT_JPEG;
  priv->cur_format.width = UVC_DEFAULT_WIDTH;
  priv->cur_format.height = UVC_DEFAULT_HEIGHT;
  priv->cur_format.frame_interval = UVC_INTERVAL_30FPS;

  uinfo("UVC instance created: devno=%d\n", priv->devno);

  return &priv->usbclass;
}

static int usbhost_uvc_connect(FAR struct usbhost_class_s *usbclass,
                               FAR const uint8_t *configdesc,
                               int desclen)
{
  FAR struct usbhost_uvc_s *priv = (FAR struct usbhost_uvc_s *)usbclass;
  char devpath[16];
  int ret;

  DEBUGASSERT(priv != NULL && configdesc != NULL &&
              desclen >= sizeof(struct usb_cfgdesc_s));

  syslog(LOG_INFO, "usbhost_uvc_connect: Connecting UVC device\n");
  syslog(LOG_INFO, "usbhost_uvc_connect: Config descriptor length: %d\n", desclen);

  /* Parse configuration descriptor */

  ret = usbhost_uvc_parse_config(priv, configdesc, desclen);
  if (ret < 0)
    {
      syslog(LOG_ERR, "usbhost_uvc_connect: Failed to parse config descriptor: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "usbhost_uvc_connect: Config descriptor parsed successfully\n");

  /* Increment reference count */

  priv->crefs++;
  DEBUGASSERT(priv->crefs == 2);

  /* Mark device as connected */

  priv->state = UVC_STATE_CONNECTED;

  /* Register video device using imgdata_register and imgsensor_register */

  syslog(LOG_INFO, "usbhost_uvc_connect: Registering UVC sensor and data\n");

  imgdata_register(&priv->imgdata);
  syslog(LOG_INFO, "usbhost_uvc_connect: imgdata registered\n");

  imgsensor_register(&priv->imgsensor);
  syslog(LOG_INFO, "usbhost_uvc_connect: imgsensor registered\n");

  syslog(LOG_INFO, "usbhost_uvc_connect: UVC sensor and data registered successfully\n");
  uinfo("  VC interface: %d\n", priv->vc_ifno);
  uinfo("  VS interface: %d\n", priv->vs_ifno);
  uinfo("  Endpoint: addr=%02x maxpkt=%d\n",
        priv->ep_addr, priv->ep_maxpacketsize);

  /* Check for disconnect during initialization */

  nxmutex_lock(&priv->lock);
  if (priv->crefs <= 2 && priv->disconnected)
    {
      ret = -ENODEV;
    }
  else
    {
      priv->crefs--;
    }

  nxmutex_unlock(&priv->lock);

  return ret;

errout:
  priv->crefs--;
  return ret;
}

static int usbhost_uvc_disconnected(FAR struct usbhost_class_s *usbclass)
{
  FAR struct usbhost_uvc_s *priv = (FAR struct usbhost_uvc_s *)usbclass;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  uinfo("UVC disconnected\n");

  /* Stop streaming if active */

  if (priv->state == UVC_STATE_STREAMING)
    {
      priv->streaming = false;
      usbhost_uvc_stop_streaming(priv);
    }

  /* Mark as disconnected */

  flags = enter_critical_section();
  priv->disconnected = true;
  priv->state = UVC_STATE_DISCONNECTED;

  /* Check if we can free now */

  if (priv->crefs == 1)
    {
      if (up_interrupt_context())
        {
          /* Defer cleanup to worker thread */

          work_queue(HPWORK, &priv->work,
                    (worker_t)usbhost_uvc_freeclass, priv, 0);
        }
      else
        {
          /* Free immediately */

          /* Free endpoints */

          if (priv->ep_stream != NULL)
            {
              DRVR_EPFREE(priv->usbclass.hport->drvr, priv->ep_stream);
            }

          /* Free buffers */

          if (priv->xfer_buf != NULL)
            {
              kmm_free(priv->xfer_buf);
            }

          if (priv->frame_buf != NULL)
            {
              kmm_free(priv->frame_buf);
            }

          /* Free device address */

          usbhost_devaddr_destroy(priv->usbclass.hport,
                                 priv->usbclass.hport->funcaddr);

          /* Disconnect from host controller */

          DRVR_DISCONNECT(priv->usbclass.hport->drvr,
                         priv->usbclass.hport);

          /* Release device number */

          usbhost_uvc_freedevno(priv);

          /* Free the class instance */

          usbhost_uvc_freeclass(priv);
        }
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_uvc_init
 *
 * Description:
 *   Initialize the USB Video Class host driver. This function should be
 *   called from board-specific initialization code to register support
 *   for UVC devices.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int usbhost_uvc_init(void)
{
  uinfo("Initializing UVC host driver\n");

  /* Register the UVC class driver */

  return usbhost_registerclass(&g_uvc_registry);
}
