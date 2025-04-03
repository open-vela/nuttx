/****************************************************************************
 * drivers/audio/audio_tunnel.c
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

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_tunnel.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define AUDIO_TUNNEL_STATE_IDLE     -1
#define AUDIO_TUNNEL_STATE_CONFIGED  0
#define AUDIO_TUNNEL_STATE_STOPPING  1
#define AUDIO_TUNNEL_STATE_STOPPED   2
#define AUDIO_TUNNEL_STATE_STARTING  3
#define AUDIO_TUNNEL_STATE_STARTED   4
#define AUDIO_TUNNEL_STATE_UNDERFLOW 5

#define AUDIO_TUNNEL_ROLE_PRODUCER   0
#define AUDIO_TUNNEL_ROLE_CONSUMER   1

#define AUDIO_TUNNEL_MSG_P2C         (1 << 15)
#define AUDIO_TUNNEL_MSG_C2P         (1 << 16)

struct audio_tunnel_s;

struct audio_peer_s
{
  struct audio_lowerhalf_s dev;  /* Audio lower half (this device) */
  FAR struct audio_tunnel_s *parent;
  struct dq_queue_s dataq;
  int state;                     /* Meet state machine request */
  int role;                      /* consumer:1 producer:0 */
};

struct audio_tunnel_s
{
  mutex_t     mutex;

  struct      audio_info_s     info;  /* Formats */
  struct      ap_buffer_info_s binfo;

  struct audio_peer_s producer;    /* Peer which produce data */
  struct audio_peer_s consumer;    /* Peer which cosume data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int audio_tunnel_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_reserve(FAR struct audio_lowerhalf_s *dev,
                                FAR void **session);
static int audio_tunnel_configure(FAR struct audio_lowerhalf_s *dev,
                                  FAR void *session,
                                  FAR const struct audio_caps_s *caps);
static int audio_tunnel_start(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
static int audio_tunnel_release(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session);

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_tunnel_stop(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_tunnel_pause(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session);
static int audio_tunnel_resume(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session);
#endif
#else
static int audio_tunnel_reserve(FAR struct audio_lowerhalf_s *dev);
static int audio_tunnel_configure(FAR struct audio_lowerhalf_s *dev,
                                  FAR const struct audio_caps_s *caps);
static int audio_tunnel_start(FAR struct audio_lowerhalf_s *dev);
static int audio_tunnel_release(FAR struct audio_lowerhalf_s *dev);

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
static int audio_tunnel_stop(FAR struct audio_lowerhalf_s *dev);
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
static int audio_tunnel_pause(FAR struct audio_lowerhalf_s *dev);
static int audio_tunnel_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int audio_tunnel_shutdown(FAR struct audio_lowerhalf_s *dev);
static int audio_tunnel_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                      FAR struct ap_buffer_s *apb);
static int audio_tunnel_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                     FAR struct ap_buffer_s *apb);
static int audio_tunnel_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audio_tunnel_ops =
{
    audio_tunnel_getcaps,         /* getcaps        */
    audio_tunnel_configure,       /* configure      */
    audio_tunnel_shutdown,        /* shutdown       */
    audio_tunnel_start,           /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
    audio_tunnel_stop,            /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
    audio_tunnel_pause,           /* pause          */
    audio_tunnel_resume,          /* resume         */
#endif
    NULL,                         /* allocbuffer    */
    NULL,                         /* freebuffer     */
    audio_tunnel_enqueuebuffer,   /* enqueue_buffer */
    audio_tunnel_cancelbuffer,    /* cancel_buffer  */
    audio_tunnel_ioctl,           /* ioctl          */
    NULL,                         /* read           */
    NULL,                         /* write          */
    audio_tunnel_reserve,         /* reserve        */
    audio_tunnel_release          /* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void audio_tunnel_notify_dqmsg(FAR struct audio_tunnel_s *tunnel,
                                      FAR struct ap_buffer_s *src,
                                      FAR struct ap_buffer_s *dst)
{
  FAR struct audio_peer_s *consumer = &tunnel->consumer;
  FAR struct audio_peer_s *producer = &tunnel->producer;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  consumer->dev.upper(consumer->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                      dst, OK, NULL);

  producer->dev.upper(producer->dev.priv, AUDIO_CALLBACK_DEQUEUE,
                      src, OK, NULL)
#else
  consumer->dev.upper(consumer->dev.priv, AUDIO_CALLBACK_DEQUEUE, dst, OK);
  producer->dev.upper(producer->dev.priv, AUDIO_CALLBACK_DEQUEUE, src, OK);
#endif
}

static int audio_tunnel_buffer_deliver(FAR struct audio_tunnel_s *tunnel,
                                       int msg_id)
{
  FAR struct audio_peer_s *consumer = &tunnel->consumer;
  FAR struct audio_peer_s *producer = &tunnel->producer;
  FAR struct ap_buffer_s *src;
  FAR struct ap_buffer_s *dst;

  nxmutex_lock(&tunnel->mutex);
  if (dq_empty(&producer->dataq))
    {
      /* producer underflow */

      if (msg_id == AUDIO_MSG_DEQUEUE)
        {
          producer->state = AUDIO_TUNNEL_STATE_UNDERFLOW;
        }

      nxmutex_unlock(&tunnel->mutex);
      goto out;
    }
  else if (dq_empty(&consumer->dataq))
    {
      nxmutex_unlock(&tunnel->mutex);
      goto out;
    }

  src = (FAR struct ap_buffer_s *)dq_remfirst(&producer->dataq);
  dst = (FAR struct ap_buffer_s *)dq_remfirst(&consumer->dataq);

  if (producer->state == AUDIO_TUNNEL_STATE_UNDERFLOW)
    {
      producer->state = AUDIO_TUNNEL_STATE_STARTED;
    }

  nxmutex_unlock(&tunnel->mutex);

  memcpy(dst->samp, src->samp, src->nbytes);
  dst->nbytes = src->nbytes;

  audio_tunnel_notify_dqmsg(tunnel, src, dst);

out:
  return OK;
}

/****************************************************************************
 * Name: audio_tunnel_state_transition
 *
 * Description:
 *   Transite the peer device state among following state machine.
 *
 * State Machine:
 *   peer1 or peer2 represents the producer and consumer, respectively.
 *
 *                 initialize
 *                     |
 *                     V
 *                +---------+                      +-----------+
 *                | IDLE    |---- configure ---->  | CONFIGURED|
 *                +---------+                      +-----------+
 *               ^   ^                                   |
 *              /    |                               peer1_start
 *             /     |                                   |
 *            /      |                                   v
 *  +---------+      |       +-----------+         +-----------+
 *  | STOPPED |  peer2_stop  | UNDERFLOW |  <      | STARTING  |
 *  +---------+      |       +-----------+ \ \     +-----------+
 *           ^       |          /           \ \          |
 *            \      |    peer1_stop         \ \     peer2_start
 *             \     |        /               \ \        |
 *              \    |       v                 \ \       V
 *               \+---------+                   >  +-----------+
 *                | STOPPING|<---- peer1_stop ---- | STARTED   |
 *                +---------+                      +-----------+
 ****************************************************************************/

static int audio_tunnel_state_transition(FAR struct audio_peer_s *peer,
                                         int reason)
{
  FAR struct audio_tunnel_s *tunnel = peer->parent;
  int count;
  int dst;

  switch (reason)
    {
      case AUDIOIOC_CONFIGURE:

          if (peer->state == AUDIO_TUNNEL_STATE_IDLE)
            {
              dst = AUDIO_TUNNEL_STATE_CONFIGED;
            }
          break;

      case AUDIOIOC_START:
      case AUDIO_MSG_START:

          if (peer->state == AUDIO_TUNNEL_STATE_CONFIGED)
            {
              dst = AUDIO_TUNNEL_STATE_STARTING;
            }
          else if (peer->state == AUDIO_TUNNEL_STATE_STARTING)
            {
              dst = AUDIO_TUNNEL_STATE_STARTED;
            }
          else
            return -ENOTTY;

          break;

      case AUDIOIOC_STOP:
      case AUDIO_MSG_STOP:

          if (peer->state == AUDIO_TUNNEL_STATE_STARTED ||
              peer->state == AUDIO_TUNNEL_STATE_UNDERFLOW)
            {
              dst = AUDIO_TUNNEL_STATE_STOPPING;
            }
          else if (peer->state == AUDIO_TUNNEL_STATE_STOPPING)
            {
              dst = AUDIO_TUNNEL_STATE_STOPPED;
            }
          else
            return -ENOTTY;

          break;
    }

    if (dst == AUDIO_TUNNEL_STATE_STOPPED)
      {
        peer->dev.upper(peer->dev.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
        dst = AUDIO_TUNNEL_STATE_IDLE;
      }
    else if (dst == AUDIO_TUNNEL_STATE_STARTED)
      {
        if (peer->role == AUDIO_TUNNEL_ROLE_CONSUMER)
          {
            nxmutex_lock(&tunnel->mutex);
            count = dq_count(&peer->dataq);
            nxmutex_unlock(&tunnel->mutex);

            if (count > 0)
              {
                audio_tunnel_buffer_deliver(tunnel, AUDIO_MSG_DEQUEUE);
              }
          }
      }

    nxmutex_lock(&tunnel->mutex);
    peer->state = dst;
    nxmutex_unlock(&tunnel->mutex);

    return OK;
}

static int audio_tunnel_msg_deliver(FAR struct audio_peer_s *peer,
                                    struct audio_msg_s msg)
{
  FAR struct audio_tunnel_s *tunnel = peer->parent;

  /* Process the message */

  switch (msg.msg_id)
    {
      case AUDIO_MSG_START:
      case AUDIO_MSG_STOP:
        peer = msg.u.data == AUDIO_TUNNEL_MSG_P2C ?
                             &tunnel->consumer :
                             &tunnel->producer;

        audio_tunnel_state_transition(peer, msg.msg_id);

        peer->dev.upper(peer->dev.priv, AUDIO_CALLBACK_MESSAGE,
                        (FAR struct ap_buffer_s *)&msg, OK);
        break;

      default:
        auderr("ERROR: Ignoring message ID %d\n", msg.msg_id);
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: audio_tunnel_getcaps
 *
 * Description:
 *   Get the audio capabilities from stub peer.
 *
 ****************************************************************************/

static int audio_tunnel_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                                FAR struct audio_caps_s *caps)
{
  FAR struct audio_peer_s *peer = (struct audio_peer_s *)dev;
  FAR struct audio_tunnel_s *tunnel = peer->parent;

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:

        caps->ac_channels = tunnel->info.channels;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:
              caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;

              if (tunnel->info.subformat == 0)
                return -ENOSYS;

              caps->ac_format.hw = (1 << (tunnel->info.subformat - 1));
              break;
            case AUDIO_FMT_MP3:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_PCM_MP3;
              caps->ac_controls.b[1] = AUDIO_SUBFMT_END;
              break;
            case AUDIO_FMT_PCM:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_PCM_S16_LE;
              caps->ac_controls.b[1] = AUDIO_SUBFMT_END;
              break;
            case AUDIO_FMT_SBC:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_SBC;
              caps->ac_controls.b[1] = AUDIO_SUBFMT_END;
              break;
            default:
              caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
              break;
          }

        break;

      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:
        caps->ac_channels = tunnel->info.channels;

        switch (caps->ac_subtype)
          {
            case AUDIO_TYPE_QUERY:

              /* Report the Sample rates we support */

              caps->ac_controls.hw[0] =
                tunnel->info.samplerate == 8000 ? AUDIO_SAMP_RATE_8K:
                tunnel->info.samplerate == 16000 ? AUDIO_SAMP_RATE_16K:
                tunnel->info.samplerate == 32000 ? AUDIO_SAMP_RATE_32K :
                tunnel->info.samplerate == 44100 ? AUDIO_SAMP_RATE_44K:
                tunnel->info.samplerate == 48000 ? AUDIO_SAMP_RATE_48K :
                AUDIO_SAMP_RATE_DEF_ALL;
              break;

            default:
              break;
          }

        break;

      default:
        caps->ac_subtype = 0;
        caps->ac_channels = 0;
        break;
    }

  /* Return the length of the audio_caps_s struct for validation of
   * proper Audio device type.
   */

  audinfo("role:%d Return %d\n", peer->role, caps->ac_len);
  return caps->ac_len;
}

/****************************************************************************
 * Name: audio_tunnel_configure
 *
 * Description:
 *   Configure the driver.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_configure(FAR struct audio_lowerhalf_s *dev,
                                  FAR void *session,
                                  FAR const struct audio_caps_s *caps)
#else
static int audio_tunnel_configure(FAR struct audio_lowerhalf_s *dev,
                                  FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct audio_peer_s *peer = (struct audio_peer_s *)dev;
  FAR struct audio_tunnel_s *tunnel = peer->parent;

  audinfo("role:%d ac_type: %d\n", peer->role, caps->ac_type);

  /* Process the configure operation */

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_OUTPUT:
      case AUDIO_TYPE_INPUT:

        tunnel->info.format = caps->ac_subtype;
        tunnel->info.samplerate = caps->ac_controls.hw[0] |
                                  (caps->ac_controls.b[3] << 16);
        tunnel->info.channels = caps->ac_channels;
        tunnel->info.subformat = caps->ac_subtype;

        audinfo("Audio type: %s\n", (caps->ac_type == AUDIO_TYPE_OUTPUT)
                                        ? "AUDIO_TYPE_OUTPUT"
                                        : "AUDIO_TYPE_INPUT");
        syslog(0, "Codec type %u %d %d\n",
                  caps->ac_subtype,
                  tunnel->info.samplerate,
                  tunnel->info.channels);
        break;

      default:
        audinfo("default case: %d\n", caps->ac_type);
        break;
    }

  audio_tunnel_state_transition(peer, AUDIOIOC_CONFIGURE);
  return OK;
}

/****************************************************************************
 * Name: audio_tunnel_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int audio_tunnel_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: audio_tunnel_start
 *
 * Description:
 *   Start the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_start(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session)
#else
static int audio_tunnel_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_peer_s *peer = (FAR struct audio_peer_s *)dev;
  struct audio_msg_s msg;
  int ret;

  msg.msg_id = AUDIO_MSG_START;
  msg.u.data = peer->role == AUDIO_TUNNEL_ROLE_PRODUCER ?
                             AUDIO_TUNNEL_MSG_P2C :
                             AUDIO_TUNNEL_MSG_C2P;

  ret = audio_tunnel_msg_deliver(peer, msg);
  if (ret == 0)
    {
      audio_tunnel_state_transition(peer, AUDIOIOC_START);
    }

  return 0;
}

/****************************************************************************
 * Name: audio_tunnel_stop
 *
 * Description:
 *   Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_stop(FAR struct audio_lowerhalf_s *dev,
                             FAR void *session)
#else
static int audio_tunnel_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct audio_peer_s *peer = (FAR struct audio_peer_s *)dev;
  struct audio_msg_s msg;
  int ret;

  msg.msg_id  = AUDIO_MSG_STOP;
  msg.u.data = peer->role == AUDIO_TUNNEL_ROLE_PRODUCER ?
                             AUDIO_TUNNEL_MSG_P2C :
                             AUDIO_TUNNEL_MSG_C2P;

  ret = audio_tunnel_msg_deliver(peer, msg);
  if (ret == 0)
    {
      audio_tunnel_state_transition(peer, AUDIOIOC_STOP);
    }

  return ret;
}
#endif

static int audio_producer_enqueuebuffer(FAR struct audio_peer_s *peer,
                                        FAR struct ap_buffer_s *apb)
{
  FAR struct audio_tunnel_s *tunnel = peer->parent;
  int ret = 0;

  /* Invalid state,  dequeue buffer and return ENOTTY */

  if (peer->state == AUDIO_TUNNEL_STATE_IDLE ||
      peer->state == AUDIO_TUNNEL_STATE_CONFIGED ||
      peer->state == AUDIO_TUNNEL_STATE_STOPPING)
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      peer->dev.upper(peer->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
      peer->dev.upper(peer->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif

      return -ENOTTY;
    }

  /* 1. add data queue */

  nxmutex_lock(&tunnel->mutex);
  dq_addlast((FAR dq_entry_t *)apb, &peer->dataq);
  nxmutex_unlock(&tunnel->mutex);

  /* 2. if underflow,  actively send enqueue msg */

  if (peer->state == AUDIO_TUNNEL_STATE_UNDERFLOW)
    {
      ret = audio_tunnel_buffer_deliver(tunnel, AUDIO_MSG_ENQUEUE);
      if (ret < 0)
        {
          auderr("ERROR: %s deliver buffer failed. %s\n",
                 __func__, strerror(ret));
        }
    }

  return ret;
}

static int audio_consumer_enqueuebuffer(FAR struct audio_peer_s *peer,
                                        FAR struct ap_buffer_s *apb)
{
  FAR struct audio_tunnel_s *tunnel = peer->parent;
  struct audio_msg_s msg;
  int ret = 0;

  /* Invalid state,  dequeue buffer and return ENOTTY */

  if (peer->state == AUDIO_TUNNEL_STATE_IDLE ||
      peer->state == AUDIO_TUNNEL_STATE_STOPPING)
    {
      auderr("ERROR: %s invalid state. %d\n", __func__, peer->state);
    }

  /* 1. add data queue */

  nxmutex_lock(&tunnel->mutex);
  dq_addlast((FAR dq_entry_t *)apb, &peer->dataq);
  nxmutex_unlock(&tunnel->mutex);

  if (peer->state == AUDIO_TUNNEL_STATE_STARTED)
    {
      /* Consumer send AUDIO_MSG_DATA_REQUEST msg to producer */

      ret = audio_tunnel_buffer_deliver(tunnel, AUDIO_MSG_DEQUEUE);
      if (ret < 0)
        {
          auderr("ERROR: %s deliver buffer failed. %s\n",
                 __func__, strerror(ret));
        }
    }

  return ret;
}

/****************************************************************************
 * Name: audio_tunnel_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_pause(FAR struct audio_lowerhalf_s *dev,
                              FAR void *session)
#else
static int audio_tunnel_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: audio_tunnel_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_resume(FAR struct audio_lowerhalf_s *dev,
                               FAR void *session)
#else
static int audio_tunnel_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

static int audio_tunnel_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                                      FAR struct ap_buffer_s *apb)
{
  FAR struct audio_peer_s *peer = (FAR struct audio_peer_s *)dev;

  if (peer->role == AUDIO_TUNNEL_ROLE_PRODUCER)
    {
      return audio_producer_enqueuebuffer(peer, apb);
    }
  else
    {
      return audio_consumer_enqueuebuffer(peer, apb);
    }
}

/****************************************************************************
 * Name: audio_tunnel_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int audio_tunnel_cancelbuffer(FAR struct audio_lowerhalf_s *dev,
                                     FAR struct ap_buffer_s *apb)
{
  return OK;
}

/****************************************************************************
 * Name: audio_tunnel_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int audio_tunnel_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                              unsigned long arg)
{
  FAR struct audio_peer_s *peer = (FAR struct audio_peer_s *)dev;
  FAR struct audio_tunnel_s *tunnel = peer->parent;
  int ret  = OK;

  audinfo("cmd=%d arg=%ld\n", cmd, arg);

  /* Deal with ioctls passed from the upper-half driver */

  switch (cmd)
    {
        /* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
         * through from the upper-half audio driver.
         */

      case AUDIOIOC_HWRESET:
        {
          audinfo("AUDIOIOC_HWRESET:\n");
        }
        break;

        /* Report our preferred buffer size and quantity */

      case AUDIOIOC_GETBUFFERINFO:
        {
          struct ap_buffer_info_s *info = (struct ap_buffer_info_s *)arg;
          info->nbuffers    = tunnel->binfo.nbuffers;
          info->buffer_size = tunnel->binfo.buffer_size;
        }
        break;

      case AUDIOIOC_SETBUFFERINFO:
        {
          FAR struct ap_buffer_info_s *info =
            (FAR struct ap_buffer_info_s *)arg;
          tunnel->binfo.nbuffers = info->nbuffers;
          tunnel->binfo.buffer_size = info->buffer_size;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  audinfo("Return OK\n");
  return ret;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_reserve(FAR struct audio_lowerhalf_s *dev,
                                FAR void **session)
#else
static int audio_tunnel_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int audio_tunnel_release(FAR struct audio_lowerhalf_s *dev,
                                FAR void *session)
#else
static int audio_tunnel_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int audio_tunnel_initialize(const FAR char *prefix)
{
  FAR struct audio_tunnel_s *tunnel;
  FAR struct audio_peer_s *producer;
  FAR struct audio_peer_s *consumer;
  char name[16];
  int ret;

  /* Allocate the tunnel audio device structure */

  tunnel = (FAR struct audio_tunnel_s *)kmm_zalloc(
                                          sizeof(struct audio_tunnel_s) +
                                          sizeof(struct audio_peer_s) +
                                          sizeof(struct audio_peer_s));
  if (!tunnel)
    {
      auderr("ERROR: Failed to allocate tunnel audio device\n");
      return -ENOMEM;
    }

  memset(&tunnel->info, 0, sizeof(struct audio_info_s));
  memset(&tunnel->binfo, 0, sizeof(struct ap_buffer_info_s));
  nxmutex_init(&tunnel->mutex);

  producer = &tunnel->producer;
  producer->dev.ops   = &g_audio_tunnel_ops;
  producer->parent    = tunnel;
  producer->role      = AUDIO_TUNNEL_ROLE_PRODUCER;
  producer->state     = AUDIO_TUNNEL_STATE_IDLE;
  dq_init(&producer->dataq);

  snprintf(name, sizeof(name), "%s0p", prefix);
  ret = audio_register(name,  &producer->dev);
  if (ret < 0)
    {
      goto error;
    }

  consumer = &tunnel->consumer;
  consumer->dev.ops   = &g_audio_tunnel_ops;
  consumer->parent    = tunnel;
  consumer->role      = AUDIO_TUNNEL_ROLE_CONSUMER;
  consumer->state     = AUDIO_TUNNEL_STATE_IDLE;
  dq_init(&consumer->dataq);

  memset(name, 0, sizeof(name));
  snprintf(name, sizeof(name), "%s0c", prefix);
  ret = audio_register(name,  &consumer->dev);
  if (ret < 0)
    {
      goto error;
    }

  return 0;

error:
  kmm_free(tunnel);
  return ret;
}
