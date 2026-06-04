/****************************************************************************
 * include/nuttx/usb/cdcacm.h
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

#ifndef __INCLUDE_NUTTX_USB_CDCACM_H
#define __INCLUDE_NUTTX_USB_CDCACM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/streams.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* CONFIG_CDCACM
 *   Enable compilation of the USB serial driver
 * CONFIG_CDCACM_EP0MAXPACKET
 *   Endpoint 0 max packet size. Default 64.
 * CONFIG_CDCACM_EPINTIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   interrupt IN operation.  Default 1.
 * CONFIG_CDCACM_EPINTIN_FSSIZE
 *   Max package size for the interrupt IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPINTIN_HSSIZE
 *   Max package size for the interrupt IN endpoint if high speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPBULKOUT
 *   The logical 7-bit address of a hardware endpoint that supports
 *   bulk OUT operation.  Default: 3
 * CONFIG_CDCACM_EPBULKOUT_FSSIZE
 *   Max package size for the bulk OUT endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPBULKOUT_HSSIZE
 *   Max package size for the bulk OUT endpoint if high speed mode.
 *   Default 512.
  * CONFIG_CDCACM_EPBULKOUT_SSSIZE
 *   Max package size for the bulk OUT endpoint if super speed mode.
 *   Default 1024.
 * CONFIG_CDCACM_EPBULKIN
 *   The logical 7-bit address of a hardware endpoint that supports
 *   bulk IN operation.  Default: 2
 * CONFIG_CDCACM_EPBULKIN_FSSIZE
 *   Max package size for the bulk IN endpoint if full speed mode.
 *   Default 64.
 * CONFIG_CDCACM_EPBULKIN_HSSIZE
 *   Max package size for the bulk IN endpoint if high speed mode.
 *   Default 512.
 * CONFIG_CDCACM_EPBULKIN_SSSIZE
 *   Max package size for the bulk IN endpoint if super speed mode.
 *   Default 1024.
 * CONFIG_CDCACM_NWRREQS and CONFIG_CDCACM_NRDREQS
 *   The number of write/read requests that can be in flight.
 *   CONFIG_CDCACM_NWRREQS includes write requests used for both the
 *   interrupt and bulk IN endpoints.  Default 4.
 * CONFIG_CDCACM_VENDORID and CONFIG_CDCACM_VENDORSTR
 *   The vendor ID code/string.  Default 0x0525 and "NuttX"
 *   0x0525 is the Netchip vendor and should not be used in any
 *   products.  This default VID was selected for compatibility with
 *   the Linux CDC ACM default VID.
 * CONFIG_CDCACM_PRODUCTID and CONFIG_CDCACM_PRODUCTSTR
 *   The product ID code/string. Default 0xa4a7 and "CDC/ACM Serial"
 *   0xa4a7 was selected for compatibility with the Linux CDC ACM
 *   default PID.
 */

/* Information needed in usbdev_devinfo_s */

#define CDCACM_NUM_EPS             (3)

#define CDCACM_EP_INTIN_IDX        (0)
#define CDCACM_EP_BULKIN_IDX       (1)
#define CDCACM_EP_BULKOUT_IDX      (2)

#define CDCACM_NCONFIGS            (1)      /* Number of configurations supported */

/* Configuration descriptor values */

#define CDCACM_CONFIGID            (1)      /* The only supported configuration ID */

#define CDCACM_NINTERFACES         (2)      /* Number of interfaces in the configuration */

/* EP0 max packet size */

#ifndef CONFIG_CDCACM_EP0MAXPACKET
#  define CONFIG_CDCACM_EP0MAXPACKET 64
#endif

/* Endpoint number and size (in bytes) of the CDC serial device-to-host (IN)
 * notification interrupt endpoint.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
#  ifndef CONFIG_CDCACM_EPINTIN
#    define CONFIG_CDCACM_EPINTIN 1
#  endif
#endif

#ifndef CONFIG_CDCACM_EPINTIN_FSSIZE
#  define CONFIG_CDCACM_EPINTIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCACM_EPINTIN_HSSIZE
#  define CONFIG_CDCACM_EPINTIN_HSSIZE 64
#endif

/* Endpoint number and size (in bytes) of the CDC device-to-host (IN) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 *
 * Ideally, the BULKOUT request size should *not* be the same size as the
 * maxpacket size.  That is because IN transfers of exactly the maxpacket
 * size will be followed by a NULL packet.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
#  ifndef CONFIG_CDCACM_EPBULKIN
#    define CONFIG_CDCACM_EPBULKIN 2
#  endif
#endif

#ifndef CONFIG_CDCACM_EPBULKIN_FSSIZE
#  define CONFIG_CDCACM_EPBULKIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCACM_EPBULKIN_HSSIZE
#  define CONFIG_CDCACM_EPBULKIN_HSSIZE 512
#endif

#ifndef CONFIG_CDCACM_EPBULKIN_SSSIZE
#  define CONFIG_CDCACM_EPBULKIN_SSSIZE 1024
#endif

#ifndef CONFIG_CDCACM_BULKIN_REQLEN
#  ifdef CONFIG_USBDEV_DUALSPEED
#    define CONFIG_CDCACM_BULKIN_REQLEN (3 * CONFIG_CDCACM_EPBULKIN_FSSIZE / 2)
#  else
#    define CONFIG_CDCACM_BULKIN_REQLEN (3 * CONFIG_CDCACM_EPBULKIN_FSSIZE / 2)
#  endif
#endif

/* Endpoint number and size (in bytes) of the CDC host-to-device (OUT) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 *
 * NOTE:  The BULKOUT request buffer size is always the same as the
 * maxpacket size.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
#  ifndef CONFIG_CDCACM_EPBULKOUT
#    define CONFIG_CDCACM_EPBULKOUT 3
#  endif
#endif

#ifndef CONFIG_CDCACM_EPBULKOUT_FSSIZE
#  define CONFIG_CDCACM_EPBULKOUT_FSSIZE 64
#endif

#ifndef CONFIG_CDCACM_EPBULKOUT_HSSIZE
#  define CONFIG_CDCACM_EPBULKOUT_HSSIZE 512
#endif

#ifndef CONFIG_CDCACM_EPBULKOUT_SSSIZE
#  define CONFIG_CDCACM_EPBULKOUT_SSSIZE 1024
#endif

/* Number of requests in the write queue.  This includes write requests used
 * for both the interrupt and bulk IN endpoints.
 */

#ifndef CONFIG_CDCACM_NWRREQS
#  define CONFIG_CDCACM_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_CDCACM_NRDREQS
#  define CONFIG_CDCACM_NRDREQS 4
#endif

/* Vendor and product IDs and strings.  The default is the Linux Netchip
 * CDC ACM VID and PID.
 */

#ifndef CONFIG_CDCACM_VENDORID
#  define CONFIG_CDCACM_VENDORID  0x0525
#endif

#ifndef CONFIG_CDCACM_PRODUCTID
#  define CONFIG_CDCACM_PRODUCTID 0xa4a7
#endif

#ifndef CONFIG_CDCACM_VENDORSTR
#  define CONFIG_CDCACM_VENDORSTR  "NuttX"
#endif

#ifndef CONFIG_CDCACM_PRODUCTSTR
#  define CONFIG_CDCACM_PRODUCTSTR "CDC ACM Serial"
#endif

#undef CONFIG_CDCACM_SERIALSTR
#define CONFIG_CDCACM_SERIALSTR "0"

#undef CONFIG_CDCACM_CONFIGSTR
#define CONFIG_CDCACM_CONFIGSTR "Bulk"

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define CDCACM_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define CDCACM_SELFPOWERED (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define CDCACM_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define CDCACM_REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/* IOCTL Commands ***********************************************************/

/* The USB serial driver will support a subset of the TIOC IOCTL commands
 * defined in include/nuttx/serial/tioctl.h.  This subset includes:
 *
 * CAICO_REGISTERCB
 *   Register a callback for serial event notification. Argument:
 *   cdcacm_callback_t.  See cdcacm_callback_t type definition below.
 *   NOTE:  The callback will most likely invoked at the interrupt level.
 *   The called back function should, therefore, limit its operations to
 *   invoking some kind of IPC to handle the serial event in some normal
 *   task environment.
 * CAIOC_GETLINECODING
 *   Get current line coding.  Argument: struct cdc_linecoding_s*.
 *   See include/nuttx/usb/cdc.h for structure definition.  This IOCTL
 *   should be called to get the data associated with the
 *   CDCACM_EVENT_LINECODING event (see devent definition below).
 * CAIOC_GETCTRLLINE
 *   Get control line status bits. Argument FAR int*.  See
 *   include/nuttx/usb/cdc.h for bit definitions.  This IOCTL should be
 *   called to get the data associated CDCACM_EVENT_CTRLLINE event (see event
 *   definition below).
 * CAIOC_NOTIFY
 *   Send a serial state to the host via the Interrupt IN endpoint.
 *   Argument: int.  This includes the current state of the carrier detect,
 *   DSR, break, and ring signal.  See "Table 69: UART State Bitmap Values"
 *   and CDC_UART_definitions in include/nuttx/usb/cdc.h.
 */

#define CAIOC_REGISTERCB    _CAIOC(0x0001)
#define CAIOC_GETLINECODING _CAIOC(0x0002)
#define CAIOC_GETCTRLLINE   _CAIOC(0x0003)
#define CAIOC_NOTIFY        _CAIOC(0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* Reported serial events.  Data is associated with CDCACM_EVENT_LINECODING
 * and CDCACM_EVENT_CTRLLINE.  The data may be obtained using CDCACM IOCTL
 * commands described above.
 *
 * CDCACM_EVENT_LINECODING - See "Table 50: Line Coding Structure" and struct
 *   cdc_linecoding_s in include/nuttx/usb/cdc.h.
 * CDCACM_EVENT_CTRLLINE - See "Table 51: Control Signal Bitmap Values for
 *   SetControlLineState" and definitions in include/nutt/usb/cdc.h
 * CDCACM_EVENT_SENDBREAK - See Paragraph "6.2.15 SendBreak." This request
 *   sends special carrier modulation that generates an RS-232 style break.
 */

enum cdcacm_event_e
{
  CDCACM_EVENT_LINECODING = 0, /* New line coding received from host */
  CDCACM_EVENT_CTRLLINE,       /* New control line status received from host */
  CDCACM_EVENT_SENDBREAK       /* Send break request received */
};

typedef CODE void (*cdcacm_callback_t)(enum cdcacm_event_e event);

struct usbdevclass_driver_s;

/* Forward declaration -- opaque to callers */

struct cdcacm_dev_s;
struct usbdev_devinfo_s;

/* User callback table.  Any field may be NULL; the cdcacm core no-ops
 * any unset callback.  Multiple optional groups:
 *   - CDC ACM class control requests (line coding, etc.)
 *   - USB lifecycle (connect/suspend)
 *   - Data plane (only the uart adapter uses these)
 *   - xmit-buffer alias protocol for the DISABLE_TXBUF zero-copy path
 */

struct cdcacm_user_ops_s
{
  /* Called from cdcacm_setup() when the host issues SET_LINE_CODING.
   * coding[0..6] holds dwDTERate / bCharFormat / bParityType / bDataBits
   * per the CDC PSTN spec; len is the byte count actually received
   * (normally 7).  Return 0 on success or a negated errno.  The cdcacm
   * core ACKs the control transfer regardless.
   */

  CODE int  (*set_line_coding)(FAR struct cdcacm_dev_s *dev,
                               FAR const uint8_t *coding, size_t len);

  /* Called from cdcacm_setup() when the host issues GET_LINE_CODING.
   * The adapter writes up to cap bytes of the current encoding state
   * into out.  Return the number of bytes written or a negated errno.
   */

  CODE int  (*get_line_coding)(FAR struct cdcacm_dev_s *dev,
                               FAR uint8_t *out, size_t cap);

  /* Called from cdcacm_setup() when the host issues
   * SET_CONTROL_LINE_STATE.  state bit 0 = DTR, bit 1 = RTS.  Return 0
   * on success or a negated errno.
   */

  CODE int  (*set_ctrl_line_state)(FAR struct cdcacm_dev_s *dev,
                                   uint16_t state);

  /* Called from cdcacm_setup() when the host issues SEND_BREAK.
   * duration is in milliseconds; 0xffff means continuous, 0 stops an
   * in-progress break.  Return 0 on success or a negated errno.
   */

  CODE int  (*send_break)(FAR struct cdcacm_dev_s *dev,
                          uint16_t duration);

  /* Called when the USB cable is plugged/unplugged or the host issues
   * SET_CONFIGURATION.  connected == true means data endpoints are
   * ready for I/O; false means they have been torn down.
   */

  CODE void (*on_connect)(FAR struct cdcacm_dev_s *dev, bool connected);

  /* Called when the USB bus enters suspend or resumes.  suspended ==
   * true means the USB clock has been gated and no transfers are
   * possible until resume.
   */

  CODE void (*on_suspend)(FAR struct cdcacm_dev_s *dev, bool suspended);

  /* Called when an OUT endpoint request completes carrying len bytes.
   * buf is valid only for the duration of the call -- the adapter must
   * copy or consume the data inline.
   */

  CODE void (*on_rx)(FAR struct cdcacm_dev_s *dev,
                     FAR const uint8_t *buf, size_t len);

  /* Called when cdcacm needs the next IN packet payload.  The adapter
   * fills dst with up to cap bytes and returns the number of bytes
   * written; returning 0 means the adapter has nothing to send right
   * now and cdcacm will skip the IN packet.
   */

  CODE int  (*pull_tx)(FAR struct cdcacm_dev_s *dev,
                       FAR uint8_t *dst, size_t cap);

  /* Called on the DISABLE_TXBUF zero-copy alias path; cdcacm asks the
   * adapter for the next xmit ring buffer.  The adapter writes the
   * buffer pointer into *buf and the available capacity into *cap.
   * Return 0 on success or a negated errno.
   */

  CODE int  (*claim_xmit_buf)(FAR struct cdcacm_dev_s *dev,
                              FAR uint8_t **buf, FAR size_t *cap);

  /* Called from wrcomplete to tell the adapter how many bytes from the
   * previously claimed xmit buffer were actually transmitted, so the
   * adapter can advance its ring tail.
   */

  CODE void (*release_xmit_buf)(FAR struct cdcacm_dev_s *dev,
                                size_t consumed);
};

/* Outstream -- naming aligns with lib_blkoutstream_open /
 * lib_mtdoutstream_open and uses the lib_blkoutstream_s / lib_mtdoutstream_s
 * convention of publishing fields directly (no opaque blob).
 */

struct cdcacm_outstream_s
{
  struct lib_outstream_s    common;
  FAR struct cdcacm_dev_s  *dev;

  /* puts/putc/flush need no stream-local sync state: blocking and
   * completion are handled by cdcacm_internal_submit through the device's
   * tx_waitsem and wrcontainer pool, so only the bound device is held here.
   */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_classobject
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured) and
 *   return the class object.
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   A pointer to the allocated class object (NULL on failure).
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
int cdcacm_classobject(int minor, FAR struct usbdev_devinfo_s *devinfo,
                       FAR struct usbdevclass_driver_s **classdev);
#endif

/****************************************************************************
 * Name: cdcacm_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameters:
 *   minor - Device minor number.  E.g., minor 0 would correspond to
 *     /dev/ttyACM0.
 *   handle - An optional opaque reference to the CDC/ACM class object that
 *     may subsequently be used with cdcacm_uninitialize().
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is returned.
 *
 ****************************************************************************/

#if defined(CONFIG_CDCACM_SERIAL) && \
    (!defined(CONFIG_USBDEV_COMPOSITE) || !defined(CONFIG_CDCACM_COMPOSITE))
int cdcacm_initialize(int minor, FAR void **handle);
#endif

/****************************************************************************
 * Name: cdcacm_uninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This function is used
 *   internally by the USB composite driver to uninitialize the CDC/ACM
 *   driver.  This same interface is available (with an untyped input
 *   parameter) when the CDC/ACM driver is used standalone.
 *
 * Input Parameters:
 *   There is one parameter, it differs in typing depending upon whether the
 *   CDC/ACM driver is an internal part of a composite device, or a
 *   standalone USB driver:
 *
 *     classdev - The class object returned by cdcacm_classobject()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_CDCACM_SERIAL
void cdcacm_uninitialize(FAR struct usbdevclass_driver_s *classdev);
#endif

/****************************************************************************
 * Name: cdcacm_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_CDCACM_COMPOSITE)
struct composite_devdesc_s;
void cdcacm_get_composite_devdesc(struct composite_devdesc_s *dev);
#endif

/****************************************************************************
 * Name: cdcacm_write
 *
 * Description:
 *   This provides a cdcacm write method for syslog devices that support
 *   multiple byte writes.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the number of characters written is returned.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CDCACM
ssize_t cdcacm_write(FAR const char *buffer, size_t buflen);
#endif

/****************************************************************************
 * Name: cdcacm_disable_syslog
 *
 * Description:
 *   Disable CDCACM syslog channel by clearing the globle pointer.
 *   This function is used in specific situation, such as must disable
 *   cdcacm log printing when usb re-enumeration.
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_CDCACM
void cdcacm_disable_syslog(void);
#endif

/****************************************************************************
 * Name: cdcacm_register
 *
 * Description:
 *   Allocate a cdcacm instance, install the supplied user ops table, and
 *   register the underlying USB device class driver.  Returns the opaque
 *   device handle through dev_out for use with the rest of the layered
 *   API.
 *
 * Input Parameters:
 *   minor     - Device minor number; same meaning as cdcacm_initialize().
 *   ops       - User callback table.  Any field may be NULL.
 *   user_priv - Opaque pointer stored in the instance and returned from
 *               cdcacm_get_user_priv().
 *   devinfo   - Composite endpoint/interface descriptor info, or NULL
 *               when running standalone.
 *   dev_out   - Location to receive the new device handle on success.
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure.
 *
 ****************************************************************************/

int  cdcacm_register(int minor,
                     FAR const struct cdcacm_user_ops_s *ops,
                     FAR void *user_priv,
                     FAR struct usbdev_devinfo_s *devinfo,
                     FAR struct cdcacm_dev_s **dev_out);

/****************************************************************************
 * Name: cdcacm_unregister
 *
 * Description:
 *   Tear down a cdcacm instance previously created by cdcacm_register().
 *   Once all in-flight users have released the device, the underlying USB
 *   device class driver is unregistered and the instance is freed.
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void cdcacm_unregister(FAR struct cdcacm_dev_s *dev);

/****************************************************************************
 * Name: cdcacm_acquire
 *
 * Description:
 *   Increment the cdcacm instance refcount, preventing teardown while the
 *   caller holds a reference.  Fails (without taking a reference) once the
 *   instance is closing, so the caller must abort whatever it was opening
 *   rather than pair the failed acquire with a later cdcacm_release().
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   true if a reference was taken; false if the instance is closing (or dev
 *   is NULL) -- in which case the caller holds no reference and must not
 *   call cdcacm_release().
 *
 ****************************************************************************/

bool cdcacm_acquire(FAR struct cdcacm_dev_s *dev);

/****************************************************************************
 * Name: cdcacm_release
 *
 * Description:
 *   Drop a reference previously taken by cdcacm_acquire().  When the
 *   refcount reaches zero and the instance is in the closing state, the
 *   instance is freed.
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void cdcacm_release(FAR struct cdcacm_dev_s *dev);

/****************************************************************************
 * Name: cdcacm_get_user_priv
 *
 * Description:
 *   Retrieve the opaque user_priv pointer that the adapter passed to
 *   cdcacm_register().
 *
 * Input Parameters:
 *   dev - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   The stored user_priv pointer, or NULL if none was set.
 *
 ****************************************************************************/

FAR void *cdcacm_get_user_priv(FAR struct cdcacm_dev_s *dev);

/****************************************************************************
 * Name: cdcacm_send_serial_state
 *
 * Description:
 *   Intended to send a SERIAL_STATE notification on the interrupt IN
 *   endpoint when the modem-status lines (DCD/DSR/RI/CTS) change.
 *
 *   Currently a stub that returns -ENOSYS.  The live notification path is
 *   cdcacm_serialstate(), reached through the uart adapter's CAIOC_NOTIFY
 *   ioctl when CONFIG_CDCACM_IFLOWCONTROL is enabled.
 *
 * Input Parameters:
 *   dev   - Device handle returned by cdcacm_register().
 *   state - Bitmap of UART state bits, see "Table 69: UART State Bitmap
 *           Values" in the CDC PSTN spec and CDC_UART_definitions in
 *           include/nuttx/usb/cdc.h.
 *
 * Returned Value:
 *   -ENOSYS (not implemented).
 *
 ****************************************************************************/

int  cdcacm_send_serial_state(FAR struct cdcacm_dev_s *dev, uint16_t state);

/****************************************************************************
 * Name: cdcacm_outstream_open
 *
 * Description:
 *   Initialize a cdcacm_outstream_s so it can be used with the standard
 *   lib_outstream_s puts/putc/flush operations.  Naming aligns with
 *   lib_blkoutstream_open() / lib_mtdoutstream_open().
 *
 * Input Parameters:
 *   stream - Caller-allocated stream object to initialize.
 *   dev    - Device handle returned by cdcacm_register().
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure.
 *
 ****************************************************************************/

int  cdcacm_outstream_open(FAR struct cdcacm_outstream_s *stream,
                           FAR struct cdcacm_dev_s *dev);

/****************************************************************************
 * Name: cdcacm_outstream_close
 *
 * Description:
 *   Release any resources owned by stream and detach it from the cdcacm
 *   instance.  After this call the stream object must not be used.
 *
 * Input Parameters:
 *   stream - Stream object previously initialized by
 *            cdcacm_outstream_open().
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void cdcacm_outstream_close(FAR struct cdcacm_outstream_s *stream);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_CDCACM_H */
