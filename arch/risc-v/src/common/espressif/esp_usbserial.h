/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_usbserial.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_USBSERIAL_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_USBSERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/serial/serial.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uart_dev_t g_uart_usbserial;

/* RAM debug marker, see esp_usbserial.c.  Read after a warm reset to learn
 * the last debug point the CPU reached. */

extern volatile uint32_t g_dbg_mark;

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_usbserial_write
 *
 * Description:
 *   Write one character through the USB serial. Used mainly for early
 *   debugging.
 *
 ****************************************************************************/

void esp_usbserial_write(char ch);

/****************************************************************************
 * Name: dbg_putc
 *
 * Description:
 *   Polled debug marker with a bounded wait for TX FIFO space.  Works with
 *   interrupts masked; never blocks forever.  See esp_usbserial.c.
 *
 ****************************************************************************/

void dbg_putc(int ch);

/****************************************************************************
 * Name: dbg_console_tx_set
 *
 * Description:
 *   Enable/disable the polled console TX in dbg_putc().  Keep enabled while
 *   printing the boot RAM marker dump; disable before returning to app so
 *   polled markers never contend with the interrupt-driven console drain.
 *
 ****************************************************************************/

void dbg_console_tx_set(bool enable);

/****************************************************************************
 * Name: dbg_mark_char
 *
 * Description:
 *   Record a character into the RAM marker and non-blocking TX it.  Intended
 *   for use inside spin-lock critical sections.  See esp_usbserial.c.
 *
 ****************************************************************************/

void dbg_mark_char(int ch);

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_USBSERIAL_H */
