/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_panic_dump.c
 *
 * Panic-time crash dump that writes DIRECTLY to the USB-Serial-JTAG TX FIFO,
 * bypassing NuttX syslog / UART0 (which is invisible on ttyACM0).
 *
 * Called at the very top of xtensa_panic() / xtensa_user_panic() so the
 * exception registers and a small stack window are captured even if the
 * syslog path itself faults (double exception) or the USB-JTAG 4096B
 * buffer overflows.
 *
 * [DIAG] Boot-crash isolation instrumentation - keep until WiFi PHY crash
 * is root-caused.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "xtensa.h"
#include "hal/usb_serial_jtag_ll.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void jtag_putc(char c)
{
  /* Poll the TX FIFO; USB-Serial-JTAG txfifo is small, host is reading. */

  while (!usb_serial_jtag_ll_txfifo_writable())
    {
    }

  usb_serial_jtag_ll_write_txfifo((const uint8_t *)&c, 1);
}

static void jtag_puts(const char *s)
{
  while (*s != '\0')
    {
      jtag_putc(*s++);
    }
}

static void jtag_hex32(uint32_t v)
{
  static const char hex[] = "0123456789abcdef";
  int i;

  for (i = 28; i >= 0; i -= 4)
    {
      jtag_putc(hex[(v >> i) & 0xf]);
    }
}

static void jtag_dec(int v)
{
  char buf[12];
  int i = 0;
  unsigned int u;

  if (v < 0)
    {
      jtag_putc('-');
      u = (unsigned int)(-v);
    }
  else
    {
      u = (unsigned int)v;
    }

  do
    {
      buf[i++] = '0' + (u % 10);
      u /= 10;
    }
  while (u != 0);

  while (i > 0)
    {
      jtag_putc(buf[--i]);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_panic_dump
 *
 * Description:
 *   Dump exception registers + a stack window over USB-Serial-JTAG.
 *   regs is the XCPT context pointer passed to xtensa_user_panic().
 *
 ****************************************************************************/

void esp32s3_panic_dump(int xptcode, uint32_t *regs)
{
  const uint32_t *sp;
  int i;

  jtag_puts("\r\n[PANIC-DUMP] xptcode=");
  jtag_dec(xptcode);
  jtag_puts(" PC=");
  jtag_hex32(regs[REG_PC]);
  jtag_puts(" PS=");
  jtag_hex32(regs[REG_PS]);
  jtag_puts(" CAUSE=");
  jtag_hex32(regs[REG_EXCCAUSE]);
  jtag_puts(" VADDR=");
  jtag_hex32(regs[REG_EXCVADDR]);
  jtag_puts("\r\n");

  jtag_puts(" A0=");
  jtag_hex32(regs[REG_A0]);
  jtag_puts(" A1=");
  jtag_hex32(regs[REG_A1]);
  jtag_puts(" A2=");
  jtag_hex32(regs[REG_A2]);
  jtag_puts(" A3=");
  jtag_hex32(regs[REG_A3]);
  jtag_puts(" A4=");
  jtag_hex32(regs[REG_A4]);
  jtag_puts(" A5=");
  jtag_hex32(regs[REG_A5]);
  jtag_puts(" A6=");
  jtag_hex32(regs[REG_A6]);
  jtag_puts(" A7=");
  jtag_hex32(regs[REG_A7]);
  jtag_puts("\r\n");

  jtag_puts(" A8=");
  jtag_hex32(regs[REG_A8]);
  jtag_puts(" A9=");
  jtag_hex32(regs[REG_A9]);
  jtag_puts(" A10=");
  jtag_hex32(regs[REG_A10]);
  jtag_puts(" A11=");
  jtag_hex32(regs[REG_A11]);
  jtag_puts(" A12=");
  jtag_hex32(regs[REG_A12]);
  jtag_puts(" A13=");
  jtag_hex32(regs[REG_A13]);
  jtag_puts(" A14=");
  jtag_hex32(regs[REG_A14]);
  jtag_puts(" A15=");
  jtag_hex32(regs[REG_A15]);
  jtag_puts("\r\n");

  /* Stack window: 32 words from the captured stack pointer. */

  sp = (const uint32_t *)regs[REG_A1];
  jtag_puts("STACK:");
  for (i = 0; i < 32; i++)
    {
      if ((i & 7) == 0)
        {
          jtag_puts("\r\n ");
          jtag_hex32((uint32_t)(uintptr_t)(sp + i));
          jtag_puts(":");
        }

      jtag_puts(" ");
      jtag_hex32(sp[i]);
    }

  jtag_puts("\r\n[PANIC-DUMP-END]\r\n");
}
