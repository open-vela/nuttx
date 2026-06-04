/****************************************************************************
 * boards/arm/t113/t113-evb/uartring/uartdiag_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal UART diagnostic for T113 5-hop ring.
 *
 * Ring wiring (UART0=console, not in ring):
 *   UART1 TX PG6  -> UART2 RX PE3
 *   UART2 TX PE2  -> UART3 RX PB7
 *   UART3 TX PB6  -> UART4 RX PE5
 *   UART4 TX PE4  -> UART5 RX PE7
 *   UART5 TX PE6  -> UART1 RX PG7
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))

#define PIO_CFG(port, r) (0x02000000 + (port) * 0x30 + (r) * 4)
#define PIO_DAT(port)    (0x02000000 + (port) * 0x30 + 0x10)

#define PB 1
#define PE 4
#define PG 6

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int test_wire(const char *name,
                     int tx_port, int tx_pin,
                     int rx_port, int rx_pin)
{
  uint32_t tx_cfg_addr = PIO_CFG(tx_port, tx_pin / 8);
  uint32_t rx_cfg_addr = PIO_CFG(rx_port, rx_pin / 8);
  int tx_shift = (tx_pin % 8) * 4;
  int rx_shift = (rx_pin % 8) * 4;
  uint32_t tx_cfg_save = REG32(tx_cfg_addr);
  uint32_t rx_cfg_save = REG32(rx_cfg_addr);
  uint32_t cfg;
  int rx_hi;
  int rx_lo;

  cfg = REG32(tx_cfg_addr);
  cfg &= ~(0xfu << tx_shift);
  cfg |=  (0x1u << tx_shift);
  REG32(tx_cfg_addr) = cfg;

  cfg = REG32(rx_cfg_addr);
  cfg &= ~(0xfu << rx_shift);
  REG32(rx_cfg_addr) = cfg;

  REG32(PIO_DAT(tx_port)) |= (1u << tx_pin);
  usleep(500);
  rx_hi = (REG32(PIO_DAT(rx_port)) >> rx_pin) & 1;

  REG32(PIO_DAT(tx_port)) &= ~(1u << tx_pin);
  usleep(500);
  rx_lo = (REG32(PIO_DAT(rx_port)) >> rx_pin) & 1;

  REG32(tx_cfg_addr) = tx_cfg_save;
  REG32(rx_cfg_addr) = rx_cfg_save;

  printf("  %s: hi->%d lo->%d %s\n", name, rx_hi, rx_lo,
         (rx_hi == 1 && rx_lo == 0) ? "OK" : "FAIL");
  return (rx_hi == 1 && rx_lo == 0) ? 0 : 1;
}

static void uart_init(uint32_t base, int uart_num)
{
  uint32_t reg;
  volatile int i;

  reg = REG32(0x0200190c);
  reg &= ~(1u << (uart_num + 16));
  REG32(0x0200190c) = reg;
  for (i = 0; i < 100; i++);
  reg |= (1u << (uart_num + 16)) | (1u << uart_num);
  REG32(0x0200190c) = reg;

  REG32(base + 0x08) = 0x06;
  REG32(base + 0x08) = 0x01;
  REG32(base + 0x0c) = 0x83;
  REG32(base + 0x04) = 0x00;
  REG32(base + 0x00) = 0x0d;
  REG32(base + 0x0c) = 0x03;
  REG32(base + 0x08) = 0x07;
  REG32(base + 0x04) = 0x00;
}

static void setup_uart_pins(void)
{
  uint32_t cfg;

  /* PG6=func2(UART1 TX), PG7=func2(UART1 RX) - DS v1.6 */

  cfg = REG32(PIO_CFG(PG, 0));
  cfg &= ~(0xffu << 24);
  cfg |=  (0x22u << 24);
  REG32(PIO_CFG(PG, 0)) = cfg;

  /* PE2=func3(UART2 TX), PE3=func3(UART2 RX) - DS v1.6 */

  cfg = REG32(PIO_CFG(PE, 0));
  cfg &= ~(0xffu << 8);
  cfg |=  (0x33u << 8);
  REG32(PIO_CFG(PE, 0)) = cfg;

  /* PB6=func7(UART3 TX), PB7=func7(UART3 RX) */

  cfg = REG32(PIO_CFG(PB, 0));
  cfg &= ~(0xffu << 24);
  cfg |=  (0x77u << 24);
  REG32(PIO_CFG(PB, 0)) = cfg;

  /* PE4=func3(UART4 TX), PE5=func3(UART4 RX) - DS v1.6 */

  cfg = REG32(PIO_CFG(PE, 0));
  cfg &= ~(0xffu << 16);
  cfg |=  (0x33u << 16);
  REG32(PIO_CFG(PE, 0)) = cfg;

  /* PE6=func3(UART5 TX), PE7=func3(UART5 RX) - DS v1.6 */

  cfg = REG32(PIO_CFG(PE, 0));
  cfg &= ~(0xffu << 24);
  cfg |=  (0x33u << 24);
  REG32(PIO_CFG(PE, 0)) = cfg;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char *argv[])
{
  /* UART base addresses: UART1-5 */

  uint32_t uart[] =
    {
      0x02500400,   /* UART1 */
      0x02500800,   /* UART2 */
      0x02500c00,   /* UART3 */
      0x02501000,   /* UART4 */
      0x02501400,   /* UART5 */
    };

  int fail = 0;
  int i;
  uint8_t byte;
  int hop;
  int tx_idx;
  int rx_idx;
  uint32_t tx;
  uint32_t rx;
  uint32_t lsr_imm;
  uint32_t pe_cfg_save;
  uint32_t cfg2;
  int samples[200];
  int s;
  uint32_t pe1s;
  uint32_t pe1c;

  printf("=== UART Ring Diagnostic (5-hop) ===\n\n");

  /* Step 1: GPIO wire test - all 5 wires */

  if (argc > 1 && argv[1][0] == 'g')
    {
      printf("Step 1: GPIO wire test\n");
      fail |= test_wire("PG6->PE3 (U1TX->U2RX)", PG, 6, PE, 3);
      fail |= test_wire("PE2->PB7 (U2TX->U3RX)", PE, 2, PB, 7);
      fail |= test_wire("PB6->PE5 (U3TX->U4RX)", PB, 6, PE, 5);
      fail |= test_wire("PE4->PE7 (U4TX->U5RX)", PE, 4, PE, 7);
      fail |= test_wire("PE6->PG7 (U5TX->U1RX)", PE, 6, PG, 7);
      if (fail)
        {
          printf("  WIRING PROBLEM.\n");
          return 1;
        }

      printf("  All wires OK.\n\n");
    }
  else
    {
      printf("Step 1: skipped (use 'uartdiag g' for GPIO test)\n\n");
    }

  /* Step 2: Init all 5 UARTs using lowsetup sequence + GPIO pins */

  printf("Step 2: Init UART1-5 (lowsetup style)\n");

  for (i = 0; i < 5; i++)
    {
      uart_init(uart[i], i + 1);
    }

  setup_uart_pins();

  for (i = 0; i < 5; i++)
    {
      printf("  UART%d: LCR=%02x IIR=%02x LSR=%02x\n", i + 1,
             REG32(uart[i] + 0x0c) & 0xff,
             REG32(uart[i] + 0x08) & 0xff,
             REG32(uart[i] + 0x14) & 0xff);
    }

  printf("\n");

  /* Step 3: Send 0x55 around the ring.
   * Write to UART1 THR, it arrives at UART2 RBR.
   * Read UART2, write to UART2 THR -> UART3 RBR.
   * ... continue until UART5 THR -> UART1 RBR.
   */

  /* Verify GPIO mux after setup */

  printf("  GPIO: PG_CFG0=%08x PE_CFG0=%08x PB_CFG0=%08x\n",
         REG32(PIO_CFG(PG, 0)),
         REG32(PIO_CFG(PE, 0)),
         REG32(PIO_CFG(PB, 0)));

  printf("\nStep 3: Send 0x55 around the ring\n");

  byte = 0x55;
  for (hop = 0; hop < 5; hop++)
    {
      tx_idx = hop;
      rx_idx = (hop + 1) % 5;
      tx = uart[tx_idx];
      rx = uart[rx_idx];

      /* Write byte to TX UART */

      REG32(tx) = byte;

      /* Sample LSR immediately + after delay */

      lsr_imm = REG32(tx + 0x14);
      usleep(1000);
      printf("  [LSR: imm=%02x after=%02x]\n",
             lsr_imm & 0xff, REG32(tx + 0x14) & 0xff);

      /* Probe: read RX pin as GPIO while TX is sending */

      if (hop == 0)
        {
          /* Temporarily set PE3 to input, send byte,
           * sample PE3 rapidly to catch UART waveform
           */

          pe_cfg_save = REG32(PIO_CFG(PE, 0));
          cfg2 = pe_cfg_save;
          cfg2 &= ~(0xfu << 12);     /* PE3 = input */
          REG32(PIO_CFG(PE, 0)) = cfg2;

          /* Also set PE10 as input to probe UART1 alt pin */

          pe1s = REG32(PIO_CFG(PE, 1));
          pe1c = pe1s;
          pe1c &= ~(0xfu << 8);    /* PE10 = input */
          REG32(PIO_CFG(PE, 1)) = pe1c;

          REG32(tx) = 0xaa;

          for (s = 0; s < 200; s++)
            {
              samples[s] = (REG32(PIO_DAT(PE)) >> 3) & 1;
            }

          REG32(PIO_CFG(PE, 1)) = pe1s;

          REG32(PIO_CFG(PE, 0)) = pe_cfg_save;

          printf("  PE3 probe (200 samples): ");
          for (s = 0; s < 200; s++)
            {
              printf("%d", samples[s]);
            }

          printf("\n");
        }

      printf("  Hop %d: UART%d TX 0x%02x -> UART%d",
             hop + 1, tx_idx + 1, byte, rx_idx + 1);

      if (REG32(rx + 0x14) & 1)
        {
          byte = REG32(rx) & 0xff;
          printf(" RX 0x%02x OK\n", byte);
        }
      else
        {
          printf(" RX: NO DATA (LSR=%02x)\n",
                 REG32(rx + 0x14) & 0xff);
          printf("  FAILED at hop %d\n", hop + 1);
          return 1;
        }
    }

  printf("\n  Ring complete! Final byte=0x%02x %s\n",
         byte, byte == 0x55 ? "MATCH" : "MISMATCH");

  return 0;
}
