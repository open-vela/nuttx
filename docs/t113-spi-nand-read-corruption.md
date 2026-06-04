# T113 SPI NAND Read Corruption — Root Cause and Fix

## Summary

On Allwinner T113-S3 with USB MSC + SPI NAND concurrent load (MSC writes
trigger FTL read-modify-write on the NAND), sporadic page-read corruption
manifested as **delta = -1** patterns: page N's data area contained
page N-1's data. Error rate ~50 per 50MB write+readback.

The fix is a **single DSB barrier** in `spi_select()` immediately
after the TCR write that raises CS on deselect.

## Root Cause

`spi_select(dev, devid, false)` writes the TCR register to assert
SS_LEVEL=1 (raise CS). On ARM Cortex-A7 this device store goes
through the CPU store buffer and is **posted** — the CPU continues
execution before the write reaches the SPI peripheral.

Immediately after `spi_select(false)` returns, the caller issues new
device writes (typically another `SPI_SELECT(true)` for the next
command). Under concurrent USB MUSB load, the variable latency
between CPU and AHB bus can widen the window such that the CS-rise
store has not yet reached the SPI peripheral when the new CS-low
store arrives. The NAND chip observes an insufficient tCSH
(chip-select hold time), which lets the NAND **drop or merge
commands** — the subsequent `PAGE_READ` with row=N may be absorbed
into the prior transaction, leaving the on-chip data cache still
holding page N-1's content. The driver then calls `READ_FROM_CACHE`
and receives page N-1.

This is **not** a T113 silicon errata or TP_EN bug. It is a
store-buffer ordering issue: the SPI peripheral observes CS
transitions out of intended program order because the first store
is still buffered.

## Fix

File: `arch/arm/src/t113/t113_spi.c`

```c
static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  ...
  else
    {
      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_SS_LEVEL);

      /* Drain the posted CS-rise store to the SPI peripheral
       * before any subsequent device write can overtake it.
       */
      __asm__ volatile("dsb sy" ::: "memory");
    }
}
```

Overhead: ~1 ns per CS toggle. No measurable performance impact.

## Why This Specific Location

A DSB at the end of `spi_exchange_pio` (before the caller's
`SPI_SELECT(false)` runs) **does not** fix the bug — the CS-rise
store occurs AFTER that barrier, so the posted-store race window
still exists.

The barrier must sit **immediately after the CS-rise store** so that
when `spi_select(false)` returns, the CS transition is guaranteed
committed.

## Experimental Evidence

| Variant                                                 | Errors / 50MB |
|---------------------------------------------------------|---------------|
| Baseline (no DSB)                                       | ~50           |
| DSB at end of `spi_exchange_pio`                        | 63            |
| DSB in `spi_select(false)` after CS-rise (this fix)     | **0 × 3 runs** |
| `up_udelay(20)` after `mx35_readbuffer`                 | 0             |
| `up_udelay(700)` before `mx35_readbuffer`               | ~50           |
| `spin_lock_irqsave` around `mx35_readbuffer` (old WA)   | 0             |

All working alternatives share the same side effect: they prevent
another device-store from racing the posted CS-rise. The DSB fix
addresses the root cause directly — no CPU time wasted on delays
or IRQ masking.

## Test Pattern

The test writes a 50 MB pattern where each 2048-byte page begins with
its page index encoded as little-endian uint32. The `delta =
got - expected` histogram concentrates at `-1` in the corrupted case
because when page N's cache load is merged/dropped, the subsequent
`READ_FROM_CACHE` returns page N-1's content.

```bash
# On host, after usbmsc starts and /dev/sdd appears:
dd if=/tmp/pattern.bin of=/dev/sdd bs=65536 status=progress
sync
dd if=/dev/sdd of=/tmp/readback.bin bs=65536 count=800 iflag=direct
python3 /tmp/verify.py /tmp/readback.bin
# Expect: "checked 25600 pages, 0 errors"
```

## Related

- The earlier "TP_EN silicon errata" hypothesis was wrong. TP_EN
  correctly pauses SCK when the RX FIFO fills and does not corrupt
  data.
- The earlier `spin_lock_irqsave(readlock)` workaround in `mx35.c`
  has been removed. The SPI driver fix makes it unnecessary.
