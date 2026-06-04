# T113-S3 dual-core (core0 NuttX master + core1 NuttX slave)

Asymmetric multiprocessing for the dual-Cortex-A7 T113-S3.  CPU0 runs
`core0`, an independent NuttX kernel that owns global hardware bringup and
acts as the CPU1 master: it ELF-loads the CPU1 image and releases CPU1 from
reset, exactly as Linux remoteproc would.  CPU1 runs `core1`, a second NuttX
kernel ELF-loaded into the shared carveout.

`core1` is delivered as a relocatable ELF (`core1.elf`) carrying a static
`.resource_table` section.  The same artifact is loadable by either master:
core0-NuttX via `rptun start /dev/rptun/core1`, or Linux remoteproc from
`/lib/firmware`.  The loader copies the `PT_LOAD` segments into DRAM and
resolves the boot entry from the ELF `e_entry` (0x46B002C0); CPU1 is released
from reset there, not at `RAM_START`, because the image base holds the vector
table.

Both defconfigs derive from `nsh/defconfig`, so both images carry the full
T113 driver feature set; the master/slave hardware partition is applied at
runtime through the Kconfig guards below.

## Kconfig roles

| Symbol                 | core0 | core1 | Meaning                                                        |
|------------------------|-------|-------|----------------------------------------------------------------|
| `T113_AMP`             | y     | y     | run-mode: two independent images + hwspinlock for shared regs |
| `T113_RPTUN_CORE1`      | y     | n     | register `/dev/rptun/core1`; core0 is the CPU1 ELF master       |
| `T113_RPTUN_SLAVE`     | n     | y     | this image is the ELF-loaded CPU1 slave                        |

`T113_RPTUN_CORE1` depends on `RPTUN` + `RPTUN_LOADER`; the firmware path is
`T113_RPTUN_CORE1_FIRMWARE_PATH` (default `/fw/core1.elf`).  `T113_RPTUN_SLAVE`
marks the slave so it skips master-owned bringup and remaps the shared vring
window Non-Cacheable.

## DRAM layout

| Region    | Range                   | Size  | Owner | Notes                                          |
|-----------|-------------------------|-------|-------|------------------------------------------------|
| TEE       | `0x40000000-0x401FFFFF` | 2 M   | -     | reserved for a future secure world (unused)    |
| core0     | `0x40200000-0x46AFFFFF` | 105 M | core0 | CONFIG_RAM_START / CONFIG_RAM_SIZE             |
| core1     | `0x46B00000+`           | 14 M  | core1 | CONFIG_RAM_START; linked below the vring window|
| carveout  | `0x47E00000-0x47EFFFFF` | 1 M   | shared| vring0 / vring1 / rpmsg buffer pool            |

`T113_RPTUN_CORE1_CARVEOUT_BASE` is `0x47E00000`.  This 1 MB section holds
the vrings and rpmsg buffer pool; the core1 image is linked at `0x46B00000`,
well below it, in the high-DDR SHM aggregation zone separate from the image.
The master maps just the 1 MB carveout Non-Cacheable so the vring window is
coherent without explicit cache maintenance on the rpmsg data path.  The
carveout extent must match the Linux DT reserved-memory node and the addresses
baked into the static `resource_table`.

## Resource ownership

| Block / driver           | core0 (master) | core1 (slave) | Notes                                                       |
|--------------------------|----------------|---------------|-------------------------------------------------------------|
| PLL_CPUX / PERI0 / APB   | owns           | inherits      | slave's `t113_clk_init()` returns immediately               |
| GIC distributor          | owns           | inherits      | guard in `t113_irq.c::up_irqinitialize`                     |
| DMA0 / MBUS gating       | owns           | skipped       | slave's `arm_dma_initialize()` returns                      |
| SPINLOCK module BGR      | owns           | skipped       | slave still constructs 32 lock dev_s                        |
| UART0 console (PF2/PF4)  | console        | n/a           | core0 nsh prompt `core0`                                    |
| UART2 console (PE2/PE3)  | n/a            | console       | core1 nsh prompt `core1`                                    |
| SPI0 / NAND / FTL / FAT  | core0 only     | n/a           | NAND owned by master; slave defconfig keeps the symbols off |
| EMAC                     | core0 only     | n/a           | single MAC, master-owned                                    |
| USB host / device        | core0 only     | n/a           | single PHY, master-owned                                    |
| HSTIMER0/1               | core0 only     | n/a           | master-owned                                                |
| Crypto Engine            | core0 only     | n/a           | single instance                                             |
| GPADC / PWM / RTC / WDT  | core0 only     | n/a           | enabled in master's nsh-derived defconfig only              |

CCU shared registers are protected by the `t113_ccu_*` helpers; under
`T113_AMP` they fall through to the hardware spinlock module
(`HWLOCK_ID_CCU`).

## Build

```sh
cmake -B build_core0 -GNinja -DBOARD_CONFIG=t113-evb:core0 ./
cmake --build build_core0

cmake -B build_core1 -GNinja -DBOARD_CONFIG=t113-evb:core1 ./
cmake --build build_core1
```

Verify the core1 entry and the master release symbol:

```sh
arm-none-eabi-objdump -h build_core1/nuttx | grep '\.text'        # 46b00000
arm-none-eabi-readelf -h build_core1/nuttx | grep 'Entry point'   # 0x46B002C0
arm-none-eabi-nm build_core0/nuttx | grep t113_release_cpu1       # resolved
```

Do not strip `build_core1/nuttx`; the rptun loader (and Linux remoteproc)
read its `.resource_table` section.

## Deploy and boot

Both images ship inside `fwromfs.img` on NAND mtd1.  boot0 loads core0; core0
mounts `/fw` and then ELF-loads core1.

1. Pack `core0.bin` and `core1.elf` into `fwromfs.img` and write it to
   mtd1.  See `boards/arm/t113/t113-evb/scripts/fwromfs/README.md`.
2. boot0 boots core0 from the cfg; core0 finishes `t113_bringup()` (clocks,
   GIC, DMA, spinlock, console) and registers `/dev/rptun/core1`.
3. At the `core0>` prompt: `rptun start /dev/rptun/core1`.  The loader opens
   `/fw/core1.elf`, copies its `PT_LOAD` segments into the carveout, resolves
   `rproc->bootaddr` from `e_entry`, cleans the dcache, and calls
   `t113_release_cpu1(entry)` to bring CPU1 out of reset at that address.
4. core1 starts executing from `0x46B002C0`, brings up its own CPU interface
   and UART2 console, and the rpmsg transport comes up over the shared vrings.

## RPMSG transport

rpmsg is live over the carveout vrings.  core0 exposes the slave console as an
rpmsg-tty endpoint (CPU name `cpu1`) and rpmsg ping has been verified
end-to-end.  After `rptun start`, core0's `ls /dev` shows `rptun/` and the
`cpu1` rpmsg-tty node, and `free` shows a `vdev0buffer` pool confirming the
OpenAMP virtio buffer-pool init.

## Monitoring

- core0: UART0 on PF2 (TX) / PF4 (RX) — see `board.h` `T113_UART0_TX_1`.
- core1: UART2 on PE2 (TX) / PE3 (RX) — see `board.h` `T113_UART2_TX_2`.

Two serial cables (or a breakout that exposes both UARTs) let you watch both
shells directly; the core1 console is also reachable as an rpmsg-tty endpoint
from core0.  core0 prompt is `core0>`; core1 prompt is `core1>`.

## Known fixes

Two CPU1-bringup bugs are fixed in the chip code and must stay fixed:

1. **Slave self-reset in `t113_irq.c:up_irqinitialize()`.**  The
   `this_cpu() == 0` block holds CPU1 in reset and initializes the GIC
   distributor — master-only work.  The slave is also `CONFIG_UP=y` and sees
   `this_cpu() == 0` on CPU1, so without a guard it would assert its own
   reset and park CPU1 in WFI forever.  The whole block is wrapped in
   `#ifndef CONFIG_T113_RPTUN_SLAVE`; the slave only runs
   `arm_gic_initialize()` for its own CPU interface.

2. **GIC SPI affinity defaults to CPU0.**  The distributor's target list for
   the slave's SPIs is inherited as `0x01` (CPU0) from the master's
   `arm_gic0_initialize()`.  Without re-targeting, UART2's IRQ (SPI 36)
   lands on the master, which has no handler and reports
   `irq_unexpected_isr: ERROR irq: 36`, so the slave NSH receives no input.
   `t113_bringup()` calls `up_affinity_irq(T113_IRQ_UART2, 1 << 1)` early on
   the slave path.  Additional slave-owned SPIs need their own line there.

## Known gaps

1. **Slave-side peripheral re-init.**  core1 inherits nsh's full driver set
   at link time but keeps each peripheral disabled at Kconfig level via the
   slave defconfig.  Re-enabling any of them on core1 requires solving the
   "single instance, two cores" problem first (refcount + hwspinlock-protected
   init, or a static per-core partition table).
2. **CPU1 cache/MMU state.**  The master cleans the dcache before releasing
   CPU1, so the segments it wrote are coherent at cold fetch.  The slave's own
   cache/MMU re-init in `arm_a_r/arm_head.S` is the canonical ARMv7-A startup;
   no extra slave-side work is needed for the config validated here.
