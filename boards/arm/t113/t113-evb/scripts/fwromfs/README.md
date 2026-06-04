# fwromfs.img — ROMFS firmware image for boot0-nuttx

After boot0-nuttx boots, it mounts the ROMFS on NAND mtd1 to `/fw`.
At the `boot0> ` NSH prompt, type `boot /fw/<x>.cfg` to load and
launch an image.  `fwromfs.img` is that ROMFS image, packed on the
host with `genromfs`.

See [README_zh.md](README_zh.md) for the Chinese version.

## Files in this directory

| File | Purpose |
|------|---------|
| `boot.cfg`       | NuttX single-image template (default; loads `nuttx.bin`) |
| `boot-linux.cfg` | Mainline Linux template (zImage + board.dtb) |
| `boot-amp.cfg`   | NuttX AMP dual-image template (core0 + core1) |
| `README.md`      | This document |
| `README_zh.md`   | Chinese version |

## cfg syntax

One `<addr_hex> <path>` pair per line (whitespace or tab separated).
`#` comment lines and blank lines are ignored.  boot0 loads each file
to its physical address, jumps to `loads[0].addr`, and passes
`loads[1].addr` in r2 (or 0 when there is only one load).

The calling convention always follows ARM Linux DT
(`r0=0, r1=0xFFFFFFFF, r2=image2_phys`): Linux reads r2 as the DTB
physical address; NuttX ignores r2.

## Pack

```sh
mkdir staging
cd staging

# 1. Copy one or more cfgs (pick which one to run at boot time).
cp ../boot.cfg .

# 2. Copy the images referenced by the cfgs.
#
#   NuttX single/SMP (use boot.cfg):
cp ~/projects/t113/build_nsh_smp/nuttx.bin .

#   or mainline Linux (use boot-linux.cfg):
# cp ~/tmp/ws/t113-mainline-fel/zImage    .
# cp ~/tmp/ws/t113-mainline-fel/board.dtb .

#   or NuttX core0 + CPU1 (use boot-amp.cfg):
# cp ~/projects/t113/build_core0/nuttx.bin core0.bin
#   CPU1 rides in fwromfs as an ELF: core0-NuttX loads it with
#   "rptun start /dev/rptun/core1".  Copy the core1 build output as
#   core1.elf.  Do NOT strip it -- the rptun loader (and Linux remoteproc)
#   read the .resource_table section from the ELF.
# cp ~/projects/t113/build_core1/nuttx core1.elf

# 3. Pack the ROMFS image.
genromfs -f /tmp/fwromfs.img -d .
```

## Deploy + boot

```sh
# Write fwromfs.img to mtd1 (offset 0x100000, 1 MB past boot0).
# -k uses skip-bad, aligned with NuttX FTL ftl_init_map on the
# read side.
xfel spinand write -k 0x100000 fwromfs.img
xfel reset

# After reset the board NAND-boots into the boot0> prompt;
# pick a cfg there:
boot0> boot /fw/boot.cfg          # boot NuttX single/SMP
boot0> boot /fw/boot-linux.cfg    # boot Linux
boot0> boot /fw/boot-amp.cfg      # boot NuttX AMP
```

## Size limits

| Limit | Source |
|-------|--------|
| cfg file ≤ 127 bytes | `BOOT_CFG_BUFSIZE = 128` (read cap; 1 byte reserved for NUL) |
| cfg lines ≤ 4 | `BOOT_MAX_LOADS = 4` |
| each image ≤ 16 MB | `IMAGE_MAX_SIZE` |
| `fwromfs.img` ≤ 32 MB | mtd1 physical partition size |

Each violation prints a `boot: ...` syslog line naming the offending
path and the relevant limit, then aborts.
