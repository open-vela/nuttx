# T113 boot0 代码体积

## 概述

T113-S3（全志，双核 Cortex-A7）的 boot0 是一个 SPL (Secondary Program
Loader)，完全在 SRAM 中运行。BROM 会将其从 SPI NAND block 0 加载到 SRAM
的 `0x20000` 处。它的职责是：

1. 初始化时钟并训练 DDR。
2. 从 SPI NAND 读取下一级固件（AP 镜像）。
3. 将其复制到 DDR 并跳转执行。

SRAM 总计 160 KB。镜像开头的 eGON header 会被 BROM 校验并读取大小，因此
每个 boot0 变体都必须控制在 SRAM 预算之内，并且必须带有合法的 eGON
header。header 的布局与校验算法参见 `tools/mksunxi.py`。

本仓库目前保留三个独立的 boot0 变体，它们互不干扰，各自服务不同场景：

| 变体                   | 构建系统            | 运行时        | 预期用途                        |
|------------------------|---------------------|---------------|---------------------------------|
| `boot0-noos/`          | 独立 Makefile       | 裸机          | 最精简、可量产的 SPL            |
| `configs/boot0-nuttx/` | NuttX CMake+Ninja   | 完整 NuttX    | 基于 NuttX、带 MTD/miniboot     |
| `configs/boot0/`       | NuttX CMake+Ninja   | 极简 NuttX    | 实验性超小体积 SPL              |

所有 NuttX 变体共享的源文件：

- `arch/arm/src/t113/t113_boot0.S` -- eGON header、异常向量、
  CPU 模式 / 缓存 / 栈初始化、可选的 SMP 次核 park、跳转到
  `_start` 或 `sys_copyself`。
- `arch/arm/src/t113/t113_load.c` -- 裸机 C 加载器：时钟初始化、
  DDR 训练、SPI NAND 读、跳转到 DDR。编译选项为
  `-O3 -fno-lto -fno-stack-protector`；运行在 SRAM `0x30000`、
  DDR 可用之前；**禁止**引用全局变量或调用 libc。

## boot0-noos 路径（独立裸机）

目录：`boards/arm/t113/t113-evb/boot0-noos/`

内容：

```
boot0-noos/
  Makefile          独立 GNU make 构建脚本
  boot0.bin         预构建产物（已入库）
  build/            上次构建的 obj / ELF 输出
    boot0.elf
    boot0_asm.o
    boot0_load.o
```

Makefile 不走 NuttX CMake/Kconfig。它直接调用 `arm-none-eabi-gcc`
编译那两个源文件，用 `boards/arm/t113/t113-evb/scripts/sram.ld` 链接：

```
SRCS_S = arch/arm/src/t113/t113_boot0.S
SRCS_C = arch/arm/src/t113/t113_load.c
CFLAGS = -mcpu=cortex-a7 -marm -O3 -fno-lto -fno-stack-protector \
         -ffunction-sections -fdata-sections -ffreestanding -nostdlib \
         -DCONFIG_T113_BOOT0_NOOS
LDFLAGS = -T scripts/sram.ld -nostdlib \
          --defsym CONFIG_IDLETHREAD_STACKSIZE=0 \
          --defsym CONFIG_ARCH_INTERRUPTSTACK=0
```

链接完成后通过 `objcopy -O binary` 生成 `boot0.bin`，然后
`tools/mksunxi.py --nand` 修补 eGON 的 `spl_size` 和 BROM 校验和
（NAND 需要 1 KB 对齐）。

使用方式（在 `boot0-noos/` 目录下）：

```
make                 # 构建 boot0.bin
make flash           # 需设备处于 FEL 模式
make clean
make CROSS=xxx-      # 覆盖交叉编译器前缀
```

flash 步骤执行：

```
xfel spinand splwrite 2048 1048576 boot0.bin
```

即将 boot0 写到 SPI NAND 偏移 `0x100000`（block 8），页大小 2048 字节。
命令以 Makefile 中的为准。

当前入库的预构建产物大小：

```
boot0-noos/boot0.bin       9216 字节
boot0-noos/build/boot0.elf 76412 字节（含调试符号；在 boot0.bin 中
                                        strip 后为 9 KB）
```

对应的 `configs/boot0-noos/defconfig` 只是一个占位 stub，目的是让 NuttX
CMake 能识别这个配置名称；实际构建由独立的 Makefile 驱动，不走 Kconfig。

## boot0-nuttx 路径（完整 NuttX）

目录：`boards/arm/t113/t113-evb/configs/boot0-nuttx/`

这是一个完整的 NuttX 配置，基于 NuttX 内核 + SPI NAND MTD 驱动 + miniboot
加载框架生成 boot0 镜像。用标准 Vela 流程构建：

```
./build.sh t113-evb/boot0-nuttx -j
```

`configs/boot0-nuttx/defconfig` 中当前设置的关键选项：

| 选项                               | 取值        | 作用                                    |
|------------------------------------|-------------|-----------------------------------------|
| `CONFIG_T113_BOOT0`                | y           | eGON header、SRAM 链接脚本              |
| `CONFIG_BOOT_MINIBOOT`             | y           | 使用 miniboot 加载框架                  |
| `CONFIG_BOOT_RUNFROMISRAM`         | y           | 从内部 SRAM 运行                        |
| `CONFIG_MINIBOOT_SLOT_PATH`        | `/dev/mtd1` | AP 镜像分区                             |
| `CONFIG_MINIBOOT_HEADER_SIZE`      | 0x0         | AP 镜像为裸二进制，无额外头             |
| `CONFIG_T113_DMA`                  | y           | SPI 启用 DMA                            |
| `CONFIG_T113_SPI0`                 | y           | SPI0 驱动（连接 SPI NAND）              |
| `CONFIG_MTD` / `MTD_PARTITION`     | y / y       | 启用 MTD 与分区支持                     |
| `CONFIG_MTD_BYTE_WRITE`            | y           | 启用字节级写入（固件升级用）            |
| `CONFIG_MTD_MX35` / `MX35_QSPI`    | y / y       | Macronix MX35 SPI NAND 驱动（QSPI）     |
| `CONFIG_MTD_GD5F` / `GD5F_QSPI`    | y / y       | Giga GD5F SPI NAND 驱动（QSPI）         |
| `CONFIG_BCH`                       | y           | BCH 库                                  |
| `CONFIG_DEBUG_CUSTOMOPT`           | y           | 自定义优化等级                          |
| `CONFIG_DEBUG_OPTLEVEL`            | `-Os`       | 面向体积的优化                          |
| `CONFIG_DEFAULT_SMALL`             | y           | 小体积默认值                            |
| `CONFIG_ARCH_INTERRUPTSTACK`       | 1024        | 共享 IRQ 栈                             |
| `CONFIG_IDLETHREAD_STACKSIZE`      | 4096        | idle 任务栈                             |
| `CONFIG_DEFAULT_TASK_STACKSIZE`    | 4096        | 默认任务栈                              |
| `CONFIG_PTHREAD_STACK_DEFAULT`     | 1024        | 默认 pthread 栈                         |
| `CONFIG_SCHED_TICKLESS`            | y           | tickless 调度                           |
| `CONFIG_SYSTEM_NSH`                | y           | 内置 NSH（便于调试 boot0）              |
| `CONFIG_NSH_PROMPT_STRING`         | `"boot0>"`  | 与完整 NuttX shell 区分                 |

`boot0-nuttx` 是"齐全"路径：保留 MTD、miniboot 框架和一个精简的 NSH，
便于工程师在 bring-up 阶段从 boot0 shell 直接访问 SPI NAND。体积因此
明显大于裸机 `boot0-noos` 变体。

## boot0 路径（实验性超小体积 NuttX）

目录：`boards/arm/t113/t113-evb/configs/boot0/`

第三个变体，目标是最小可用的 NuttX-based boot0。保留在树中用于
实验，但目前存在若干已知问题（见下方"注意事项"）。

关键设置：

| 选项                                | 取值   | 意图                                           |
|-------------------------------------|--------|------------------------------------------------|
| `CONFIG_T113_BOOT0`                 | y      | eGON header、SRAM 链接脚本                     |
| `CONFIG_INIT_NONE`                  | y      | 不创建 init 任务（由 board 钩子完成跳转）      |
| `CONFIG_LTO_FULL`                   | y      | 全量 LTO                                       |
| `CONFIG_DEBUG_OPTLEVEL`             | `-Os`  | 面向体积的优化                                 |
| `CONFIG_ARM_DCACHE_DISABLE`         | y      | 跳过 D-cache 初始化路径                        |
| `CONFIG_ARCH_MINIMAL_VECTORTABLE`   | y      | 压缩向量表                                     |
| `CONFIG_ARCH_NUSER_INTERRUPTS`      | 1      | 只保留 timer 中断槽                            |
| `CONFIG_ARCH_INTERRUPTSTACK`        | 512    | 更紧的 IRQ 栈                                  |
| `CONFIG_MTD` / `MTD_MX35`           | y      | 仅保留 MTD + MX35 驱动                         |
| `CONFIG_SYSLOG_NONE`                | y      | 禁用日志                                       |
| `CONFIG_NAME_MAX`                   | 8      | 缩短 inode 名称表                              |
| `CONFIG_DEV_CONSOLE`（未设置）      | n      | 不挂控制台设备                                 |
| `CONFIG_ARCH_FPU`（未设置）         | n      | 不保存 VFP/NEON 上下文                         |

### 注意事项

该 defconfig 中有两处选项引用了当前树里并不存在的符号：

- `CONFIG_MTD_READONLY` 仅在 `drivers/mtd/gd5f.c` 中出现
  （gate 一段写入代码），但 `drivers/mtd/Kconfig` 中**没有**对应的
  `config MTD_READONLY` 声明。因此 defconfig 里的这一行对 MTD 栈其余部分
  无效，特别是**并未**从 `ftl.c` 或 `mx35.c` 中裁剪写路径。
- `CONFIG_INIT_NONE` 在 Kconfig 层面是合法的，但
  `sched/init/nx_bringup.c` 在未被任何条件保护的位置仍然留着
  `#error No initialization mechanism selected (CONFIG_INIT_NONE)`
  （约 70 行）。虽然 `task_spawn` 调用处（约 296 行）外层包了
  `#ifndef CONFIG_INIT_NONE`，但要让该变体以 `INIT_NONE=y` 完整编译
  通过，仍需先移除前面那个 `#error`。

请将 `configs/boot0/` 视为研究性产物：它记录了想法方向，但还需要
Kconfig / 源码改动才能编译通过。今天真正能构建的两条路径是
`boot0-noos`（裸机）和 `boot0-nuttx`（完整 NuttX）。

## eGON header 与 BROM 流程

T113-S3 上每个可被 BROM 加载的镜像都以一个 eGON header 开头
（.bin 文件最前面的 64 字节）。布局如下：

```
offset 0x00: 4 字节 ARM 跳转指令（跨过 header）
offset 0x04: 8 字节魔数 "eGON.BT0"
offset 0x0C: 4 字节校验和（由 mksunxi.py 回填）
offset 0x10: 4 字节 spl_size（由 mksunxi.py 回填）
offset 0x14: 4 字节 header 大小（0x30）
offset 0x18: 4 字节 header 版本
offset 0x1C: 4 字节返回值
offset 0x20: 4 字节建议运行地址
```

校验和算法：将 offset `0x0C` 用种子 `0x5F0A6C39` 替换，然后对
`[0, spl_size)` 范围内的所有 32-bit word 求和，结果写回 offset `0x0C`。

对齐要求（来自 T113 BROM 反汇编）：

- SPI NAND：1 KB（`bfc r0, #10, #22`，BROM 0x1778）
- SPI NOR： 512 B（`bfc r0, #9, #23`，BROM 0x1AD8）

`tools/mksunxi.py --nand <file>` 或 `--nor <file>` 会按介质要求对齐
`spl_size` 并重算校验和。三个 boot0 变体都会经过它后处理。

## 烧录

从 FEL 模式（`reboot 99` 或 JLink 触发后）执行：

```
xfel spinand splwrite 2048 1048576 <path-to-boot0.bin>
```

`2048` 是 SPI NAND 页大小，`1048576` 是写入偏移
（`0x100000`，block 8）。对 `boot0-noos`，`make flash` 已经包含
这条命令。

AP 镜像烧录和完整文件系统部署请参考顶层 boards/t113-evb 的 flash /
deploy 文档。

## 测量体积

对独立 `boot0-noos` 构建：

```
cd boards/arm/t113/t113-evb/boot0-noos
make
arm-none-eabi-size build/boot0.elf
ls -l boot0.bin
```

对任一基于 NuttX 的变体，构建完成后：

```
arm-none-eabi-size nuttx
ls -l nuttx.bin
```

按符号粒度查看：

```
arm-none-eabi-nm --size-sort --radix=d nuttx | tail -40
```

`tools/bloaty` 或 `tools/size.py` 也可做 section 级归因。**不要**在设计文档里
引用不是从当前树的实际构建中抓来的体积数字，这类数字很快就会过期。
