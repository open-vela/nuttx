# fwromfs.img — boot0-nuttx 启动用的 ROMFS firmware image

boot0-nuttx 启动后挂 NAND mtd1 的 ROMFS 到 `/fw`,在 `boot0> ` NSH
prompt 上敲 `boot /fw/<x>.cfg` 加载并启动 image。fwromfs.img 就是该
ROMFS image,由 host 端 `genromfs` 打包。

英文版见 [README.md](README.md)。

## 这个目录的内容

| 文件 | 用途 |
|------|------|
| `boot.cfg`       | NuttX 单 image 启动模板(默认,加载 `nuttx.bin`)|
| `boot-linux.cfg` | mainline Linux 启动模板(zImage + board.dtb)|
| `boot-amp.cfg`   | NuttX AMP 双 image 启动模板(core0 + core1)|
| `README.md`      | 英文版 |
| `README_zh.md`   | 本文档 |

## cfg 文法

每行一个 `<addr_hex> <path>` 对(空格或制表符分隔),`#` 注释行 + 空
行忽略。boot0 加载每个文件到对应物理地址,跳 `loads[0].addr`,
r2 = `loads[1].addr`(单 image 时 r2 = 0)。

调用约定永远按 ARM Linux DT(`r0=0, r1=0xFFFFFFFF, r2=image2_phys`):
Linux 读 r2 作 DTB,NuttX 忽略 r2。

## 打包流程

```sh
mkdir staging
cd staging

# 1. 拷一个或多个 cfg(运行时挑哪个用)
cp ../boot.cfg .

# 2. 拷 cfg 引用的 image 文件
#
#   NuttX 单核/SMP(用 boot.cfg):
cp ~/projects/t113/build_nsh_smp/nuttx.bin .

#   或 mainline Linux(用 boot-linux.cfg):
# cp ~/tmp/ws/t113-mainline-fel/zImage    .
# cp ~/tmp/ws/t113-mainline-fel/board.dtb .

#   或 NuttX core0 + CPU1(用 boot-amp.cfg):
# cp ~/projects/t113/build_core0/nuttx.bin core0.bin
#   CPU1 以 ELF 形式放进 fwromfs:core0-NuttX 用
#   "rptun start /dev/rptun/core1" 加载它。把 core1 的构建产物拷成
#   core1.elf,且不要 strip —— rptun loader(以及 Linux remoteproc)需要读取
#   ELF 里的 .resource_table 段。
# cp ~/projects/t113/build_core1/nuttx core1.elf

# 3. 打 ROMFS image
genromfs -f /tmp/fwromfs.img -d .
```

## 部署 + 启动

```sh
# 把 fwromfs.img 写进 mtd1(boot0 1MB 之后,offset 0x100000)。
# -k 走 skip-bad,与 NuttX FTL 的 ftl_init_map 对齐。
xfel spinand write -k 0x100000 fwromfs.img
xfel reset

# 复位后板子从 NAND boot 进 boot0> prompt,在 prompt 上挑一个 cfg:
boot0> boot /fw/boot.cfg          # 启动 NuttX 单核/SMP
boot0> boot /fw/boot-linux.cfg    # 启动 Linux
boot0> boot /fw/boot-amp.cfg      # 启动 NuttX AMP
```

## 大小约束

| 限制 | 来源 |
|------|------|
| cfg 单文件 ≤ 127 字节 | `BOOT_CFG_BUFSIZE = 128`(读上限,留 1 字节给 NUL)|
| cfg 行数 ≤ 4 | `BOOT_MAX_LOADS = 4` |
| 单 image ≤ 16 MB | `IMAGE_MAX_SIZE` |
| fwromfs.img ≤ 32 MB | mtd1 物理分区大小 |

任一约束被破坏时,boot0 会打印一行 `boot: ...` syslog,指出违规的
路径和相关上限,然后中止启动。
