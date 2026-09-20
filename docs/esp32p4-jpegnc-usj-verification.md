# ESP32-P4 jpegnc cleanup hang + USJ console wedge — 验证报告

- **日期**: 2026-08-11
- **分支**: `feat/esp32p4-support`
- **设备**: ESP32-P4(USJ console 115200,`/dev/ttyACM*`)
- **修复**: 方案 A(userland cleanup 移除多余 QBUF)+ 方案 B(driver `out_pending` 门控)

---

## 1. 问题现象

在 app 阶段运行 `jpegenc`(V4L2 M2M JPEG 编码,/dev/video1)后出现两类问题:

1. **jpegenc cleanup 卡死**:编码完成后进程在 cleanup 阶段挂起,console 无新输出,设备不再响应。
2. **USJ console 乱码/wedge**:console 输出 `'k'` flood 或完全静默;`echo` 等命令无回显,串口被 `nsh>` 之外的噪声占据。

两者叠加表现为:运行 `jpegenc` 后系统「假死」。

## 2. 根因分析

### 2.1 jpegenc cleanup 卡死(方案 A 针对)

`jpegenc_main.c` 的 cleanup 段在 `VIDIOC_STREAMOFF` 之前对已 `DQBUF` 的 capture buffer 再次执行 `VIDIOC_QBUF`。

该 re-queue 会让 v4l2_m2m 核心再次触发 `CODEC_CAPTURE_AVAILABLE -> jpeg_encode_one()`,对**已经消费过的 output frame 进行第二次编码**。此时驱动与中断栈相互耦合,编码第二次进入时与 console flush 抢占资源,最终 cleanup 阻塞、`fflush`/`tcdrain` 永不返回。

> 该 re-queue 本身无意义:`close(fd)` 会释放所有 buffer。

### 2.2 USJ console wedge(interrupt path)

- USJ TX 只有 **64 字节 FIFO**;USB IN 事务以 **ZLP(zero-length packet)** 结束不完整事务。
- `IN_EMPTY` 是**电平**中断:只要 FIFO 非空就持续触发,若驱动在 FIFO drain 后未正确 disarm,则中断风暴持续。
- 原 `esp_txint()` 无条件 disarm IN_EMPTY,配合 `esp_sendbuf`(interrupt path)的 chunk flush 时序,大块输出(boot M dump、jpegnc 编码日志)时事务边界错位,IN_EMPTY 状态残留 → 中断反复触发 → console wedge。

### 2.3 console 乱码 'k' flood 根因

另有一条独立根因链(UART0 遗留中断风暴):

- ROM 把 **UART0(intr source 31)** 路由到 CPU int 5(CLIC 21),但软件从未注册服务函数。
- JPEG 编码器与 UART0 共享 `cpuint 5` 后,无人服务的 UART0 中断变成 **level storm**,产生 `'k'`(0x6b)flood → 冲垮 console → jpegnc hang。
- boot 后由 bringup 显式清理:写 `UART_INT_CLR=0xffffffff`、`UART_INT_ENA=0`,并把 `INTR_CORE0 map[31]` 改为 `INT_MUX_DISABLED_INTNO`,消灭该风暴。

## 3. 修复方案

### 方案 A — userland(`apps/examples/jpegenc`)

`jpegenc_main.c` cleanup 段**移除多余的 `VIDIOC_QBUF`**,仅保留两个 `VIDIOC_STREAMOFF` + `close()`,并加注释说明原因。

### 方案 B — driver(`esp_jpeg_enc.c`)

`jpeg_encode_one()` 增加 `out_pending` 门控:

```c
if (d->engine == NULL || !d->out_pending)
    return 0;
```

- `out_pending` 在 OUTPUT 队列有 pending frame 时才置位。
- 防止 CAPTURE_AVAILABLE 误触发对已消费 frame 的二次编码(与方案 A 双保险)。

### console 门控(`esp_usbserial.c`)

- `dbg_putc()`:写 RAM marker 后,`if (!g_dbg_console_tx) return;` 门控 polled TX。
- `dbg_console_tx_set(bool)`:**boot 阶段**开启 polled console TX(M dump 必须走 console);**app 阶段**关闭,只写 RAM marker,从源头避免 app 大块输出触发 USJ wedge。
- `esp_sendbuf()`(interrupt path)chunk + flush-once;`esp_send()`(polled path)per-byte + flush。
- ZLP/IN_EMPTY:interrupt handler 在 xmit drain 后、FIFO free 时发 ZLP 并 disarm。

## 4. 验证证据(boot M dump)

多次 boot 的 M dump 一致给出成功路径证据:

```
m  = 0x0000007a ('z')          -> jpegnc COMPLETE
sreg = 323220 (0x4EE94)        -> JPEG encoded size (bytes)
fsrc = 0x444F4E45 ("DONE")     -> jfd DONE sentinel(flush 完成、进程干净退出)
SOI  = 0xe0ffd8ff              -> JPEG 文件头(FFD8 = SOI)有效
```

关键行(节选自 boot dump):

```
ABCDM0000007a:...:e0ffd8ffC:00004642:...
```

两次独立日志(`final_verify2.log`、`diag_jpegnc.log`)均检出 `0000007a` 与 `e0ffd8ff`,确认:

- 编码成功,JPEG 文件头完整(SOI 存在);
- app 走到 DONE sentinel(cleanup 不再卡死);
- boot 阶段 console 正常输出 M dump(USJ TX 门控在 boot 期开启)。

> USJ app 阶段静默仍偶发(见 §6),但 boot 与诊断路径已稳定。

## 5. 提交清单

| 仓库 | Commit | 说明 |
|------|--------|------|
| nuttx | `08af6ac5989` | `fix(esp32p4): jpegenc cleanup hang + USJ console wedge`(方案 A+B、console 门控、UART0 风暴修复) |
| nuttx | `5ec26140474` | `debug(esp32p4): RAM markers, dispatch storm counter, M dump`(调试基础设施) |
| nuttx | `f6f596dc903` | `chore(esp32p4): ignore session logs and unused jpeg driver draft` |
| apps | `d401a6bf2` | `examples/jpegenc: add V4L2 M2M JPEG encode example`(方案 A userland) |

未提交(非本次任务):`crypto/mbedtls mbedtls_config.h`、`examples/camcap`、`nist-sts` 子模块、`logs/`(已 gitignore)。

## 6. 遗留问题

1. **USJ app 阶段静默仍偶发**:`dbg_console_tx_set(false)` 关闭了 app 的 console TX,大块 printf 在 app 阶段可能无回显;恢复方案为发空命令触发 NSH 新提示符冲刷(`final_verify3.py` 已就绪,待下次设备上电验证)。
2. **`esp_eth: create link timer failed`**:每次 boot 出现一次,不影响本次功能,独立跟踪。

## 7. 复现与回归

- `jpegenc`(需 `CONFIG_EXAMPLES_JPEGENC` + `CONFIG_ESP32P4_JPEG_ENCODER`):
  - 输入:`/data/capture.rgb565`(RGB565,1228800 bytes)
  - 输出:`/data/capture.jpg`(校验 SOI `e0ffd8ff`)
- 回归检查点:boot M dump `m='z'` + `fsrc=0x444F4E45` + `sreg>0`;`echo` 回显正常。
