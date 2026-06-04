# T113 cdcacm 远端测试包

Staging:`opt7050:~/remote-deploy/t113-cdcacm-bench/`

## 文件清单

| 文件 | 用途 |
|---|---|
| `nuttx.bin` | cdcopt build 产物(sha256 `e79f63cd...`,size 327258) |
| `boot_capture.py` | pyserial **预先 open** ttyACM0/ACM1 → FEL+xfel → 抓 boot log。诊断 boot 阶段用 |
| `bench.py` | 双串口 bench(NSH 控制 `/dev/ttyACM0` + bulk 数据 `/dev/ttyACMx`),3 case × 20MB |
| `remote_bench.sh` | 一键自动:FEL → xfel → bench(包内 journalctl PID bug 已修) |

## 物理拓扑(已知)

- `/dev/ttyACM0` = CH343 if00 = **NSH 控制通道**(要确认接到板子 UART0)
- `/dev/ttyACM1` = CH343 if02
- 板子 USB OTG 接 opt7050,sercon 后**应该**枚举出新 ttyACMx = bulk 通道

## 手动操作

### 0. 准备(每次重测前)

```bash
ssh opt7050
cd ~/remote-deploy/t113-cdcacm-bench
ls -1 /dev/ttyACM* | tee /tmp/pre_tty.list   # 记录启动前 tty 列表
sha256sum nuttx.bin                          # 确认是 e79f63cd... 开头
```

### 1A. 仅诊断 boot(看 boot log)

```bash
python3 boot_capture.py
# log 落 ~/remote-deploy/t113-cdcacm-bench/diag-pyserial/{ttyACM0,ttyACM1}.{log,bin} + jlink.log + xfel.log
```

### 1B. J-Link halt 看 CPU 当前 PC(确认 nuttx 是否真在跑)

```bash
printf 'connect\nhalt\nregs\nexit\n' | timeout 15 \
  JLinkExe -device Cortex-A7 -if JTAG -speed 20000 -nogui 1
# PC ∈ [0x40000000, 0x40051488] 区间 = nuttx 在跑
# PC = 0 / BROM region / 别处       = boot 失败
```

### 1C. 完整 bench(前提:boot 已 OK)

```bash
bash remote_bench.sh
# 结果:results/{B1,B2,B3}.log + summary.json + dmesg.log
```

或分步手动:

```bash
# FEL
printf 'connect\nhalt\nw4 0x07090100, 0x5AA50001\nw4 0x020500B8, 0x16AA0000\nw4 0x020500A8, 0x16AA0001\nexit\n' \
  | timeout 15 JLinkExe -device Cortex-A7 -if JTAG -speed 20000 -nogui 1
sleep 1
xfel version
xfel ddr t113-s3
xfel write 0x40000000 ~/remote-deploy/t113-cdcacm-bench/nuttx.bin
xfel exec 0x40000000

# 等启动后,跑 bench
sleep 2
python3 bench.py /dev/ttyACM0
```

### 2. 释放 tty(verify 没人占)

```bash
lsof /dev/ttyACM0 /dev/ttyACM1 2>/dev/null
# 空 = 释放;有 PID 就 kill
```

### 3. 拉 log 回本地(在本地跑)

```bash
mkdir -p ~/projects/t113/cdcopt/cdcbench/manual-$(date -u +%Y%m%dT%H%M%SZ)
rsync -av opt7050:remote-deploy/t113-cdcacm-bench/{diag-pyserial,results}/ \
  ~/projects/t113/cdcopt/cdcbench/manual-$(date -u +%Y%m%dT%H%M%SZ)/
```

### 4. cleanup(任务收尾时)

```bash
ssh opt7050 'rm -rf ~/remote-deploy/t113-cdcacm-bench'
```

## 已知问题(诊断历史)

- **boot 后 USB CDC 不 enum**:dmesg 只看到 BROM FEL device(1f3a:efe8) disconnect,无新 USB device → nuttx OTG 没起来
- **CH343 两口都 0 字节**:UART0 console 输出**没到**任何 CH343 口
- 两种可能:nuttx 早期 hang(在 console init 之前)/CH343 物理接的不是 t113 UART0
- 用 1B (J-Link halt + regs) 可以判定 CPU 在哪
