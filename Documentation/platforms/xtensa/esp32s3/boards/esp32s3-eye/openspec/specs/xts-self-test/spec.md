## ADDED Requirements

### Requirement: XTS 自测套件
系统 SHALL 提供按 Vela xTS 全集 V2.0 通用自测用例标准的自动化测试脚本，覆盖 ESP32-S3-EYE 板核心能力。

#### Scenario: 测试用例数量
- **WHEN** 执行自测脚本
- **THEN** 至少跑 20 个核心 test case，覆盖系统内核 / 系统应用 / 驱动 BSP / 性能 / EYE 专属能力 5 个大类

#### Scenario: 通过率达标
- **WHEN** 自测套件执行完成
- **THEN** 输出通过率 ≥ 95%（≥ 19/20 PASS）

### Requirement: 系统内核测试
自测套件 SHALL 验证 NuttX 内核核心能力：内存管理、调度、系统调用、OS 测试套件。

#### Scenario: mm 内存测试
- **WHEN** 执行 `mm` 命令
- **THEN** 输出 `TEST COMPLETE`，无 ERROR

#### Scenario: ostest 综合测试
- **WHEN** 执行 `ostest` 命令
- **THEN** 输出 `ostest_main: Exiting with status 0`，包含 mutex/sem/mqueue/signals/timers 全部子测试 PASS

#### Scenario: getprime 算力测试
- **WHEN** 执行 `getprime` 命令
- **THEN** 输出 `getprime took <N> msec`，N > 0

#### Scenario: hello 用户态测试
- **WHEN** 执行 `hello` 命令
- **THEN** 输出 `Hello, World!!`

### Requirement: 系统应用测试
自测套件 SHALL 验证系统应用基本功能：reboot、free、df。

#### Scenario: 启动正常
- **WHEN** 执行 `uname -a`
- **THEN** 输出包含 `NuttX` 字符串

#### Scenario: free 显示 RAM
- **WHEN** 执行 `free`
- **THEN** 输出包含 total/used/free 三列并显示 8MB 量级数字

#### Scenario: df 显示文件系统
- **WHEN** 执行 `df`
- **THEN** 输出包含 `Mounted on` 表头

### Requirement: 驱动 BSP 测试
自测套件 SHALL 验证关键驱动设备节点存在和工作：Flash/GPIO/I2C/SPI/UART/RNG。

#### Scenario: 设备节点完整
- **WHEN** 执行 `ls /dev`
- **THEN** 输出包含 `console`, `fb0`, `i2c0`, `null`, `random`, `timer0`, `ttyACM0`, `video0`, `zero`

#### Scenario: I2C/SPI 替代验证
- **WHEN** 执行 `ls /dev/i2c0 /dev/fb0 /dev/video0`
- **THEN** 三个设备节点都存在不报 No such file

#### Scenario: 随机数生成
- **WHEN** 执行 `rand 5`
- **THEN** 输出包含数字（≥ 3 位数字）

### Requirement: EYE 专属能力测试
自测套件 SHALL 验证 ESP32-S3-EYE 板特有的硬件：OV2640 + ST7789 + I2C bus 扫描 + fb 命令。

#### Scenario: i2c bus 扫描
- **WHEN** 执行 `i2c bus`
- **THEN** 输出 `Bus 0: YES` 表示 I2C0 总线就绪（OV2640 SCCB 接此总线）

#### Scenario: fb framebuffer 测试
- **WHEN** 执行 `fb` 命令
- **THEN** 命令完成不挂起，LCD 屏幕显示嵌套同心矩形 pattern

#### Scenario: camera 拍照测试
- **WHEN** 执行 `camera 1` 命令
- **THEN** OV2640 配置 QVGA RGB565 格式，CAM driver 完成 1 张图像捕获，LCD 显示拍照画面

### Requirement: 测试报告生成
自测套件 SHALL 生成结构化的 Markdown 报告，包含通过率、分类统计、详细 case 结果、关键证据。

#### Scenario: 报告输出文件
- **WHEN** 测试套件执行完成
- **THEN** 生成 `/tmp/xts_results.json` 和 `/tmp/xts_report.md` 两个文件

#### Scenario: 报告内容完整性
- **WHEN** 检查 xts_report.md 内容
- **THEN** 包含测试基本信息（时间/测试人/板子/系统/标准）、测试总览（通过率）、分类统计、详细 case 表格、关键证据代码块、结论 5 个章节

### Requirement: 飞书云文档发布
测试报告 SHALL 可通过 feishu MCP 工具发布为飞书云文档供团队查看。

#### Scenario: 飞书文档创建
- **WHEN** 调用 `feishu-mcp create-doc` 上传 markdown 报告
- **THEN** 返回有效的 doc_url，文档可在浏览器打开
