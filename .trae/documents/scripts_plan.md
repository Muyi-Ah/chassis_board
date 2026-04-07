# Python 脚本开发计划 (串口收发)

## 1. 摘要 (Summary)
根据需求，在项目根目录新建 `scripts` 文件夹，并编写两个基于 Python 的串口通信脚本：
- `rx_csv.py`: 接收 STM32 (如 USART2 波特率 921600) 发送的逗号分隔数据，并自动带上时间戳保存为 CSV 文件。
- `tx_cmd.py`: 配合 STM32 的接收功能，发送纯文本格式的指令（类似 Vofa+ FireWater 协议）。支持交互式参数输入，并在按下 `ESC` 键时触发停止发送的指令逻辑。
两个脚本均在运行时自动扫描可用的 COM 端口并提供列表供用户手动选择。

## 2. 现状分析 (Current State Analysis)
- 当前项目下还没有专用的上位机/测试脚本目录。
- STM32 端 `USART2` 已经配置为 `921600` 波特率，且目前使用纯文本格式向外发送数据 (`motor1.RPM, ...\r\n`)。
- 发送给 STM32 的指令（下行指令）也采用相同的纯文本方式最便于解析和调试。
- 用户的 Python 环境使用 `uv` 进行依赖管理，因此不需要脚本代为执行 `pip install`，仅需在脚本头部或注释中声明依赖。

## 3. 拟进行的更改 (Proposed Changes)

### 3.1 创建脚本目录
- **路径**: `scripts/`（位于项目根目录）

### 3.2 编写接收脚本 `scripts/rx_csv.py`
- **依赖**: `pyserial`
- **功能**:
  - 启动时调用 `serial.tools.list_ports.comports()` 获取当前设备所有串口，打印列表并要求用户输入数字选择。
  - 打开串口后，以当前时间生成类似 `data_20260407_213000.csv` 的文件。
  - 进入循环读取串口数据 (`readline`)，解码为字符串并原样写入 CSV 文件（因为 STM32 端已经是逗号分隔的格式）。
  - 支持 `Ctrl+C` 优雅退出并关闭文件/串口。

### 3.3 编写发送脚本 `scripts/tx_cmd.py`
- **依赖**: `pyserial`, `keyboard`
- **功能**:
  - 同样具备自动扫描并选择串口的功能。
  - 提供一个命令输入循环 (CLI)：用户可以输入指令和参数（例如 `start`，`motor 1 1000`）。
  - 使用 `keyboard.on_press_key('esc', callback)` 注册一个后台热键监听。当按下 `ESC` 时，脚本会自动向串口发送预定的停止指令（例如 `CMD:STOP\r\n`），并中断可能正在进行的连续发送逻辑。
  - **指令协议规范 (纯文本)**：
    - 开始发送: `CMD:START\r\n`
    - 停止发送 (ESC触发): `CMD:STOP\r\n`
    - 电机控制示例: `MOTOR:<id>:<rpm>\r\n` (后续 STM32 端可依此字符串规则利用 `sscanf` 或 `strtok` 解析)。

## 4. 假设与决策 (Assumptions & Decisions)
- **波特率硬编码**: 默认波特率为 `921600`，与 `usart.c` 中 USART2 的配置保持一致。如果用户需要其它波特率，可以在脚本开头的常量中修改。
- **发送模式**: 按下 ESC 是发送一个停止指令让 STM32 停下（而非单纯断开 Python 串口）。为了扩展性，脚本结构被设计成可以轻松添加新指令分支。
- **环境依赖**: 用户将自行通过 `uv pip install pyserial keyboard` 或其他 `uv` 方式安装依赖，脚本中将添加相关的注释提醒。

## 5. 验证步骤 (Verification Steps)
1. 检查根目录下是否成功生成 `scripts/rx_csv.py` 和 `scripts/tx_cmd.py`。
2. （用户操作）通过 `uv` 补齐 `pyserial` 和 `keyboard` 依赖。
3. （用户操作）在终端中运行 `python scripts/rx_csv.py`，测试其能否成功扫描串口并记录 CSV 文件。
4. （用户操作）在终端中运行 `python scripts/tx_cmd.py`，测试输入指令和按下 `ESC` 时，控制台能否正常打印对应下发的字符串。