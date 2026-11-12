# 示例索引

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + MCU 串口转 CAN 桥 + Damiao 串口桥）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选串口桥链路：`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。
- 通用 MCU 串口转 CAN 桥：`--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。注意与 Damiao 专用串口桥 `dm-serial` 是两种协议：`mcu-serial` 由桥固件透传经典 CAN 帧，`dm-serial` 承载 Damiao 串口协议。适配经典 CAN 全品牌（damiao / robstride / myactuator / hightorque；hexfellow 仅 CAN-FD 不适用）；桥固件在 bus-off / 发送失败 / 接收丢弃时会主动上报结构化链路状态；四个前端（motor_abi / Python / motor_cli / ws_gateway）均支持。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


这里是当前 `motorbridge` 跨语言示例的总入口。

> English version: [README.md](README.md)

## 覆盖范围

- Rust CLI: `motor_cli/src/main.rs`
- C ABI 示例: `examples/c/c_abi_demo.c`
- C++ ABI 示例: `examples/cpp/cpp_abi_demo.cpp`
- Python ctypes 示例: `examples/python/python_ctypes_demo.py`
- 多厂商位置同步脚本: `examples/python/four_vendor_pos_sync.py`
- WS 四电机同步上位机: `examples/web/ws_quad_sync_hmi.html`
- WS 四路独立滑杆上位机: `examples/http_quad_control_demo/README.zh-CN.md`
- Python SDK 示例: `bindings/python/examples/*`
- C++ wrapper 示例: `bindings/cpp/examples/*`
- Damiao 调参总表:
  - `../motor_cli/DAMIAO_API.md`
  - `../motor_cli/DAMIAO_API.zh-CN.md`
- RobStride 参数/API 总表:
  - `../motor_cli/ROBSTRIDE_API.md`
  - `../motor_cli/ROBSTRIDE_API.zh-CN.md`
- MyActuator 指令/模式总表:
  - `../motor_cli/MYACTUATOR_API.md`
  - `../motor_cli/MYACTUATOR_API.zh-CN.md`

## 示例支持的厂商

- Damiao:
  - 模式: `enable`、`disable`、`mit`、`pos-vel`、`vel`、`force-pos`
  - 寄存器和改 ID 流程仍主要走 CLI、Python SDK 和校准工具
- RobStride:
  - 模式: `ping`、`enable`、`disable`、`mit`、`vel`、`read-param`、`write-param`
  - 参数示例走 RobStride 的 ABI / binding 接口
- MyActuator:
  - 模式: `scan`、`enable`、`disable`、`stop`、`status`、`current`、`vel`、`pos`、`version`、`mode-query`
  - CLI 的 `pos`/`vel` 输入统一用弧度制（内部会转换到协议角度）

## CAN 初始化

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
ip -details link show can0
```

## Windows 实验支持（PCAN-USB）

项目主线仍以 Linux 为主。Windows 支持为实验性能力，当前通过 PEAK PCAN 后端实现。

- 安装 PEAK 驱动与 PCAN-Basic 运行时（`PCANBasic.dll`）。
- Windows 通道建议使用 `can0@1000000`。

Windows 快速验证命令：

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

## 快速开始

Damiao 的 Rust CLI 示例:

```bash
cargo run -p motor_cli --release -- \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```

RobStride 的 Rust CLI 示例:

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode ping
```

RobStride 读参数:

```bash
cargo run -p motor_cli --release -- \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019
```

MyActuator 的 Rust CLI 示例:

```bash
cargo run -p motor_cli --release -- \
  --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 \
  --mode pos --pos 3.1416 --max-speed 5.236 --loop 1 --dt-ms 50
```

## 跨语言 ABI 示例

Python ctypes:

```bash
cargo build -p motor_abi --release
python3 examples/python/python_ctypes_demo.py --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
python3 examples/python/python_ctypes_demo.py --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

C:

```bash
cargo build -p motor_abi --release
cc examples/c/c_abi_demo.c -I motor_abi/include -L target/release -lmotor_abi -o c_abi_demo
LD_LIBRARY_PATH=target/release ./c_abi_demo --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
LD_LIBRARY_PATH=target/release ./c_abi_demo --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

C++:

```bash
cargo build -p motor_abi --release
g++ -std=c++17 examples/cpp/cpp_abi_demo.cpp -I motor_abi/include -L target/release -lmotor_abi -o cpp_abi_demo
LD_LIBRARY_PATH=target/release ./cpp_abi_demo --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
LD_LIBRARY_PATH=target/release ./cpp_abi_demo --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode mit
```

## 推荐的高层示例

- Python SDK:
  - `bindings/python/examples/python_wrapper_demo.py`
  - `bindings/python/examples/robstride_wrapper_demo.py`
- C++ wrapper:
  - `bindings/cpp/examples/cpp_wrapper_demo.cpp`
  - `bindings/cpp/examples/robstride_wrapper_demo.cpp`

## 建议验证顺序

1. 先做总线全厂商扫描。
2. 验证 Damiao 控制链路（MIT 或速度）。
3. 验证 RobStride 控制链路（ping/读参/速度）。
4. 验证 MyActuator 控制链路（位置或速度）。
5. 验证 Python binding 示例（Damiao + RobStride）。
6. 验证 C++ binding 示例（Damiao + RobStride）。

快速命令：

```bash
# 1) 统一扫描
cargo run -p motor_cli --release -- --vendor all --channel can0 --mode scan --start-id 1 --end-id 255

# 2) Damiao 速度控制
cargo run -p motor_cli --release -- --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode vel --vel 0.5 --loop 40 --dt-ms 50

# 3) RobStride ping + 速度
cargo run -p motor_cli --release -- --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD --mode ping
cargo run -p motor_cli --release -- --vendor robstride --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD --mode vel --vel 0.3 --loop 40 --dt-ms 50

# 4) MyActuator 位置模式（弧度）
cargo run -p motor_cli --release -- --vendor myactuator --channel can0 --model X8 --motor-id 1 --feedback-id 0x241 --mode pos --pos 3.1416 --max-speed 5.236 --loop 1 --dt-ms 50

# 5) 多厂商位置同步脚本（Damiao x2 + MyActuator + HighTorque）
python3 examples/python/four_vendor_pos_sync.py \
  damiao 0x01 damiao 0x07 myactuator 1 hightorque 1 \
  --pos 1.57 --damiao-model-by-id "0x01=4340P,0x07=4310" --stagger-ms 50

# 6) Web 上位机（单拖杆同步四电机角度，走 ws_gateway）
cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
python3 -m http.server 18080
# 浏览器打开: http://127.0.0.1:18080/examples/web/ws_quad_sync_hmi.html
```

## 说明

- `id-dump` 仍偏 Damiao 工作流；`id-set` 支持 Damiao 和 RobStride 电机 device ID 修改。统一 `scan` 已支持 Rust CLI（`--vendor all`）和 Python SDK CLI（`motorbridge.cli scan --vendor all`）。
- RobStride 目前重点覆盖 `ping`、参数访问、MIT、速度控制。
