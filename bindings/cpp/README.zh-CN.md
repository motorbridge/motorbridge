# motorbridge C++ 绑定

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + CAN-FD + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- CAN-FD 链路可通过 CLI（`--transport socketcanfd`）和 C++ SDK（`Controller::from_socketcanfd(...)`）使用，Hexfellow 必须走该链路。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


这是基于 `motor_abi` 的 RAII 风格 C++ 包装层。

> English version: [README.md](README.md)

## ABI 元数据 helper

- `motorbridge::abi_version()` 返回当前加载的 ABI 库版本。
- `motorbridge::abi_capabilities_json()` 返回当前加载 ABI 的能力 JSON。
- `bindings/api_surface.json` 用于跟踪 binding 接口对齐。

## Damiao 置零规则（dm-serial）

- Damiao 场景下，`set_zero_position()` 前先调用 `disable()`。
- 核心层已加入防护：非失能状态调用 `set_zero_position()` 会被拒绝。
- 核心层在 `set_zero_position()` 后内置固定稳定等待（约 `20ms`）。
- C++/ABI 函数签名未变，本次属于核心行为防护升级。

## Controller 入口

- `Controller(channel)`（SocketCAN/PCAN 路径）
- `Controller::from_socketcanfd(channel)`（CAN-FD 路径，Hexfellow 必须使用）
- `Controller::from_dm_serial(serial_port, baud)`（仅 Damiao 串口桥）
- `add_damiao_motor(motor_id, feedback_id, model)`
- `add_hexfellow_motor(motor_id, feedback_id, model)`
- `add_myactuator_motor(motor_id, feedback_id, model)`
- `add_robstride_motor(motor_id, feedback_id, model)`
- `add_hightorque_motor(motor_id, feedback_id, model)`
- `add_fashionstar_motor(motor_id, feedback_id, model)`

## 快速开始

Damiao:

```cpp
#include "motorbridge/motorbridge.hpp"

int main() {
  motorbridge::Controller ctrl("can0");
  auto motor = ctrl.add_damiao_motor(0x01, 0x11, "4340P");
  ctrl.enable_all();
  motor.ensure_mode(motorbridge::Mode::MIT, 1000);
  motor.send_mit(0.0f, 0.0f, 20.0f, 1.0f, 0.0f);
  ctrl.shutdown();
  return 0;
}
```

Damiao 串口桥示例：

```cpp
#include "motorbridge/motorbridge.hpp"

int main() {
  auto ctrl = motorbridge::Controller::from_dm_serial("/dev/ttyACM1", 921600);
  auto motor = ctrl.add_damiao_motor(0x04, 0x14, "4310");
  ctrl.enable_all();
  motor.send_mit(0.5f, 0.0f, 20.0f, 1.0f, 0.0f);
  ctrl.shutdown();
  return 0;
}
```

RobStride:

```cpp
#include "motorbridge/motorbridge.hpp"

int main() {
  motorbridge::Controller ctrl("can0");
  auto motor = ctrl.add_robstride_motor(127, 0xFD, "rs-00");
  auto ids = motor.robstride_ping();
  float pos = motor.robstride_get_param_f32(0x7019);
  ctrl.shutdown();
  return static_cast<int>(ids.first == 127 && pos > -1000.0f);
}
```

RobStride 专用 wrapper helper 包括：

- `robstride_ping()`
- `robstride_ping_host_id(host_id, timeout_ms)`
- `robstride_get_param_f32_host_id(param_id, host_id, timeout_ms)`
- `robstride_get_fault_report()`
- `robstride_set_device_id(new_device_id)`
- `robstride_set_active_report(enabled)`
- `robstride_get_param_i8/u8/u16/u32/f32(param_id, timeout_ms)`
- `robstride_write_param_i8/u8/u16/u32/f32(param_id, value)`

MyActuator:

```cpp
#include "motorbridge/motorbridge.hpp"

int main() {
  motorbridge::Controller ctrl("can0");
  auto motor = ctrl.add_myactuator_motor(1, 0x241, "X8");
  ctrl.enable_all();
  motor.ensure_mode(motorbridge::Mode::POS_VEL, 1000);
  motor.send_pos_vel(3.1416f, 2.0f);  // rad / rad/s
  ctrl.shutdown();
  return 0;
}
```

Hexfellow（仅 CAN-FD）：

```cpp
#include "motorbridge/motorbridge.hpp"

int main() {
  auto ctrl = motorbridge::Controller::from_socketcanfd("can0");
  auto motor = ctrl.add_hexfellow_motor(0x01, 0x00, "hexfellow");
  ctrl.enable_all();
  motor.ensure_mode(motorbridge::Mode::MIT, 1000);  // Hexfellow 仅支持 MIT / POS_VEL
  motor.send_mit(0.8f, 1.0f, 30.0f, 1.0f, 0.1f);
  ctrl.shutdown();
  return 0;
}
```

## 示例程序

- `examples/cpp_wrapper_demo.cpp`
- `examples/hexfellow_canfd_demo.cpp`（Hexfellow，CAN-FD，仅 MIT / POS_VEL）
- `examples/robstride_wrapper_demo.cpp`
- `examples/full_modes_demo.cpp`
- `examples/pid_register_tune_demo.cpp`
- `examples/scan_ids_demo.cpp`（Damiao 历史辅助）
- `examples/pos_ctrl_demo.cpp`
- `examples/pos_repl_demo.cpp`

通过 Rust CLI 统一扫描:

```bash
cargo run -p motor_cli --release -- \
  --vendor all --channel can0 --mode scan --start-id 1 --end-id 255
```

通过 Rust CLI 使用 HighTorque：

```bash
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode read
```

通过 C++ 绑定使用 HighTorque：

```cpp
#include "motorbridge/motorbridge.hpp"

int main() {
  motorbridge::Controller ctrl("can0");
  auto motor = ctrl.add_hightorque_motor(1, 0x01, "hightorque");
  motor.send_mit(3.1416f, 0.8f, 0.0f, 0.0f, 0.8f);  // kp/kd 保留用于统一签名
  motor.request_feedback();
  auto st = motor.get_state();
  ctrl.shutdown();
  return st.has_value() ? 0 : 1;
}
```

## Windows 实验支持（PCAN-USB）

项目主线仍以 Linux 为主。Windows 支持为实验性能力，当前通过 PEAK PCAN 后端实现。

- 安装 PEAK 驱动与 PCAN-Basic 运行时（`PCANBasic.dll`）。
- Windows 下验证命令建议使用 `can0@1000000`。

验证命令：

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

## 构建

```bash
cargo build -p motor_abi --release
cmake -S bindings/cpp -B bindings/cpp/build \
  -DMOTORBRIDGE_ABI_LIBRARY=$PWD/target/release/libmotor_abi.so
cmake --build bindings/cpp/build -j
```

## 端到端示例命令

```bash
export LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH}

# Damiao wrapper 示例
./bindings/cpp/build/cpp_wrapper_demo \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 20 --dt-ms 20

# RobStride wrapper 示例：ping
./bindings/cpp/build/robstride_wrapper_demo \
  --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD --mode ping

# RobStride wrapper 示例：速度
./bindings/cpp/build/robstride_wrapper_demo \
  --channel can0 --model rs-06 --motor-id 127 --feedback-id 0xFD \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```
