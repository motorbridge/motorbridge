# Windows 分发与 SDK 使用指南

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。

完整的跨平台排障流程（PCAN + CANable candleLight/gs_usb）见 [can_debugging.md](can_debugging.md)。

> **平台特性**：Windows 下 PCAN 驱动不支持高频 `CAN_Initialize`/`CAN_Uninitialize`，反复重开总线会触发 `PCAN_ERROR_INITIALIZE` 失败（Linux SocketCAN 无此问题）。这会影响 RobStride 扫描等"每对 (motor_id, feedback_id) 重开总线"的逻辑，详见 [pcan_reinit_limit.md](pcan_reinit_limit.md)。

本文说明 `motorbridge` 在 Windows 上如何分发与使用。

## 产物类型对照

- Python 用户：
  - 安装 `.whl`（`motorbridge-<ver>-cp3xx-...-win_amd64.whl`）
- C/C++ 用户：
  - 使用 ABI `.zip`（`motorbridge-abi-<ver>-windows-x86_64.zip`）
- Linux 专用包：
  - `.deb` 仅适用于 Ubuntu/Linux，Windows 不能安装
- MSI：
  - 可选增强项，不是 SDK 使用的必要条件

## 运行时依赖

- 已安装 PEAK PCAN 驱动
- 已安装 PCAN-Basic 运行时（提供 `PCANBasic.dll`）

## 本地构建 Windows ABI

```bash
cargo build -p motor_abi --release
```

预期产物：

- `target/release/motor_abi.dll`
- `target/release/motor_abi.lib`

## 本地构建 Windows Python Wheel

```bash
python -m pip install --user wheel
set MOTORBRIDGE_LIB=%CD%\\target\\release\\motor_abi.dll
set MOTORBRIDGE_WS_GATEWAY_BIN=%CD%\\target\\release\\ws_gateway.exe
python -m pip wheel --no-build-isolation bindings/python -w bindings/python/dist
```

说明：

- wheel 构建会自动把 ABI DLL 打进包内。
- 若找不到 ABI DLL，会直接失败并提示路径，避免产出不可用 wheel。

## 安装并验证 Python SDK

```bash
python -m pip install bindings/python/dist/motorbridge-*.whl
python -c "from motorbridge import Controller; c=Controller('can0@1000000'); print('ok'); c.close()"
```

## 安装并验证 C/C++ ABI

1. 下载 `motorbridge-abi-<ver>-windows-x86_64.zip`。
2. 解压 include/lib 到依赖目录。
3. 在工程里链接 `motor_abi.dll` 与 import lib。

最小 ctypes 验证：

```python
import ctypes
lib = ctypes.CDLL("motor_abi.dll")
lib.motor_controller_new_socketcan.argtypes = [ctypes.c_char_p]
lib.motor_controller_new_socketcan.restype = ctypes.c_void_p
ptr = lib.motor_controller_new_socketcan(b"can0@1000000")
assert ptr
lib.motor_controller_free(ptr)
```

## Windows 通道与波特率约定

- `can0` 映射 `PCAN_USBBUS1`
- `can1` 映射 `PCAN_USBBUS2`
- 波特率后缀：`can0@1000000`

## 推荐验证命令

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```
