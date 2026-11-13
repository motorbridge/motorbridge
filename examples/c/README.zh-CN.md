# C ABI 示例

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + CAN-FD + MCU 串口转 CAN 桥 + Damiao 串口桥）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- Hexfellow 示例需使用 CAN-FD 路径（`motor_controller_new_socketcanfd(...)` / CLI `--transport socketcanfd`）。
- 仅 Damiao 可选串口桥链路：`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。
- 通用 MCU 串口转 CAN 桥：`--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。注意与 Damiao 专用串口桥 `dm-serial` 是两种协议：`mcu-serial` 由桥固件透传经典 CAN 帧，`dm-serial` 承载 Damiao 串口协议。适配经典 CAN 全品牌（damiao / robstride / myactuator / hightorque / huayi；hexfellow 仅 CAN-FD 不适用）；桥固件在 bus-off / 发送失败 / 接收丢弃时会主动上报结构化链路状态；四个前端（motor_abi / Python / motor_cli / ws_gateway）均支持。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


这里是直接调用 `motor_abi` 的 C 示例。

> English version: [README.md](README.md)

## 文件

- `c_abi_demo.c`: Damiao + RobStride 的统一示例
- `hexfellow_canfd_demo.c`: Hexfellow CAN-FD 示例（仅 `mit` / `pos-vel`）

覆盖范围:

- Damiao: `enable`、`disable`、`mit`、`pos-vel`、`vel`、`force-pos`
- RobStride: `ping`、`enable`、`disable`、`mit`、`pos-vel`、`vel`、`read-param`、`write-param`
  - 说明：力矩/电流仅参数级可用（通过 `write-param` 写 `iq_ref`/限幅参数），不是统一高层模式
- Hexfellow（仅 CAN-FD）: `mit`、`pos-vel`

## 构建

```bash
cargo build -p motor_abi --release
cc examples/c/c_abi_demo.c -I motor_abi/include -L target/release -lmotor_abi -o c_abi_demo
cc examples/c/hexfellow_canfd_demo.c -I motor_abi/include -L target/release -lmotor_abi -o hexfellow_canfd_demo
LD_LIBRARY_PATH=target/release ./c_abi_demo --help
```

## 示例

Damiao MIT:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```

RobStride ping:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

RobStride 读取位置参数:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019 --param-type f32
```

RobStride 低增益 MIT:

```bash
LD_LIBRARY_PATH=target/release ./c_abi_demo \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode mit --ensure-strict 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
```

Hexfellow（仅 CAN-FD）:

```bash
LD_LIBRARY_PATH=target/release ./hexfellow_canfd_demo \
  --channel can0 --motor-id 0x01 --feedback-id 0x00 --mode mit --loop 20 --dt-ms 50
```
