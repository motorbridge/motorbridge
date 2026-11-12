# Python ctypes 示例

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + MCU 串口转 CAN 桥 + Damiao 串口桥）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- 仅 Damiao 可选串口桥链路：`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。
- 通用 MCU 串口转 CAN 桥：`--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`。注意与 Damiao 专用串口桥 `dm-serial` 是两种协议：`mcu-serial` 由桥固件透传经典 CAN 帧，`dm-serial` 承载 Damiao 串口协议。适配经典 CAN 全品牌（damiao / robstride / myactuator / hightorque；hexfellow 仅 CAN-FD 不适用）；桥固件在 bus-off / 发送失败 / 接收丢弃时会主动上报结构化链路状态；四个前端（motor_abi / Python / motor_cli / ws_gateway）均支持。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


这里的 Python 示例直接通过 `ctypes` 调用 Rust ABI。

> English version: [README.md](README.md)

## 文件

- `python_ctypes_demo.py`: 统一的双 vendor 示例
- `four_vendor_pos_sync.py`: 多厂商并发位置控制脚本

覆盖范围:

- Damiao: `enable`、`disable`、`mit`、`pos-vel`、`vel`、`force-pos`
- RobStride: `ping`、`enable`、`disable`、`mit`、`pos-vel`、`vel`、`read-param`、`write-param`
  - 说明：力矩/电流仅参数级可用（通过 `write-param` 写 `iq_ref`/限幅参数），不是统一高层模式

## 构建与运行

```bash
cargo build -p motor_abi --release
python3 examples/python/python_ctypes_demo.py --help
```

## 示例

Damiao MIT:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20
```

RobStride ping:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping
```

RobStride 读参数:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode read-param --param-id 0x7019 --param-type f32
```

RobStride 写参数:

```bash
python3 examples/python/python_ctypes_demo.py \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode write-param --param-id 0x700A --param-type f32 --param-value 0.2
```

多厂商位置同步（Damiao x2 + MyActuator + HighTorque）：

```bash
python3 examples/python/four_vendor_pos_sync.py \
  damiao 0x01 damiao 0x07 myactuator 1 hightorque 1 \
  --pos 1.57 \
  --damiao-model-by-id "0x01=4340P,0x07=4310" \
  --stagger-ms 50
```

仅预览将执行的 `motor_cli` 命令：

```bash
python3 examples/python/four_vendor_pos_sync.py \
  damiao:0x01 damiao:0x07 myactuator:1 hightorque:1 \
  --pos 1.57 --dry-run
```
