# 最终用户操作手册（PCAN 主链路 + MCU 串口桥 + Damiao 串口桥备用链路）

本文面向实际部署与运维。默认链路是标准 CAN（Linux SocketCAN / Windows PCAN）；通用 MCU 串口转 CAN 桥（`mcu-serial`）与 Damiao 串口桥（`dm-serial`）作为串口备用链路，两者是不同的协议。

## 1. 总体原则

- 默认使用标准 CAN 链路：
  - Linux: `can0` / `can1`（SocketCAN；PCAN 或 CANable candleLight/gs_usb 初始化后得到的接口）
  - Windows: `can0@1000000`（映射到 PCAN 通道）
- 当你使用通用 MCU 串口转 CAN 桥（如 XIAO-C5 桥固件，透传经典 CAN 帧）时，可使用：
  - `--transport mcu-serial --serial-port <tty> --serial-baud <baud>`
- 仅当你使用 Damiao 私有转接板串口桥时，才使用：
  - `--transport dm-serial --serial-port <tty> --serial-baud <baud>`
- `mcu-serial` 支持 `damiao` / `robstride` / `myactuator` / `hightorque`（经典 CAN 品牌；hexfellow 仅 CAN-FD 不适用）。
- `dm-serial` 仅支持 `--vendor damiao`，不用于 RobStride / MyActuator / HighTorque / Hexfellow。

## 2. 链路选择

- 场景 A（推荐，通用）：同一总线上有多个厂商或希望统一调试
  - 用标准 CAN（SocketCAN/PCAN）
- 场景 B（通用 MCU 桥）：只有 USB 串口、没有 CAN 网卡，或希望经典 CAN 品牌共用一条串口链路
  - 用 `mcu-serial`
- 场景 C（Damiao 专板）：Damiao 转接板直连串口，且你需要其私有桥接协议
  - 用 `dm-serial`

## 3. Linux 主链路（SocketCAN）

### 3.1 PCAN（默认推荐）

```bash
scripts/can_restart.sh can0
ip -details link show can0
lsmod | grep -E 'peak_usb|can_raw|can_dev'
```

### 3.2 CANable candleLight / gs_usb

```bash
scripts/canable_restart.sh can0
ip -details link show can0
lsmod | grep -E 'gs_usb|can_raw|can_dev'
```

说明：
- CANable 必须是 candleLight/gs_usb 固件；普通 USB 串口转 CAN 路径不作为当前推荐/验证链路。
- PCAN 和 CANable 初始化完成后，上层都使用 `--channel can0`。

## 4. Linux 备用链路（Damiao 串口桥）

当 Damiao 转接板通过串口直接桥接 CAN 时，使用：

```bash
cargo run -p motor_cli --release -- --vendor damiao \
  --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --mode scan --start-id 1 --end-id 16
```

控制示例：

```bash
cargo run -p motor_cli --release -- --vendor damiao \
  --transport dm-serial --serial-port /dev/ttyACM1 --serial-baud 921600 \
  --model 4310 --motor-id 0x04 --feedback-id 0x14 \
  --mode mit --verify-model 0 --ensure-mode 0 \
  --pos 0.5 --vel 0 --kp 20 --kd 1 --tau 0 --loop 80 --dt-ms 20
```

## 5. Windows 主链路（PCAN）

先安装 PEAK 驱动和 `PCANBasic.dll`，然后使用：

```bash
cargo run -p motor_cli --release -- --vendor damiao \
  --channel can0@1000000 --model 4310 --mode scan --start-id 1 --end-id 16
```

说明：
- `can0` 映射 `PCAN_USBBUS1`，`can1` 映射 `PCAN_USBBUS2`。
- Windows 也可使用 Damiao 串口桥（`dm-serial`），串口名常见为 `COM3`、`COM4` 等。

## 6. 多厂商统一控制建议

- 你要统一控制 Damiao + 其他厂商时：
  - 必须走标准 CAN（SocketCAN/PCAN）
- 你使用 Damiao 串口桥时：
  - 只保证 Damiao 协议路径可用
  - 文档和 CLI 会明确提示其为 Damiao-only

## 7. 常见问题与排障

### 7.1 `if_nametoindex failed for can0: No such device`

原因：`can0` 未创建或已掉线。  
处理：重新执行第 3 章起链流程，确认 `ip -details link show can0` 可见。

### 7.2 `model handshake failed reading PMAX(rid=21)`

常见原因：
- 电机 ID / 反馈 ID 不匹配
- 链路不通（接线、终端电阻、电源）
- 用错链路（本应走 Damiao 串口桥却走了 `can0`，或反之）

建议顺序：
1. 先 `--mode scan` 确认在线 ID。
2. 再用命中的 `motor-id/feedback-id` 做 `--loop 1` 小步验证。
3. 仍失败时抓包（`candump can0`）或切换到 `dm-serial` 交叉验证。

### 7.3 两块 Damiao 转接板跑一段时间后失联

优先排查：
- USB 供电/线材/Hub 稳定性
- 串口设备号漂移（`/dev/ttyACM*` 变化）
- 多适配器竞争（接口名冲突、多个 CAN 设备同时在线）

建议：
- 使用固定规则（udev）绑定设备名
- 每路独立接口名与启动脚本
- 持续监控 `dmesg` / `journalctl -k`

## 8. 推荐运行基线

- 生产默认：
  - Linux：SocketCAN（PCAN 或 CANable candleLight/gs_usb）
  - Windows：PCAN
- Damiao 串口桥：
  - 作为备用链路启用
  - 在文档与命令中显式加 `--transport dm-serial`

## 9. 夹爪电机校准提示

Damiao 电机使用单圈编码器，位置范围为约 ±2 圈（±PMAX rad），**断电后零点会丢失**。当电机应用于夹爪时，每次上电需要执行校准流程以确定零位：

1. 用 MIT 模式低力矩推向机械限位（碰到即停，不会硬撞）：
   ```bash
   # Linux
   motor_cli --vendor damiao --channel can0 --model 4310 \
     --motor-id 0x07 --feedback-id 0x17 \
     --mode mit --pos -5 --vel 0 --kp 5 --kd 0.5 --tau 0 --loop 100 --dt-ms 20

   # Windows
   motor_cli --vendor damiao --channel can0@1000000 --model 4310 \
     --motor-id 0x07 --feedback-id 0x17 \
     --mode mit --pos -5 --vel 0 --kp 5 --kd 0.5 --tau 0 --loop 100 --dt-ms 20
   ```
2. 等反馈中 `vel ≈ 0` 且 `pos` 稳定后，设置当前为零点：
   ```bash
   # Linux
   motor_cli --vendor damiao --channel can0 --model 4310 \
     --motor-id 0x07 --feedback-id 0x17 --mode set-zero

   # Windows
   motor_cli --vendor damiao --channel can0@1000000 --model 4310 \
     --motor-id 0x07 --feedback-id 0x17 --mode set-zero
   ```
3. 校准后用 MIT 模式夹取（力控安全）或 pos-vel 模式张开（快速精确）。

> **注意：** MIT 模式是夹爪的推荐控制模式——`kp` 控制夹持刚度，`tau` 限制最大力矩，不会夹坏物体。

## 10. 相关文档

- CLI 参考：[cli.md](cli.md)
- ABI 参考：[abi.md](abi.md)
- CAN 调试：[can_debugging.md](can_debugging.md)
- 顶层说明：[`README.zh-CN.md`](../../README.zh-CN.md)
