# Python Binding 快速上手（pip 优先）

这个目录是一套全新的“新手优先”文档，面向 pip 安装用户。

目标：安装包 -> 扫描设备 -> 跑通电机示例，一步步快速上手。

## 1）安装

```bash
python3 -m pip install motorbridge
```

如果你要测试 TestPyPI 预发布版本：

```bash
python3 -m pip install -i https://test.pypi.org/simple/ motorbridge==<版本号>
```

## 2）硬件通道说明

- Linux SocketCAN 常用通道：`can0`、`can1`
- Windows PCAN 常用通道：`can0@1000000`、`can1@1000000`
- 测试时尽量保证总线上只有一个发送端。

### 三种链路怎么选（很重要）

- `TRANSPORT = "auto"` 或 `"socketcan"`：
  使用标准 CAN 通道（`CHANNEL` 生效）。
- `TRANSPORT = "mcu-serial"`：
  走通用 MCU 串口转 CAN 桥（`SERIAL_PORT` / `SERIAL_BAUD` 生效），
  覆盖经典 CAN 品牌（damiao / robstride / myactuator / hightorque）。
  注意与 Damiao 专用 `dm-serial` 串口桥是两种协议。
- `TRANSPORT = "dm-serial"`：
  走 Damiao 串口桥（`SERIAL_PORT` / `SERIAL_BAUD` 生效），只支持 Damiao。
- `TRANSPORT = "dm-device"`：
  走 DaMiao DM_Device SDK（`DM_DEVICE_TYPE` / `DM_CHANNEL` 生效），只支持 Damiao。
  适配器需要处于 USB 模式。扫描脚本中 `DM_CHANNEL = None` 表示扫描所选适配器
  的全部通道：`usb2canfd` 单路 `0`，`usb2canfd-dual` 为
  `0`/`1`，`linkx4c` 为 SDK 通道 `0..3`；设置 `DM_CHANNEL` 时只扫一路。

快速判断：
- 你用 CAN 设备（PCAN/CANable candleLight/gs_usb）就用 `auto/socketcan`。
- 你用通用 MCU 串口转 CAN 桥（如 XIAO-C5 桥固件）就用 `mcu-serial`。
- 你用 Damiao 串口桥（如 `/dev/ttyACM0`）就用 `dm-serial`。
- 你用 Damiao USB2CANFD / USB2CANFD_DUAL / LINKX4C 就用 `dm-device`；
  通过 `DM_DEVICE_TYPE` 选择设备类型，通过 `DM_CHANNEL=0/1/2/3`
  选择物理通道。

## 3）快速命令（无需源码）

```bash
# 用已安装 CLI 做全品牌扫描
motorbridge-cli scan --vendor all --channel can0 --start-id 1 --end-id 255

# 单电机 Damiao 控制（示例 ID）
motorbridge-cli run --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode pos-vel --pos 1.0 --vlim 1.0 --loop 60 --dt-ms 20
```

## 4）本目录运行方式

`get_started` 目录下的可运行脚本统一放在 `courses/`。
这样入口更单一，按课程顺序执行更容易理解。

### 顶部常量参数含义（小白版）

- `TRANSPORT`：链路类型（`auto/socketcan/socketcanfd/mcu-serial/dm-serial/dm-device`）
- `CHANNEL`：CAN 接口名（Linux: `can0/can1`；Windows: `can0@1000000`）
- `VENDOR`：扫描厂商（`all` 最常用）
- `MOTOR_ID`：电机控制 ID
- `FEEDBACK_ID`：反馈帧 ID
- `MODEL`：电机型号字符串
- `TARGET_POS` / `POS`：目标角度（弧度）
- `V_LIMIT`：位置模式速度上限
- `LOOP`：循环次数（越大发送越久）
- `DT_MS`：循环周期毫秒（总线忙时增大到 30/50）
- `SERIAL_PORT` / `SERIAL_BAUD`：`mcu-serial` 与 `dm-serial` 使用
- `DM_DEVICE_TYPE` / `DM_CHANNEL`：仅 `dm-device` 使用；支持
  `usb2canfd` / `usb2canfd-dual` / `linkx4c`
  （扫描脚本里 `DM_CHANNEL=None` 表示扫描所选适配器全部通道）

## 5）课程化系列（强烈推荐）

如果你想按“真实使用流程”系统学习，请直接看 `courses/`：

- `00-enable-and-status.py`：使能 + 状态查询
- `01-scan.py`：扫描设备
- `02-register-rw.py`：参数/寄存器读写
- `03-mode-switch-method.py`：模式切换方法
- `04-mode-mit.py`：MIT 模式
- `05-mode-pos-vel.py`：POS_VEL 模式
- `06-mode-vel.py`：VEL 模式
- `07-mode-force-pos.py`：FORCE_POS 模式
- `08-mode-mixed-switch.py`：模式混合切换
- `09-multi-motor.py`：多电机控制

推荐学习顺序：

```bash
python3 bindings/python/get_started/courses/00-enable-and-status.py
python3 bindings/python/get_started/courses/01-scan.py
python3 bindings/python/get_started/courses/03-mode-switch-method.py
```

## 6）现在就运行（courses）

```bash
python3 bindings/python/get_started/courses/00-enable-and-status.py
python3 bindings/python/get_started/courses/01-scan.py
python3 bindings/python/get_started/courses/09-multi-motor.py
```

## 7）常见问题

- `os error 105`：发送过快或有其它程序同时发包；把 `--dt-ms` 提高到 30/50。
- 电机无响应：先检查布线、波特率、motor/feedback ID。
- CANable 设备：先初始化 candleLight/gs_usb 对应的 SocketCAN 接口再运行示例。

## 8）`poll_feedback_once()` 版本说明

- `<= v0.1.6`：状态查询脚本建议保留手动 `poll_feedback_once()`。
- `v0.1.7+`：默认已启用后台轮询，通常可不再手动调用 `poll_feedback_once()`。
- 课程脚本保留该调用，是为了兼容旧版本与统一教学调用风格。

## 9）下一步文档

- Python 总览：`bindings/python/README.zh-CN.md`
- 完整示例目录：`bindings/python/examples/READMEzh_cn.md`
- CLI 全参数文档：`motor_cli/README.zh-CN.md`
- 课程总纲（全接口导向）：`bindings/python/get_started/courses/README.zh-CN.md`
- Mintlify 文档站（教程 + API）：`../motorbridge-docs`
