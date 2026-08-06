# motorbridge Python SDK

<!-- channel-compat-note -->
## 通道兼容说明（PCAN + CANable candleLight/gs_usb + CAN-FD + Damiao 串口桥 + DM_Device）

- Linux SocketCAN 直接使用已初始化的接口名：`can0`、`can1`。CANable 请刷 candleLight/gs_usb 固件，让系统识别为 `can0` 这类 SocketCAN 接口。
- 标准 CAN 推荐 PCAN 或 CANable candleLight/gs_usb。
- CAN-FD 链路可通过 CLI（`--transport socketcanfd`）和 Python SDK（`Controller.from_socketcanfd(...)`）使用，Hexfellow 必须走该链路。
- 仅 Damiao 可选两类适配器链路：串口桥 `--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`，以及 DM_Device SDK `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`。DM_Device 链路当前只配 Damiao 电机协议使用，适配器需处于 USB 模式。
- 仅 Damiao 可选 DM_Device SDK 链路：
  `--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c`；
  Python SDK 入口为 `Controller.from_dm_device(...)`。适配器需要处于 USB 模式。
  通道映射：`usb2canfd` => `0`，`usb2canfd-dual` =>
  `0`/`1`，`linkx4c` => SDK 通道 `0..3`。Linux x86_64 下
  USB2CANFD_DUAL 的通道 0/1 和 LINKX4C 通道 `0..3` 扫描已实测通过。
- Damiao 串口桥完整接口与命令模板见 `motor_cli/README.zh-CN.md` 第 `3.6` 节（英文见 `motor_cli/README.md`）。
- Linux SocketCAN 下 `--channel` 不要带 `@bitrate`（例如 `can0@1000000` 无效）。
- Windows（PCAN 后端）中，`can0/can1` 映射 `PCAN_USBBUS1/2`，可选 `@bitrate` 后缀。


这是基于 `motor_abi` 的 Python 绑定层。

> English version: [README.md](README.md)

## README 导航（先看哪个）

如果你是第一次接触这个目录，建议按下面顺序阅读：

1. 本文档 [README.zh-CN.md](README.zh-CN.md)  
   作用：Python binding 总览（安装、API 范围、常用命令）。
2. [examples/READMEzh_cn.md](examples/READMEzh_cn.md)（中文） / [examples/README.md](examples/README.md)（英文）  
   作用：所有 Python 示例的入口说明（从最简单到高级示例）。
2.5. [`../motorbridge-docs`](../../../motorbridge-docs)
   作用：正式 Mintlify 文档站入口（教程 + API 手册风格）。
3. [get_started/README.zh-CN.md](get_started/README.zh-CN.md) / [get_started/README.md](get_started/README.md)  
   作用：pip 安装用户的快速上手路径（安装 -> 扫描 -> 运行）。
4. [DAMIAO_PYTHON_REFERENCE.zh-CN.md](DAMIAO_PYTHON_REFERENCE.zh-CN.md)  
   作用：Damiao Python 接口参考，偏“按接口查参数”。
5. [DAMIAO_binding.md](DAMIAO_binding.md)  
   作用：Damiao 绑定实现说明，偏“原理/实现细节”。
6. [README.md](README.md)  
   作用：英文版总览（给英文协作成员）。

补充：
- 如果你主要想“马上跑起来”，优先看 `examples/READMEzh_cn.md` 的“新手优先（最简单的 2 个示例）”。
- 如果你主要想查 CLI 参数，去 `../../motor_cli/README.zh-CN.md`。

## 范围

- 当前目标包版本：`0.5.1`。
- wheel 会随包携带当前平台对应的 `motor_abi` 和 `ws_gateway`，但不再内置
  DaMiao DM_Device SDK runtime。真正使用 `Controller.from_dm_device(...)`、
  Python CLI `--transport dm-device` 或
  `motorbridge-gateway --transport dm-device` 时，motorbridge 会按当前
  OS/架构解析 runtime。如果缺少库，会提示需要哪个文件、GitHub 下载地址以及
  可以放到哪些路径。这样 Linux manylinux wheel 能通过 `auditwheel`，运行时
  准备过程也保持显式可控。
- DM_Device runtime 控制项：
  - 查看所需 runtime 与安装路径：`motorbridge-install-dm-device`
  - 显式下载到用户 cache：`motorbridge-install-dm-device --download`
  - 只打印解析路径：`motorbridge-install-dm-device --print-path`
  - 使用手动安装的 SDK runtime：`MOTOR_DM_DEVICE_LIB=/path/to/libdm_device.so`
  - 使用源码树位置：`third_party/dm_device/v1.1.0/<platform>/<arch>/<runtime>`
  - 显式下载时使用内网镜像：`MOTOR_DM_DEVICE_DOWNLOAD_BASE_URL=https://.../third_party/dm_device/v1.1.0`
  - 指定 cache 目录：`MOTOR_DM_DEVICE_CACHE_DIR=/path/to/cache`
- DM_Device runtime 支持矩阵：

| 平台 / 架构 | 官方 Python wheel | DM_Device runtime 可用 | runtime 文件 | OS/runtime ABI 依赖 | 硬件实测状态 |
| --- | --- | --- | --- | --- | --- |
| Linux x86_64 | 支持 | 支持 | `linux/x86_64/libdm_device.so` | 需要 `libusb-1.0.so.0`，以及带 `GLIBCXX_3.4.32` 的 `libstdc++.so.6`，`GLIBC_2.14+` | 已实测 USB2CANFD_DUAL channel 0/1 和 LINKX4C 通道 `0..3` 扫描 |
| Linux aarch64 | 支持 | 支持 | `linux/arm64/libdm_device.so` | 需要 `libusb-1.0.so.0`，`GLIBC_2.17+`，`GLIBCXX_3.4.22+` | 待对应主机验证 |
| Windows x86_64 | 支持 | 支持 | `windows/msvc/dm_device.dll` | 需要 libusb runtime/驱动和 Microsoft Visual C++ runtime（`MSVCP140*.dll`、`VCRUNTIME140*.dll`） | 待对应主机验证 |
| macOS arm64 | 支持 | 支持 | `macos/arm64/libdm_device.dylib` | 链接系统 `libc++`、`libSystem`、`libobjc`；最低 macOS 版本待主机验证 | 待对应主机验证 |
| macOS x86_64 | 不发布官方 wheel | 仅源码/手动安装 | `macos/x86_64/libdm_device.dylib` | 链接系统 `libc++`、`libSystem`、`libobjc`；最低 macOS 版本待主机验证 | 待对应主机验证 |
| 其他系统/架构 | 不支持 | 不支持 | 无 vendored runtime | 不支持 | 不支持 |

- ABI 元数据 helper：
  - `motorbridge.abi_version()` 返回当前加载的 ABI 库版本。
  - `motorbridge.abi_capabilities()` 返回当前加载 ABI 的能力 JSON（Python `dict`）。
- `0.4.5` 优化 Damiao `ensure_mode` 读回校验：校验失败后会重试写入模式寄存器。
  同时文档补充 RobStride `robstride_cia402` 和 `robstride_mit` 两条实验 CLI
  路径；这些路径用于测试/协议 bring-up，当前还不是生产可用链路。
- `0.4.4` 新增 Damiao `dm-device` 传输、Python
  `Controller.from_dm_device(...)`、Python CLI `--transport dm-device`，并在
  目标平台存在 vendored SDK runtime 时显式解析
  `libdm_device.so`/`.dylib`/`.dll`。
- Python CLI 使用 `--transport dm-device` 扫描时，不传 `--dm-channel` 会扫描
  所选适配器的全部通道：`usb2canfd` 为 `0`，`usb2canfd-dual` 为 `0|1`，
  `linkx4c` 的 SDK 通道 `0..3`；传 `--dm-channel ...` 则只扫一路。
- `0.4.2` 优化 Damiao `dm-serial` 多电机控制：串口无待读字节时 `recv(0ms)`
  会非阻塞返回，并将有界同步读取的串口 read timeout 降到 1 ms。
- `0.4.1` 新增 ABI 版本/能力发现，并让 C++ RobStride wrapper 与 Python SDK
  对齐。接口对齐清单见 `bindings/api_surface.json`。
- `0.4.1` 包含随包 `ws_gateway` 在 Damiao `dm-serial` 整臂扫描时的会话释放
  与串口争用问题，并新增 `damiao_state_many` 用于浏览器多关节状态同步。
- `0.3.9` 的 RobStride `request_feedback()` 语义修正和 `0.3.8` 的 PP/CSP
  专用位置控制接口继续保留，同时保持 Python binding 公开 API 向后兼容。
- RobStride 参数写入默认不等待状态 ack；需要恢复同步等待时可设置
  `MOTORBRIDGE_ROBSTRIDE_WRITE_ACK_TIMEOUT_MS`。
- Python CLI 现在实现为 `motorbridge.cli` 包，但 `motorbridge-cli`、
  `python -m motorbridge.cli`、`python -m motorbridge`、
  `from motorbridge.cli import main` 和旧式扁平 run 参数继续可用。
- 高层 API: `Controller`、`Motor`、`Mode`
- CLI: `motorbridge-cli`
- 网关启动命令（pip 安装后进入 PATH）：
  - `motorbridge-gateway -- --bind 127.0.0.1:9002 ...`
  - 该命令实际启动随 Python wheel 打包的 Rust `ws_gateway` 二进制，
    因此 `state_stream`、`damiao_state_many`、`param_stream`、
    `damiao_param_stream`、`robstride_param_stream` 等 WS JSON op 由随包网关版本直接支持。
- 安全说明：
  - 本地使用建议保持回环地址 `127.0.0.1`。
  - 若绑定到非回环地址（`0.0.0.0` 或网卡 IP），启动前必须设置 `MOTORBRIDGE_WS_TOKEN`。
  - 客户端需在握手中携带 token：`x-motorbridge-token` 或 `Authorization: Bearer ...`。
- macOS 运行说明（仅当出现动态库加载错误时需要）：
  - 通用方式获取网关路径（不写死本机路径）：
    `GW="$(python3 -c "import motorbridge, pathlib; print(pathlib.Path(motorbridge.__file__).resolve().parent/'bin'/'ws_gateway')")"`
  - 使用包内 `lib` 目录设置动态库路径：
    `PKG_DIR="$(python3 -c "import motorbridge, pathlib; print(pathlib.Path(motorbridge.__file__).resolve().parent)")"`
    `DYLD_LIBRARY_PATH="$PKG_DIR/lib:${DYLD_LIBRARY_PATH:-}" "$GW" --bind 127.0.0.1:9002 --vendor damiao --channel can0 --model auto --motor-id 0x01 --feedback-id 0x11 --dt-ms 20`
- Controller 构造入口：
  - `Controller(channel="can0")`（SocketCAN/PCAN 路径）
  - `Controller.from_socketcanfd(channel="can0")`（CAN-FD 路径，Hexfellow 必须使用）
  - `Controller.from_dm_serial(serial_port="/dev/ttyACM0", baud=921600)`（仅 Damiao 串口桥）
  - `Controller.from_dm_device(dm_device_type="usb2canfd-dual", dm_channel="0")` / `Controller.from_dm_device(dm_device_type="linkx4c", dm_channel="0")`（仅 Damiao DM_Device SDK 链路）
- 厂商入口:
  - Damiao: `add_damiao_motor(...)`
  - Hexfellow: `add_hexfellow_motor(...)`
  - MyActuator: `add_myactuator_motor(...)`
  - RobStride: `add_robstride_motor(...)`
  - HighTorque: `add_hightorque_motor(...)`
- 状态查询统一范式：
  - 推荐统一使用 `request_feedback() -> poll_feedback_once() -> get_state()`。
  - RobStride 私有协议没有单次“请求状态帧”命令；RobStride 的 `request_feedback()` 是非阻塞 no-op。连通性检查请用 `robstride_ping()`，连续状态请用主动上报，需要新鲜位置/速度请读类型化参数。

## 统一模式映射摘要（顶层协议 -> 厂商原生）

| 顶层统一模式 | Damiao | RobStride | Hexfellow | MyActuator | HighTorque |
| --- | --- | --- | --- | --- | --- |
| `Mode.MIT` | 原生 MIT | 原生 MIT | 原生 MIT（模式 5） | 不支持 | 映射到原生 pos+vel+tqe |
| `Mode.POS_VEL` | 原生 POS_VEL | 映射到原生 Position（`run_mode=1` + `vel_max(0x7024)` + `loc_ref(0x7016)`） | 原生 POS_VEL（模式 1） | Position 设定流程 | 映射到原生 pos+vel+tqe |
| `Mode.VEL` | 原生 VEL | 原生 Velocity | 不支持 | 原生 Velocity 设定流程 | 原生速度命令 |
| `Mode.FORCE_POS` | 原生 FORCE_POS | 不支持 | 不支持 | 不支持 | 映射到原生 pos+vel+tqe |

说明：

- RobStride 统一高层当前覆盖 `MIT` / `POS_VEL` / `VEL`。
- `TORQUE/CURRENT` 对 RobStride 仍为参数级能力（`robstride_write_param_*`），尚未提供独立统一模式。
- RobStride 建议默认使用 `feedback-id=0xFD`；扫描默认尝试 `0xFD,0xFF,0xFE,0x00,0xAA`。
- RobStride 的 `feedback_id` / `host_id` 不是电机 `device_id`；扫描命中的电机 ID 看 `probe` / `device_id`。

## 快速开始

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    motor = ctrl.add_damiao_motor(0x01, 0x11, "4340P")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(0.0, 0.0, 20.0, 1.0, 0.0)
    motor.request_feedback()
    ctrl.poll_feedback_once()
    print(motor.get_state())
    motor.close()
```

最简单控制示例（统一接口，转到目标角度）：

```python
from motorbridge import Controller, Mode

TARGET_POS = 1.0  # 目标角度(rad)

with Controller("can0") as ctrl:
    motor = ctrl.add_damiao_motor(0x01, 0x11, "4340P")  # 可替换为你的 id / model
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(TARGET_POS, 0.0, 20.0, 1.0, 0.0)
    motor.request_feedback()
    ctrl.poll_feedback_once()
    print("state=", motor.get_state())
    motor.close()
```

```python
from motorbridge import Controller, Mode

TARGET_POS = 1.0  # 目标角度(rad)

with Controller("can0") as ctrl:
motor = ctrl.add_robstride_motor(127, 0xFD, "rs-00")  # 可替换为你的 id / model
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(TARGET_POS, 0.0, 8.0, 0.2, 0.0)
    motor.request_feedback()
    ctrl.poll_feedback_once()
    print("state=", motor.get_state())
    motor.close()
```

Damiao 串口桥示例：

```python
from motorbridge import Controller, Mode

TARGET_POS = 0.5  # 目标角度(rad)

with Controller.from_dm_serial("/dev/ttyACM1", 921600) as ctrl:
    motor = ctrl.add_damiao_motor(0x04, 0x14, "4310")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(TARGET_POS, 0.0, 20.0, 1.0, 0.0)  # 控制到目标角度
    motor.request_feedback()
    ctrl.poll_feedback_once()
    print(motor.get_state())
    motor.close()
```

RobStride 快速示例:

```python
from motorbridge import Controller, Mode

TARGET_POS = 1.0  # 目标角度(rad)

with Controller("can0") as ctrl:
motor = ctrl.add_robstride_motor(127, 0xFD, "rs-00")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)
    motor.send_mit(TARGET_POS, 0.0, 8.0, 0.2, 0.0)  # 控制到目标角度
    motor.request_feedback()
    ctrl.poll_feedback_once()
    print(motor.get_state())
    motor.close()
```

MyActuator 快速示例:

```python
from motorbridge import Controller, Mode

with Controller("can0") as ctrl:
    motor = ctrl.add_myactuator_motor(1, 0x241, "X8")
    ctrl.enable_all()
    motor.ensure_mode(Mode.POS_VEL, 1000)
    motor.send_pos_vel(3.1416, 2.0)  # rad / rad/s
    print(motor.get_state())
    motor.close()
```

Hexfellow 快速示例（仅 CAN-FD）:

```python
from motorbridge import Controller, Mode

with Controller.from_socketcanfd("can0") as ctrl:
    motor = ctrl.add_hexfellow_motor(1, 0x00, "hexfellow")
    ctrl.enable_all()
    motor.ensure_mode(Mode.MIT, 1000)      # Hexfellow 仅支持 MIT / POS_VEL
    motor.send_mit(0.8, 1.0, 30.0, 1.0, 0.1)
    print(motor.get_state())
    motor.close()
```

## CLI 示例

Damiao:

```bash
motorbridge-cli run \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --mode mit --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 50 --dt-ms 20

# 也兼容 Rust CLI 风格的改 ID：
motorbridge-cli run \
  --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --set-motor-id 0x02 --set-feedback-id 0x12 --store 1 --verify-id 1
```

RobStride:

```bash
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 \
  --mode ping
```

RobStride MIT 快速验证：

```bash
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 2 --feedback-id 0xFD \
  --mode mit --ensure-strict 1 \
  --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 \
  --loop 100 --dt-ms 20
```

RobStride 位置目标，已与 WS gateway 的原生寄存器路径对齐
（`vel_max` `0x7024`、`loc_kp` `0x701E`、`loc_ref` `0x7016`）：

```bash
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel \
  --pos 1.5 --vlim 1.0 --loc-kp 5.0 \
  --loop 1 --dt-ms 20

motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 \
  --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel \
  --pos -1.5 --vlim 1.0 --loc-kp 5.0 \
  --loop 1 --dt-ms 20
```

RobStride 读参数:

```bash
motorbridge-cli robstride-read-param \
  --channel can0 --model rs-00 --motor-id 127 --param-id 0x7019 --type f32

# Python CLI 也兼容 Rust CLI 风格的 run/read-param：
motorbridge-cli run \
  --vendor robstride --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD \
  --mode read-param --param-id 0x7019 --param-type f32
```

Damiao 参数读写:

```bash
motorbridge-cli damiao-read-param \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --param-id 21 --type f32

motorbridge-cli damiao-write-param \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --param-id 8 --type u32 --value 0x01 --verify 1
```

统一扫描（所有 vendor）:

```bash
motorbridge-cli scan --vendor all --channel can0 --start-id 0x01 --end-id 0xFF
```

RobStride 单独扫描和改 ID：

```bash
motorbridge-cli scan \
  --vendor robstride --channel can0 --start-id 1 --end-id 127 \
  --feedback-ids 0xFD,0xFF,0xFE,0x00,0xAA

motorbridge-cli id-set \
  --vendor robstride --channel can0 \
  --motor-id 127 --feedback-id 0xFD \
  --new-motor-id 126 --store 1 --verify 1
```

通过绑定使用 HighTorque：

```python
from motorbridge import Controller

with Controller("can0") as ctrl:
    motor = ctrl.add_hightorque_motor(1, 0x01, "hightorque")
    motor.send_mit(3.1416, 0.8, 0.0, 0.0, 0.8)  # kp/kd 参数保留，但协议本身不使用
    motor.request_feedback()
    print(motor.get_state())
    motor.close()
```

通过 Rust CLI 使用 HighTorque：

```bash
cargo run -p motor_cli --release -- \
  --vendor hightorque --channel can0 --motor-id 1 --mode read
```

## Windows 实验支持（PCAN-USB）

项目主线仍以 Linux 为主。Windows 支持为实验性能力，当前通过 PEAK PCAN 后端实现。

- 安装 PEAK 驱动与 PCAN-Basic 运行时（`PCANBasic.dll`）。
- Windows 下建议使用 `can0@1000000`（映射到 `PCAN_USBBUS1`，1Mbps）。

建议先用 Rust CLI 做快速验证：

```bash
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode scan --start-id 1 --end-id 16
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4340P --motor-id 0x01 --feedback-id 0x11 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
cargo run -p motor_cli --release -- --vendor damiao --channel can0@1000000 --model 4310 --motor-id 0x07 --feedback-id 0x17 --mode pos-vel --pos 3.1416 --vlim 2.0 --loop 1 --dt-ms 20
```

Windows 本地 wheel 构建：

```bash
python -m pip install --user wheel
set MOTORBRIDGE_LIB=%CD%\\target\\release\\motor_abi.dll
set MOTORBRIDGE_WS_GATEWAY_BIN=%CD%\\target\\release\\ws_gateway.exe
python -m pip wheel --no-build-isolation bindings/python -w bindings/python/dist
python -m pip install bindings/python/dist/motorbridge-*.whl
```

## 示例程序

- Damiao wrapper 示例: `examples/python_wrapper_demo.py`
- Hexfellow CAN-FD 示例: `examples/hexfellow_canfd_demo.py`（仅 MIT / POS_VEL）
- Damiao 维护接口示例: `examples/damiao_maintenance_demo.py`
- Damiao 寄存器读写示例: `examples/damiao_register_rw_demo.py`
- Damiao 串口桥链路示例: `examples/damiao_dm_serial_demo.py`
- RobStride wrapper 示例: `examples/robstride_wrapper_demo.py`
- Damiao 全模式示例: `examples/full_modes_demo.py`
- Damiao 扫描 / 调参 / 位置辅助:
  - `examples/scan_ids_demo.py`
  - `examples/pid_register_tune_demo.py`
  - `examples/pos_ctrl_demo.py`
  - `examples/pos_repl_demo.py`

详细见 [examples/READMEzh_cn.md](examples/READMEzh_cn.md)（中文）或 [examples/README.md](examples/README.md)（英文）。

## Damiao 全覆盖状态

Python 示例中 Damiao 用法已覆盖到位：

- 控制模式：`mit` / `pos-vel` / `vel` / `force-pos`
- 传输链路：`Controller(channel)`、`Controller.from_socketcanfd(...)`、`Controller.from_dm_serial(...)` 与 `Controller.from_dm_device(...)`
- 维护接口：`clear_error`、`set_zero_position`、`set_can_timeout_ms`、`request_feedback`
  - Damiao 置零规范：先 `disable()`，再 `set_zero_position()`
  - Python 不暴露置零等待参数；核心层内置固定 `20ms` 稳定等待
- 寄存器接口：`get/write f32`、`get/write u32`、`store_parameters`

## RobStride 维护说明

- `clear_error()` 已通过和 Damiao 一致的统一电机方法支持。
- `robstride_set_active_report(True/False)` 可开启/关闭 RobStride 通信类型 `24` 主动状态上报。
- 开启主动上报后，后台 polling 可以直接消费状态帧并更新 `get_state()` 缓存，不必每次额外发送查询命令；`request_feedback()` 仍保留为兼容/手动刷新接口。

## CLI run 模式参数有效性表

`motorbridge-cli run` 使用统一命令入口，但不同品牌/模式只会消费该协议真正支持的参数。

| 品牌 | 模式 | 有效参数 | 说明 |
| --- | --- | --- | --- |
| Damiao | `mit` | `--pos --vel --kp --kd --tau` | 原生 MIT 帧 |
| Damiao | `pos-vel` | `--pos --vlim` | 原生位置-速度帧 |
| Damiao | `vel` | `--vel` | 原生速度帧 |
| Damiao | `force-pos` | `--pos --vlim --ratio` | 原生力位混控帧 |
| RobStride | `mit` | `--pos --vel --kp --kd --tau` | 原生 MIT 帧 |
| RobStride | `pos-vel` | `--pos --vlim --loc-kp` | 映射到原生 Position 模式；未传 `--loc-kp` 时接受 `--kp` 作为 fallback |
| RobStride | `vel` | `--vel` | 原生速度模式 |
| HighTorque | `mit` | `--pos --vel --tau` | `--kp/--kd` 为统一签名兼容参数，`ht_can v1.5.5` 会忽略 |
| Hexfellow | `mit` | `--pos --vel --kp --kd --tau` | CAN-FD 路径 |
| Hexfellow | `pos-vel` | `--pos --vlim` | CAN-FD 路径 |

RobStride `pos-vel` 的 `--vel`、`--kd`、`--tau` 是无效参数：该路径实际写入
`limit_spd`、`loc_kp`、`loc_ref`。Rust CLI 和 Python CLI 在用户显式传入这些参数时会输出 warning。

## 端到端示例命令

```bash
# 先构建 ABI
cargo build -p motor_abi --release
export PYTHONPATH=bindings/python/src
export LD_LIBRARY_PATH=$PWD/target/release:${LD_LIBRARY_PATH}

# Damiao wrapper 示例
python3 bindings/python/examples/python_wrapper_demo.py \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \
  --pos 0 --vel 0 --kp 20 --kd 1 --tau 0 --loop 20 --dt-ms 20

# RobStride wrapper 示例：ping
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode ping

# RobStride wrapper 示例：清故障
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD --mode clear-error

# RobStride wrapper 示例：主动上报初始化
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode write-param --param-id 0x7026 --param-type u16 --param-value 3

python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode active-report --active-report 1

# RobStride wrapper 示例：位置命令
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20

# RobStride wrapper 示例：速度
python3 bindings/python/examples/robstride_wrapper_demo.py \
  --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \
  --mode vel --vel 0.3 --loop 40 --dt-ms 50
```

## 说明

- `id-dump` 仍是 Damiao 工作流；`id-set` 支持 Damiao 和 RobStride；`scan` 支持 `damiao|hexfellow|myactuator|robstride|hightorque|all`。
- RobStride `id-set` 中，`--new-motor-id` 修改 `device_id`；`--feedback-id` 仍是上位机侧 host_id。
- RobStride `motor_id` / `device_id` 会校验为 `1..255`；`feedback_id` / `host_id` 会校验为 `0..255`，避免 `ctypes` 静默截断。
- Python CLI 与 Rust CLI 在生产常用的 Damiao / RobStride 工作流上已经对齐：扫描、使能/失能、控制、改 ID、参数读写、RobStride 清故障和主动上报。Rust CLI 仍保留更多 HighTorque/MyActuator/Hexfellow 的底层调试入口。
- RobStride 扫描会通过指定 host_id 的 ABI helper 精确探测每个 `--feedback-ids`；非法 host_id 会直接报错，不会静默回退。
- MyActuator 在 ABI wrapper 中不支持 `Mode.MIT` 与 `send_force_pos`。
- Hexfellow 在 ABI wrapper 中支持 `MIT` 与 `POS_VEL`，`VEL` / `FORCE_POS` 会返回不支持。
- Damiao 的完整调参参考仍保留在:
  - [DAMIAO_API.md](DAMIAO_API.md)
  - [DAMIAO_API.zh-CN.md](DAMIAO_API.zh-CN.md)

## PyPI 自动发布（GitHub Actions）

仓库已新增 `.github/workflows/pypi-publish.yml`。

- Tag 自动发布策略：
  - 推送 `vX.Y.Z` -> 同一套产物同时发布到 TestPyPI 和 PyPI
- 手动发布：在 GitHub Actions 运行 `Python Publish`，可选：
  - `testpypi`（仅发布 TestPyPI）
  - `pypi`（仅发布 PyPI）

### 一次性配置（token 模式）

1. 在 PyPI 创建 API token，并配置仓库 secret：`PYPI_API_TOKEN`。
2. 在 TestPyPI 创建 API token，并配置仓库 secret：`TEST_PYPI_API_TOKEN`。
3. 每次上传必须使用全新版本号（例如 `0.1.6`、`0.1.7`）。
