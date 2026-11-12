# Python Binding 课程总览（流程化 + 全接口导向）

这套课程是 `bindings/python` 的实操主线，目标是让你按“真实电机使用流程”掌握全部核心接口。

课程文件规则：

- `xx-*.py`：可直接运行
- `xx-*.md`：该课讲解、参数说明、注意事项

---

## 一、先建立统一心智模型

真实项目里建议始终按这条链路理解：

1. 选链路（`Controller(...)` / `from_socketcanfd(...)` / `from_dm_serial(...)` / `from_dm_device(...)`）
2. 加电机（`add_*_motor(...)`）
3. 使能（`enable_all()`）
4. 模式（`ensure_mode(...)`）
5. 下发（`send_*`）
6. 反馈（`request_feedback()`）
7. 读状态（`get_state()`）

状态获取版本说明：

- `<= v0.1.6`：建议手动 `poll_feedback_once()`
- `v0.1.7+`：默认后台轮询，通常可直接 `get_state()`

---

## 二、课程顺序（按工程实践）

### 00 `enable-and-status`

目标：

- 跑通“能通信、能使能、能拿状态”最小闭环

重点接口：

- `enable_all`
- `request_feedback`
- `get_state`
- `poll_feedback_once`（兼容）

### 01 `scan`

目标：

- 扫描在线电机，确认 `motor_id / feedback_id / model`

重点接口/命令：

- `motorbridge-cli scan`

### 02 `register-rw`

目标：

- 读写寄存器参数，理解“控制参数”和“设备参数”

重点接口：

- `get_register_u32/f32`
- `write_register_u32/f32`
- `store_parameters`

### 03 `mode-switch-method`

目标：

- 统一模式切换方法，避免“命令和模式不匹配”

重点接口：

- `ensure_mode(Mode.*, timeout_ms)`

### 04 / 05 / 06 / 07 单模式课程

目标：

- 分别掌握 `MIT / POS_VEL / VEL / FORCE_POS` 的独立使用

重点接口：

- `send_mit`
- `send_pos_vel`
- `send_vel`
- `send_force_pos`

### 08 `mode-mixed-switch`

目标：

- 在一个程序中安全切换多个模式

重点接口：

- `ensure_mode` + 各 `send_*`

### 09 `multi-motor`

目标：

- 同厂商多电机、跨厂商多控制器统一查询

重点接口：

- `add_*_motor`
- `request_feedback`
- `get_state`

---

## 三、全 Binding 接口速查（不含糊版）

### Controller 接口

构造：

- `Controller(channel="can0")`
- `Controller.from_socketcanfd(channel="can0")`
- `Controller.from_dm_serial(serial_port, baud)`
- `Controller.from_dm_device(dm_device_type="usb2canfd-dual", dm_channel="0")`
  或 `Controller.from_dm_device(dm_device_type="linkx4c", dm_channel="0")`

生命周期：

- `close()`
- `shutdown()`
- `close_bus()`

控制器级操作：

- `enable_all()`
- `disable_all()`
- `poll_feedback_once()`

挂载电机：

- `add_damiao_motor(...)`
- `add_robstride_motor(...)`
- `add_myactuator_motor(...)`
- `add_hightorque_motor(...)`
- `add_hexfellow_motor(...)`

### Motor 接口

生命周期与维护：

- `close()`
- `enable()` / `disable()`
- `clear_error()`
- `set_zero_position()`

模式与控制：

- `ensure_mode(mode, timeout_ms=1000)`
- `send_mit(pos, vel, kp, kd, tau)`
- `send_pos_vel(pos, vlim)`
- `send_vel(vel)`
- `send_force_pos(pos, vlim, ratio)`

反馈与状态：

- `request_feedback()`
- `get_state()` -> `MotorState | None`

通用寄存器：

- `set_can_timeout_ms(timeout_ms)`
- `store_parameters()`
- `write_register_u32/f32(...)`
- `get_register_u32/f32(...)`

RobStride 专属：

- `robstride_ping()`
- `robstride_set_device_id(...)`
- `robstride_get_param_* / robstride_write_param_*`

---

## 四、正常电机使用习惯（强烈建议）

1. 先扫再控，不要盲发。
2. 同时只保留一个发送程序，避免 `os error 105`。
3. 控制周期先保守：`DT_MS=20~50`。
4. 先 `enable + status`，再进入控制模式。
5. 报错先查三件事：ID、波特率、链路类型（socketcan/socketcanfd/mcu-serial/dm-serial/dm-device）。

---

## 五、推荐学习命令

```bash
python3 bindings/python/get_started/courses/00-enable-and-status.py
python3 bindings/python/get_started/courses/01-scan.py
python3 bindings/python/get_started/courses/03-mode-switch-method.py
python3 bindings/python/get_started/courses/09-multi-motor.py
```

---

## 六、与 Mintlify 文档联动

如果你希望看“教程化 + API 手册化”的完整文档站，请看：

- `../motorbridge-docs`
- 重点页：`sdk/python/reference.mdx`、`zh/sdk/python/reference.mdx`

本地预览：

```bash
cd ../motorbridge-docs
npx mintlify dev
```


通道排障参考：请查看 docs/zh/can_debugging.md（包含 can0 的配置说明）。
