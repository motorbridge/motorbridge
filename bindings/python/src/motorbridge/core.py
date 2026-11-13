from __future__ import annotations

import ctypes
from ctypes import c_float, c_int8, c_uint8, c_uint16, c_uint32

from .abi import CState, CStateMetadata, get_abi
from .dm_device_runtime import ensure_dm_device_runtime
from .errors import CallError
from .models import Mode, MotorState, MotorStateSnapshot


def _err_text() -> str:
    msg = get_abi().lib.motor_last_error_message()
    return msg.decode() if msg else "unknown error"


def _ok(rc: int, what: str) -> None:
    if rc != 0:
        raise CallError(f"{what} failed: {_err_text()}")


class Controller:
    def __init__(self, channel: str = "can0") -> None:
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcan(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcan failed: {_err_text()}")

    @classmethod
    def from_socketcanfd(cls, channel: str = "can0") -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcanfd(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcanfd failed: {_err_text()}")
        return self

    @classmethod
    def from_dm_serial(cls, serial_port: str = "/dev/ttyACM0", baud: int = 921600) -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_serial(serial_port.encode(), int(baud))
        if not self._ptr:
            raise CallError(f"new_dm_serial failed: {_err_text()}")
        return self

    @classmethod
    def from_mcu_serial(cls, serial_port: str = "/dev/ttyACM0", baud: int = 921600) -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_mcu_serial(serial_port.encode(), int(baud))
        if not self._ptr:
            raise CallError(f"new_mcu_serial failed: {_err_text()}")
        return self

    @classmethod
    def from_dm_device(
        cls,
        dm_device_type: str = "usb2canfd-dual",
        dm_channel: str = "0",
    ) -> "Controller":
        self = cls.__new__(cls)
        ensure_dm_device_runtime(quiet=True)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_device(
            dm_device_type.encode(),
            dm_channel.encode(),
        )
        if not self._ptr:
            raise CallError(f"new_dm_device failed: {_err_text()}")
        return self

    def close(self) -> None:
        if self._ptr:
            self._abi.lib.motor_controller_free(self._ptr)
            self._ptr = None

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("controller is closed")
        return self._ptr

    def shutdown(self) -> None:
        _ok(self._abi.lib.motor_controller_shutdown(self._require_open()), "controller_shutdown")

    def close_bus(self) -> None:
        _ok(self._abi.lib.motor_controller_close_bus(self._require_open()), "controller_close_bus")

    def enable_all(self) -> None:
        _ok(self._abi.lib.motor_controller_enable_all(self._require_open()), "enable_all")

    def disable_all(self) -> None:
        _ok(self._abi.lib.motor_controller_disable_all(self._require_open()), "disable_all")

    def poll_feedback_once(self) -> None:
        _ok(
            self._abi.lib.motor_controller_poll_feedback_once(self._require_open()),
            "poll_feedback_once",
        )

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_damiao_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_damiao_motor failed: {_err_text()}")
        return Motor(m)

    def add_hexfellow_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_hexfellow_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_hexfellow_motor failed: {_err_text()}")
        return Motor(m)

    def add_myactuator_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_myactuator_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_myactuator_motor failed: {_err_text()}")
        return Motor(m)

    def add_robstride_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        if not 1 <= int(motor_id) <= 255:
            raise ValueError(f"RobStride motor_id must be in 1..255, got {motor_id}")
        if not 0 <= int(feedback_id) <= 255:
            raise ValueError(f"RobStride feedback_id/host_id must be in 0..255, got {feedback_id}")
        m = self._abi.lib.motor_controller_add_robstride_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_robstride_motor failed: {_err_text()}")
        return Motor(m)

    def add_hightorque_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        m = self._abi.lib.motor_controller_add_hightorque_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_hightorque_motor failed: {_err_text()}")
        return Motor(m)

    def add_huayi_motor(self, motor_id: int, feedback_id: int, model: str) -> "Motor":
        """注册一台华翼 CANopen 关节电机。motor_id 即 CANopen 节点号
        (1..=127, SDO 收发 COB-ID 由此推出); feedback_id=0 表示沿用节点号。
        与 RobStride-CiA402 节点可混挂同一条经典 CAN 总线(共享接收线程,
        按 arbitration id 分发)。"""
        if not 1 <= int(motor_id) <= 127:
            raise ValueError(f"Huayi motor_id must be a CANopen node id in 1..127, got {motor_id}")
        if not 0 <= int(feedback_id) <= 127:
            raise ValueError(f"Huayi feedback_id must be in 0..127, got {feedback_id}")
        m = self._abi.lib.motor_controller_add_huayi_motor(
            self._require_open(), motor_id, feedback_id, model.encode()
        )
        if not m:
            raise CallError(f"add_huayi_motor failed: {_err_text()}")
        return Motor(m)

    def huayi_scan_ids(self, start_id: int = 1, end_id: int = 127, timeout_ms: int = 100) -> list[int]:
        """CANopen 节点扫描(厂商无关探测过程): 先广播 NMT start, 再对
        start_id..=end_id 逐个探测 6041h/6061h/603Fh, 任一应答即视为在线。
        返回在线节点号列表。适合配置前发现总线上所有 CANopen 节点
        (包括其他 CANopen 厂商的)。注意: 直连探测会与核心接收线程竞争,
        控制器上已注册电机时报错——先扫描再 add_huayi_motor, 或用同一
        总线上另一个控制器。"""
        if not 1 <= int(start_id) <= 127 or not 1 <= int(end_id) <= 127 or start_id > end_id:
            raise ValueError(f"invalid scan range {start_id}..{end_id} (expected 1..=127, start<=end)")
        span = end_id - start_id + 1
        nodes = (c_uint16 * span)()
        count = c_uint32(0)
        _ok(
            self._abi.lib.motor_controller_huayi_scan_ids(
                self._require_open(), start_id, end_id, timeout_ms, nodes, ctypes.byref(count)
            ),
            "huayi_scan_ids",
        )
        return [int(nodes[i]) for i in range(int(count.value))]

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        try:
            self.shutdown()
        finally:
            self.close()


class Motor:
    def __init__(self, ptr: int) -> None:
        self._abi = get_abi()
        self._ptr = ptr

    def close(self) -> None:
        if self._ptr:
            self._abi.lib.motor_handle_free(self._require_open())
            self._ptr = None

    def _require_open(self) -> int:
        if not self._ptr:
            raise CallError("motor handle is closed")
        return self._ptr

    def enable(self) -> None:
        _ok(self._abi.lib.motor_handle_enable(self._require_open()), "enable")

    def disable(self) -> None:
        _ok(self._abi.lib.motor_handle_disable(self._require_open()), "disable")

    def clear_error(self) -> None:
        _ok(self._abi.lib.motor_handle_clear_error(self._require_open()), "clear_error")

    def set_zero_position(self) -> None:
        _ok(self._abi.lib.motor_handle_set_zero_position(self._require_open()), "set_zero_position")

    def ensure_mode(self, mode: Mode | int, timeout_ms: int = 1000) -> None:
        """切换控制模式(统一编号 1=MIT / 2=POS_VEL / 3=VEL / 4=FORCE_POS)。
        各厂商内部编号不同: Huayi 走 CANopen 6060h(1=PP, 3=PV, 4=力矩,
        5=MIT——**注意 5 在 RobStride 私有协议里是 CSP, 勿混淆**), FORCE_POS
        无对应模式报错; 模式切换含 6061h 回读校验, 校验通过后缓存, 后续
        命令免握手轮次(掉使能/急停/清错后缓存失效)。"""
        _ok(self._abi.lib.motor_handle_ensure_mode(self._require_open(), int(mode), timeout_ms), "ensure_mode")

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        _ok(self._abi.lib.motor_handle_send_mit(self._require_open(), pos, vel, kp, kd, tau), "send_mit")

    def send_pos_vel(self, pos: float, vlim: float) -> None:
        _ok(self._abi.lib.motor_handle_send_pos_vel(self._require_open(), pos, vlim), "send_pos_vel")

    def robstride_send_pos_vel_pp(self, pos: float, vel_max: float, acc_set: float) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_send_pos_vel_pp(self._require_open(), pos, vel_max, acc_set),
            "robstride_send_pos_vel_pp",
        )

    def robstride_send_pos_vel_csp(self, pos: float, vlim: float) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_send_pos_vel_csp(self._require_open(), pos, vlim),
            "robstride_send_pos_vel_csp",
        )

    def send_vel(self, vel: float) -> None:
        _ok(self._abi.lib.motor_handle_send_vel(self._require_open(), vel), "send_vel")

    def send_force_pos(self, pos: float, vlim: float, ratio: float) -> None:
        _ok(
            self._abi.lib.motor_handle_send_force_pos(self._require_open(), pos, vlim, ratio),
            "send_force_pos",
        )

    def request_feedback(self) -> None:
        """请求电机反馈帧(统一入口, 各厂商语义):
        HighTorque 2026-10-23 起为 **DM 语义非阻塞**; 2026-10-24 起连发
        **两帧** 2 字节查询——`17 01`(0x01-0x03 位置/速度/力矩)+
        `16 0E`(0x0E 温度 + 0x0F 错误码, 地址连续并入一帧)——即返回,
        回帧由后台收帧线程入缓存: `27 01` 回帧锚定一次状态计数
        (sequence +1), `26 0E` 回帧就地补丁 t_mos/fault_code(不推进
        计数, 两回帧合为一次状态更新), 之后 get_state /
        get_state_snapshot 读取。掉线不表现为本方法报错(无回复即静默),
        需要新鲜度判定用 get_state_snapshot 的 received_at; 需要"发同帧
        并阻塞等回"用 hightorque_get_register(0x01, 3)。Damiao/
        MyActuator 同为非阻塞; Robstride 为空操作(无单次状态查询命令);
        Hexfellow 为阻塞查询(300ms); Huayi 在 TPDO 周期上报已激活时为
        空操作(对齐 RobStride 流式语义), 未激活时自动激活出厂 TPDO
        (10ms 事件计时器)——之后状态经后台收帧线程持续入缓存。"""
        _ok(self._abi.lib.motor_handle_request_feedback(self._require_open()), "request_feedback")

    def hightorque_send_pos_vel_acc(self, pos: float, vlim: float, acc: float) -> None:
        """HighTorque 梯形控制(07 2d, 固件 v4.6.0+): 目标位置(rad) +
        速度上限(rad/s) + 加速度上限(rad/s²)。仅 HighTorque。"""
        _ok(
            self._abi.lib.motor_handle_hightorque_send_pos_vel_acc(
                self._require_open(), pos, vlim, acc
            ),
            "hightorque_send_pos_vel_acc",
        )

    def hightorque_send_pos_classic(self, pos: float, tqe: float | None = None) -> None:
        """HighTorque 普通位置模式(07 07): 目标位置(rad) + 力矩上限(Nm)。
        tqe=None 表示力矩无限制(0x8000 哨兵); 力矩按型号自适应补偿。仅 HighTorque。"""
        _ok(
            self._abi.lib.motor_handle_hightorque_send_pos_classic(
                self._require_open(), pos, float("nan") if tqe is None else tqe
            ),
            "hightorque_send_pos_classic",
        )

    def hightorque_get_register(
        self, addr: int, count: int = 1, timeout_ms: int = 500
    ) -> list[float]:
        """HighTorque 通用寄存器读(17 族): 从 addr 连续读 count(1-3) 个寄存器。
        2026-10-22 起读取固定按 Int16(ty 参数已删除), 返回**换算后物理量**
        (位置/速度/加速度 → rad 族, 力矩套型号 k 系数, 电流/电压/温度/PWM/
        PID 按各自 scale; 无换算关系的寄存器 1:1 返回原值)。仅 HighTorque。

        地址准入(同日追补决策): addr 及 count 覆盖的每个地址都必须在
        HighTorque 寄存器表内(0x00-0x45 中 xlsx 列出的 43 个)——缺号与
        表外地址(0x46-0xFF)一律报错(整段拒绝, 不发帧); write 同此规则。

        ⚠ 签名变更(旧 (addr, ty, count)): 旧的按位置传参调用
        `get_register(0x01, 1, 3)` 会静默变成 (addr=0x01, count=1,
        timeout_ms=3)——升级脚本时请核对参数位置。"""
        if not 1 <= int(count) <= 3:
            raise ValueError(f"count must be in 1..3, got {count}")
        out = (c_float * count)()
        _ok(
            self._abi.lib.motor_handle_hightorque_get_register(
                self._require_open(), int(addr), int(count), int(timeout_ms), out
            ),
            "hightorque_get_register",
        )
        return [float(v) for v in out]

    def hightorque_request_register(self, addr: int, count: int = 1) -> None:
        """HighTorque 非阻塞寄存器查询(DM 风格"请求即发送"): 只发 17 族
        读请求帧(2026-10-22 起固定 Int16, ty 参数已删除), 立即返回不等
        回帧。地址准入同 hightorque_get_register(须在寄存器表内, 缺号/
        表外报错)。回复由 Rust 后台接收线程解码, 按地址写入电机寄存器缓存;
        之后 hightorque_get_cached_register 读最近值(非阻塞, 已换算为
        物理量), 或 hightorque_get_register 阻塞等新鲜值。适合固定频率
        监测循环: 发完即走, 下一拍读缓存。仅 HighTorque。"""
        if not 1 <= int(count) <= 3:
            raise ValueError(f"count must be in 1..3, got {count}")
        _ok(
            self._abi.lib.motor_handle_hightorque_request_register(
                self._require_open(), int(addr), int(count)
            ),
            "hightorque_request_register",
        )

    def hightorque_get_cached_register(self, addr: int) -> float | None:
        """HighTorque 读寄存器缓存最近值(不发帧不等待, 已换算为物理量)。
        None = 从未收到过该地址的回复, 或地址不在寄存器表内(缺号/表外
        一律 None)。注意不区分新鲜/陈旧(电机掉线后停留在最后一值); 需要
        新鲜值用 hightorque_get_register, 或先
        hightorque_request_register 发查询、隔一拍回读本缓存。仅 HighTorque。"""
        has = c_uint8(0)
        val = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_hightorque_get_cached_register(
                self._require_open(), int(addr), ctypes.byref(has), ctypes.byref(val)
            ),
            "hightorque_get_cached_register",
        )
        return float(val.value) if has.value else None

    def hightorque_send_torque(self, torque_nm: float) -> None:
        """HighTorque 力矩模式(05 13): 直接下发目标力矩(Nm, 物理量,
        型号系数补偿)。无需预先切模式。仅 HighTorque。"""
        _ok(
            self._abi.lib.motor_handle_hightorque_send_torque(
                self._require_open(), torque_nm
            ),
            "hightorque_send_torque",
        )

    def hightorque_send_dq_voltage(self, voltage_v: float) -> None:
        """HighTorque DQ 电压模式(01 00 08 05 1B): Q 轴电压(V, 0.1 V/LSB),
        模式写与寄存器写拼一帧。协议只定义 Q 分量。仅 HighTorque。"""
        _ok(
            self._abi.lib.motor_handle_hightorque_send_dq_voltage(
                self._require_open(), voltage_v
            ),
            "hightorque_send_dq_voltage",
        )

    def hightorque_send_dq_current(self, current_a: float) -> None:
        """HighTorque DQ 电流模式(01 00 09 05 1C): Q 轴电流(A, 0.1 A/LSB),
        模式写与寄存器写拼一帧。仅 HighTorque。"""
        _ok(
            self._abi.lib.motor_handle_hightorque_send_dq_current(
                self._require_open(), current_a
            ),
            "hightorque_send_dq_current",
        )

    def hightorque_brake(self) -> None:
        """HighTorque 刹车(模式 15, 01 00 0F): 三相短接阻尼刹车。仅 HighTorque。"""
        _ok(
            self._abi.lib.motor_handle_hightorque_brake(self._require_open()),
            "hightorque_brake",
        )

    def hightorque_set_run_mode(self, mode: int) -> None:
        """HighTorque 运行模式切换(01 00 <mode> 写模式寄存器 0x000)。
        mode 取 0-17(表2; 0=停止并清错, 8=DQ 电压, 9=DQ 电流, 10=位置,
        15=刹车)。真发帧, 区别于 ensure_mode 的仅校验语义。仅 HighTorque。"""
        if not 0 <= int(mode) <= 17:
            raise ValueError(f"run mode must be in 0..17, got {mode}")
        _ok(
            self._abi.lib.motor_handle_hightorque_set_run_mode(
                self._require_open(), int(mode)
            ),
            "hightorque_set_run_mode",
        )

    def hightorque_write_register(self, addr: int, value: int, config_class: bool = False) -> None:
        """HighTorque 通用寄存器写(05 <addr> <val16>)。config_class=False 走
        控制通道(控制目标类寄存器), True 走扩展 0x8000|id 帧(配置类)。
        addr 须在 HighTorque 寄存器表内(0x00-0x45 中 xlsx 列出的 43 个,
        缺号/表外地址报错且不发帧, 与读同规则)。
        模式寄存器 0x000 不适用, 请用 hightorque_set_run_mode。仅 HighTorque。"""
        if not -32768 <= int(value) <= 32767:
            raise ValueError(f"value must fit int16, got {value}")
        _ok(
            self._abi.lib.motor_handle_hightorque_write_register(
                self._require_open(), int(addr), int(value), 1 if config_class else 0
            ),
            "hightorque_write_register",
        )

    def hightorque_set_periodic_report(self, period_ms: int) -> None:
        """HighTorque 周期上报配置(05 B4 02 <t_ms16>, 扩展帧)。
        period_ms>=1 开启电机按周期主动发 27 01 状态帧; 0 停止上报。仅 HighTorque。"""
        if not 0 <= int(period_ms) <= 65535:
            raise ValueError(f"period_ms must fit uint16, got {period_ms}")
        _ok(
            self._abi.lib.motor_handle_hightorque_set_periodic_report(
                self._require_open(), int(period_ms)
            ),
            "hightorque_set_periodic_report",
        )

    # ------------------------------------------------------------------
    # Huayi (CANopen) vendor-private interface
    # ------------------------------------------------------------------

    def huayi_sdo_read_u16(self, index: int, sub: int, timeout_ms: int = 1000) -> int:
        """Huayi SDO 读 u16。index 为 16 位对象字典索引(如 0x6041 状态字),
        sub 为子索引。对象含义查华翼手册的对象字典章节。仅 Huayi。"""
        out = c_uint16(0)
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_read_u16(
                self._require_open(), int(index), int(sub), int(timeout_ms), ctypes.byref(out)
            ),
            "huayi_sdo_read_u16",
        )
        return int(out.value)

    def huayi_sdo_read_i16(self, index: int, sub: int, timeout_ms: int = 1000) -> int:
        """Huayi SDO 读 i16(如 6071h 目标力矩 ‰、2204h 温度 0.1°C)。仅 Huayi。"""
        out = c_int16(0)
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_read_i16(
                self._require_open(), int(index), int(sub), int(timeout_ms), ctypes.byref(out)
            ),
            "huayi_sdo_read_i16",
        )
        return int(out.value)

    def huayi_sdo_read_i32(self, index: int, sub: int, timeout_ms: int = 1000) -> int:
        """Huayi SDO 读 i32。仅 Huayi。"""
        out = c_int32(0)
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_read_i32(
                self._require_open(), int(index), int(sub), int(timeout_ms), ctypes.byref(out)
            ),
            "huayi_sdo_read_i32",
        )
        return int(out.value)

    def huayi_sdo_read_f32(self, index: int, sub: int, timeout_ms: int = 1000) -> float:
        """Huayi SDO 读 f32(位置/速度/加速度, 单位为输出轴 Rev 族)。仅 Huayi。"""
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_read_f32(
                self._require_open(), int(index), int(sub), int(timeout_ms), ctypes.byref(out)
            ),
            "huayi_sdo_read_f32",
        )
        return float(out.value)

    def huayi_sdo_write_u16(self, index: int, sub: int, value: int) -> None:
        """Huayi SDO 写 u16。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_write_u16(
                self._require_open(), int(index), int(sub), int(value)
            ),
            "huayi_sdo_write_u16",
        )

    def huayi_sdo_write_i16(self, index: int, sub: int, value: int) -> None:
        """Huayi SDO 写 i16。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_write_i16(
                self._require_open(), int(index), int(sub), int(value)
            ),
            "huayi_sdo_write_i16",
        )

    def huayi_sdo_write_i32(self, index: int, sub: int, value: int) -> None:
        """Huayi SDO 写 i32。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_write_i32(
                self._require_open(), int(index), int(sub), int(value)
            ),
            "huayi_sdo_write_i32",
        )

    def huayi_sdo_write_f32(self, index: int, sub: int, value: float) -> None:
        """Huayi SDO 写 f32。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_sdo_write_f32(
                self._require_open(), int(index), int(sub), float(value)
            ),
            "huayi_sdo_write_f32",
        )

    def huayi_send_torque(self, torque_nm: float) -> None:
        """Huayi 力矩模式(6060h=4): 目标力矩 Nm → 6071h Int16, 单位为
        6076h 峰值力矩的千分比(换算在后端, 峰值首次读取后缓存)。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_send_torque(
                self._require_open(), float(torque_nm)
            ),
            "huayi_send_torque",
        )

    def huayi_send_mit_compressed(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        """Huayi 压缩 MIT 实时通道(手册 §5.2): RPDO4 重映射为 2004h_02/_03,
        每条指令一帧 8 字节经典帧发往 0x500+节点号。64 位控制字按
        2004h 范围寄存器线性映射打包 Pdes(16b)/Vdes(12b)/Kp(12b)/Kd(12b)/
        Tff(12b)。参数为统一 SI 单位(rad, rad/s, Nm/rad, Nm·s/rad, Nm),
        Kp/Kd 单位链: Nm/rad →(×2π)→ Nm/Rev →(线性映射)→ 12 位字段。
        首次调用自动完成 RPDO 重映射与范围寄存器读取(随后各仅一帧)。
        慢速 2003h SDO 路径为通用的 send_mit。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_send_mit_compressed(
                self._require_open(),
                float(pos),
                float(vel),
                float(kp),
                float(kd),
                float(tau),
            ),
            "huayi_send_mit_compressed",
        )

    def huayi_position_preset(self, position_rev: float) -> None:
        """Huayi 位置预设(3001h, 魔数 "pres"): 将当前位置重新解释为
        position_rev(输出轴转数, |值|≤0.5)。**必须处于 Switch On
        Disabled 状态**——先 disable() 再调用, 否则报错(与 Damiao
        set_zero_position 的"先 disable"守卫同构)。set_zero_position()
        即本方法的 0 值形式。仅 Huayi。"""
        _ok(
            self._abi.lib.motor_handle_huayi_position_preset(
                self._require_open(), float(position_rev)
            ),
            "huayi_position_preset",
        )

    def huayi_configure_periodic_report(self, period_ms: int) -> None:
        """Huayi 周期上报(对齐 RobStride set_active_report 的角色):
        period_ms>0 激活三个出厂 TPDO——TPDO2(位置/速度)为**锚定帧**
        (seq+1、刷新时间戳), TPDO1(错误码/状态字/模式)与 TPDO3(力矩/
        双温度)为**就地补丁帧**(不推进 seq, 同 hightorque 两帧反馈模型),
        事件计时器=period_ms; period_ms=0 关闭(COB-ID 置无效位, 映射
        保留, 再激活即恢复)。范围 1..=65535 ms。仅 Huayi。"""
        if not 0 <= int(period_ms) <= 65535:
            raise ValueError(f"period_ms must fit uint16, got {period_ms}")
        _ok(
            self._abi.lib.motor_handle_huayi_configure_periodic_report(
                self._require_open(), int(period_ms)
            ),
            "huayi_configure_periodic_report",
        )

    def huayi_get_error_code(self) -> int:
        """Huayi 最近缓存的错误码(603Fh/EMCY, u16; 0=无错误, 不发帧)。
        从未收到过反馈时报错(先 configure_periodic_report 或
        request_feedback)。错误码含义查手册表(0x2310 过流等)。
        get_state 的 fault_code 字段是本值的低字节。仅 Huayi。"""
        out = c_uint16(0)
        _ok(
            self._abi.lib.motor_handle_huayi_get_error_code(
                self._require_open(), ctypes.byref(out)
            ),
            "huayi_get_error_code",
        )
        return int(out.value)

    def huayi_get_error_history(self) -> list[int]:
        """Huayi 错误历史(1003h 子索引遍历): 最多返回 32 条错误码,
        从旧到新。仅 Huayi。"""
        codes = (c_uint16 * 32)()
        count = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_huayi_get_error_history(
                self._require_open(), codes, ctypes.byref(count)
            ),
            "huayi_get_error_history",
        )
        return [int(codes[i]) for i in range(int(count.value))]

    def huayi_set_node_id(self, new_id: int) -> None:
        """Huayi 节点号修改(2001h_1): 节点复位后生效。new_id 1..=127。
        仅 Huayi。"""
        if not 1 <= int(new_id) <= 127:
            raise ValueError(f"Huayi new_id must be a CANopen node id in 1..127, got {new_id}")
        _ok(
            self._abi.lib.motor_handle_huayi_set_node_id(
                self._require_open(), int(new_id)
            ),
            "huayi_set_node_id",
        )

    def huayi_nmt(self, command: int, node: int = 0) -> None:
        """Huayi 总线上的 NMT 主站命令(COB-ID 0x000, [command, node])。
        command: 0x01 start / 0x02 stop / 0x80 进入预操作 / 0x81 节点复位 /
        0x82 通信复位; node=0 广播总线上所有 CANopen 节点(含其他厂商)。
        发完即走不等回复。仅 Huayi 电机句柄(命令作用于整条总线)。
        指向本节点(或广播)的复位会使本地缓存失效: 0x81 清空模式/PP 握手/
        TPDO 标记, 0x82 / 0x80 / 0x02 清 TPDO 标记——下一条命令自动重走
        完整握手, 不会跳过驱动器已忘记的步骤。"""
        if int(node) < 0 or int(node) > 127:
            raise ValueError(f"NMT node must be in 0..127, got {node}")
        _ok(
            self._abi.lib.motor_handle_huayi_nmt(self._require_open(), int(command), int(node)),
            "huayi_nmt",
        )

    def set_can_timeout_ms(self, timeout_ms: int) -> None:
        _ok(self._abi.lib.motor_handle_set_can_timeout_ms(self._require_open(), timeout_ms), "set_can_timeout_ms")

    def store_parameters(self) -> None:
        _ok(self._abi.lib.motor_handle_store_parameters(self._require_open()), "store_parameters")

    def write_register_f32(self, rid: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_write_register_f32(self._require_open(), rid, value), "write_register_f32")

    def write_register_u32(self, rid: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_write_register_u32(self._require_open(), rid, value), "write_register_u32")

    def get_register_f32(self, rid: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_get_register_f32(
                self._require_open(), rid, timeout_ms, ctypes.byref(out)
            ),
            "get_register_f32",
        )
        return float(out.value)

    def get_register_u32(self, rid: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_get_register_u32(
                self._require_open(), rid, timeout_ms, ctypes.byref(out)
            ),
            "get_register_u32",
        )
        return int(out.value)

    def robstride_ping(self) -> tuple[int, int]:
        device_id = c_uint8(0)
        responder_id = c_uint8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_ping(
                self._require_open(), ctypes.byref(device_id), ctypes.byref(responder_id)
            ),
            "robstride_ping",
        )
        return int(device_id.value), int(responder_id.value)

    def robstride_ping_host_id(self, host_id: int, timeout_ms: int = 500) -> tuple[int, int]:
        if not 0 <= int(host_id) <= 255:
            raise ValueError(f"RobStride host_id must be in 0..255, got {host_id}")
        device_id = c_uint8(0)
        responder_id = c_uint8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_ping_host_id(
                self._require_open(), host_id, timeout_ms, ctypes.byref(device_id), ctypes.byref(responder_id)
            ),
            "robstride_ping_host_id",
        )
        return int(device_id.value), int(responder_id.value)

    def robstride_get_param_f32_host_id(self, param_id: int, host_id: int, timeout_ms: int = 1000) -> float:
        if not 0 <= int(host_id) <= 255:
            raise ValueError(f"RobStride host_id must be in 0..255, got {host_id}")
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_f32_host_id(
                self._require_open(), param_id, host_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_f32_host_id",
        )
        return float(out.value)

    def robstride_get_fault_report(self) -> tuple[int, int]:
        fault_raw = c_uint32(0)
        warning_raw = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_fault_report(
                self._require_open(), ctypes.byref(fault_raw), ctypes.byref(warning_raw)
            ),
            "robstride_get_fault_report",
        )
        return int(fault_raw.value), int(warning_raw.value)

    def robstride_set_device_id(self, new_device_id: int) -> None:
        if not 1 <= int(new_device_id) <= 255:
            raise ValueError(f"RobStride new_device_id must be in 1..255, got {new_device_id}")
        _ok(self._abi.lib.motor_handle_robstride_set_device_id(self._require_open(), new_device_id), "robstride_set_device_id")

    def robstride_set_active_report(self, enabled: bool) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_set_active_report(
                self._require_open(), 1 if enabled else 0
            ),
            "robstride_set_active_report",
        )

    def robstride_write_param_i8(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_i8(self._require_open(), param_id, value), "robstride_write_param_i8")

    def robstride_write_param_u8(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_u8(self._require_open(), param_id, value), "robstride_write_param_u8")

    def robstride_write_param_u16(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_u16(self._require_open(), param_id, value), "robstride_write_param_u16")

    def robstride_write_param_u32(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_u32(self._require_open(), param_id, value), "robstride_write_param_u32")

    def robstride_write_param_f32(self, param_id: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_robstride_write_param_f32(self._require_open(), param_id, value), "robstride_write_param_f32")

    def robstride_get_param_i8(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_int8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_i8(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_i8",
        )
        return int(out.value)

    def robstride_get_param_u8(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint8(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_u8(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_u8",
        )
        return int(out.value)

    def robstride_get_param_u16(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint16(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_u16(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_u16",
        )
        return int(out.value)

    def robstride_get_param_u32(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_u32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_u32",
        )
        return int(out.value)

    def robstride_get_param_f32(self, param_id: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_robstride_get_param_f32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "robstride_get_param_f32",
        )
        return float(out.value)

    def damiao_get_param_f32(self, param_id: int, timeout_ms: int = 1000) -> float:
        out = c_float(0.0)
        _ok(
            self._abi.lib.motor_handle_damiao_get_param_f32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "damiao_get_param_f32",
        )
        return float(out.value)

    def damiao_get_param_u32(self, param_id: int, timeout_ms: int = 1000) -> int:
        out = c_uint32(0)
        _ok(
            self._abi.lib.motor_handle_damiao_get_param_u32(
                self._require_open(), param_id, timeout_ms, ctypes.byref(out)
            ),
            "damiao_get_param_u32",
        )
        return int(out.value)

    def damiao_write_param_f32(self, param_id: int, value: float) -> None:
        _ok(self._abi.lib.motor_handle_damiao_write_param_f32(self._require_open(), param_id, value), "damiao_write_param_f32")

    def damiao_write_param_u32(self, param_id: int, value: int) -> None:
        _ok(self._abi.lib.motor_handle_damiao_write_param_u32(self._require_open(), param_id, value), "damiao_write_param_u32")

    def get_state(self) -> MotorState | None:
        st = CState()
        _ok(self._abi.lib.motor_handle_get_state(self._require_open(), ctypes.byref(st)), "get_state")
        if not st.has_value:
            return None
        return MotorState(
            can_id=int(st.can_id),
            arbitration_id=int(st.arbitration_id),
            status_code=int(st.status_code),
            pos=float(st.pos),
            vel=float(st.vel),
            torq=float(st.torq),
            t_mos=float(st.t_mos),
            t_rotor=float(st.t_rotor),
            fault_code=int(st.fault_code),
        )

    def get_state_snapshot(self) -> MotorStateSnapshot | None:
        state = CState()
        metadata = CStateMetadata()
        _ok(
            self._abi.lib.motor_handle_get_state_snapshot(
                self._require_open(), ctypes.byref(state), ctypes.byref(metadata)
            ),
            "get_state_snapshot",
        )
        if not state.has_value:
            return None
        return MotorStateSnapshot(
            state=MotorState(
                can_id=int(state.can_id),
                arbitration_id=int(state.arbitration_id),
                status_code=int(state.status_code),
                pos=float(state.pos),
                vel=float(state.vel),
                torq=float(state.torq),
                t_mos=float(state.t_mos),
                t_rotor=float(state.t_rotor),
            ),
            feedback_age_s=int(metadata.feedback_age_ns) / 1_000_000_000,
            feedback_sequence=int(metadata.feedback_sequence),
        )
