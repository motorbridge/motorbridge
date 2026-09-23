use motor_core::bus::CanFrame;
use std::f32::consts::PI;

const TWO_PI: f32 = PI * 2.0;

#[derive(Debug, Clone, Copy)]
pub struct HightorqueFeedbackState {
    pub can_id: u8,
    pub arbitration_id: u32,
    /// 运行模式(表2)。寄存器读回复时为 0;状态反馈帧时为 byte0。
    pub status_code: u8,
    /// 故障码(表3)。寄存器读回复时为 0;状态反馈帧时为 byte1。
    pub fault_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

impl HightorqueFeedbackState {
    /// 解析 `status_code`(byte0)为运行模式;寄存器读回复(status_code=0 且非状态帧)
    /// 与未知码返回 `None`。
    pub fn run_mode(&self) -> Option<RunMode> {
        RunMode::from_code(self.status_code)
    }

    /// 故障码的人读描述(表3)。
    pub fn fault_description(&self) -> &'static str {
        decode_fault(self.fault_code)
    }
}

/// 运行模式(表2)。状态反馈帧的 byte0 即为该值。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RunMode {
    Stop = 0,               // 停机/清除错误
    Error = 1,              // 故障
    Ready2 = 2,             // 就绪
    Ready3 = 3,             // 就绪
    Ready4 = 4,             // 就绪
    Pwm = 5,                // PWM 输出
    Voltage = 6,            // 电压输出
    FocVoltage = 7,         // FOC 电压
    DqVoltage = 8,          // DQ 电压
    DqCurrent = 9,          // DQ 电流
    Position = 10,          // 位置模式
    Timeout = 11,           // 超时
    ZeroSpeed = 12,         // 零速
    Range = 13,             // 量程
    MeasureInductance = 14, // 测量电感
    Brake = 15,             // 制动
    Reserved16 = 16,
    Reserved17 = 17,
}

impl RunMode {
    /// 从 byte0 解析运行模式;超出 0-17 返回 `None`。
    pub fn from_code(code: u8) -> Option<Self> {
        Some(match code {
            0 => Self::Stop,
            1 => Self::Error,
            2 => Self::Ready2,
            3 => Self::Ready3,
            4 => Self::Ready4,
            5 => Self::Pwm,
            6 => Self::Voltage,
            7 => Self::FocVoltage,
            8 => Self::DqVoltage,
            9 => Self::DqCurrent,
            10 => Self::Position,
            11 => Self::Timeout,
            12 => Self::ZeroSpeed,
            13 => Self::Range,
            14 => Self::MeasureInductance,
            15 => Self::Brake,
            16 => Self::Reserved16,
            17 => Self::Reserved17,
            _ => return None,
        })
    }
}

/// 故障码(表3)→ 人读描述。8-31 与 >47 为规范未定义,返回"未知故障码"。
pub fn decode_fault(code: u8) -> &'static str {
    match code {
        0 => "正常",
        // 1-7:UART/DMA 相关错误(规范未逐条命名,统一归类)
        1..=7 => "UART/DMA 错误",
        32 => "校准错误",
        33 => "驱动故障",
        34 => "过压",
        35 => "编码器错误",
        36 => "未校准",
        37 => "PWM 周期错误",
        38 => "过温",
        39 => "起始位置超范围",
        40 => "欠压",
        41 => "配置已变更",
        42 => "无效角度",
        43 => "无效位置",
        44 => "驱动使能故障",
        45 => "停止位置误用",
        46 => "时序错误",
        47 => "反电动势前馈错误",
        _ => "未知故障码",
    }
}

/// Extract the motor id from a reply frame's arbitration ID.
///
/// Per the Hightorque CAN protocol (v2.0.0 firmware `motor.c::motor_process_state_all`,
/// `id = fdcan_rx_header.Identifier >> 8`), the motor always places its id in the
/// *high* byte of the reply ID (`source = motor_id`, `dest = 0`, reply bit = 0),
/// for both standard 11-bit and extended 29-bit frames. The previous extended-frame
/// branch read the low byte (`& 0x7F`), which only matched `motor_id == 0`.
///
/// Returns `None` for frames addressed to a non-zero destination (other hosts) —
/// replies to host 0 always carry `dest = 0`.
pub(crate) fn reply_motor_id(frame: &CanFrame) -> Option<u8> {
    if (frame.arbitration_id & 0x00FF) != 0 {
        return None;
    }
    Some(((frame.arbitration_id >> 8) & 0x7F) as u8)
}

pub(crate) fn decode_read_reply(
    frame: CanFrame,
    c: TorqueCoeff,
) -> Option<HightorqueFeedbackState> {
    if frame.dlc < 8 {
        return None;
    }
    if frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    let can_id = reply_motor_id(&frame)?;
    let pos_raw = i16::from_le_bytes([frame.data[2], frame.data[3]]);
    let vel_raw = i16::from_le_bytes([frame.data[4], frame.data[5]]);
    let tqe_raw = i16::from_le_bytes([frame.data[6], frame.data[7]]);
    Some(HightorqueFeedbackState {
        can_id,
        arbitration_id: frame.arbitration_id,
        status_code: 0,
        fault_code: 0,
        pos: pos_raw as f32 * 0.0001 * TWO_PI,
        vel: vel_raw as f32 * 0.00025 * TWO_PI,
        torq: tqe_restore_raw(tqe_raw, c),
        t_mos: 0.0,
        t_rotor: 0.0,
    })
}

/// 解析控制命令附带的状态反馈帧(参考固件 `motor.c::motor_process_state`
/// 的 `len==8 && p_data[0]!=0x27` 分支):byte0=mode(表2),byte1=fault(表3),
/// byte2-7 = pos/vel/tqe int16(小端),标度与寄存器读回复一致。
///
/// 寄存器读回复(0x27...)、设置 ACK(0x41,7 字节)、固件版本回复(5 字节)
/// 均不满足此条件,会被返回 `None`,由各自的处理路径负责。
pub(crate) fn decode_status_frame(
    frame: &CanFrame,
    c: TorqueCoeff,
) -> Option<HightorqueFeedbackState> {
    if frame.dlc < 8 || frame.data[0] == 0x27 {
        return None;
    }
    let can_id = reply_motor_id(frame)?;
    let pos_raw = i16::from_le_bytes([frame.data[2], frame.data[3]]);
    let vel_raw = i16::from_le_bytes([frame.data[4], frame.data[5]]);
    let tqe_raw = i16::from_le_bytes([frame.data[6], frame.data[7]]);
    Some(HightorqueFeedbackState {
        can_id,
        arbitration_id: frame.arbitration_id,
        status_code: frame.data[0],
        fault_code: frame.data[1],
        pos: pos_raw as f32 * 0.0001 * TWO_PI,
        vel: vel_raw as f32 * 0.00025 * TWO_PI,
        torq: tqe_restore_raw(tqe_raw, c),
        t_mos: 0.0,
        t_rotor: 0.0,
    })
}

/// 被动反馈分派:先试寄存器读回复,再试状态反馈帧。
pub(crate) fn decode_feedback(frame: CanFrame, c: TorqueCoeff) -> Option<HightorqueFeedbackState> {
    decode_read_reply(frame, c).or_else(|| decode_status_frame(&frame, c))
}

// ----- 设置 ACK 与固件版本回复(参考固件 motor.c 设置/版本解析分支)-----
//
// 设置类命令成功后,电机回 7 字节 ACK:`0x41 0x01 0x04` + `"OK\r\n"`。
// 固件版本查询(`0x15 0xB5 0x02`)回 5 字节,签名 `data[1]==0xB5 && data[2]==0x02`,
// 版本号按半位打包(镜像 `motor.c`):major=data[4]>>4、minor=(data[4]&0x0F)|(data[3]>>4)、
// patch=data[3]&0x0F。这两类帧都是"事务性回复",不属于被动反馈(decode_feedback 返回 None),
// 由各自的活动读取路径(send_with_ack / request_firmware_version)消费。

/// 设置类命令的成功 ACK(7 字节):`0x41 0x01 0x04` + `"OK\r\n"`。
pub const SETTING_ACK: [u8; 7] = [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A];

/// 判断一帧是否为设置 ACK(dlc==7 且前 7 字节精确匹配 `SETTING_ACK`)。
pub(crate) fn is_setting_ack(frame: &CanFrame) -> bool {
    frame.dlc == 7 && frame.data.starts_with(&SETTING_ACK)
}

/// 电机固件版本(major.minor.patch,各 4 位编码于 5 字节回复)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareVersion {
    pub major: u8,
    pub minor: u8,
    pub patch: u8,
}

impl FirmwareVersion {
    pub fn as_tuple(self) -> (u8, u8, u8) {
        (self.major, self.minor, self.patch)
    }
}

/// 解析固件版本回复(5 字节,签名 `data[1]==0xB5 && data[2]==0x02`)。
/// 长度或签名不符时返回 `None`(不回退到固件的 3.9.1 默认值,避免伪造版本)。
pub(crate) fn decode_version_reply(frame: &CanFrame) -> Option<FirmwareVersion> {
    if frame.dlc != 5 || frame.data[1] != 0xB5 || frame.data[2] != 0x02 {
        return None;
    }
    Some(FirmwareVersion {
        major: frame.data[4] >> 4,
        minor: (frame.data[4] & 0x0F) | (frame.data[3] >> 4),
        patch: frame.data[3] & 0x0F,
    })
}

// ----- P2-16 单位与数据类型选择 -----
//
// 对照参考固件 `convert.c`:`pos_vel_type_t`(RADIAN_2PI/ANGLE_360/TURNS,
// 默认 `MOTOR_DATA_TYPE_FLAG = TURNS`)与 `data_type_t`(TFLOAT/TINT32/TINT16;
// TINT8 在固件 switch 中未实现但 §1.3 cmd 位含 int8 且 rint8 scale 仍传入)。
// 写路径 `conv_to_turns(v,unit) → xxx_float2int(turns,type) = turns*rintX`;
// 读路径 `xxx_int2float(raw,type) = raw/rintX → conv_from_turns(turns,unit)`。

/// 角度单位(参考固件 `pos_vel_type_t`)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AngleUnit {
    /// 弧度制(`RADIAN_2PI`,固件 `MY_2PI` 基准)。crate 默认。
    Rad,
    /// 角度制(`ANGLE_360`,0–360°)。
    Degree,
    /// 圈数制(`TURNS`,固件默认 `MOTOR_DATA_TYPE_FLAG`)。
    Turn,
}

/// 数据类型(参考固件 `data_type_t` + §1.3 cmd 半位 `[3:2]`)。
///
/// 注:固件 `data_float2int`/`data_int2float` 的 switch 未实现 `TINT8`
/// (default → `MOTOR_ERR()`),但协议 §1.3 的 cmd 位编码含 int8,且各
/// `*_float2int` 仍传入 rint8 scale,故此处保留 `Int8` 并取该 scale。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataType {
    Int8,
    Int16,
    Int32,
    Float,
}

/// 某物理量在 Int8/Int16/Int32 下的 raw scale(Float 为恒等 1.0,不查表)。
/// 逐项对照 `convert.c` 各 `*_float2int` 的 rint8/rint16/rint32 入参。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuantityScale {
    pub int8: f32,
    pub int16: f32,
    pub int32: f32,
}

impl QuantityScale {
    const fn scale(self, ty: DataType) -> f32 {
        match ty {
            DataType::Int8 => self.int8,
            DataType::Int16 => self.int16,
            DataType::Int32 => self.int32,
            DataType::Float => 1.0,
        }
    }
}

/// 位置 raw scale(对照 `pos_float2int`:rint8=100, rint16=10000, rint32=100000)。
pub const POS_SCALE: QuantityScale = QuantityScale {
    int8: 100.0,
    int16: 10000.0,
    int32: 100000.0,
};
/// 速度 raw scale(对照 `vel_float2int`:rint8=100, rint16=4000, rint32=100000)。
pub const VEL_SCALE: QuantityScale = QuantityScale {
    int8: 100.0,
    int16: 4000.0,
    int32: 100000.0,
};
/// 力矩 raw scale(对照 `tqe_float2int`:rint8=2, rint16=100, rint32=1000)。
pub const TQE_SCALE: QuantityScale = QuantityScale {
    int8: 2.0,
    int16: 100.0,
    int32: 1000.0,
};
/// 加速度 raw scale(对照 `acc_float2int`:rint8=20, rint16=1000, rint32=100000)。
pub const ACC_SCALE: QuantityScale = QuantityScale {
    int8: 20.0,
    int16: 1000.0,
    int32: 100000.0,
};
/// 电流 raw scale(对照 `cur_float2int`:rint8=1, rint16=10, rint32=1000)。
pub const CUR_SCALE: QuantityScale = QuantityScale {
    int8: 1.0,
    int16: 10.0,
    int32: 1000.0,
};
/// 电压 raw scale(对照 `vol_float2int`:rint8=2, rint16=10, rint32=1000)。
pub const VOL_SCALE: QuantityScale = QuantityScale {
    int8: 2.0,
    int16: 10.0,
    int32: 1000.0,
};
/// PID raw scale(对照 `pid_float2int`:rint8=1, rint16=10, rint32=1000)。
pub const PID_SCALE: QuantityScale = QuantityScale {
    int8: 1.0,
    int16: 10.0,
    int32: 1000.0,
};

/// 单位→圈数(镜像 `conv_to_turns`)。
pub fn to_turns(value: f32, unit: AngleUnit) -> f32 {
    match unit {
        AngleUnit::Rad => value / TWO_PI,
        AngleUnit::Degree => value / 360.0,
        AngleUnit::Turn => value,
    }
}

/// 圈数→单位(镜像 `conv_from_turns`)。
pub fn from_turns(turns: f32, unit: AngleUnit) -> f32 {
    match unit {
        AngleUnit::Rad => turns * TWO_PI,
        AngleUnit::Degree => turns * 360.0,
        AngleUnit::Turn => turns,
    }
}

/// 数据类型对应的 raw 对称饱和限(对照 `convert.c::data_limit` 的 max 入参:
/// TINT16=32760、TINT32=2147483640)。注:固件 `data_limit` 负侧实现有笔误
/// (`return -min` → 负饱和返回正值),此处修正为对称饱和,避免负向溢出误为正满量程。
const fn int_clamp_limit(ty: DataType) -> f32 {
    match ty {
        DataType::Int8 => 127.0,
        DataType::Int16 => 32760.0,
        DataType::Int32 => 2_147_483_640.0,
        DataType::Float => f32::INFINITY,
    }
}

/// 写:物理量(指定单位)→ float raw(单位先经 `conv_to_turns` 转圈,再乘 scale;
/// 镜像 `xxx_float2int` 的 `in_data * rintX`)。已按 `ty` 对称饱和到对应整型限。
/// `DataType::Float` 原样返回(不乘 scale、不饱和)。
pub fn encode_unit_raw(value: f32, unit: AngleUnit, ty: DataType, scale: QuantityScale) -> f32 {
    if ty == DataType::Float {
        return value;
    }
    let turns = to_turns(value, unit);
    let raw = turns * scale.scale(ty);
    let lim = int_clamp_limit(ty);
    raw.clamp(-lim, lim)
}

/// 读:float raw → 物理量(指定单位;镜像 `xxx_int2float` 的 `raw / rintX`
/// 再 `conv_from_turns`)。`DataType::Float` 原样返回。
pub fn decode_unit_raw(raw: f32, unit: AngleUnit, ty: DataType, scale: QuantityScale) -> f32 {
    if ty == DataType::Float {
        return raw;
    }
    let turns = raw / scale.scale(ty);
    from_turns(turns, unit)
}

/// i16 raw 编码便捷路径(默认 Rad+Int16,crate 现有命令路径沿用)。
fn to_i16_raw(value: f32, unit: AngleUnit, scale: QuantityScale) -> i16 {
    let raw = encode_unit_raw(value, unit, DataType::Int16, scale);
    (raw.round() as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

pub(crate) fn rad_to_pos_raw(rad: f32) -> i16 {
    to_i16_raw(rad, AngleUnit::Rad, POS_SCALE)
}

pub(crate) fn radps_to_vel_raw(radps: f32) -> i16 {
    to_i16_raw(radps, AngleUnit::Rad, VEL_SCALE)
}

/// 力矩补偿系数 {k, d}(参考固件 `motor_tqe_adj_t`)。
///
/// 写力矩:`raw = tqe_adjust(tau) * 100 = ((tau - d) / k) * 100`;
/// 读力矩:`tau = tqe_restore(raw * 0.01) = raw * 0.01 * k + d`。
/// 当前固件所有型号的 `d` 均为 0(注释里的 d 值已禁用)。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TorqueCoeff {
    pub k: f32,
    pub d: f32,
}

/// 电机型号(参考固件 `motor_type_t`)。决定力矩 raw↔Nm 的补偿系数。
/// 变体名镜像固件枚举码(如 `M60SG_35`),不改为驼峰以保持与源码对照可读。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum MotorModel {
    Null,
    M3536_32,
    M4438_30,
    M4438_32,
    M4538_19,
    M5043_20,
    M5046_20,
    M5047_09,
    M5047_36,
    /// HTDW-5036-02-CNE(50mm 行星关节, 减速比 36, Kt=0.54 Nm/A)。
    M5036_36,
    M6056_36,
    M7256_35,
    M60SG_35,
    M60BM_35,
    /// 通用/无明确型号(k=0.5, d=0)。固件在型号未知时用此项。
    General,
}

impl MotorModel {
    /// 力矩补偿系数(逐项对照参考固件 `motor_tqe_adj[]`)。
    pub const fn coeff(self) -> TorqueCoeff {
        match self {
            Self::Null => TorqueCoeff { k: 0.0, d: 0.0 },
            Self::M3536_32 => TorqueCoeff {
                k: 0.458_105,
                d: 0.0,
            },
            Self::M4438_30 => TorqueCoeff {
                k: 0.525_600,
                d: 0.0,
            },
            Self::M4438_32 => TorqueCoeff {
                k: 0.485_565,
                d: 0.0,
            },
            Self::M4538_19 => TorqueCoeff {
                k: 0.493_835,
                d: 0.0,
            },
            Self::M5043_20 => TorqueCoeff { k: 0.966, d: 0.0 },
            Self::M5046_20 => TorqueCoeff {
                k: 0.533_654,
                d: 0.0,
            },
            Self::M5047_09 => TorqueCoeff {
                k: 0.547_474,
                d: 0.0,
            },
            Self::M5047_36 => TorqueCoeff { k: 0.35, d: 0.0 },
            Self::M5036_36 => TorqueCoeff { k: 0.54, d: 0.0 },
            Self::M6056_36 => TorqueCoeff { k: 0.677, d: 0.0 },
            Self::M7256_35 => TorqueCoeff {
                k: 0.676_524,
                d: 0.0,
            },
            Self::M60SG_35 => TorqueCoeff { k: 0.794_2, d: 0.0 },
            Self::M60BM_35 => TorqueCoeff { k: 0.794_2, d: 0.0 },
            Self::General => TorqueCoeff { k: 0.5, d: 0.0 },
        }
    }

    /// 从型号提示字符串解析。接受 `5046-20` / `5046_20` / `M5046_20` /
    /// `ht-5046-20` / `5036` / `ht-5036` 等大小写与分隔符变体;通用占位(`""`/`hightorque`/`ht`/
    /// `auto`/`default`/`general`)返回 [`MotorModel::General`];无法识别返回 `None`。
    pub fn from_hint(hint: &str) -> Option<Self> {
        let s = hint.trim().to_ascii_lowercase();
        match s.as_str() {
            "" | "hightorque" | "ht" | "auto" | "default" | "general" => {
                return Some(Self::General)
            }
            "null" | "none" => return Some(Self::Null),
            _ => {}
        }
        let mut t = s.replace('-', "_");
        for prefix in ["hightorque_", "ht_", "m"] {
            if let Some(rest) = t.strip_prefix(prefix) {
                t = rest.to_string();
                break;
            }
        }
        Some(match t.as_str() {
            "3536_32" => Self::M3536_32,
            "4438_30" => Self::M4438_30,
            "4438_32" => Self::M4438_32,
            "4538_19" => Self::M4538_19,
            "5043_20" => Self::M5043_20,
            "5046_20" => Self::M5046_20,
            "5047_09" => Self::M5047_09,
            "5047_36" => Self::M5047_36,
            // HTDW-5036-02-CNE: 后缀按 SDK 约定取减速比(36), 非产品商业代号 -02。
            // 裸 "5036"、减速比式 "5036_36"、商业代号 "5036_02" 均映射到唯一 5036 变体。
            "5036" | "5036_36" | "5036_02" => Self::M5036_36,
            "6056_36" => Self::M6056_36,
            "7256_35" => Self::M7256_35,
            "60sg_35" => Self::M60SG_35,
            "60bm_35" => Self::M60BM_35,
            _ => return None,
        })
    }
}

/// 写力矩:真实 Nm → int16 raw(经 `tqe_adjust` + TINT16 scale=100)。
/// `k == 0`(MNULL)时返回 0,避免除零。
pub(crate) fn tqe_adjust_to_raw(tau_nm: f32, c: TorqueCoeff) -> i16 {
    if c.k == 0.0 {
        return 0;
    }
    let v = (((tau_nm - c.d) / c.k) * 100.0).round() as i32;
    v.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// 读力矩:int16 raw → 真实 Nm(经 TINT16 scale=0.01 + `tqe_restore`)。
pub(crate) fn tqe_restore_raw(raw: i16, c: TorqueCoeff) -> f32 {
    raw as f32 * 0.01 * c.k + c.d
}

/// 加速度(rad/s²)→梯形模式 int16 原始值。
///
/// 参考固件 `motor_set_pos_vel_acc` 经 `conv_to_turns` → `acc_float2int(TINT16)`
/// 落地,`data_float2int` 对 TINT16 用 scale=1000(转/秒²→raw),即
/// `acc_raw = acc_turns * 1000 = (acc_rad_s2 / 2π) * 1000`。
///
/// 注:`libelybot_can.c` 的 `motor_control_pos_vel_acc` 注释写"单位 0.01 转/秒²"
/// (scale 100),与 `convert.c` 实际代码路径(scale 1000)不一致;此处跟随实际
/// 代码路径,待真机校准后确认。
pub(crate) fn radps2_to_acc_raw(radps2: f32) -> i16 {
    to_i16_raw(radps2, AngleUnit::Rad, ACC_SCALE)
}

// ----- MIT 模式(电机固件 v4.6.0+)-----
//
// 参考 `motor_control_pos_vel_tqe_kp_kd`(`libelybot_can.c`):帧 ID = `0x18000 | id`
// (扩展 29 位),8 字节位打包:pos(16 bit)、vel(12)、tqe(12)、kp(12)、kd(12)。
// 量程来自 `motor_set_pos_vel_tqe_kp_kd`(`motor_control.c`):pos/vel 以"圈"为单位
// (经 conv_to_turns),tqe 以 Nm 为单位(型号自适应见 P2-15,此处先不修正)。

pub const MIT_ID_PREFIX: u32 = 0x18000;

pub const MIT_POS_MIN: f32 = -3.2768;
pub const MIT_POS_MAX: f32 = 3.2767;
pub const MIT_VEL_MIN: f32 = -2.0;
pub const MIT_VEL_MAX: f32 = 2.0;
pub const MIT_TQE_MIN: f32 = -10.0;
pub const MIT_TQE_MAX: f32 = 10.0;
pub const MIT_KP_MIN: f32 = -400.0;
pub const MIT_KP_MAX: f32 = 400.0;
pub const MIT_KD_MIN: f32 = -100.0;
pub const MIT_KD_MAX: f32 = 100.0;

/// 镜像参考固件 `mit_float2int`:`raw = (x - x_min) * ((1<<bits) / span)`,
/// 并饱和到 `[0, (1<<bits)-1]`(否则 12 位字段取满量程时会得到 1<<12=4096,
/// 高位溢出后位打包错位)。
pub(crate) fn mit_float2int(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let scale = ((1u32 << bits) as f32) / span;
    let raw = (x - x_min) * scale;
    let max = ((1u32 << bits) - 1) as f32;
    raw.clamp(0.0, max) as u16
}

/// 编码 MIT 帧。输入:pos(rad)、vel(rad/s)、tqe(Nm)、kp、kd 无量纲。
pub(crate) fn encode_mit_frame(
    pos_rad: f32,
    vel_radps: f32,
    tqe_nm: f32,
    kp: f32,
    kd: f32,
) -> [u8; 8] {
    let pos_turns = pos_rad / TWO_PI;
    let vel_turns = vel_radps / TWO_PI;
    let pos_u = mit_float2int(pos_turns, MIT_POS_MIN, MIT_POS_MAX, 16);
    let vel_u = mit_float2int(vel_turns, MIT_VEL_MIN, MIT_VEL_MAX, 12);
    let tqe_u = mit_float2int(tqe_nm, MIT_TQE_MIN, MIT_TQE_MAX, 12);
    let kp_u = mit_float2int(kp, MIT_KP_MIN, MIT_KP_MAX, 12);
    let kd_u = mit_float2int(kd, MIT_KD_MIN, MIT_KD_MAX, 12);
    let b0 = (pos_u & 0xff) as u8;
    let b1 = ((pos_u >> 8) & 0xff) as u8;
    let b2 = (vel_u & 0xff) as u8;
    let vel_hi = ((vel_u >> 8) & 0x0f) as u8;
    let tqe_lo = (tqe_u & 0x0f) as u8;
    let b3 = vel_hi | (tqe_lo << 4);
    let b4 = ((tqe_u >> 4) & 0xff) as u8;
    let b5 = (kp_u & 0xff) as u8;
    let kp_hi = ((kp_u >> 8) & 0x0f) as u8;
    let kd_lo = (kd_u & 0x0f) as u8;
    let b6 = kp_hi | (kd_lo << 4);
    let b7 = ((kd_u >> 4) & 0xff) as u8;
    [b0, b1, b2, b3, b4, b5, b6, b7]
}

// ----- 通用寄存器读取(协议 §1.3)-----
//
// 读命令字节(cmd)= `[7:4]=0001 读 / 0010 回复`, `[3:2]` 类型(00=int8,
// 01=int16, 10=int32, 11=float), `[1:0]` 数量(01=1, 10=2, 11=3)。
// 多对 `(cmd, addr)` 可拼接,总长 ≤ 8 字节。回复镜像(cmd 高半字节=0010),
// 每对形如 `(reply_cmd, start_addr, value0, value1, ...)`,地址连续递增。

/// 寄存器数据类型(对应协议 cmd 的 `[3:2]` 两位)。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterType {
    Int8 = 0b00,
    Int16 = 0b01,
    Int32 = 0b10,
    Float = 0b11,
}

impl RegisterType {
    /// 从 cmd 字节的 `[3:2]` 两位反解类型。
    pub(crate) fn from_cmd_bits(bits: u8) -> Option<Self> {
        match bits & 0b11 {
            0 => Some(Self::Int8),
            1 => Some(Self::Int16),
            2 => Some(Self::Int32),
            3 => Some(Self::Float),
            _ => None,
        }
    }

    pub(crate) fn cmd_bits(self) -> u8 {
        self as u8
    }

    pub(crate) fn size(self) -> usize {
        match self {
            Self::Int8 => 1,
            Self::Int16 => 2,
            Self::Int32 | Self::Float => 4,
        }
    }
}

/// 读到的寄存器原始值。
#[derive(Debug, Clone, Copy)]
pub enum RegisterValue {
    I8(i8),
    I16(i16),
    I32(i32),
    F32(f32),
}

impl RegisterValue {
    /// 以 f32 形式取值(用于位置/速度/力矩等的后续标度换算)。
    pub fn as_f32(self) -> f32 {
        match self {
            Self::I8(v) => v as f32,
            Self::I16(v) => v as f32,
            Self::I32(v) => v as f32,
            Self::F32(v) => v,
        }
    }
}

/// 一条读请求:从 `addr` 起连续读 `count` 个 `ty` 类型数据(count ∈ 1..=3)。
#[derive(Debug, Clone, Copy)]
pub struct ReadCmd {
    pub addr: u8,
    pub ty: RegisterType,
    pub count: u8,
}

impl ReadCmd {
    fn cmd_byte(self) -> Option<u8> {
        let count = self.count;
        if !(1..=3).contains(&count) {
            return None;
        }
        // count 1/2/3 恰好等于 2 位编码 01/10/11
        Some(0b0001_0000 | (self.ty.cmd_bits() << 2) | (count & 0b11))
    }
}

const REPLY_NIBBLE: u8 = 0b0010_0000;

/// 将多对读请求拼成一帧 CAN 数据(≤ 8 字节)。超过 8 字节返回 `None`。
pub(crate) fn encode_read(cmds: &[ReadCmd]) -> Option<Vec<u8>> {
    let mut out = Vec::with_capacity(8);
    for cmd in cmds {
        let cb = cmd.cmd_byte()?;
        out.push(cb);
        out.push(cmd.addr);
    }
    if out.len() > 8 {
        return None;
    }
    Some(out)
}

/// 解析寄存器读回复。返回 `(addr, value)` 列表,地址按 count 连续递增。
///
/// 非回复帧(首字节高半字节 != 0010)或长度不足时返回 `None`。
pub(crate) fn decode_register_reply(data: &[u8]) -> Option<Vec<(u8, RegisterValue)>> {
    let mut out = Vec::new();
    let mut pos = 0usize;
    while pos < data.len() {
        let cmd = data[pos];
        if cmd & 0xF0 != REPLY_NIBBLE {
            return None;
        }
        if pos + 1 >= data.len() {
            return None;
        }
        let addr = data[pos + 1];
        let ty = RegisterType::from_cmd_bits((cmd >> 2) & 0b11)?;
        let count = (cmd & 0b11) as usize;
        if count == 0 {
            return None;
        }
        pos += 2;
        let size = ty.size();
        for i in 0..count {
            let start = pos + i * size;
            if start + size > data.len() {
                return None;
            }
            let buf = &data[start..start + size];
            let value = match ty {
                RegisterType::Int8 => RegisterValue::I8(i8::from_le_bytes([buf[0]])),
                RegisterType::Int16 => RegisterValue::I16(i16::from_le_bytes([buf[0], buf[1]])),
                RegisterType::Int32 => {
                    RegisterValue::I32(i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]))
                }
                RegisterType::Float => {
                    RegisterValue::F32(f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]))
                }
            };
            out.push(((addr.wrapping_add(i as u8)), value));
        }
        pos += count * size;
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reply_frame(arbitration_id: u32, is_extended: bool) -> CanFrame {
        CanFrame {
            arbitration_id,
            data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
            dlc: 8,
            is_extended,
            is_rx: true,
        }
    }

    fn frame_with(arbitration_id: u32, data: [u8; 8], dlc: u8, is_extended: bool) -> CanFrame {
        CanFrame {
            arbitration_id,
            data,
            dlc,
            is_extended,
            is_rx: true,
        }
    }

    /// 测试用通用系数(k=0.5, d=0)。
    const GEN_COEFF: TorqueCoeff = TorqueCoeff { k: 0.5, d: 0.0 };

    // The bug: extended-frame replies put the motor id in the HIGH byte
    // (e.g. id=1 → 0x0100), but the old code read the LOW byte (`& 0x7F` → 0),
    // so only motor_id 0 ever matched.
    #[test]
    fn reply_motor_id_extended_frame_uses_high_byte() {
        assert_eq!(reply_motor_id(&reply_frame(0x0100, true)), Some(1));
    }

    #[test]
    fn reply_motor_id_standard_frame_uses_high_byte() {
        assert_eq!(reply_motor_id(&reply_frame(0x0100, false)), Some(1));
    }

    #[test]
    fn reply_motor_id_larger_id_extended() {
        // id=16 → reply ID 0x1000 (extended, since >0x7FF)
        assert_eq!(reply_motor_id(&reply_frame(0x1000, true)), Some(16));
    }

    #[test]
    fn reply_motor_id_rejects_nonzero_dest() {
        // dest != 0 → addressed to another host, not ours
        assert_eq!(reply_motor_id(&reply_frame(0x0101, true)), None);
    }

    #[test]
    fn decode_read_reply_extended_frame_decodes_correct_id() {
        let frame = reply_frame(0x0100, true);
        let state = decode_read_reply(frame, GEN_COEFF).expect("extended reply should decode");
        assert_eq!(state.can_id, 1);
    }

    // ----- 通用寄存器读取 -----

    #[test]
    fn encode_read_single_int16_count3_matches_0x17_0x01() {
        let cmds = [ReadCmd {
            addr: 0x01,
            ty: RegisterType::Int16,
            count: 3,
        }];
        assert_eq!(encode_read(&cmds), Some(vec![0x17, 0x01]));
    }

    #[test]
    fn encode_read_rejects_invalid_count() {
        let cmds = [ReadCmd {
            addr: 0x01,
            ty: RegisterType::Int16,
            count: 0,
        }];
        assert_eq!(encode_read(&cmds), None);
    }

    #[test]
    fn encode_read_rejects_over_8_bytes() {
        // 5 pairs = 10 bytes > 8
        let cmds = [
            ReadCmd {
                addr: 0x01,
                ty: RegisterType::Int8,
                count: 1,
            },
            ReadCmd {
                addr: 0x02,
                ty: RegisterType::Int8,
                count: 1,
            },
            ReadCmd {
                addr: 0x03,
                ty: RegisterType::Int8,
                count: 1,
            },
            ReadCmd {
                addr: 0x04,
                ty: RegisterType::Int8,
                count: 1,
            },
            ReadCmd {
                addr: 0x05,
                ty: RegisterType::Int8,
                count: 1,
            },
        ];
        assert_eq!(encode_read(&cmds), None);
    }

    #[test]
    fn decode_register_reply_pos_vel_tqe() {
        // 0x27 0x01, pos=0x1234, vel=0x0009, tqe=0x0000 (spec 示例)
        let data = [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00];
        let vals = decode_register_reply(&data).expect("reply decodes");
        assert_eq!(vals.len(), 3);
        assert_eq!(vals[0].0, 0x01);
        assert!(matches!(vals[0].1, RegisterValue::I16(0x1234)));
        assert_eq!(vals[1].0, 0x02);
        assert!(matches!(vals[1].1, RegisterValue::I16(0x0009)));
        assert_eq!(vals[2].0, 0x03);
        assert!(matches!(vals[2].1, RegisterValue::I16(0x0000)));
    }

    #[test]
    fn decode_register_reply_float_single() {
        // cmd = 0001(read)|11(float)|01(count1) = 0x1D; reply = 0x2D
        let val = 1.5f32.to_le_bytes();
        let mut data = vec![0x2D, 0x0F];
        data.extend_from_slice(&val);
        let vals = decode_register_reply(&data).expect("reply decodes");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0F);
        assert!(matches!(vals[0].1, RegisterValue::F32(v) if (v - 1.5).abs() < 1e-6));
    }

    #[test]
    fn decode_register_reply_rejects_status_frame() {
        // 状态反馈帧首字节=mode(高半字节 0/1),不是回复(0010)
        let data = [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00];
        assert!(decode_register_reply(&data).is_none());
    }

    #[test]
    fn decode_register_reply_rejects_non_reply_first_byte() {
        let data = [0x17, 0x01, 0, 0, 0, 0, 0, 0]; // 0x17 高半字节=0001,非回复
        assert!(decode_register_reply(&data).is_none());
    }

    #[test]
    fn decode_register_reply_multi_pair() {
        // 两对:int16 count1 @0x01, int16 count1 @0x0F
        // pair1: 0x25 0x01 (reply int16 count1), data 0x34 0x12
        // pair2: 0x25 0x0F, data 0x09 0x00
        let data = [0x25, 0x01, 0x34, 0x12, 0x25, 0x0F, 0x09, 0x00];
        let vals = decode_register_reply(&data).expect("reply decodes");
        assert_eq!(vals.len(), 2);
        assert_eq!(vals[0].0, 0x01);
        assert_eq!(vals[1].0, 0x0F);
    }

    // ----- 状态反馈帧(P1-4) -----

    #[test]
    fn decode_status_frame_extracts_mode_and_fault() {
        // mode=0x0A(位置模式), fault=0, pos=0x1234, vel=0x0009, tqe=0x0000
        let frame = frame_with(
            0x0100,
            [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            8,
            true,
        );
        let state = decode_status_frame(&frame, GEN_COEFF).expect("status frame decodes");
        assert_eq!(state.can_id, 1);
        assert_eq!(state.status_code, 0x0A);
        assert_eq!(state.fault_code, 0x00);
        assert_eq!(state.pos, 0x1234i16 as f32 * 0.0001 * TWO_PI);
        assert_eq!(state.vel, 9.0 * 0.00025 * TWO_PI);
        assert_eq!(state.torq, 0.0);
    }

    #[test]
    fn decode_status_frame_carries_fault_code() {
        // mode=0x02(ready), fault=0x22(34=过压)
        let frame = frame_with(0x0100, [0x02, 0x22, 0, 0, 0, 0, 0, 0], 8, true);
        let state = decode_status_frame(&frame, GEN_COEFF).expect("status frame decodes");
        assert_eq!(state.status_code, 0x02);
        assert_eq!(state.fault_code, 0x22);
    }

    #[test]
    fn decode_status_frame_rejects_register_reply_and_short() {
        // 0x27 寄存器回复不应被当状态帧
        let reg = frame_with(0x0100, [0x27, 0x01, 0, 0, 0, 0, 0, 0], 8, true);
        assert!(decode_status_frame(&reg, GEN_COEFF).is_none());
        // 短帧
        let short = frame_with(0x0100, [0x0A, 0x00, 0, 0, 0, 0, 0, 0], 5, true);
        assert!(decode_status_frame(&short, GEN_COEFF).is_none());
    }

    #[test]
    fn decode_feedback_dispatches_both_paths() {
        // 寄存器读回复走 read_reply 分支(mode=0, fault=0)
        let reg = frame_with(
            0x0100,
            [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            8,
            true,
        );
        let s = decode_feedback(reg, GEN_COEFF).expect("feedback decodes");
        assert_eq!(s.status_code, 0);
        assert_eq!(s.fault_code, 0);

        // 状态反馈帧走 status 分支
        let st = frame_with(
            0x0100,
            [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            8,
            true,
        );
        let s = decode_feedback(st, GEN_COEFF).expect("feedback decodes");
        assert_eq!(s.status_code, 0x0A);
    }

    #[test]
    fn decode_feedback_rejects_ack_and_version() {
        // 设置 ACK(7 字节)与版本回复(5 字节)都不属于反馈
        let ack = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            7,
            true,
        );
        assert!(decode_feedback(ack, GEN_COEFF).is_none());
        let ver = frame_with(0x0100, [0x00, 0xB5, 0x02, 0x12, 0x34, 0, 0, 0], 5, true);
        assert!(decode_feedback(ver, GEN_COEFF).is_none());
    }

    // ----- MIT 模式位打包(P1-2)-----
    //
    // 验证 `encode_mit_frame` 的位布局(pos@byte0-1、vel@byte2+byte3高半位、
    // tqe@byte3低半位+byte4、kp@byte5+byte6高半位、kd@byte6低半位+byte7):
    // 逐字段置最大值、其余置最小值,只应翻转该字段对应的位。

    #[test]
    fn mit_float2int_midpoint_is_half_scale() {
        // 对称量程,中点 0 应映射到半量程。
        assert_eq!(mit_float2int(0.0, -10.0, 10.0, 12), 2048);
        assert_eq!(mit_float2int(0.0, -2.0, 2.0, 12), 2048);
        assert_eq!(mit_float2int(0.0, -400.0, 400.0, 12), 2048);
    }

    #[test]
    fn encode_mit_frame_all_min_is_all_zero() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        assert_eq!(d, [0, 0, 0, 0, 0, 0, 0, 0]);
    }

    #[test]
    fn encode_mit_frame_all_max_is_all_ff() {
        let d = encode_mit_frame(
            MIT_POS_MAX * TWO_PI,
            MIT_VEL_MAX * TWO_PI,
            MIT_TQE_MAX,
            MIT_KP_MAX,
            MIT_KD_MAX,
        );
        assert_eq!(d, [0xFF; 8]);
    }

    #[test]
    fn encode_mit_frame_pos_occupies_bytes_0_1() {
        let d = encode_mit_frame(
            MIT_POS_MAX * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        assert_eq!(d[0], 0xFF);
        assert_eq!(d[1], 0xFF);
        assert_eq!(d[2], 0x00);
        assert_eq!(d[3], 0x00);
        assert_eq!(d[4], 0x00);
        assert_eq!(d[5], 0x00);
        assert_eq!(d[6], 0x00);
        assert_eq!(d[7], 0x00);
    }

    #[test]
    fn encode_mit_frame_vel_occupies_byte2_and_high_nibble_of_byte3() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MAX * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        // byte2 = vel 低 8 位(0xFF),byte3 高半位 = vel 高 4 位(0x0F),
        // byte3 低半位 = tqe 低 4 位(0)→ byte3 = 0x0F。
        assert_eq!(d[0], 0x00);
        assert_eq!(d[1], 0x00);
        assert_eq!(d[2], 0xFF);
        assert_eq!(d[3], 0x0F);
        assert_eq!(d[4], 0x00);
    }

    #[test]
    fn encode_mit_frame_tqe_occupies_low_nibble_of_byte3_and_byte4() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MAX,
            MIT_KP_MIN,
            MIT_KD_MIN,
        );
        // byte3 低半位 = tqe 低 4 位(0x0F)<<4 → 0xF0,byte4 = tqe 高 8 位(0xFF)。
        assert_eq!(d[2], 0x00);
        assert_eq!(d[3], 0xF0);
        assert_eq!(d[4], 0xFF);
        assert_eq!(d[5], 0x00);
    }

    #[test]
    fn encode_mit_frame_kp_occupies_byte5_and_high_nibble_of_byte6() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MAX,
            MIT_KD_MIN,
        );
        assert_eq!(d[5], 0xFF);
        assert_eq!(d[6], 0x0F);
        assert_eq!(d[7], 0x00);
    }

    #[test]
    fn encode_mit_frame_kd_occupies_low_nibble_of_byte6_and_byte7() {
        let d = encode_mit_frame(
            MIT_POS_MIN * TWO_PI,
            MIT_VEL_MIN * TWO_PI,
            MIT_TQE_MIN,
            MIT_KP_MIN,
            MIT_KD_MAX,
        );
        assert_eq!(d[5], 0x00);
        assert_eq!(d[6], 0xF0);
        assert_eq!(d[7], 0xFF);
    }

    // ----- 梯形模式加速度标度(P1-3)-----

    #[test]
    fn radps2_to_acc_raw_one_rev_per_s2_is_1000() {
        // 1 圈/s² = 2π rad/s² → acc_turns=1 → raw=1000(scale=1000)。
        let raw = radps2_to_acc_raw(TWO_PI);
        assert_eq!(raw, 1000);
    }

    #[test]
    fn radps2_to_acc_raw_saturates() {
        // 远超 i16 量程 → 饱和到固件 data_limit 的 ±32760(非 i16::MAX,
        // 预留 0x8000 作 NAN_INT16 "无限制" 哨兵,对照 convert.c)。
        let raw = radps2_to_acc_raw(1.0e9);
        assert_eq!(raw, 32760);
        let raw = radps2_to_acc_raw(-1.0e9);
        assert_eq!(raw, -32760);
    }

    // ----- P2-16 单位与数据类型 -----

    #[test]
    fn quantity_scales_match_convert_c() {
        // pos_float2int(in, type) = data_float2int(in, type, 100, 10000, 100000)
        assert_eq!(
            POS_SCALE,
            QuantityScale {
                int8: 100.0,
                int16: 10000.0,
                int32: 100000.0
            }
        );
        // vel: (100, 4000, 100000)
        assert_eq!(
            VEL_SCALE,
            QuantityScale {
                int8: 100.0,
                int16: 4000.0,
                int32: 100000.0
            }
        );
        // tqe: (2, 100, 1000)
        assert_eq!(
            TQE_SCALE,
            QuantityScale {
                int8: 2.0,
                int16: 100.0,
                int32: 1000.0
            }
        );
        // acc: (20, 1000, 100000)
        assert_eq!(
            ACC_SCALE,
            QuantityScale {
                int8: 20.0,
                int16: 1000.0,
                int32: 100000.0
            }
        );
        // cur: (1, 10, 1000)
        assert_eq!(
            CUR_SCALE,
            QuantityScale {
                int8: 1.0,
                int16: 10.0,
                int32: 1000.0
            }
        );
        // vol: (2, 10, 1000)
        assert_eq!(
            VOL_SCALE,
            QuantityScale {
                int8: 2.0,
                int16: 10.0,
                int32: 1000.0
            }
        );
        // pid: (1, 10, 1000)
        assert_eq!(
            PID_SCALE,
            QuantityScale {
                int8: 1.0,
                int16: 10.0,
                int32: 1000.0
            }
        );
    }

    #[test]
    fn to_turns_from_turns_are_inverse() {
        for unit in [AngleUnit::Rad, AngleUnit::Degree, AngleUnit::Turn] {
            for v in [0.0f32, 1.0, 0.5, -2.0, 3.5] {
                let turns = to_turns(v, unit);
                let back = from_turns(turns, unit);
                assert!((back - v).abs() < 1e-5, "unit {unit:?} v={v} back={back}");
            }
        }
        // 各单位 1 圈等价:Rad=2π、Degree=360、Turn=1
        assert!((to_turns(TWO_PI, AngleUnit::Rad) - 1.0).abs() < 1e-5);
        assert!((to_turns(360.0, AngleUnit::Degree) - 1.0).abs() < 1e-5);
        assert_eq!(to_turns(1.0, AngleUnit::Turn), 1.0);
    }

    /// 各 (AngleUnit, DataType) 组合的 encode→decode 往返,以 pos 为例。
    #[test]
    fn encode_decode_unit_round_trip_all_combos() {
        for unit in [AngleUnit::Rad, AngleUnit::Degree, AngleUnit::Turn] {
            for ty in [
                DataType::Int8,
                DataType::Int16,
                DataType::Int32,
                DataType::Float,
            ] {
                // 取一个在所有类型量程内的值:0.3 圈
                let v_in_unit = match unit {
                    AngleUnit::Rad => 0.3 * TWO_PI,
                    AngleUnit::Degree => 0.3 * 360.0,
                    AngleUnit::Turn => 0.3,
                };
                let raw = encode_unit_raw(v_in_unit, unit, ty, POS_SCALE);
                let back = decode_unit_raw(raw, unit, ty, POS_SCALE);
                // 量化误差:Float 无损,整型 ≤ 1/scale
                let tol = match ty {
                    DataType::Float => 1e-5,
                    DataType::Int8 => 1.0 / POS_SCALE.int8,
                    DataType::Int16 => 1.0 / POS_SCALE.int16,
                    DataType::Int32 => 1.0 / POS_SCALE.int32,
                };
                assert!(
                    (back - v_in_unit).abs() < tol,
                    "unit {unit:?} ty {ty:?} raw={raw} back={back} tol {tol}"
                );
            }
        }
    }

    #[test]
    fn encode_unit_raw_float_is_identity() {
        // Float 类型不乘 scale、不饱和,原样返回
        let raw = encode_unit_raw(1.5, AngleUnit::Rad, DataType::Float, POS_SCALE);
        assert_eq!(raw, 1.5);
        let back = decode_unit_raw(1.5, AngleUnit::Rad, DataType::Float, POS_SCALE);
        assert_eq!(back, 1.5);
    }

    #[test]
    fn encode_unit_raw_int16_matches_legacy_rad_to_pos_raw() {
        // P2-16 泛化路径应与原 Rad+Int16 固定常量路径一致。
        for rad in [0.0f32, 0.5 * TWO_PI, 1.0 * TWO_PI, -0.25 * TWO_PI] {
            let via_unit =
                encode_unit_raw(rad, AngleUnit::Rad, DataType::Int16, POS_SCALE).round() as i32;
            let via_legacy = rad_to_pos_raw(rad) as i32;
            assert_eq!(via_unit, via_legacy, "rad={rad}");
        }
    }

    #[test]
    fn encode_unit_raw_saturation_limits() {
        // int8: ±127
        assert_eq!(
            encode_unit_raw(1.0e9, AngleUnit::Turn, DataType::Int8, POS_SCALE),
            127.0
        );
        assert_eq!(
            encode_unit_raw(-1.0e9, AngleUnit::Turn, DataType::Int8, POS_SCALE),
            -127.0
        );
        // int16: ±32760
        assert_eq!(
            encode_unit_raw(1.0e9, AngleUnit::Turn, DataType::Int16, POS_SCALE),
            32760.0
        );
        assert_eq!(
            encode_unit_raw(-1.0e9, AngleUnit::Turn, DataType::Int16, POS_SCALE),
            -32760.0
        );
        // int32: ±2147483640
        assert_eq!(
            encode_unit_raw(1.0e15, AngleUnit::Turn, DataType::Int32, POS_SCALE),
            2_147_483_640.0
        );
        assert_eq!(
            encode_unit_raw(-1.0e15, AngleUnit::Turn, DataType::Int32, POS_SCALE),
            -2_147_483_640.0
        );
        // float: 不饱和
        assert_eq!(
            encode_unit_raw(1.0e15, AngleUnit::Turn, DataType::Float, POS_SCALE),
            1.0e15
        );
    }

    /// 单位换算正确性:1 圈经 Degree+Int16 编码,raw 应 = 1 * 360/360 * 10000 = 10000。
    #[test]
    fn encode_unit_raw_degree_one_rev_is_10000_int16() {
        let raw = encode_unit_raw(360.0, AngleUnit::Degree, DataType::Int16, POS_SCALE);
        assert_eq!(raw, 10000.0);
    }

    /// 圈数制 + Int16:1.0 圈 → 1 * 10000 = 10000(对照固件默认 TURNS)。
    #[test]
    fn encode_unit_raw_turn_one_rev_is_10000_int16() {
        let raw = encode_unit_raw(1.0, AngleUnit::Turn, DataType::Int16, POS_SCALE);
        assert_eq!(raw, 10000.0);
        // decode 往返
        assert!(
            (decode_unit_raw(10000.0, AngleUnit::Turn, DataType::Int16, POS_SCALE) - 1.0).abs()
                < 1e-6
        );
    }

    // ----- 设置 ACK(P1-6)-----

    #[test]
    fn is_setting_ack_matches_exact_frame() {
        let f = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            7,
            true,
        );
        assert!(is_setting_ack(&f));
    }

    #[test]
    fn is_setting_ack_rejects_wrong_dlc_or_bytes() {
        // dlc != 7
        let long = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            8,
            true,
        );
        assert!(!is_setting_ack(&long));
        // 字节错乱(0x4E 代替 0x4F)
        let bad = frame_with(
            0x0100,
            [0x41, 0x01, 0x04, 0x4E, 0x4B, 0x0D, 0x0A, 0],
            7,
            true,
        );
        assert!(!is_setting_ack(&bad));
        // 状态帧
        let status = frame_with(0x0100, [0x0A, 0x00, 0, 0, 0, 0, 0, 0], 8, true);
        assert!(!is_setting_ack(&status));
    }

    // ----- 固件版本回复(P1-5)-----
    //
    // 参考 `motor.c::motor_process_state` 版本分支的半位布局。

    #[test]
    fn decode_version_reply_extracts_major_minor_patch() {
        // data[3]=0x12 → patch=2, minor_hi=1;data[4]=0x34 → major=3, minor_lo=4
        // → major=3, minor=4|1=5, patch=2
        let f = frame_with(0x0100, [0x15, 0xB5, 0x02, 0x12, 0x34, 0, 0, 0], 5, true);
        let v = decode_version_reply(&f).expect("version decodes");
        assert_eq!(v.as_tuple(), (3, 5, 2));
    }

    #[test]
    fn decode_version_reply_zero_version() {
        let f = frame_with(0x0100, [0x00, 0xB5, 0x02, 0x00, 0x00, 0, 0, 0], 5, true);
        let v = decode_version_reply(&f).expect("version decodes");
        assert_eq!(v.as_tuple(), (0, 0, 0));
    }

    #[test]
    fn decode_version_reply_max_nibbles() {
        // data[3]=0xFF → patch=0xF, minor_hi=0xF;data[4]=0xFF → major=0xF, minor_lo=0xF
        // → major=15, minor=15|15=15, patch=15
        let f = frame_with(0x0100, [0x00, 0xB5, 0x02, 0xFF, 0xFF, 0, 0, 0], 5, true);
        let v = decode_version_reply(&f).expect("version decodes");
        assert_eq!(v.as_tuple(), (15, 15, 15));
    }

    #[test]
    fn decode_version_reply_rejects_wrong_signature_and_length() {
        // 签名错(data[2]!=0x02)
        let bad_sig = frame_with(0x0100, [0x00, 0xB5, 0x03, 0x12, 0x34, 0, 0, 0], 5, true);
        assert!(decode_version_reply(&bad_sig).is_none());
        // 长度错(8 字节,即使前 5 字节匹配也不接收,避免与状态帧混淆)
        let bad_len = frame_with(
            0x0100,
            [0x00, 0xB5, 0x02, 0x12, 0x34, 0x00, 0x00, 0x00],
            8,
            true,
        );
        assert!(decode_version_reply(&bad_len).is_none());
    }

    #[test]
    fn run_mode_from_code_covers_0_through_17() {
        for code in 0u8..=17 {
            assert!(
                RunMode::from_code(code).is_some(),
                "code {code} should map to a RunMode"
            );
        }
        // 18 及以上不在表2 范围
        assert_eq!(RunMode::from_code(18), None);
        assert_eq!(RunMode::from_code(255), None);
    }

    #[test]
    fn run_mode_from_code_matches_repr() {
        assert_eq!(RunMode::from_code(0), Some(RunMode::Stop));
        assert_eq!(RunMode::from_code(1), Some(RunMode::Error));
        assert_eq!(RunMode::from_code(5), Some(RunMode::Pwm));
        assert_eq!(RunMode::from_code(10), Some(RunMode::Position));
        assert_eq!(RunMode::from_code(15), Some(RunMode::Brake));
        assert_eq!(RunMode::from_code(17), Some(RunMode::Reserved17));
    }

    #[test]
    fn decode_fault_known_codes() {
        assert_eq!(decode_fault(0), "正常");
        // 1-7 统一归类为 UART/DMA 错误
        for c in 1u8..=7 {
            assert_eq!(decode_fault(c), "UART/DMA 错误", "code {c}");
        }
        assert_eq!(decode_fault(32), "校准错误");
        assert_eq!(decode_fault(33), "驱动故障");
        assert_eq!(decode_fault(34), "过压");
        assert_eq!(decode_fault(35), "编码器错误");
        assert_eq!(decode_fault(36), "未校准");
        assert_eq!(decode_fault(37), "PWM 周期错误");
        assert_eq!(decode_fault(38), "过温");
        assert_eq!(decode_fault(39), "起始位置超范围");
        assert_eq!(decode_fault(40), "欠压");
        assert_eq!(decode_fault(41), "配置已变更");
        assert_eq!(decode_fault(42), "无效角度");
        assert_eq!(decode_fault(43), "无效位置");
        assert_eq!(decode_fault(44), "驱动使能故障");
        assert_eq!(decode_fault(45), "停止位置误用");
        assert_eq!(decode_fault(46), "时序错误");
        assert_eq!(decode_fault(47), "反电动势前馈错误");
    }

    #[test]
    fn decode_fault_unknown_codes() {
        // 8-31 与 >47 为规范未定义
        assert_eq!(decode_fault(8), "未知故障码");
        assert_eq!(decode_fault(31), "未知故障码");
        assert_eq!(decode_fault(48), "未知故障码");
        assert_eq!(decode_fault(255), "未知故障码");
    }

    #[test]
    fn feedback_state_run_mode_and_fault_description() {
        let mut s = HightorqueFeedbackState {
            can_id: 1,
            arbitration_id: 0x0100,
            status_code: 10,
            fault_code: 34,
            pos: 0.0,
            vel: 0.0,
            torq: 0.0,
            t_mos: 0.0,
            t_rotor: 0.0,
        };
        assert_eq!(s.run_mode(), Some(RunMode::Position));
        assert_eq!(s.fault_description(), "过压");
        // 未知码
        s.status_code = 99;
        s.fault_code = 200;
        assert_eq!(s.run_mode(), None);
        assert_eq!(s.fault_description(), "未知故障码");
    }

    // ----- P2-15 力矩型号自适应 -----

    /// 逐型号系数对照参考固件 `convert.c::motor_tqe_adj[]`。所有 d=0。
    #[test]
    fn motor_model_coeff_matches_firmware_table() {
        assert_eq!(MotorModel::Null.coeff(), TorqueCoeff { k: 0.0, d: 0.0 });
        assert_eq!(
            MotorModel::M3536_32.coeff(),
            TorqueCoeff {
                k: 0.458_105,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M4438_30.coeff(),
            TorqueCoeff {
                k: 0.525_600,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M4438_32.coeff(),
            TorqueCoeff {
                k: 0.485_565,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M4538_19.coeff(),
            TorqueCoeff {
                k: 0.493_835,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M5043_20.coeff(),
            TorqueCoeff { k: 0.966, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M5046_20.coeff(),
            TorqueCoeff {
                k: 0.533_654,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M5047_09.coeff(),
            TorqueCoeff {
                k: 0.547_474,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M5047_36.coeff(),
            TorqueCoeff { k: 0.35, d: 0.0 }
        );
        // HTDW-5036-02-CNE: Kt=0.54 Nm/A(电机端), 减速比 36。
        assert_eq!(
            MotorModel::M5036_36.coeff(),
            TorqueCoeff { k: 0.54, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M6056_36.coeff(),
            TorqueCoeff { k: 0.677, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M7256_35.coeff(),
            TorqueCoeff {
                k: 0.676_524,
                d: 0.0
            }
        );
        assert_eq!(
            MotorModel::M60SG_35.coeff(),
            TorqueCoeff { k: 0.794_2, d: 0.0 }
        );
        assert_eq!(
            MotorModel::M60BM_35.coeff(),
            TorqueCoeff { k: 0.794_2, d: 0.0 }
        );
        assert_eq!(MotorModel::General.coeff(), TorqueCoeff { k: 0.5, d: 0.0 });
    }

    #[test]
    fn from_hint_accepts_placeholders_as_general() {
        for s in ["", "hightorque", "ht", "auto", "default", "general"] {
            assert_eq!(
                MotorModel::from_hint(s),
                Some(MotorModel::General),
                "placeholder {s:?} → General"
            );
        }
    }

    #[test]
    fn from_hint_accepts_specific_model_codes() {
        assert_eq!(MotorModel::from_hint("5046-20"), Some(MotorModel::M5046_20));
        assert_eq!(MotorModel::from_hint("5046_20"), Some(MotorModel::M5046_20));
        assert_eq!(
            MotorModel::from_hint("M5046_20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(
            MotorModel::from_hint("ht-5046-20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(
            MotorModel::from_hint("hightorque-5046-20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(
            MotorModel::from_hint("m5046_20"),
            Some(MotorModel::M5046_20)
        );
        // HTDW-5036-02: 裸型号与带减速比变体均解析为 M5036_36
        assert_eq!(MotorModel::from_hint("5036"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("5036_36"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("M5036_36"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("ht-5036"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("ht-5036-36"), Some(MotorModel::M5036_36));
        // 商业代号 -02 (HTDW-5036-02) 亦映射到同一变体(-02 非减速比)
        assert_eq!(MotorModel::from_hint("5036-02"), Some(MotorModel::M5036_36));
        assert_eq!(MotorModel::from_hint("5036_02"), Some(MotorModel::M5036_36));
        // 大小写不敏感
        assert_eq!(
            MotorModel::from_hint("M5046_20"),
            Some(MotorModel::M5046_20)
        );
        assert_eq!(MotorModel::from_hint("5046-20"), Some(MotorModel::M5046_20));
    }

    #[test]
    fn from_hint_rejects_unknown_model_codes() {
        assert_eq!(MotorModel::from_hint("9999_99"), None);
        assert_eq!(MotorModel::from_hint("garbage"), None);
    }

    #[test]
    fn from_hint_accepts_null_and_none() {
        assert_eq!(MotorModel::from_hint("null"), Some(MotorModel::Null));
        assert_eq!(MotorModel::from_hint("none"), Some(MotorModel::Null));
    }

    /// `tqe_adjust_to_raw` / `tqe_restore_raw` 在各型号系数下应近似互逆。
    /// 跳过 Null(k=0,恒返回 0)。
    #[test]
    fn tqe_adjust_restore_round_trip_per_model() {
        let models = [
            MotorModel::M3536_32,
            MotorModel::M4438_30,
            MotorModel::M4438_32,
            MotorModel::M4538_19,
            MotorModel::M5043_20,
            MotorModel::M5046_20,
            MotorModel::M5047_09,
            MotorModel::M5047_36,
            MotorModel::M5036_36,
            MotorModel::M6056_36,
            MotorModel::M7256_35,
            MotorModel::M60SG_35,
            MotorModel::M60BM_35,
            MotorModel::General,
        ];
        for m in models {
            let c = m.coeff();
            for tau in [0.0f32, 0.5, 1.0, 2.0, -1.0, 5.0] {
                let raw = tqe_adjust_to_raw(tau, c);
                let restored = tqe_restore_raw(raw, c);
                // TINT16 scale=100 的量化误差 ≤ k * 0.01
                let tol = (c.k * 0.01).max(1e-6);
                assert!(
                    (restored - tau).abs() < tol,
                    "model {m:?} k={} tau={tau} raw={raw} restored={restored} (tol {tol})",
                    c.k
                );
            }
        }
    }

    /// Null 型号(k=0)写力矩恒为 0,避免除零。
    #[test]
    fn tqe_adjust_to_raw_null_model_returns_zero() {
        let c = MotorModel::Null.coeff();
        assert_eq!(tqe_adjust_to_raw(5.0, c), 0);
        assert_eq!(tqe_restore_raw(0, c), 0.0);
    }

    /// General(k=0.5):1.0 Nm → ((1.0/0.5)*100) = 200;raw 200 → 200*0.01*0.5 = 1.0。
    #[test]
    fn tqe_adjust_to_raw_general_k_half() {
        let c = MotorModel::General.coeff();
        assert_eq!(c.k, 0.5);
        assert_eq!(tqe_adjust_to_raw(1.0, c), 200);
        assert_eq!(tqe_restore_raw(200, c), 1.0);
    }

    /// 量化误差:General k=0.5 下,0.005 Nm 不足一个 raw 步进(0.01*0.5=0.005)。
    #[test]
    fn tqe_adjust_to_raw_quantizes_to_steps_of_k_times_001() {
        let c = MotorModel::General.coeff();
        // 0.007 Nm → 0.007/0.5=0.014 → *100=1.4 → round=1 → raw=1 → restore=0.005
        let raw = tqe_adjust_to_raw(0.007, c);
        let restored = tqe_restore_raw(raw, c);
        assert_eq!(raw, 1);
        assert!((restored - 0.005).abs() < 1e-6, "restored={restored}");
    }
}
