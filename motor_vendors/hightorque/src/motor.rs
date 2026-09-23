use crate::protocol::{
    decode_feedback, decode_read_reply, decode_register_reply, decode_version_reply,
    encode_mit_frame, encode_read, is_setting_ack, rad_to_pos_raw, radps2_to_acc_raw,
    radps_to_vel_raw, reply_motor_id, tqe_adjust_to_raw, FirmwareVersion, HightorqueFeedbackState,
    MotorModel, ReadCmd, RegisterValue, TorqueCoeff, MIT_ID_PREFIX,
};
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

pub struct HightorqueMotor {
    pub motor_id: u16,
    feedback_id: u16,
    model: String,
    /// 力矩 raw↔Nm 补偿系数(由 `model` 解析,参考固件 `motor_tqe_adj[]`,见 P2-15)。
    torque_coeff: TorqueCoeff,
    bus: Arc<dyn CanBus>,
    state: Mutex<Option<HightorqueFeedbackState>>,
    /// 设置 ACK 序号:每收到一帧 ACK 由 `process_feedback_frame` 递增,
    /// `send_with_ack` 轮询该序号判定 ACK 是否到达(参照 robstride crate)。
    ack_seq: AtomicU64,
}

impl HightorqueMotor {
    pub(crate) fn new(motor_id: u16, feedback_id: u16, model: &str, bus: Arc<dyn CanBus>) -> Self {
        let torque_coeff = MotorModel::from_hint(model)
            .unwrap_or(MotorModel::General)
            .coeff();
        Self {
            motor_id,
            feedback_id,
            model: model.to_string(),
            torque_coeff,
            bus,
            state: Mutex::new(None),
            ack_seq: AtomicU64::new(0),
        }
    }

    /// 当前力矩补偿系数(供上层/测试诊断)。
    pub fn torque_coeff(&self) -> TorqueCoeff {
        self.torque_coeff
    }

    pub fn latest_state(&self) -> Option<HightorqueFeedbackState> {
        self.state.lock().ok().and_then(|g| *g)
    }

    pub fn enable(&self) -> Result<()> {
        Err(MotorError::Unsupported(
            "enable is not supported for HighTorque protocol".to_string(),
        ))
    }

    pub fn disable(&self) -> Result<()> {
        self.send_stop()
    }

    pub fn clear_error(&self) -> Result<()> {
        Err(MotorError::InvalidArgument(
            "clear_error is not supported for HighTorque".to_string(),
        ))
    }

    pub fn set_zero_position(&self) -> Result<()> {
        // 设置当前位置为零点(`0x40 0x01 0x04 0x64 0x20 0x63 0x0A`,参考固件
        // `libelybot_can.c::rezero_pos`)。该指令只在 RAM 中改位置偏置,无机
        // 械动作(固件注释"此指令只是在 RAM 中修改");fire-and-forget,不
        // 等 ACK、不 sleep,随即 store_parameters 落盘——对齐参考固件,该固
        // 件 rezero_pos/conf_write 本就是 fire-and-forget(0x41 ACK 时序不可
        // 靠,参考固件也从不主动等它)。
        self.send_query(&[0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A], 7)?;
        // RAM 偏置写入与随后 conf_write 之间留一短停顿,避免背靠背扩展帧。
        std::thread::sleep(Duration::from_millis(50));
        self.store_parameters()
    }

    pub fn ensure_control_mode(&self, mode: u32, _timeout: Duration) -> Result<()> {
        // 表2 运行模式:0-15 已定义,16/17 保留;超出范围拒绝。
        if mode <= 17 {
            Ok(())
        } else {
            Err(MotorError::InvalidArgument(format!(
                "HighTorque run mode must be 0-17 (表2), got {mode}"
            )))
        }
    }

    /// MIT 模式(电机固件 v4.6.0+):`0x18000 | id` 扩展帧,位打包
    /// pos(16)/vel(12)/tqe(12)/kp(12)/kd(12)。输出力矩 =
    /// (目标位置 - 当前位置)·kp + (目标速度 - 当前速度)·kd + 前馈力矩。
    ///
    /// 量程:pos ±3.2768 圈、vel ±2.0 圈/s、tqe ±10 Nm、kp ±400、kd ±100。
    /// 力矩的型号自适应(`tqe_adjust`)与 PID 的型号自适应(`pid_adjust`)
    /// 见 P2-15,当前未修正。
    pub fn send_cmd_mit(
        &self,
        target_position: f32,
        target_velocity: f32,
        stiffness: f32,
        damping: f32,
        feedforward_torque: f32,
    ) -> Result<()> {
        let data = encode_mit_frame(
            target_position,
            target_velocity,
            feedforward_torque,
            stiffness,
            damping,
        );
        self.send_raw(MIT_ID_PREFIX | u32::from(self.motor_id), &data, 8, true)
    }

    /// 梯形模式(电机固件 v4.6.0+):`0x07 0x2d`,`[pos@2-3, vel@4-5, acc@6-7]`
    /// int16 小端。以目标速度、目标加速度运动到目标位置。`0x8000` 表示无限制。
    pub fn send_cmd_pos_vel_acc(
        &self,
        target_position: f32,
        velocity_limit: f32,
        acceleration_limit: f32,
    ) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let vel_raw = radps_to_vel_raw(velocity_limit);
        let acc_raw = radps2_to_acc_raw(acceleration_limit);
        let mut data = [0x07, 0x2d, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&pos_raw.to_le_bytes());
        data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        data[6..8].copy_from_slice(&acc_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_pos_vel(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let vel_raw = radps_to_vel_raw(velocity_limit);
        self.send_cmd_pos_vel_tqe(pos_raw, vel_raw, i16::MIN)
    }

    /// 普通位置模式(§1.2.1 / 参考固件 `motor_control_pos`):`0x07 0x07`,
    /// `[pos@2-3, 0x0000@4-5, tqe@6-7]`,8 字节,小端。区别于协同模式 `0x07 0x35`
    /// (P1-2 路径),此模式只给目标位置 + 最大力矩,速度字段固定 0。
    ///
    /// `torque_limit_nm = None` 表示力矩无限制(`0x8000`)。帧走标准 11 位、
    /// ID = motor_id(P2-12/13)。力矩按型号自适应补偿(P2-15,`tqe_adjust`:
    /// `raw = ((tau - d) / k) * 100`),型号由 `model` 解析,未知型号走 General(k=0.5)。
    pub fn send_cmd_pos_classic(
        &self,
        target_position: f32,
        torque_limit_nm: Option<f32>,
    ) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let tqe_raw = match torque_limit_nm {
            Some(nm) => tqe_adjust_to_raw(nm, self.torque_coeff),
            None => i16::MIN, // 0x8000 = 无限制
        };
        let mut data = [0x07, 0x07, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&pos_raw.to_le_bytes());
        // bytes 4-5 固定 0x0000(参考固件 `motor_control_pos` 模板,该字段在纯位置模式下不用)
        data[6..8].copy_from_slice(&tqe_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_vel(&self, target_velocity: f32) -> Result<()> {
        let vel_raw = radps_to_vel_raw(target_velocity);
        let mut data = [0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
        data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        data[6..8].copy_from_slice(&i16::MIN.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_force_pos(
        &self,
        _target_position: f32,
        _velocity_limit: f32,
        _torque_limit_ratio: f32,
    ) -> Result<()> {
        Err(MotorError::InvalidArgument(
            "send_force_pos is not supported for HighTorque ABI; use send_mit/send_pos_vel"
                .to_string(),
        ))
    }

    pub fn store_parameters(&self) -> Result<()> {
        // 落盘到 flash(`0x05 0xB3 0x02 0x00 0x00`,参考固件 `conf_write`)。
        // fire-and-forget:参考固件本就如此,0x41 ACK 时序不可靠(见
        // set_zero_position 注释);固件建议 conf_write 后重新上电使配置生效。
        self.send_query(&[0x05, 0xB3, 0x02, 0x00, 0x00], 5)
    }

    /// 发送设置类命令并等待 ACK(7 字节 `0x41 0x01 0x04 OK\r\n`)。
    /// ACK 帧由被动反馈路径(`process_feedback_frame`)递增 `ack_seq`,此处轮询该序号。
    /// 超时未收到 ACK 时返回 `MotorError::Timeout`(不重试)。
    pub fn send_with_ack(&self, payload: &[u8], dlc: u8, timeout: Duration) -> Result<()> {
        let start_seq = self.ack_seq.load(Ordering::Acquire);
        self.send_query(payload, dlc)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.ack_seq.load(Ordering::Acquire) > start_seq {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(4));
        }
        Err(MotorError::Timeout(format!(
            "HighTorque setting ACK timeout on motor id {}",
            self.motor_id
        )))
    }

    /// 查询固件版本(`0x15 0xB5 0x02`,参考固件 `send_read_motor_version`)。
    /// 回复 5 字节,签名 `data[1]==0xB5 && data[2]==0x02`,版本号按半位解包。
    pub fn request_firmware_version(&self, timeout: Duration) -> Result<FirmwareVersion> {
        self.send_query(&[0x15, 0xB5, 0x02], 3)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let left = deadline.saturating_duration_since(Instant::now());
            if let Some(frame) = self.bus.recv(left.min(Duration::from_millis(20)))? {
                if reply_motor_id(&frame) != Some(self.motor_id as u8) {
                    continue;
                }
                if let Some(v) = decode_version_reply(&frame) {
                    return Ok(v);
                }
            }
        }
        Err(MotorError::Timeout(format!(
            "HighTorque firmware version timeout on motor id {}",
            self.motor_id
        )))
    }

    pub fn request_motor_feedback(&self, timeout: Duration) -> Result<()> {
        self.send_query(&[0x17, 0x01, 0, 0, 0, 0, 0, 0], 8)?;
        self.wait_status(timeout)
    }

    /// 通用寄存器读取(协议 §1.3):发送一组 `(cmd, addr)` 读请求并等待回复。
    ///
    /// 返回 `(addr, value)` 列表,地址按每对 count 连续递增。回复帧经
    /// `reply_motor_id` 校验电机 id 后,由 `decode_register_reply` 解析。
    pub fn read_registers(
        &self,
        cmds: &[ReadCmd],
        timeout: Duration,
    ) -> Result<Vec<(u8, RegisterValue)>> {
        let payload = encode_read(cmds).ok_or_else(|| {
            MotorError::InvalidArgument(
                "read_registers: cmd list too long or count out of range".to_string(),
            )
        })?;
        self.send_query(&payload, payload.len() as u8)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let left = deadline.saturating_duration_since(Instant::now());
            if let Some(frame) = self.bus.recv(left.min(Duration::from_millis(20)))? {
                if reply_motor_id(&frame) != Some(self.motor_id as u8) {
                    continue;
                }
                if let Some(vals) = decode_register_reply(&frame.data[..frame.dlc as usize]) {
                    return Ok(vals);
                }
            }
        }
        Err(MotorError::Timeout(format!(
            "HighTorque register read timeout on motor id {}",
            self.motor_id
        )))
    }

    fn send_stop(&self) -> Result<()> {
        self.send_control(&[0x01, 0x00, 0x00], 3)
    }

    fn send_cmd_pos_vel_tqe(&self, pos_raw: i16, vel_raw: i16, tqe_raw: i16) -> Result<()> {
        let mut data = [0x07, 0x35, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&vel_raw.to_le_bytes());
        data[4..6].copy_from_slice(&tqe_raw.to_le_bytes());
        data[6..8].copy_from_slice(&pos_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    /// 控制类命令(vel/pos_vel/pos_vel_acc/pos_classic/stop/brake):**标准 11 位帧**,
    /// ID = `motor_id`,不置回复总开关 bit15(参考固件 `motor_control_*` 均
    /// `can_send(hfdcanx, id, ...)`,见 P2-12/P2-13)。
    fn send_control(&self, payload: &[u8], dlc: u8) -> Result<()> {
        self.send_raw(u32::from(self.motor_id), payload, dlc, false)
    }

    /// 查询/配置类命令(read/rezero/store/conf_write/version):**扩展 29 位帧**,
    /// ID = `0x8000 | motor_id`,置回复总开关 bit15(参考固件这些路径均
    /// `can_send(hfdcanx, 0x8000 | id, ...)`,见 P2-12/P2-13)。
    fn send_query(&self, payload: &[u8], dlc: u8) -> Result<()> {
        self.send_raw(u32::from(0x8000u16 | self.motor_id), payload, dlc, true)
    }

    fn send_raw(
        &self,
        arbitration_id: u32,
        payload: &[u8],
        dlc: u8,
        is_extended: bool,
    ) -> Result<()> {
        let mut data = [0u8; 8];
        data[..payload.len().min(8)].copy_from_slice(&payload[..payload.len().min(8)]);
        self.bus.send(CanFrame {
            arbitration_id,
            data,
            dlc: dlc.min(8),
            is_extended,
            is_rx: false,
        })
    }

    fn wait_status(&self, timeout: Duration) -> Result<()> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let left = deadline.saturating_duration_since(Instant::now());
            if let Some(frame) = self.bus.recv(left.min(Duration::from_millis(20)))? {
                if let Some(state) = decode_read_reply(frame, self.torque_coeff) {
                    if state.can_id as u16 == self.motor_id {
                        if let Ok(mut g) = self.state.lock() {
                            *g = Some(state);
                        }
                        return Ok(());
                    }
                }
            }
        }
        Err(MotorError::Timeout(format!(
            "HighTorque read timeout on motor id {}",
            self.motor_id
        )))
    }
}

impl MotorDevice for HightorqueMotor {
    fn vendor(&self) -> &'static str {
        "hightorque"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        self.feedback_id
    }

    fn enable(&self) -> Result<()> {
        HightorqueMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        HightorqueMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if !frame.is_rx {
            return false;
        }
        if reply_motor_id(frame) != Some(self.motor_id as u8) {
            return false;
        }
        // 8 字节:寄存器读回复(0x27...)或控制命令附带的状态反馈(byte0=mode)。
        // 7 字节:设置 ACK(0x41...)。5 字节:固件版本回复(0xB5 0x02)。
        // 其余长度(如 3 字节)不归被动反馈,由活动路径的 bus.recv 直接消费。
        match frame.dlc {
            8 => true,
            7 => is_setting_ack(frame),
            5 => decode_version_reply(frame).is_some(),
            _ => false,
        }
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        if reply_motor_id(&frame) != Some(self.motor_id as u8) {
            return Ok(());
        }
        if is_setting_ack(&frame) {
            self.ack_seq.fetch_add(1, Ordering::Release);
            return Ok(());
        }
        if let Some(state) = decode_feedback(frame, self.torque_coeff) {
            if state.can_id as u16 == self.motor_id {
                self.state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".to_string()))?
                    .replace(state);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use std::sync::Arc;
    use std::time::Duration;

    fn make_motor(id: u16) -> (Arc<MockBus>, Arc<HightorqueMotor>) {
        let bus: Arc<MockBus> = Arc::new(MockBus::new());
        let motor = Arc::new(HightorqueMotor::new(id, 0, "ht-test", bus.clone()));
        (bus, motor)
    }

    fn ack_frame(id: u16) -> CanFrame {
        CanFrame {
            arbitration_id: u32::from(id) << 8,
            data: [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            dlc: 7,
            is_extended: true,
            is_rx: true,
        }
    }

    fn version_frame(id: u16, b3: u8, b4: u8) -> CanFrame {
        CanFrame {
            arbitration_id: u32::from(id) << 8,
            data: [0x15, 0xB5, 0x02, b3, b4, 0, 0, 0],
            dlc: 5,
            is_extended: true,
            is_rx: true,
        }
    }

    #[test]
    fn process_feedback_frame_bumps_ack_seq_on_setting_ack() {
        let (_bus, motor) = make_motor(1);
        let before = motor.ack_seq.load(Ordering::Acquire);
        motor.process_feedback_frame(ack_frame(1)).expect("process");
        assert_eq!(motor.ack_seq.load(Ordering::Acquire), before + 1);
    }

    #[test]
    fn process_feedback_frame_ignores_ack_for_other_motor() {
        let (_bus, motor) = make_motor(1);
        let before = motor.ack_seq.load(Ordering::Acquire);
        // id=2 的 ACK 不应被 id=1 的电机消费
        motor.process_feedback_frame(ack_frame(2)).expect("process");
        assert_eq!(motor.ack_seq.load(Ordering::Acquire), before);
    }

    #[test]
    fn send_with_ack_returns_ok_when_ack_arrives() {
        let (bus, motor) = make_motor(1);
        let m = motor.clone();
        // 模拟被动反馈路径在命令发出后递补 ACK 序号
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(15));
            m.process_feedback_frame(ack_frame(1)).expect("process");
        });
        let r = motor.send_with_ack(&[0x05, 0xB3, 0x02, 0, 0], 5, Duration::from_millis(500));
        handle.join().expect("thread join");
        assert!(r.is_ok(), "send_with_ack should succeed on ACK: {:?}", r);
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert_eq!(&sent[0].data[..5], &[0x05, 0xB3, 0x02, 0, 0]);
    }

    #[test]
    fn send_with_ack_times_out_without_ack() {
        let (_bus, motor) = make_motor(1);
        let r = motor.send_with_ack(&[0x05, 0xB3, 0x02, 0, 0], 5, Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "expected timeout, got {r:?}"
        );
    }

    #[test]
    fn request_firmware_version_decodes_reply() {
        let (bus, motor) = make_motor(1);
        // data[3]=0x12, data[4]=0x34 → major=3, minor=5, patch=2
        bus.push_rx(version_frame(1, 0x12, 0x34));
        let v = motor
            .request_firmware_version(Duration::from_millis(200))
            .expect("version");
        assert_eq!(v.as_tuple(), (3, 5, 2));
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(&sent[0].data[..3], &[0x15, 0xB5, 0x02]);
    }

    #[test]
    fn request_firmware_version_times_out_without_reply() {
        let (_bus, motor) = make_motor(1);
        let r = motor.request_firmware_version(Duration::from_millis(20));
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "expected timeout, got {r:?}"
        );
    }

    #[test]
    fn accepts_frame_admits_status_register_ack_and_version() {
        let (_bus, motor) = make_motor(1);
        // 8 字节状态帧
        let status = CanFrame {
            arbitration_id: 0x0100,
            data: [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&status));
        // 7 字节 ACK
        assert!(motor.accepts_frame(&ack_frame(1)));
        // 5 字节版本回复
        assert!(motor.accepts_frame(&version_frame(1, 0x12, 0x34)));
        // 其它电机的帧不接受
        assert!(!motor.accepts_frame(&ack_frame(2)));
        // 3 字节短帧不接受
        let short = CanFrame {
            arbitration_id: 0x0100,
            data: [0x01, 0x00, 0x00, 0, 0, 0, 0, 0],
            dlc: 3,
            is_extended: true,
            is_rx: true,
        };
        assert!(!motor.accepts_frame(&short));
    }

    #[test]
    fn ensure_control_mode_accepts_0_through_17() {
        let (_bus, motor) = make_motor(1);
        for mode in 0..=17 {
            assert!(
                motor.ensure_control_mode(mode, Duration::ZERO).is_ok(),
                "mode {mode} should be accepted"
            );
        }
    }

    #[test]
    fn ensure_control_mode_rejects_above_17() {
        let (_bus, motor) = make_motor(1);
        for mode in [18, 19, 100, u32::MAX] {
            assert!(
                motor.ensure_control_mode(mode, Duration::ZERO).is_err(),
                "mode {mode} should be rejected"
            );
        }
    }

    /// P2-12/13:控制命令走标准 11 位帧、ID=motor_id,不置回复位。
    #[test]
    fn control_commands_use_standard_frame_bare_id() {
        let (bus, motor) = make_motor(1);
        // 速度命令(0x07 0x07)
        motor.send_cmd_vel(1.0).expect("vel");
        // 协同位置-速度(0x07 0x35)
        motor.send_cmd_pos_vel(0.5, 1.0).expect("pos_vel");
        // 梯形(0x07 0x2d)
        motor
            .send_cmd_pos_vel_acc(0.5, 1.0, 0.5)
            .expect("pos_vel_acc");
        // 停止(0x01 0x00 0x00)
        motor.send_stop().expect("stop");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 4);
        for f in sent.iter() {
            assert!(!f.is_extended, "控制帧应为标准 11 位");
            assert_eq!(f.arbitration_id, 1, "控制帧 ID 应为裸 motor_id");
            assert!(!f.is_rx);
        }
        // 不应为 0x8000|id(那是查询帧)
        assert_ne!(sent[0].arbitration_id, 0x8001);
    }

    /// P2-12/13:查询/配置命令走扩展 29 位帧、ID=0x8000|motor_id,置回复位。
    #[test]
    fn query_commands_use_extended_frame_with_reply_bit() {
        let (bus, motor) = make_motor(1);
        // 寄存器读取(0x17 0x01 派生的 read 命令)
        motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x01,
                    ty: crate::protocol::RegisterType::Int16,
                    count: 3,
                }],
                Duration::from_millis(10),
            )
            .ok();
        // 固件版本查询(0x15 0xB5 0x02)
        motor
            .request_firmware_version(Duration::from_millis(10))
            .ok();
        let sent = bus.sent.lock().expect("sent");
        assert!(sent.len() >= 2);
        for f in sent.iter() {
            assert!(f.is_extended, "查询帧应为扩展 29 位");
            assert_eq!(f.arbitration_id, 0x8001, "查询帧 ID 应为 0x8000|motor_id");
            assert!(!f.is_rx);
        }
    }

    /// P2-14:普通位置模式 `0x07 0x07` 字节布局,力矩 None → 0x8000(无限制),
    /// 标准 11 位帧。对照参考固件 `motor_control_pos` 模板 `[0x07,0x07,pos@2-3,
    /// 0x00,0x00,tqe@6-7]`。
    #[test]
    fn send_cmd_pos_classic_byte_layout() {
        let (bus, motor) = make_motor(1);
        // 目标位置 0.5 圈 → raw = 0.5 * 10000 = 5000 = 0x1388
        motor
            .send_cmd_pos_classic(0.5 * std::f32::consts::TAU, None)
            .expect("pos_classic");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert!(!f.is_extended, "普通位置帧应为标准 11 位");
        assert_eq!(f.arbitration_id, 1);
        assert_eq!(&f.data[0..2], &[0x07, 0x07]);
        // pos = 5000 LE
        assert_eq!(i16::from_le_bytes([f.data[2], f.data[3]]), 5000);
        // bytes 4-5 固定 0x0000
        assert_eq!(&f.data[4..6], &[0x00, 0x00]);
        // tqe = 0x8000(无限制)
        assert_eq!(i16::from_le_bytes([f.data[6], f.data[7]]), i16::MIN);
    }

    /// P2-14 + P2-15:给定力矩时,classic pos 的 tqe 字段 = tqe_adjust_to_raw(nm, coeff)。
    /// 测试电机 model="ht-test" → General(k=0.5),故 1.0 Nm → ((1.0/0.5)*100) = 200。
    #[test]
    fn send_cmd_pos_classic_with_torque_limit() {
        let (bus, motor) = make_motor(1);
        assert_eq!(motor.torque_coeff().k, 0.5, "测试电机应为 General k=0.5");
        // 1.0 Nm → raw = 200 = 0x00C8
        motor
            .send_cmd_pos_classic(0.0, Some(1.0))
            .expect("pos_classic");
        let f = &bus.sent.lock().expect("sent")[0];
        assert_eq!(i16::from_le_bytes([f.data[6], f.data[7]]), 200);
    }
}
