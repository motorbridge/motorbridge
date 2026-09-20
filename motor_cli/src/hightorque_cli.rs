use crate::args::{get_f32, get_i16, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_core::CanFrame;
use std::collections::HashMap;
use std::convert::TryFrom;
use std::sync::Arc;
use std::time::{Duration, Instant};

const TWO_PI: f32 = std::f32::consts::PI * 2.0;

#[derive(Debug, Clone, Copy)]
struct HighTorqueStatus {
    motor_id: u16,
    pos_raw: i16,
    vel_raw: i16,
    tqe_raw: i16,
}

impl HighTorqueStatus {
    fn pos_turns(self) -> f32 {
        self.pos_raw as f32 * 0.0001
    }

    fn pos_rad(self) -> f32 {
        self.pos_turns() * TWO_PI
    }

    fn vel_rps(self) -> f32 {
        self.vel_raw as f32 * 0.00025
    }

    fn vel_rad_s(self) -> f32 {
        self.vel_rps() * TWO_PI
    }
}

fn can_ext_id_for_motor(motor_id: u16) -> u32 {
    u32::from(0x8000u16 | motor_id)
}

fn send_ext(
    bus: &dyn CanBus,
    motor_id: u16,
    payload: &[u8],
) -> Result<(), Box<dyn std::error::Error>> {
    if payload.len() > 8 {
        return Err("payload too long (max 8 bytes)".into());
    }
    let mut data = [0u8; 8];
    data[..payload.len()].copy_from_slice(payload);
    bus.send(CanFrame {
        arbitration_id: can_ext_id_for_motor(motor_id),
        data,
        dlc: payload.len() as u8,
        is_extended: true,
        is_rx: false,
    })?;
    Ok(())
}

fn decode_read_reply(frame: CanFrame) -> Option<HighTorqueStatus> {
    if frame.dlc < 8 {
        return None;
    }
    if frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    let motor_id = if !frame.is_extended && (frame.arbitration_id & 0x00FF) == 0 {
        ((frame.arbitration_id >> 8) & 0x7F) as u16
    } else {
        (frame.arbitration_id & 0x7FF) as u16
    };
    Some(HighTorqueStatus {
        motor_id,
        pos_raw: i16::from_le_bytes([frame.data[2], frame.data[3]]),
        vel_raw: i16::from_le_bytes([frame.data[4], frame.data[5]]),
        tqe_raw: i16::from_le_bytes([frame.data[6], frame.data[7]]),
    })
}

fn wait_status_for_motor(
    bus: &dyn CanBus,
    motor_id: u16,
    timeout: Duration,
) -> Result<Option<HighTorqueStatus>, Box<dyn std::error::Error>> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        if let Some(frame) = bus.recv(left.min(Duration::from_millis(20)))? {
            if let Some(status) = decode_read_reply(frame) {
                if status.motor_id == motor_id {
                    return Ok(Some(status));
                }
            }
        }
    }
    Ok(None)
}

fn print_status(prefix: &str, s: HighTorqueStatus) {
    println!(
        "{} id={} pos_raw={} vel_raw={} tqe_raw={} pos_rad={:+.4} vel_rad_s={:+.4} pos_turn={:+.4} vel_rps={:+.4}",
        prefix,
        s.motor_id,
        s.pos_raw,
        s.vel_raw,
        s.tqe_raw,
        s.pos_rad(),
        s.vel_rad_s(),
        s.pos_turns(),
        s.vel_rps()
    );
}

fn pos_raw_from_args(args: &HashMap<String, String>) -> Result<i16, String> {
    if args.contains_key("raw-pos") {
        return get_i16(args, "raw-pos", 0);
    }
    if args.contains_key("pos-deg") {
        let deg = get_f32(args, "pos-deg", 0.0)?;
        return Ok(round_to_i16_saturated(deg / 360.0 * 10_000.0));
    }
    if args.contains_key("pos") {
        let rad = get_f32(args, "pos", 0.0)?;
        return Ok(round_to_i16_saturated(rad / TWO_PI * 10_000.0));
    }
    Ok(0)
}

fn round_to_i16_saturated(v: f32) -> i16 {
    (v.round() as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

fn vel_raw_from_args(args: &HashMap<String, String>) -> Result<i16, String> {
    if args.contains_key("raw-vel") {
        return get_i16(args, "raw-vel", 0);
    }
    if args.contains_key("vel-deg-s") {
        let deg_s = get_f32(args, "vel-deg-s", 0.0)?;
        return Ok(round_to_i16_saturated(deg_s / 360.0 / 0.00025));
    }
    if args.contains_key("vel") {
        let rad_s = get_f32(args, "vel", 0.0)?;
        return Ok(round_to_i16_saturated(rad_s / TWO_PI / 0.00025));
    }
    Ok(0)
}

fn tqe_raw_from_args(args: &HashMap<String, String>) -> Result<i16, String> {
    if args.contains_key("raw-tqe") {
        return get_i16(args, "raw-tqe", 0);
    }
    if args.contains_key("tau") {
        let tau = get_f32(args, "tau", 0.0)?;
        return Ok(round_to_i16_saturated(tau * 100.0));
    }
    Ok(0)
}

/// Open the CAN bus for the requested transport. Routes universal transports
/// (socketcan / mcu-serial) through `open_transport` so the platform driver
/// construction lives in one place (core). HighTorque uses standard CAN only,
/// so socketcanfd is rejected (it is not a CAN-FD device here); damiao-only
/// transports are rejected. Returns `Arc` because `open_transport` returns
/// `Arc<dyn CanBus>`; the raw send/recv helpers take `&dyn CanBus`, which both
/// `Arc` and `Box` deref to identically.
fn open_hightorque_bus(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<Arc<dyn CanBus>, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcan" => open_transport(Transport::SocketCan, &p)?,
        "mcu-serial" => open_transport(Transport::McuSerial, &p)?,
        "socketcanfd" => {
            return Err(
                "transport socketcanfd unsupported (hightorque uses standard CAN only)".into(),
            )
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (hightorque supports auto|socketcan|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown HighTorque transport: {transport} (expected auto|socketcan|mcu-serial)"
            )
            .into());
        }
    };
    Ok(bus)
}

pub fn run_hightorque(
    args: &HashMap<String, String>,
    channel: &str,
    motor_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "ping");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let bus = open_hightorque_bus(&transport, channel, &serial_port, serial_baud)?;

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?.clamp(1, 127);
        let end_id = get_u16_hex_or_dec(args, "end-id", 32)?.clamp(1, 127);
        if start_id > end_id {
            return Err("invalid scan range after clamp (start-id > end-id)".into());
        }
        println!(
            "[scan] probing hightorque IDs {}..{} on {} by 0x17/0x01",
            start_id, end_id, channel
        );
        let mut hits = 0usize;
        for id in start_id..=end_id {
            send_ext(bus.as_ref(), id, &[0x17, 0x01, 0, 0, 0, 0, 0, 0])?;
            if let Some(s) = wait_status_for_motor(bus.as_ref(), id, Duration::from_millis(80))? {
                print_status("[hit]", s);
                hits += 1;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        println!("[scan] done vendor=hightorque hits={hits}");
        bus.shutdown()?;
        return Ok(());
    }

    let mut send_count = loop_n.max(1);
    if matches!(mode.as_str(), "scan" | "ping" | "read") {
        send_count = 1;
    }

    if mode == "mit" && (args.contains_key("kp") || args.contains_key("kd")) {
        let kp = get_f32(args, "kp", 0.0)?;
        let kd = get_f32(args, "kd", 0.0)?;
        println!(
            "[info] vendor=hightorque mode=mit ignores --kp/--kd in ht_can v1.5.5 (received kp={:.3}, kd={:.3})",
            kp, kd
        );
    }

    for i in 0..send_count {
        match mode.as_str() {
            "ping" | "read" => {
                send_ext(bus.as_ref(), motor_id, &[0x17, 0x01, 0, 0, 0, 0, 0, 0])?;
                if let Some(s) =
                    wait_status_for_motor(bus.as_ref(), motor_id, Duration::from_millis(500))?
                {
                    print_status("[ok]", s);
                } else {
                    return Err(format!(
                        "hightorque {} timeout on id={} (request cmd=0x17,0x01)",
                        mode, motor_id
                    )
                    .into());
                }
            }
            "pos" => {
                let pos = pos_raw_from_args(args)?;
                let tqe = tqe_raw_from_args(args)?;
                println!(
                    "[tx] mode=pos id={} pos_raw={} tqe_raw={}",
                    motor_id, pos, tqe
                );
                let mut data = [0x07, 0x07, 0x0A, 0x05, 0x00, 0x00, 0x80, 0x00];
                data[2..4].copy_from_slice(&pos.to_le_bytes());
                data[6..8].copy_from_slice(&tqe.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            "vel" => {
                let vel = vel_raw_from_args(args)?;
                let tqe = tqe_raw_from_args(args)?;
                println!(
                    "[tx] mode=vel id={} vel_raw={} tqe_raw={}",
                    motor_id, vel, tqe
                );
                let mut data = [0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
                data[4..6].copy_from_slice(&vel.to_le_bytes());
                data[6..8].copy_from_slice(&tqe.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            "tqe" => {
                let tqe = tqe_raw_from_args(args)?;
                println!("[tx] mode=tqe id={} tqe_raw={}", motor_id, tqe);
                let mut data = [0x05, 0x13, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
                data[2..4].copy_from_slice(&tqe.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data[..4])?;
            }
            "mit" => {
                let pos = pos_raw_from_args(args)?;
                let vel = vel_raw_from_args(args)?;
                let tqe = tqe_raw_from_args(args)?;
                println!(
                    "[tx] mode=mit(id: {}) -> cmd=pos-vel-tqe pos_raw={} vel_raw={} tqe_raw={}",
                    motor_id, pos, vel, tqe
                );
                let mut data = [0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                data[2..4].copy_from_slice(&vel.to_le_bytes());
                data[4..6].copy_from_slice(&tqe.to_le_bytes());
                data[6..8].copy_from_slice(&pos.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            "volt" => {
                let vol = get_i16(args, "raw-vol", 0)?;
                let mut data = [0x01, 0x00, 0x08, 0x05, 0x1B, 0x00, 0x00];
                data[5..7].copy_from_slice(&vol.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            "cur" => {
                let cur = get_i16(args, "raw-cur", 0)?;
                let mut data = [0x01, 0x00, 0x09, 0x05, 0x1C, 0x00, 0x00];
                data[5..7].copy_from_slice(&cur.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            "pos-vel-tqe" => {
                let pos = get_i16(args, "raw-pos", 0)?;
                let vel = get_i16(args, "raw-vel", 0)?;
                let tqe = get_i16(args, "raw-tqe", 0)?;
                let mut data = [0x07, 0x35, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
                data[2..4].copy_from_slice(&vel.to_le_bytes());
                data[4..6].copy_from_slice(&tqe.to_le_bytes());
                data[6..8].copy_from_slice(&pos.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            "stop" => {
                send_ext(bus.as_ref(), motor_id, &[0x01, 0x00, 0x00])?;
            }
            "brake" => {
                send_ext(bus.as_ref(), motor_id, &[0x01, 0x00, 0x0F])?;
            }
            "conf-write" => {
                send_ext(bus.as_ref(), motor_id, &[0x05, 0xB3, 0x02, 0x00, 0x00])?;
            }
            "rezero" => {
                send_ext(
                    bus.as_ref(),
                    motor_id,
                    &[0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A],
                )?;
                std::thread::sleep(Duration::from_secs(1));
                send_ext(bus.as_ref(), motor_id, &[0x05, 0xB3, 0x02, 0x00, 0x00])?;
            }
            "timed-read" => {
                let t_ms = get_i16(args, "period-ms", 100)?;
                let mut data = [0x05, 0xB4, 0x02, 0x00, 0x00];
                data[3..5].copy_from_slice(&t_ms.to_le_bytes());
                send_ext(bus.as_ref(), motor_id, &data)?;
            }
            _ => {
                return Err(format!(
                "unknown hightorque mode: {}. expected ping|scan|read|mit|pos|vel|tqe|volt|cur|pos-vel-tqe|stop|brake|rezero|conf-write|timed-read",
                mode
            )
            .into());
            }
        }
        if send_count > 1 {
            println!("[loop] #{i} sent mode={mode}");
        }
        if i + 1 < send_count {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    bus.shutdown()?;
    Ok(())
}
