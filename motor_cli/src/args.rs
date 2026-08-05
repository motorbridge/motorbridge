use std::collections::HashMap;

pub fn parse_args() -> HashMap<String, String> {
    let mut out = HashMap::new();
    let mut it = std::env::args().skip(1).peekable();
    while let Some(k) = it.next() {
        if k == "-h" || k == "--help" || k == "help" {
            out.insert("help".to_string(), "1".to_string());
            continue;
        }
        if k == "-v" || k == "--version" {
            out.insert("version".to_string(), "1".to_string());
            continue;
        }
        // Ignore common cargo-only flag if user accidentally passes it to binary.
        if k == "--release" {
            continue;
        }
        if !k.starts_with("--") {
            // Accept the Python CLI style `motor_cli scan --vendor ...` as a
            // shorthand for the Rust CLI's historical `--mode scan` form.
            if !out.contains_key("mode") && is_mode_word(&k) {
                out.insert("mode".to_string(), k);
            }
            continue;
        }
        let key = k.trim_start_matches("--").to_string();
        match it.peek() {
            Some(v) if !v.starts_with("--") => {
                if let Some(val) = it.next() {
                    out.insert(key, val);
                }
            }
            _ => {
                out.insert(key, "1".to_string());
            }
        }
    }
    out
}

fn is_mode_word(s: &str) -> bool {
    matches!(
        s,
        "scan"
            | "ping"
            | "enable"
            | "disable"
            | "stop"
            | "status"
            | "current"
            | "vel"
            | "pos"
            | "version"
            | "mode-query"
            | "read"
            | "mit"
            | "pos-vel"
            | "pos-vel-pp"
            | "pos-vel-csp"
            | "force-pos"
            | "zero"
            | "set-zero"
            | "save"
            | "zero-by-offset"
            | "read-param"
            | "write-param"
            | "tqe"
            | "volt"
            | "cur"
            | "pos-vel-tqe"
            | "brake"
            | "rezero"
            | "conf-write"
            | "timed-read"
            | "clear-error"
            | "clear-fault"
            | "quick-stop"
            | "emergency-stop"
            | "watchdog"
            | "set-protocol"
            | "protocol-switch"
            | "fault"
            | "set-mode"
            | "set-can-id"
            | "set-host-id"
            | "active-report"
            | "get-protocol"
            | "query-protocol"
            | "protocol"
            | "get-mode"
            | "query-mode"
    )
}

pub fn get_str(args: &HashMap<String, String>, key: &str, default: &str) -> String {
    args.get(key)
        .cloned()
        .unwrap_or_else(|| default.to_string())
}

pub fn get_f32(args: &HashMap<String, String>, key: &str, default: f32) -> Result<f32, String> {
    match args.get(key) {
        Some(v) => v
            .parse::<f32>()
            .map_err(|e| format!("invalid --{key}: {e}")),
        None => Ok(default),
    }
}

pub fn get_i16(args: &HashMap<String, String>, key: &str, default: i16) -> Result<i16, String> {
    match args.get(key) {
        Some(v) => v
            .parse::<i16>()
            .map_err(|e| format!("invalid --{key}: {e}")),
        None => Ok(default),
    }
}

pub fn get_u64(args: &HashMap<String, String>, key: &str, default: u64) -> Result<u64, String> {
    match args.get(key) {
        Some(v) => v
            .parse::<u64>()
            .map_err(|e| format!("invalid --{key}: {e}")),
        None => Ok(default),
    }
}

pub fn parse_u16_hex_or_dec(s: &str, key: &str) -> Result<u16, String> {
    if let Some(hex) = s.strip_prefix("0x") {
        u16::from_str_radix(hex, 16).map_err(|e| format!("invalid --{key}: {e}"))
    } else {
        s.parse::<u16>()
            .map_err(|e| format!("invalid --{key}: {e}"))
    }
}

pub fn get_u16_hex_or_dec(
    args: &HashMap<String, String>,
    key: &str,
    default: u16,
) -> Result<u16, String> {
    match args.get(key) {
        Some(v) => parse_u16_hex_or_dec(v, key),
        None => Ok(default),
    }
}

pub fn get_opt_u16_hex_or_dec(
    args: &HashMap<String, String>,
    key: &str,
) -> Result<Option<u16>, String> {
    match args.get(key) {
        Some(v) => Ok(Some(parse_u16_hex_or_dec(v, key)?)),
        None => Ok(None),
    }
}

pub fn get_u16_list_hex_or_dec(
    args: &HashMap<String, String>,
    key: &str,
    default: &[u16],
) -> Result<Vec<u16>, String> {
    let Some(raw) = args.get(key) else {
        return Ok(default.to_vec());
    };
    let mut out = Vec::new();
    for part in raw.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        let value = parse_u16_hex_or_dec(part, key)?;
        if !out.contains(&value) {
            out.push(value);
        }
    }
    if out.is_empty() {
        return Err(format!("invalid --{key}: expected comma-separated id list"));
    }
    Ok(out)
}

pub fn print_help() {
    println!(
        "motor_cli\n\
Usage:\n\
  motor_cli -h | --help\n\
  motor_cli -v | --version\n\
  motor_cli scan --vendor robstride --start-id 1 --end-id 127\n\
  motor_cli --vendor damiao --mode scan --start-id 1 --end-id 16\n\
  motor_cli --vendor robstride --mode ping --motor-id 127 --feedback-id 0xFF\n\n\
Behavior:\n\
  no arguments: print this help (no motor command is sent)\n\n\
Mode shorthand:\n\
  A bare mode word (for example `scan`) is accepted as shorthand for `--mode scan`.\n\n\
CLI form:\n\
  motor_cli --vendor damiao --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 \\\n\
    --mode mit --pos 0 --vel 0 --kp 2 --kd 1 --tau 0 --loop 200 --dt-ms 20\n\n\
Vendors:\n\
  --vendor damiao    default\n\
  --vendor robstride\n\
  --vendor hightorque (native ht_can v1.5.5 direct-CAN mode)\n\
  --vendor hexfellow (CANopen over dedicated CAN-FD path)\n\
  --vendor robstride_cia402 (RobStride CANopen/CiA402 over classic CAN; experimental/incomplete)\n\
  --vendor robstride_mit (RobStride F_CMD=2 MIT protocol over classic CAN standard frames; experimental/incomplete)\n\
  --vendor myactuator\n\
  --vendor all       scan all vendors\n\n\
Damiao modes:\n\
  --mode scan | enable | disable | mit | pos-vel | vel | force-pos | read-param | write-param\n\n\
RobStride modes:\n\
  --mode ping | scan | enable | disable | stop | clear-error | active-report | zero | set-zero | save | zero-by-offset | get-protocol | get-mode | set-protocol | mit | pos-vel | pos-vel-pp | pos-vel-csp | vel | read-param | write-param\n\n\
RobStride CiA402 modes:\n\
  --mode scan | status | enable | disable | quick-stop | clear-error | zero | watchdog | get-protocol | set-protocol | pos-vel | vel | mit | torque\n\n\
RobStride MIT protocol modes:\n\
  --mode scan | status | enable | disable | clear-error | zero | set-mode | set-can-id | set-host-id | get-protocol | set-protocol | save | active-report | mit | pos-vel | vel | read-param | write-param\n\n\
HighTorque modes:\n\
  --mode ping | scan | read | mit | pos | vel | tqe | volt | cur | pos-vel-tqe | stop | brake | rezero | conf-write | timed-read\n\n\
MyActuator modes:\n\
  --mode scan | enable | disable | stop | set-zero | status | current | vel | pos | version | mode-query\n\n\
\n\
Hexfellow modes:\n\
  --mode scan | status | enable | disable | pos-vel | mit\n\n\
\n\
Common args:\n\
  --transport   auto|socketcan|socketcanfd|dm-serial|dm-device (default auto; dm-serial/dm-device are Damiao-only)\n\
  --channel      default can0\n\
  --serial-port  default /dev/ttyACM0 (used when --transport dm-serial)\n\
  --serial-baud  default 921600 (used when --transport dm-serial)\n\
  --dm-device-type  usb2canfd|usb2canfd-dual|linkx4c, default usb2canfd-dual (used when --transport dm-device)\n\
  --dm-channel      SDK channel number: usb2canfd=0, usb2canfd-dual=0|1, linkx4c=0|1|2|3 (control default 0; scan omitted scans all)\n\
  --model        default depends on vendor (damiao=4340, robstride=rs-00, robstride_cia402=rs-00, robstride_mit=rs-00, hightorque=hightorque[hint only], myactuator=X8)\n\
  --motor-id     default 0x01\n\
  --feedback-id  default 0x11 for Damiao, 0xFD for RobStride/RobStride MIT host_id, ignored/0 for RobStride CiA402, 0x01 for HighTorque, 0x241 for MyActuator\n\
  --loop         send cycles, default 1\n\
  --dt-ms        period ms, default 20\n\
  --ensure-mode  1/0, default 1\n\n\
ID change support by vendor:\n\
  Damiao: supported (`--set-motor-id` and optional `--set-feedback-id`)\n\
  RobStride: supported (`--set-motor-id`)\n\
  HighTorque / Hexfellow / MyActuator: not supported in unified CLI path\n\n\
Damiao extras:\n\
  --verify-model 1/0, default 1\n\
  --verify-timeout-ms  default 500\n\
  --verify-tol   default 0.2\n\
  --set-motor-id <id> --set-feedback-id <id> --store 1/0 --verify-id 1/0\n\n\
RobStride extras:\n\
  --param-id <hex|dec>      for read-param / write-param\n\
  --param-value <number>    for write-param\n\
  --type u32|f32            for Damiao read-param / write-param (default f32)\n\
  --loc-kp <float>          for pos-vel native position-loop gain; --kp is accepted as fallback\n\
  --acc <rad/s^2>           for pos-vel-pp acc_set(0x7025), default 10\n\
  --feedback-ids <list>     for scan host_id candidates, default 0xFD,0xFF,0xFE,0x00,0xAA\n\
  --timeout-ms <ms>         for scan ping timeout, default 80\n\
  --param-timeout-ms <ms>   for scan parameter fallback timeout, default 120\n\
  --active-report 1/0       for active-report mode, default 1 (RobStride comm_type=24)\n\
  --protocol private|canopen|mit|0|1|2 for set-protocol mode; power-cycle required\n\
  --wait-ack 1/0            for private set-protocol, default 1\n\
  --zero-exp 1/0            for zero/set-zero, default 0 (run experimental sequence: disable -> set-zero -> optional save)\n\
  --offset-negate 1/0       accepted for zero-by-offset, but zero-by-offset is temporarily disabled\n\
  --store 1/0               for write-param store/save, default 0; for zero-exp, default 1\n\
  --start-id <hex|dec>      for scan, default 1\n\
  --end-id <hex|dec>        for scan, default 255\n\
  Note: RobStride feedback_id/host_id (for example 0xFD/0xFE) is not motor_id/device_id.\n\
  Note: zero-by-offset currently prints a firmware-safety warning and sends no calibration CAN frames.\n\
  (scan auto-fallbacks to blind pulse probing if no ping/parameter replies)\n\
\n\
RobStride CiA402 extras:\n\
  --watchdog-s <seconds>       for watchdog mode; 0 disables\n\
  --watchdog-raw <u32>         raw watchdog value; manual says 20000 means 1s\n\
  --position-window <rad>      optional PP/CSP position window (6067)\n\
  --position-window-time-ms    optional PP/CSP position window time (6068)\n\
  --protocol private|canopen|mit|0|1|2 for set-protocol mode; power-cycle required\n\
\n\
RobStride MIT protocol extras:\n\
  --run-mode mit|position|velocity|0|1|2 for set-mode\n\
  --set-motor-id <id>       for set-can-id\n\
  --set-host-id <id>        for set-host-id\n\
  --protocol private|canopen|mit|0|1|2 for set-protocol; power-cycle required\n\
  --current/--ilim <A>      current limit for --mode vel\n\
  --param-id <hex|dec> --param-value <number> --param-type f32|u8|i8|u16|i16|u32|i32\n\
\n\
Run-mode effective arguments:\n\
  Damiao mit:        --pos --vel --kp --kd --tau\n\
  Damiao pos-vel:    --pos --vlim\n\
  Damiao vel:        --vel\n\
  Damiao force-pos:  --pos --vlim --ratio\n\
  RobStride mit:     --pos --vel --kp --kd --tau\n\
  RobStride pos-vel: --pos --vlim --loc-kp (or --kp fallback); --vel/--kd/--tau are ignored with warning\n\
  RobStride vel:     --vel\n\
  RobStride CiA402 pos-vel: --pos --vlim [--acc] [--position-window] [--position-window-time-ms]\n\
  RobStride CiA402 vel:     --vel\n\
  RobStride CiA402 torque:  --tau\n\
  RobStride MIT mit:        --pos --vel --kp --kd --tau\n\
  RobStride MIT pos-vel:    --pos --vlim\n\
  RobStride MIT vel:        --vel --current(or --ilim)\n\
  HighTorque mit:    --pos --vel --tau; --kp/--kd are ignored by ht_can v1.5.5\n\
  Hexfellow mit:     --pos --vel --kp --kd --tau\n\
  Hexfellow pos-vel: --pos --vlim\n\
\n\
MyActuator extras:\n\
  --current <A>          for --mode current\n\
  --vel <rad/s>          for --mode vel\n\
  --pos <rad>            for --mode pos\n\
  --max-speed <rad/s>    for --mode pos (default 8.726646 ~= 500 deg/s)\n\
  --start-id/--end-id    for --mode scan (range 1..32)\n\
\n\
HighTorque extras:\n\
  unified args: --pos(rad) --vel(rad/s) --tau(Nm)\n\
  alt args: --pos-deg --vel-deg-s\n\
  raw args: --raw-pos --raw-vel --raw-tqe (--mode pos/vel/tqe/mit)\n\
  --kp/--kd are accepted for unified MIT signature but ignored by ht_can v1.5.5\n\
  --loop/--dt-ms are supported for repeated send cadence\n\
\n\
All-vendor scan:\n\
  --vendor all --mode scan   run Damiao + RobStride + HighTorque + MyActuator scan in one command\n\
  optional model hints: --damiao-model ... --robstride-model ... --hightorque-model(hint only) ... --myactuator-model ...\n\
\n\
Examples:\n\
  motor_cli --vendor robstride --channel can0 --model rs-00 --motor-id 127 --mode ping\n\
\n\
  motor_cli --vendor robstride --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD \\\n\
    --mode read-param --param-id 0x7019\n\
\n\
  motor_cli --vendor robstride --channel can0 --model rs-00 --motor-id 127 --feedback-id 0xFD \\\n\
    --mode pos-vel --pos 1.5 --vlim 1.0 --loc-kp 5.0 --loop 1 --dt-ms 20\n\
\n\
  motor_cli --vendor robstride --channel can0 --model rs-00 --motor-id 2 --feedback-id 0xFD \\\n\
    --mode mit --ensure-mode 1 --pos 0.5 --vel 0 --kp 20.0 --kd 0.5 --tau 0 --loop 100 --dt-ms 20\n"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_u16_hex_or_dec_supports_both_formats() {
        assert_eq!(parse_u16_hex_or_dec("0x10", "x").expect("hex"), 16);
        assert_eq!(parse_u16_hex_or_dec("255", "x").expect("dec"), 255);
    }

    #[test]
    fn parse_u16_hex_or_dec_rejects_invalid_values() {
        assert!(parse_u16_hex_or_dec("0xZZ", "x").is_err());
        assert!(parse_u16_hex_or_dec("-1", "x").is_err());
    }

    #[test]
    fn get_u16_hex_or_dec_uses_default_when_missing() {
        let args = HashMap::new();
        assert_eq!(
            get_u16_hex_or_dec(&args, "motor-id", 0x01).expect("default"),
            0x01
        );
    }

    #[test]
    fn recognizes_positional_mode_words() {
        assert!(is_mode_word("scan"));
        assert!(is_mode_word("read-param"));
        assert!(!is_mode_word("can0"));
    }

    #[test]
    fn get_u16_list_hex_or_dec_parses_and_deduplicates() {
        let mut args = HashMap::new();
        args.insert("feedback-ids".to_string(), "0xFD, 0xFF,253".to_string());
        assert_eq!(
            get_u16_list_hex_or_dec(&args, "feedback-ids", &[0xAA]).expect("list"),
            vec![0xFD, 0xFF]
        );
    }
}
