use crate::model::{ServerConfig, Target, Transport, Vendor};

use super::common::parse_hex_or_dec;

pub(crate) fn parse_args() -> Result<ServerConfig, String> {
    let mut bind = "127.0.0.1:9002".to_string();
    let mut vendor = Vendor::Damiao;
    let mut transport = Transport::Auto;
    let mut channel = "can0".to_string();
    let mut serial_port = "/dev/ttyACM0".to_string();
    let mut serial_baud = 921600u32;
    let mut dm_device_type = "usb2canfd-dual".to_string();
    let mut dm_channel = "0".to_string();
    let mut model = "auto".to_string();
    let mut motor_id = 0x01u16;
    let mut feedback_id = 0x11u16;
    let mut dt_ms = 20u64;

    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut i = 0usize;
    while i < args.len() {
        let k = &args[i];
        if k == "--help" || k == "-h" {
            println!(
                "ws_gateway\n\
Usage:\n\
  ws_gateway --bind 127.0.0.1:9002\n\
  cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002\n\
  motorbridge-gateway -- --bind 127.0.0.1:9002\n\
\n\
Router mode (recommended):\n\
  Start with only --bind. The web UI/WS messages choose vendor, model, IDs, channel, and scan/control target dynamically.\n\
  This is the safest default for mixed buses and current motorbridge-studio usage.\n\
\n\
Examples:\n\
  cargo run -p ws_gateway --release -- --bind 127.0.0.1:9002\n\
\n\
  cargo run -p ws_gateway --release -- \\\n\
    --bind 127.0.0.1:9002 --vendor robstride --transport socketcan \\\n\
    --channel can0 --model rs-00 --motor-id 0x01 --feedback-id 0xFD --dt-ms 20\n\
\n\
  cargo run -p ws_gateway --release -- \\\n\
    --bind 127.0.0.1:9002 --vendor damiao --transport dm-serial \\\n\
    --serial-port /dev/ttyACM0 --serial-baud 921600 --dt-ms 20\n\
\n\
Optional defaults (only used when WS message omits target fields):\n\
  --vendor damiao|robstride|hexfellow|myactuator|hightorque|fashionstar\n\
  --transport auto|socketcan|socketcanfd|dm-serial|dm-device\n\
  --channel can0                SocketCAN/CAN-FD interface, for example can0/can1\n\
  --serial-port /dev/ttyACM0    used only by --transport dm-serial\n\
  --serial-baud 921600          used only by --transport dm-serial\n\
  --dm-device-type usb2canfd-dual  used only by --transport dm-device\n\
  --dm-channel 0                dm-device SDK channel number; scan JSON omitted dm_channel scans all adapter channels\n\
  --model auto                  default model hint when WS message omits model\n\
  --motor-id 0x01               default command/device ID when WS message omits motor_id\n\
  --feedback-id 0x11            default feedback/host ID; RobStride defaults to 0xFD if omitted\n\
  --dt-ms 20                    backend control/update period in milliseconds\n\
\n\
Transport notes:\n\
  socketcan:   classic Linux SocketCAN, usually can0/can1 after adapter setup\n\
  socketcanfd: CAN-FD path, required for Hexfellow\n\
  dm-serial:   Damiao serial bridge only; does not support RobStride/MyActuator/Hexfellow\n\
  dm-device:   DaMiao DM_Device SDK USB2CANFD/USB2CANFD_DUAL/LINKX4C transport, Damiao only\n\
\n\
Platform hints:\n\
  Linux SocketCAN: use --channel can0 after bringing the CAN interface UP.\n\
  Windows PCAN/socketcan-style channel: use forms such as --channel can0@1000000 when supported by the runtime.\n\
  dm-serial ports: Linux /dev/ttyACM0 or /dev/ttyUSB0, macOS /dev/tty.usbmodem*, Windows COM3.\n\
\n\
Security:\n\
  Non-loopback bind requires env MOTORBRIDGE_WS_TOKEN\n\
  macOS/Linux: export MOTORBRIDGE_WS_TOKEN=your-token\n\
  macOS/Linux one-shot: MOTORBRIDGE_WS_TOKEN=your-token motorbridge-gateway -- --bind 0.0.0.0:9002\n\
  PowerShell: $env:MOTORBRIDGE_WS_TOKEN=\"your-token\"\n\
  Client auth: x-motorbridge-token, Authorization: Bearer <token>, or ?motorbridge_ws_token=<token>\n\
\n\
After startup:\n\
  When you see \"ws_gateway listening on ws://127.0.0.1:9002\", connect the UI to ws://127.0.0.1:9002.\n"
            );
            std::process::exit(0);
        }
        if k == "--version" || k == "-V" {
            println!("ws_gateway {}", env!("CARGO_PKG_VERSION"));
            std::process::exit(0);
        }
        let next = args
            .get(i + 1)
            .ok_or_else(|| format!("missing value for {k}"))?;
        match k.as_str() {
            "--bind" => bind = next.clone(),
            "--vendor" => vendor = Vendor::from_str(next)?,
            "--transport" => transport = Transport::from_str(next)?,
            "--channel" => channel = next.clone(),
            "--serial-port" => serial_port = next.clone(),
            "--serial-baud" => {
                serial_baud = next
                    .parse::<u32>()
                    .map_err(|e| format!("invalid --serial-baud: {e}"))?;
            }
            "--dm-device-type" => dm_device_type = next.clone(),
            "--dm-channel" => dm_channel = next.clone(),
            "--model" => model = next.clone(),
            "--motor-id" => motor_id = parse_hex_or_dec(next)?,
            "--feedback-id" => feedback_id = parse_hex_or_dec(next)?,
            "--dt-ms" => {
                dt_ms = next
                    .parse::<u64>()
                    .map_err(|e| format!("invalid --dt-ms: {e}"))?;
            }
            _ => return Err(format!("unknown arg: {k}")),
        }
        i += 2;
    }

    if vendor == Vendor::Robstride {
        if model == "4340P" || model == "4340" {
            model = "rs-00".to_string();
        }
        if feedback_id == 0x11 {
            feedback_id = 0xFD;
        }
    } else if vendor == Vendor::Myactuator {
        if model == "4340P" || model == "4340" {
            model = "X8".to_string();
        }
        if feedback_id == 0x11 {
            feedback_id = 0x241;
        }
    } else if vendor == Vendor::Hexfellow {
        if model == "4340P" || model == "4340" {
            model = "hexfellow".to_string();
        }
        if feedback_id == 0x11 {
            feedback_id = 0x00;
        }
    } else if vendor == Vendor::Hightorque {
        if model == "4340P" || model == "4340" {
            model = "hightorque".to_string();
        }
        if feedback_id == 0x11 {
            feedback_id = 0x01;
        }
    } else if vendor == Vendor::Fashionstar {
        if model == "4340P" || model == "4340" {
            model = "fashionstar".to_string();
        }
        if feedback_id == 0x11 {
            feedback_id = motor_vendor_fashionstar::CAN_ID;
        }
    }

    Ok(ServerConfig {
        bind,
        target: Target {
            vendor,
            transport,
            channel,
            serial_port,
            serial_baud,
            dm_device_type,
            dm_channel,
            model,
            motor_id,
            feedback_id,
        },
        dt_ms,
    })
}
