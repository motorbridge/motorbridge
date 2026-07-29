mod args;
mod cyberbeast_cli;
mod damiao_cli;
mod hexfellow_cli;
mod hightorque_cli;
mod myactuator_cli;
mod robstride_cia402_cli;
mod robstride_cli;
mod robstride_mit_cli;

use args::{get_str, get_u16_hex_or_dec, print_help};
use cyberbeast_cli::run_cyberbeast;
use damiao_cli::run_damiao;
use hexfellow_cli::run_hexfellow;
use hightorque_cli::run_hightorque;
use myactuator_cli::run_myactuator;
use robstride_cia402_cli::run_robstride_cia402;
use robstride_cli::run_robstride;
use robstride_mit_cli::run_robstride_mit;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = args::parse_args();
    if args.is_empty() || args.contains_key("help") {
        print_help();
        return Ok(());
    }
    if args.contains_key("version") {
        println!("motor_cli {}", env!("CARGO_PKG_VERSION"));
        return Ok(());
    }

    let vendor = get_str(&args, "vendor", "damiao");
    let channel = get_str(&args, "channel", "can0");
    let transport = get_str(&args, "transport", "auto");
    let dm_device_type = get_str(&args, "dm-device-type", "usb2canfd-dual");
    let default_model =
        if vendor == "robstride" || vendor == "robstride_cia402" || vendor == "robstride_mit" {
            "rs-00"
        } else if vendor == "hightorque" {
            "hightorque"
        } else if vendor == "myactuator" {
            "X8"
        } else if vendor == "hexfellow" {
            "hexfellow"
        } else if vendor == "cyberbeast" {
            "odrive-default"
        } else {
            "4340"
        };
    let model = get_str(&args, "model", default_model);
    let motor_id = get_u16_hex_or_dec(&args, "motor-id", 0x01)?;
    let feedback_default = match vendor.as_str() {
        "robstride" | "robstride_mit" => 0x00FD,
        "robstride_cia402" | "hexfellow" => 0x0000,
        "hightorque" => 0x0001,
        "myactuator" => 0x0241,
        "cyberbeast" => 0x0000,
        _ => 0x0011,
    };
    let feedback_id = get_u16_hex_or_dec(&args, "feedback-id", feedback_default)?;
    let mode = get_str(
        &args,
        "mode",
        match vendor.as_str() {
            "robstride" => "ping",
            "robstride_mit" | "robstride_cia402" | "myactuator" | "hexfellow" => "status",
            "hightorque" => "read",
            "cyberbeast" => "status",
            "all" => "scan",
            _ => "mit",
        },
    );
    let dm_channel = if mode == "scan" && transport == "dm-device" {
        args.get("dm-channel")
            .cloned()
            .unwrap_or_else(|| "all".to_string())
    } else {
        get_str(&args, "dm-channel", "0")
    };

    let model_is_default = !args.contains_key("model");
    let motor_id_is_default = !args.contains_key("motor-id");
    let feedback_id_is_default = !args.contains_key("feedback-id");
    let start_id_is_default = !args.contains_key("start-id");
    let end_id_is_default = !args.contains_key("end-id");
    let default_tag = |is_default: bool| if is_default { " (default)" } else { "" };

    if mode == "scan" {
        let scan_start = get_u16_hex_or_dec(&args, "start-id", 1)?;
        let scan_end = get_u16_hex_or_dec(&args, "end-id", 255)?;
        println!(
            "vendor={} transport={} channel={}{} mode=scan model_hint={}{} base_feedback_id=0x{:X}{} scan_range={}{}..{}{}",
            vendor,
            transport,
            channel,
            if transport == "dm-device" {
                format!(" dm_device_type={} dm_channel={}", dm_device_type, dm_channel)
            } else {
                String::new()
            },
            model,
            default_tag(model_is_default),
            feedback_id,
            default_tag(feedback_id_is_default),
            scan_start,
            default_tag(start_id_is_default),
            scan_end,
            default_tag(end_id_is_default),
        );
    } else {
        println!(
            "vendor={} transport={} channel={}{} model={}{} motor_id=0x{:X}{} feedback_id=0x{:X}{} mode={}",
            vendor,
            transport,
            channel,
            if transport == "dm-device" {
                format!(" dm_device_type={} dm_channel={}", dm_device_type, dm_channel)
            } else {
                String::new()
            },
            model,
            default_tag(model_is_default),
            motor_id,
            default_tag(motor_id_is_default),
            feedback_id,
            default_tag(feedback_id_is_default),
            mode
        );
    }

    if transport == "dm-serial" && vendor != "damiao" {
        return Err("transport=dm-serial is currently supported only for --vendor damiao".into());
    }
    if transport == "dm-device" && vendor != "damiao" {
        return Err("transport=dm-device is currently supported only for --vendor damiao".into());
    }
    if vendor == "hexfellow" && transport != "auto" && transport != "socketcanfd" {
        return Err("vendor=hexfellow supports only --transport auto|socketcanfd".into());
    }

    if vendor == "all" {
        if mode != "scan" {
            return Err("vendor=all currently supports --mode scan only".into());
        }
        let damiao_model = get_str(&args, "damiao-model", "4340P");
        let robstride_model = get_str(&args, "robstride-model", "rs-00");
        let hightorque_model = get_str(&args, "hightorque-model", "hightorque");
        let myactuator_model = get_str(&args, "myactuator-model", "X8");
        println!(
            "[scan-all] running Damiao scan with model_hint={}, RobStride scan with model_hint={}, HighTorque scan(by ht_can) with model_hint={}, then MyActuator scan with model_hint={}",
            damiao_model, robstride_model, hightorque_model, myactuator_model
        );
        run_damiao(&args, &channel, &damiao_model, motor_id, 0x0011)?;
        run_robstride(
            &args,
            &channel,
            &robstride_model,
            motor_id,
            0x00FD,
            "robstride",
        )?;
        let mut rs_cia402_args = args.clone();
        rs_cia402_args.insert("mode".to_string(), "scan".to_string());
        let _ = run_robstride_cia402(&rs_cia402_args, &channel, &robstride_model, motor_id, 0);
        let mut rs_mit_args = args.clone();
        rs_mit_args.insert("mode".to_string(), "scan".to_string());
        let _ = run_robstride_mit(&rs_mit_args, &channel, &robstride_model, motor_id, 0x00FD);
        let mut ht_args = args.clone();
        ht_args.insert("mode".to_string(), "scan".to_string());
        if !ht_args.contains_key("start-id") {
            ht_args.insert("start-id".to_string(), "1".to_string());
        }
        if !ht_args.contains_key("end-id") {
            ht_args.insert("end-id".to_string(), "32".to_string());
        }
        let _ = hightorque_model;
        run_hightorque(&ht_args, &channel, motor_id)?;
        run_myactuator(&args, &channel, &myactuator_model, motor_id, 0x0241)?;
        return Ok(());
    }

    match vendor.as_str() {
        "damiao" => run_damiao(&args, &channel, &model, motor_id, feedback_id),
        "robstride" => run_robstride(&args, &channel, &model, motor_id, feedback_id, "robstride"),
        "robstride_cia402" => run_robstride_cia402(&args, &channel, &model, motor_id, feedback_id),
        "robstride_mit" => run_robstride_mit(&args, &channel, &model, motor_id, feedback_id),
        "hightorque" => run_hightorque(&args, &channel, motor_id),
        "myactuator" => run_myactuator(&args, &channel, &model, motor_id, feedback_id),
        "hexfellow" => run_hexfellow(&args, &channel, &model, motor_id, feedback_id),
        "cyberbeast" => run_cyberbeast(&args, &channel, &model, motor_id, feedback_id),
        _ => Err(format!("unknown vendor: {vendor}").into()),
    }
}
