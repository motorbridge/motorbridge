use crate::model::{Target, Vendor};
use motor_vendor_robstride::ParameterValue as RobstrideParameterValue;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::time::Duration;

use super::{
    as_u16, as_u64, parse_hex_or_dec, parse_id_list_csv, parse_transport_in_msg,
    parse_vendor_in_msg,
};
use crate::vendors::cyberbeast_ws::cmd_scan_cyberbeast;
use crate::vendors::damiao_ws::cmd_scan_damiao;
use crate::vendors::hightorque_ws::{send_hightorque_ext, wait_hightorque_status_for_motor};
use crate::vendors::transport_ws::{
    myactuator_feedback_default, open_hexfellow_controller, open_hightorque_bus,
    open_myactuator_controller, open_robstride_controller,
};

fn emit_scan_progress<F>(emit: &mut F, data: Value)
where
    F: FnMut(Value),
{
    emit(json!({
        "type": "scan_progress",
        "op": "scan",
        "data": data,
    }));
}

fn cmd_scan_robstride_with_progress<F>(
    v: &Value,
    base: &Target,
    emit: &mut F,
) -> Result<Value, String>
where
    F: FnMut(Value),
{
    let debug = std::env::var("MOTORBRIDGE_WS_DEBUG").is_ok();
    let mut target = base.clone();
    if let Some(channel) = v.get("channel").and_then(Value::as_str) {
        target.channel = channel.to_string();
    }
    if let Some(model) = v.get("model").and_then(Value::as_str) {
        target.model = model.to_string();
    }
    if target.model == "4340" || target.model == "4340P" || target.model == "auto" {
        target.model = "rs-00".to_string();
    }
    let transport = parse_transport_in_msg(v, base.transport)?;
    let start_id = as_u16(v, "start_id", 1);
    let end_id = as_u16(v, "end_id", 255);
    let timeout_ms = as_u64(v, "timeout_ms", 120);
    let strict_timeout = v
        .get("strict_timeout")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let probe_timeout_ms = if strict_timeout {
        timeout_ms
    } else {
        timeout_ms.min(120)
    };
    let param_id = as_u16(v, "param_id", 0x7019);
    let read_param_fallback = v
        .get("read_param_fallback")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    let scan_all_feedback_ids = v
        .get("scan_all_feedback_ids")
        .and_then(Value::as_bool)
        .unwrap_or(false);
    if end_id < start_id {
        return Err("end_id must be >= start_id".to_string());
    }

    let mut feedback_ids: Vec<u16> = Vec::new();
    let mut push_unique = |id: u16| {
        if !feedback_ids.contains(&id) {
            feedback_ids.push(id);
        }
    };
    match v.get("feedback_ids") {
        Some(Value::Array(arr)) => {
            for item in arr {
                if let Some(id) = item
                    .as_u64()
                    .and_then(|n| u16::try_from(n).ok())
                    .or_else(|| item.as_str().and_then(|s| parse_hex_or_dec(s).ok()))
                {
                    push_unique(id);
                }
            }
        }
        Some(Value::String(s)) => {
            for id in parse_id_list_csv(s) {
                push_unique(id);
            }
        }
        _ => {
            // RobStride feedback_id is the host-side ID. These defaults match the CLI scan.
            for id in [0xFD, 0xFF, 0xFE, 0x00, 0xAA] {
                push_unique(id);
            }
        }
    }
    if feedback_ids.is_empty() {
        return Err("feedback_ids must not be empty".to_string());
    }
    if let Some(id) = feedback_ids.iter().find(|id| **id > 0xFF) {
        return Err(format!("feedback_id out of range: 0x{id:X}"));
    }

    emit_scan_progress(
        emit,
        json!({
            "vendor": "robstride",
            "phase": "start",
            "start_id": start_id,
            "end_id": end_id,
            "feedback_ids": feedback_ids,
            "model": target.model,
            "probe_timeout_ms": probe_timeout_ms,
        }),
    );

    let mut hits_by_mid = BTreeMap::new();
    for fid in &feedback_ids {
        let hit_count_before = hits_by_mid.len();
        if debug {
            eprintln!("[ws_gateway] robstride scan fid=0x{fid:X} open");
        }
        let ctrl = open_robstride_controller(&target, transport)?;
        let mut bound = false;
        for mid in start_id..=end_id {
            if hits_by_mid.contains_key(&mid) {
                continue;
            }
            if debug {
                eprintln!("[ws_gateway] robstride scan probe=0x{mid:X} fid=0x{fid:X}");
            }
            emit_scan_progress(
                emit,
                json!({
                    "vendor": "robstride",
                    "phase": "probe",
                    "probe": mid,
                    "feedback_id": fid,
                }),
            );
            let motor = match ctrl.add_motor(mid, *fid, &target.model) {
                Ok(m) => m,
                Err(e) => {
                    if debug {
                        eprintln!(
                            "[ws_gateway] robstride scan add_motor failed probe=0x{mid:X} fid=0x{fid:X}: {e}"
                        );
                    }
                    emit_scan_progress(
                        emit,
                        json!({
                            "vendor": "robstride",
                            "phase": "error",
                            "probe": mid,
                            "feedback_id": fid,
                            "error": e.to_string(),
                        }),
                    );
                    continue;
                }
            };
            bound = true;
            if let Ok(p) = motor.ping_with_host_id(*fid, Duration::from_millis(probe_timeout_ms)) {
                let hit = json!({
                    "probe": mid,
                    "via": "ping",
                    "feedback_id": fid,
                    "device_id": p.device_id,
                    "responder_id": p.responder_id
                });
                hits_by_mid.insert(mid, hit.clone());
                emit_scan_progress(
                    emit,
                    json!({
                        "vendor": "robstride",
                        "phase": "hit",
                        "hit": hit,
                    }),
                );
                continue;
            }
            let mut found_by_param = false;
            if read_param_fallback {
                if let Ok(RobstrideParameterValue::F32(val)) = motor.get_parameter_with_host_id(
                    param_id,
                    *fid,
                    Duration::from_millis(probe_timeout_ms),
                ) {
                    let hit = json!({
                        "probe": mid,
                        "via": "read_param",
                        "feedback_id": fid,
                        "param_id": format!("0x{param_id:04X}"),
                        "value": val
                    });
                    hits_by_mid.insert(mid, hit.clone());
                    emit_scan_progress(
                        emit,
                        json!({
                            "vendor": "robstride",
                            "phase": "hit",
                            "hit": hit,
                        }),
                    );
                    found_by_param = true;
                }
            }
            if !found_by_param {
                emit_scan_progress(
                    emit,
                    json!({
                        "vendor": "robstride",
                        "phase": "no_reply",
                        "probe": mid,
                        "feedback_id": fid,
                    }),
                );
            }
        }
        if bound {
            if debug {
                eprintln!("[ws_gateway] robstride scan fid=0x{fid:X} close_bus");
            }
            let _ = ctrl.close_bus();
            #[cfg(target_os = "windows")]
            std::thread::sleep(Duration::from_millis(20));
        }
        if !scan_all_feedback_ids && hits_by_mid.len() > hit_count_before {
            break;
        }
    }
    let hits = hits_by_mid.into_values().collect::<Vec<_>>();

    emit_scan_progress(
        emit,
        json!({
            "vendor": "robstride",
            "phase": "done",
            "count": hits.len(),
            "start_id": start_id,
            "end_id": end_id,
        }),
    );

    Ok(json!({
        "vendor": "robstride",
        "transport": transport.as_str(),
        "count": hits.len(),
        "start_id": start_id,
        "end_id": end_id,
        "feedback_ids": feedback_ids,
        "model": target.model,
        "probe_timeout_ms": probe_timeout_ms,
        "read_param_fallback": read_param_fallback,
        "scan_all_feedback_ids": scan_all_feedback_ids,
        "strict_timeout": strict_timeout,
        "hits": hits,
    }))
}

pub(crate) fn cmd_scan_robstride_progress<F>(
    v: &Value,
    base: &Target,
    emit: &mut F,
) -> Result<Value, String>
where
    F: FnMut(Value),
{
    cmd_scan_robstride_with_progress(v, base, emit)
}

fn cmd_scan_myactuator(v: &Value, base: &Target) -> Result<Value, String> {
    let transport = parse_transport_in_msg(v, base.transport)?;
    let start_id = as_u16(v, "start_id", 1);
    let end_id_in = as_u16(v, "end_id", 32);
    if start_id == 0 || end_id_in == 0 || start_id > 32 || start_id > end_id_in {
        return Err("invalid scan range: expected start in 1..32 and start<=end".to_string());
    }
    let end_id = end_id_in.min(32);
    let timeout_ms = as_u64(v, "timeout_ms", 100);
    let ctrl = open_myactuator_controller(base, transport)?;
    let mut hits = Vec::new();
    for id in start_id..=end_id {
        let fid = myactuator_feedback_default(id);
        let m = match ctrl.add_motor(id, fid, &base.model) {
            Ok(m) => m,
            Err(_) => continue,
        };
        let _ = m.request_version_date();
        if let Ok(version) = m.await_version_date(Duration::from_millis(timeout_ms)) {
            hits.push(json!({
                "probe": id,
                "motor_id": id,
                "feedback_id": fid,
                "version": version
            }));
        }
        std::thread::sleep(Duration::from_millis(3));
    }
    let _ = ctrl.close_bus();
    Ok(json!({
        "vendor": "myactuator",
        "transport": transport.as_str(),
        "count": hits.len(),
        "start_id": start_id,
        "end_id": end_id,
        "hits": hits,
    }))
}

fn cmd_scan_hexfellow(v: &Value, base: &Target) -> Result<Value, String> {
    let transport = parse_transport_in_msg(v, base.transport)?;
    let start_id = as_u16(v, "start_id", 1);
    let end_id = as_u16(v, "end_id", 32);
    let timeout_ms = as_u64(v, "timeout_ms", 200);
    let ctrl = open_hexfellow_controller(base, transport)?;
    let found = ctrl
        .scan_ids(start_id, end_id, Duration::from_millis(timeout_ms))
        .map_err(|e| e.to_string())?;
    let mut hits = Vec::new();
    for h in found {
        hits.push(json!({
            "node_id": h.node_id,
            "sw_ver": h.sw_ver,
            "peak_torque_raw": h.peak_torque_raw,
            "kp_kd_factor_raw": h.kp_kd_factor_raw,
            "dev_type": h.dev_type,
        }));
    }
    let _ = ctrl.close_bus();
    Ok(json!({
        "vendor": "hexfellow",
        "transport": transport.as_str(),
        "count": hits.len(),
        "start_id": start_id,
        "end_id": end_id,
        "hits": hits,
    }))
}

fn cmd_scan_hightorque(v: &Value, base: &Target) -> Result<Value, String> {
    let transport = parse_transport_in_msg(v, base.transport)?;
    let start_id = as_u16(v, "start_id", 1).clamp(1, 127);
    let end_id = as_u16(v, "end_id", 32).clamp(1, 127);
    if start_id > end_id {
        return Err("invalid scan range after clamp (start_id > end_id)".to_string());
    }
    let timeout_ms = as_u64(v, "timeout_ms", 80);
    let bus = open_hightorque_bus(base, transport)?;
    let mut hits = Vec::new();
    for id in start_id..=end_id {
        send_hightorque_ext(bus.as_ref(), id, &[0x17, 0x01, 0, 0, 0, 0, 0, 0])?;
        if let Some(s) =
            wait_hightorque_status_for_motor(bus.as_ref(), id, Duration::from_millis(timeout_ms))?
        {
            hits.push(json!({
                "motor_id": s.motor_id,
                "pos_raw": s.pos_raw,
                "vel_raw": s.vel_raw,
                "tqe_raw": s.tqe_raw
            }));
        }
        std::thread::sleep(Duration::from_millis(2));
    }
    let _ = bus.shutdown();
    Ok(json!({
        "vendor": "hightorque",
        "transport": transport.as_str(),
        "count": hits.len(),
        "start_id": start_id,
        "end_id": end_id,
        "hits": hits,
    }))
}

pub(crate) fn cmd_scan(v: &Value, base: &Target) -> Result<Value, String> {
    match parse_vendor_in_msg(v, base.vendor)? {
        Vendor::Damiao => cmd_scan_damiao(v, base),
        Vendor::Robstride => cmd_scan_robstride_with_progress(v, base, &mut |_| {}),
        Vendor::Hexfellow => cmd_scan_hexfellow(v, base),
        Vendor::Myactuator => cmd_scan_myactuator(v, base),
        Vendor::Hightorque => cmd_scan_hightorque(v, base),
        Vendor::CyberBeast => cmd_scan_cyberbeast(v, base),
    }
}

pub(crate) fn cmd_batch_scan(v: &Value, base: &Target) -> Result<Value, String> {
    let mut requests: Vec<Value> = Vec::new();
    if let Some(items) = v
        .get("scans")
        .or_else(|| v.get("requests"))
        .or_else(|| v.get("items"))
        .and_then(Value::as_array)
    {
        for item in items {
            let mut req = v.clone();
            if let Some(obj) = req.as_object_mut() {
                obj.remove("scans");
                obj.remove("requests");
                obj.remove("items");
                obj.insert("op".to_string(), Value::String("scan".to_string()));
                if let Some(item_obj) = item.as_object() {
                    for (key, value) in item_obj {
                        obj.insert(key.clone(), value.clone());
                    }
                } else if let Some(vendor) = item.as_str() {
                    obj.insert("vendor".to_string(), Value::String(vendor.to_string()));
                }
            }
            requests.push(req);
        }
    } else if let Some(vendors) = v.get("vendors").and_then(Value::as_array) {
        for vendor in vendors {
            let Some(vendor) = vendor.as_str() else {
                continue;
            };
            let mut req = v.clone();
            if let Some(obj) = req.as_object_mut() {
                obj.remove("vendors");
                obj.insert("op".to_string(), Value::String("scan".to_string()));
                obj.insert("vendor".to_string(), Value::String(vendor.to_string()));
                if let Some(vendor_cfg) = v.get(vendor).and_then(Value::as_object) {
                    for (key, value) in vendor_cfg {
                        obj.insert(key.clone(), value.clone());
                    }
                }
            }
            requests.push(req);
        }
    }

    if requests.is_empty() {
        return Err("batch_scan requires scans/requests/items or vendors".to_string());
    }

    let mut results = Vec::new();
    let mut all_hits = Vec::new();
    let mut ok_count = 0usize;
    for req in requests {
        let vendor = req
            .get("vendor")
            .and_then(Value::as_str)
            .unwrap_or("unknown")
            .to_string();
        match cmd_scan(&req, base) {
            Ok(data) => {
                ok_count += 1;
                if let Some(hits) = data.get("hits").and_then(Value::as_array) {
                    for hit in hits {
                        let mut hit = hit.clone();
                        if let Some(obj) = hit.as_object_mut() {
                            obj.entry("vendor".to_string())
                                .or_insert_with(|| Value::String(vendor.clone()));
                        }
                        all_hits.push(hit);
                    }
                }
                results.push(json!({
                    "ok": true,
                    "vendor": vendor,
                    "data": data,
                }));
            }
            Err(error) => {
                results.push(json!({
                    "ok": false,
                    "vendor": vendor,
                    "error": error,
                }));
            }
        }
    }

    if ok_count == 0 {
        let errors = results
            .iter()
            .filter_map(|r| {
                let vendor = r.get("vendor").and_then(Value::as_str).unwrap_or("unknown");
                let error = r.get("error").and_then(Value::as_str)?;
                Some(format!("{vendor}: {error}"))
            })
            .collect::<Vec<_>>()
            .join("; ");
        return Err(format!("batch_scan failed for all vendors: {errors}"));
    }

    Ok(json!({
        "count": all_hits.len(),
        "hits": all_hits,
        "results": results,
    }))
}
