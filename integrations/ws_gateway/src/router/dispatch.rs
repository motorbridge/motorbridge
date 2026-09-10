use crate::commands::{cmd_batch_scan, cmd_scan, cmd_set_id, cmd_verify, parse_vendor_in_msg};
use crate::model::{ControllerHandle, Vendor};
use crate::router::stream::ParamStream;
use crate::session::SessionCtx;
use serde_json::Value;

use super::handlers;

pub(crate) fn dispatch_op(
    op: &str,
    v: &Value,
    ctx: &mut SessionCtx,
    state_stream_enabled: &mut bool,
    param_stream: &mut ParamStream,
    dt_ms: u64,
) -> Result<serde_json::Value, String> {
    if let Some(r) =
        handlers::connection::handle(op, v, ctx, state_stream_enabled, param_stream, dt_ms)
    {
        return r;
    }
    if let Some(r) = handlers::control::handle(op, v, ctx) {
        return r;
    }
    if let Some(r) = handlers::control_aux::handle(op, v, ctx) {
        return r;
    }
    if let Some(r) = handlers::register::handle(op, v, ctx) {
        return r;
    }

    match op {
        "batch_scan" => {
            release_session_before_scan(v, ctx, state_stream_enabled, param_stream);
            cmd_batch_scan(v, &ctx.target)
        }
        "scan" => {
            release_session_before_scan(v, ctx, state_stream_enabled, param_stream);
            cmd_scan(v, &ctx.target)
        }
        "set_id" => {
            release_session_before_stateless(ctx, state_stream_enabled, param_stream);
            cmd_set_id(v, &ctx.target)
        }
        "verify" => {
            release_session_before_stateless(ctx, state_stream_enabled, param_stream);
            cmd_verify(v, &ctx.target)
        }
        _ => Err(format!("unsupported op: {op}")),
    }
}

pub(crate) fn release_session_before_scan(
    v: &Value,
    ctx: &mut SessionCtx,
    state_stream_enabled: &mut bool,
    param_stream: &mut ParamStream,
) {
    let may_scan_damiao = scan_request_may_include_vendor(v, ctx.target.vendor, Vendor::Damiao);
    let may_scan_robstride =
        scan_request_may_include_vendor(v, ctx.target.vendor, Vendor::Robstride);

    let should_release = matches!(ctx.controller, Some(ControllerHandle::Damiao(_)))
        && may_scan_damiao
        || matches!(ctx.controller, Some(ControllerHandle::Robstride(_))) && may_scan_robstride;
    if !should_release {
        return;
    }

    *state_stream_enabled = false;
    param_stream.enabled = false;
    ctx.disconnect(false);

    #[cfg(target_os = "windows")]
    {
        let gap_ms = if may_scan_damiao { 50 } else { 20 };
        std::thread::sleep(std::time::Duration::from_millis(gap_ms));
    }
}

/// `set_id`/`verify` are stateless ops that open their own bus on
/// `ctx.target.channel`. PCAN (Windows/macOS) allows only one initialized
/// handle per channel, so if the session currently holds that channel — which
/// is the common case, since `set_target` is typically followed by stream
/// enable commands that lazily reconnect via `ensure_connected` — the
/// stateless op's `CAN_Initialize` would fail with `PCAN_ERROR_INITIALIZE`.
/// Release the session bus first so the channel is free for the op to use.
/// Unlike scan, no vendor matching is needed: these ops always target
/// `ctx.target.channel`, the same channel a connected session occupies.
pub(crate) fn release_session_before_stateless(
    ctx: &mut SessionCtx,
    state_stream_enabled: &mut bool,
    param_stream: &mut ParamStream,
) {
    if ctx.controller.is_none() {
        return;
    }
    *state_stream_enabled = false;
    param_stream.enabled = false;
    ctx.disconnect(false);

    #[cfg(target_os = "windows")]
    {
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
}

fn scan_request_may_include_vendor(v: &Value, default_vendor: Vendor, vendor: Vendor) -> bool {
    let mut has_nested_scan_items = false;
    for key in ["vendors", "scans", "requests", "items"] {
        let Some(items) = v.get(key).and_then(Value::as_array) else {
            continue;
        };
        has_nested_scan_items = true;
        for item in items {
            if item
                .as_str()
                .map(|s| s.eq_ignore_ascii_case(vendor.as_str()))
                == Some(true)
            {
                return true;
            }
            if item
                .get("vendor")
                .and_then(Value::as_str)
                .map(|s| s.eq_ignore_ascii_case(vendor.as_str()))
                == Some(true)
            {
                return true;
            }
        }
    }
    if has_nested_scan_items {
        return false;
    }

    if parse_vendor_in_msg(v, default_vendor).ok() == Some(vendor) {
        return true;
    }
    false
}
