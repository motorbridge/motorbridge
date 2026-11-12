use crate::commands::{as_bool, as_u64, parse_hex_or_dec};
use crate::model::Vendor;
use serde_json::Value;

pub(crate) const DAMIAO_REALTIME_OBSERVATION_PARAMS: &[u16] = &[10, 21, 22, 23, 60, 61, 62, 80, 81];

pub(crate) const DAMIAO_FULL_OBSERVATION_PARAMS: &[u16] = &[
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 50, 51, 52, 53, 54, 55, 56, 59, 60, 61, 62, 63, 64,
    65, 80, 81,
];

// RobStride stream params must stay inside the protocol section 4
// "read/write single parameter list" (0x70xx): the gateway reads them via
// type-17 single-parameter frames. The manual's 0x30xx observation block is
// host-tool serial protocol only and is rejected by real firmware with a
// non-zero read status, so it must not appear here.
pub(crate) const ROBSTRIDE_REALTIME_OBSERVATION_PARAMS: &[u16] =
    &[0x7005, 0x7019, 0x701A, 0x701B, 0x701C];

pub(crate) const ROBSTRIDE_FULL_OBSERVATION_PARAMS: &[u16] = &[
    0x7005, 0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014, 0x7016, 0x7017, 0x7018, 0x7019, 0x701A,
    0x701B, 0x701C, 0x701E, 0x701F, 0x7020, 0x7021, 0x7022, 0x7024, 0x7025, 0x7026, 0x7028, 0x7029,
    0x702A, 0x702B, 0x702C, 0x702D, 0x702E,
];

#[derive(Debug, Clone)]
pub(crate) struct ParamStream {
    pub(crate) enabled: bool,
    pub(crate) tick_div: u64,
    pub(crate) tick_counter: u64,
    pub(crate) timeout_ms: u64,
    pub(crate) params: Vec<u16>,
    pub(crate) vendor: Option<Vendor>,
}

impl Default for ParamStream {
    fn default() -> Self {
        Self {
            enabled: false,
            tick_div: 50,
            tick_counter: 0,
            timeout_ms: 80,
            params: ROBSTRIDE_REALTIME_OBSERVATION_PARAMS.to_vec(),
            vendor: None,
        }
    }
}

impl ParamStream {
    pub(crate) fn apply_message(
        &mut self,
        v: &Value,
        dt_ms: u64,
        vendor: Vendor,
    ) -> Result<(), String> {
        self.enabled = as_bool(v, "enabled", false);
        self.vendor = Some(vendor);
        self.timeout_ms = as_u64(v, "timeout_ms", self.timeout_ms).clamp(20, 1000);
        let interval_ms = as_u64(
            v,
            "interval_ms",
            self.tick_div.saturating_mul(dt_ms).max(dt_ms),
        )
        .clamp(dt_ms.max(1), 10_000);
        self.tick_div = interval_ms.div_ceil(dt_ms.max(1)).max(1);
        self.tick_counter = 0;

        if let Some(params) = parse_param_list(v)? {
            self.params = params;
        } else {
            let profile = v
                .get("profile")
                .and_then(Value::as_str)
                .unwrap_or("realtime")
                .trim()
                .to_ascii_lowercase();
            self.params = default_params_for_vendor(vendor, &profile).to_vec();
        }

        Ok(())
    }
}

pub(crate) fn default_params_for_vendor(vendor: Vendor, profile: &str) -> &'static [u16] {
    let full = matches!(profile, "full" | "all" | "observed" | "observation");
    match (vendor, full) {
        (Vendor::Damiao, true) => DAMIAO_FULL_OBSERVATION_PARAMS,
        (Vendor::Damiao, false) => DAMIAO_REALTIME_OBSERVATION_PARAMS,
        (Vendor::Robstride, true) => ROBSTRIDE_FULL_OBSERVATION_PARAMS,
        (Vendor::Robstride, false) => ROBSTRIDE_REALTIME_OBSERVATION_PARAMS,
        (Vendor::Fashionstar, _) => &[0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x17, 0x18],
        _ => &[],
    }
}

fn parse_param_list(v: &Value) -> Result<Option<Vec<u16>>, String> {
    let Some(raw) = v.get("params").or_else(|| v.get("param_ids")) else {
        return Ok(None);
    };
    let mut out = Vec::new();
    match raw {
        Value::Array(items) => {
            for item in items {
                let id = match item {
                    Value::Number(n) => n.as_u64().map(|x| x as u16),
                    Value::String(s) => Some(parse_hex_or_dec(s)?),
                    _ => None,
                }
                .ok_or_else(|| format!("invalid param id: {item}"))?;
                if !out.contains(&id) {
                    out.push(id);
                }
            }
        }
        Value::String(s) => {
            for part in s.split(',') {
                let trimmed = part.trim();
                if trimmed.is_empty() {
                    continue;
                }
                let id = parse_hex_or_dec(trimmed)?;
                if !out.contains(&id) {
                    out.push(id);
                }
            }
        }
        _ => return Err("params must be an array or comma-separated string".to_string()),
    }
    if out.is_empty() {
        return Err("params must not be empty".to_string());
    }
    Ok(Some(out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn param_stream_uses_robstride_realtime_profile_by_default() {
        let mut stream = ParamStream::default();
        stream
            .apply_message(
                &json!({"enabled": true, "interval_ms": 1000}),
                20,
                Vendor::Robstride,
            )
            .expect("valid stream config");
        assert!(stream.enabled);
        assert_eq!(stream.tick_div, 50);
        assert_eq!(stream.params, ROBSTRIDE_REALTIME_OBSERVATION_PARAMS);
    }

    #[test]
    fn param_stream_accepts_custom_hex_params() {
        let mut stream = ParamStream::default();
        stream
            .apply_message(
                &json!({"enabled": true, "params": ["0x7019", "0x701A", 28700, "0x701A"]}),
                20,
                Vendor::Robstride,
            )
            .expect("valid stream config");
        assert_eq!(stream.params, vec![0x7019, 0x701A, 0x701C]);
    }

    #[test]
    fn param_stream_supports_damiao_profile() {
        let mut stream = ParamStream::default();
        stream
            .apply_message(
                &json!({"enabled": true, "profile": "realtime"}),
                20,
                Vendor::Damiao,
            )
            .expect("valid stream config");
        assert_eq!(stream.params, DAMIAO_REALTIME_OBSERVATION_PARAMS);
    }
}
