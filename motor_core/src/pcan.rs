#![cfg(any(target_os = "windows", target_os = "macos"))]

use crate::bus::{CanBus, CanFrame};
use crate::error::{MotorError, Result};
use libloading::Library;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

type TPCANHandle = u16;
type TPCANStatus = u32;

const PCAN_USBBUS1: TPCANHandle = 0x51;

const PCAN_ERROR_OK: TPCANStatus = 0x00000;
const PCAN_ERROR_QRCVEMPTY: TPCANStatus = 0x00020;
const PCAN_ERROR_ILLHW: TPCANStatus = 0x01400;
const PCAN_ERROR_INITIALIZE: TPCANStatus = 0x4000000;

const PCAN_MESSAGE_STANDARD: u8 = 0x00;
const PCAN_MESSAGE_EXTENDED: u8 = 0x02;
const RECONNECT_MAX_ATTEMPTS: usize = 3;
const RECONNECT_BACKOFF_MS: [u64; RECONNECT_MAX_ATTEMPTS] = [20, 100, 300];

#[repr(C)]
struct TPCANMsg {
    id: u32,
    msg_type: u8,
    len: u8,
    data: [u8; 8],
}

#[repr(C)]
struct TPCANTimestamp {
    millis: u32,
    millis_overflow: u16,
    micros: u16,
}

#[cfg(target_os = "windows")]
type CanInitializeFn = unsafe extern "system" fn(TPCANHandle, u16, u8, u32, u16) -> TPCANStatus;
#[cfg(target_os = "windows")]
type CanUninitializeFn = unsafe extern "system" fn(TPCANHandle) -> TPCANStatus;
#[cfg(target_os = "windows")]
type CanReadFn =
    unsafe extern "system" fn(TPCANHandle, *mut TPCANMsg, *mut TPCANTimestamp) -> TPCANStatus;
#[cfg(target_os = "windows")]
type CanWriteFn = unsafe extern "system" fn(TPCANHandle, *const TPCANMsg) -> TPCANStatus;

#[cfg(target_os = "macos")]
type CanInitializeFn = unsafe extern "C" fn(TPCANHandle, u16, u8, u32, u16) -> TPCANStatus;
#[cfg(target_os = "macos")]
type CanUninitializeFn = unsafe extern "C" fn(TPCANHandle) -> TPCANStatus;
#[cfg(target_os = "macos")]
type CanReadFn =
    unsafe extern "C" fn(TPCANHandle, *mut TPCANMsg, *mut TPCANTimestamp) -> TPCANStatus;
#[cfg(target_os = "macos")]
type CanWriteFn = unsafe extern "C" fn(TPCANHandle, *const TPCANMsg) -> TPCANStatus;

struct PcanApi {
    _lib: Library,
    can_initialize: CanInitializeFn,
    can_uninitialize: CanUninitializeFn,
    can_read: CanReadFn,
    can_write: CanWriteFn,
}

fn library_candidates(home: Option<&Path>) -> Vec<PathBuf> {
    if cfg!(target_os = "macos") {
        let mut candidates = vec![PathBuf::from("libPCBUSB.dylib")];
        if let Some(home) = home {
            candidates.push(home.join(".local/lib/libPCBUSB.dylib"));
        }
        candidates.push(PathBuf::from("PCBUSB"));
        candidates
    } else {
        vec![PathBuf::from("PCANBasic.dll")]
    }
}

fn library_load_error(load_errors: &[String]) -> MotorError {
    let errors = load_errors.join("; ");
    if cfg!(target_os = "macos") {
        MotorError::Unsupported(format!(
            "load PCBUSB failed (tried: {errors}). Install MacCAN PCBUSB runtime (libPCBUSB.dylib)."
        ))
    } else {
        MotorError::Unsupported(format!(
            "load PCANBasic.dll failed: {errors}. Install PEAK PCAN-Basic runtime."
        ))
    }
}

impl PcanApi {
    fn load() -> Result<Self> {
        let home = std::env::var_os("HOME").map(PathBuf::from);
        let lib_candidates = library_candidates(home.as_deref());

        let mut load_errors = Vec::new();
        let mut loaded: Option<Library> = None;
        for name in &lib_candidates {
            match unsafe { Library::new(name) } {
                Ok(lib) => {
                    loaded = Some(lib);
                    break;
                }
                Err(e) => load_errors.push(format!("{}: {e}", name.display())),
            }
        }

        let lib = loaded.ok_or_else(|| library_load_error(&load_errors))?;

        let can_initialize = unsafe {
            *lib.get::<CanInitializeFn>(b"CAN_Initialize\0")
                .map_err(|e| {
                    MotorError::Unsupported(format!("load symbol CAN_Initialize failed: {e}"))
                })?
        };
        let can_uninitialize = unsafe {
            *lib.get::<CanUninitializeFn>(b"CAN_Uninitialize\0")
                .map_err(|e| {
                    MotorError::Unsupported(format!("load symbol CAN_Uninitialize failed: {e}"))
                })?
        };
        let can_read = unsafe {
            *lib.get::<CanReadFn>(b"CAN_Read\0")
                .map_err(|e| MotorError::Unsupported(format!("load symbol CAN_Read failed: {e}")))?
        };
        let can_write = unsafe {
            *lib.get::<CanWriteFn>(b"CAN_Write\0").map_err(|e| {
                MotorError::Unsupported(format!("load symbol CAN_Write failed: {e}"))
            })?
        };

        Ok(Self {
            _lib: lib,
            can_initialize,
            can_uninitialize,
            can_read,
            can_write,
        })
    }
}

fn global_api() -> Result<Arc<PcanApi>> {
    static API: OnceLock<std::result::Result<Arc<PcanApi>, String>> = OnceLock::new();
    let cached = API.get_or_init(|| PcanApi::load().map(Arc::new).map_err(|e| e.to_string()));
    match cached {
        Ok(api) => Ok(Arc::clone(api)),
        Err(msg) => Err(MotorError::Unsupported(msg.clone())),
    }
}

fn bitrate_to_btr0btr1(bitrate: u32) -> Option<u16> {
    match bitrate {
        1_000_000 => Some(0x0014),
        800_000 => Some(0x0016),
        500_000 => Some(0x001C),
        250_000 => Some(0x011C),
        125_000 => Some(0x031C),
        100_000 => Some(0x432F),
        95_238 => Some(0xC34E),
        83_333 => Some(0x852B),
        50_000 => Some(0x472F),
        47_619 => Some(0x1414),
        33_333 => Some(0x8B2F),
        20_000 => Some(0x532F),
        10_000 => Some(0x672F),
        5_000 => Some(0x7F7F),
        _ => None,
    }
}

fn parse_channel_and_bitrate(input: &str) -> Result<(TPCANHandle, u16)> {
    let trimmed = input.trim();
    let (chan_part, bitrate_part) = match trimmed.split_once('@') {
        Some((c, b)) => (c.trim(), Some(b.trim())),
        None => (trimmed, None),
    };

    let bitrate = match bitrate_part {
        Some(raw) => raw.parse::<u32>().map_err(|e| {
            MotorError::InvalidArgument(format!("invalid bitrate in channel '{input}': {e}"))
        })?,
        None => 1_000_000,
    };
    let btr = bitrate_to_btr0btr1(bitrate).ok_or_else(|| {
        MotorError::InvalidArgument(format!(
            "unsupported bitrate {bitrate}, expected one of [1000000,800000,500000,250000,125000,100000,50000,20000,10000,5000]"
        ))
    })?;

    if chan_part.eq_ignore_ascii_case("can0") {
        return Ok((PCAN_USBBUS1, btr));
    }
    if let Some(idx_str) = chan_part.strip_prefix("can") {
        let idx = idx_str.parse::<u16>().map_err(|e| {
            MotorError::InvalidArgument(format!("invalid can index in channel '{input}': {e}"))
        })?;
        let handle = PCAN_USBBUS1
            .checked_add(idx)
            .ok_or_else(|| MotorError::InvalidArgument("channel index overflow".to_string()))?;
        return Ok((handle, btr));
    }
    let chan_upper = chan_part.to_ascii_uppercase();
    if let Some(idx_str) = chan_upper.strip_prefix("PCAN_USBBUS") {
        let one_based = idx_str.parse::<u16>().map_err(|e| {
            MotorError::InvalidArgument(format!("invalid PCAN_USBBUS in channel '{input}': {e}"))
        })?;
        if one_based == 0 {
            return Err(MotorError::InvalidArgument(
                "PCAN_USBBUS index must start from 1".to_string(),
            ));
        }
        let handle = PCAN_USBBUS1
            .checked_add(one_based - 1)
            .ok_or_else(|| MotorError::InvalidArgument("channel index overflow".to_string()))?;
        return Ok((handle, btr));
    }
    if let Some(hex) = chan_part.strip_prefix("0x") {
        let handle = u16::from_str_radix(hex, 16).map_err(|e| {
            MotorError::InvalidArgument(format!("invalid hex handle in channel '{input}': {e}"))
        })?;
        return Ok((handle, btr));
    }
    if let Ok(handle) = chan_part.parse::<u16>() {
        return Ok((handle, btr));
    }

    Err(MotorError::InvalidArgument(format!(
        "unsupported channel '{input}', use can0/canN/PCAN_USBBUS1/0x51 and optional '@bitrate'"
    )))
}

fn pcan_status_to_error(prefix: &str, status: TPCANStatus) -> MotorError {
    let name = match status {
        PCAN_ERROR_ILLHW => "PCAN_ERROR_ILLHW",
        PCAN_ERROR_INITIALIZE => "PCAN_ERROR_INITIALIZE",
        _ => "PCAN_ERROR_UNKNOWN",
    };
    MotorError::Io(format!("{prefix}: PCAN status=0x{status:08X} ({name})"))
}

pub struct PcanBus {
    api: Arc<PcanApi>,
    handle: TPCANHandle,
    btr0btr1: u16,
    io_lock: Mutex<()>,
    active: Mutex<bool>,
}

impl PcanBus {
    pub fn open(channel: &str) -> Result<Self> {
        let api = global_api()?;
        let (handle, btr0btr1) = parse_channel_and_bitrate(channel)?;

        let status = unsafe { (api.can_initialize)(handle, btr0btr1, 0, 0, 0) };
        if status != PCAN_ERROR_OK {
            return Err(pcan_status_to_error("PCAN initialize failed", status));
        }

        Ok(Self {
            api,
            handle,
            btr0btr1,
            io_lock: Mutex::new(()),
            active: Mutex::new(true),
        })
    }

    fn reconnect_locked(&self, active: &mut bool) -> Result<()> {
        for attempt in 0..RECONNECT_MAX_ATTEMPTS {
            let _ = unsafe { (self.api.can_uninitialize)(self.handle) };
            let status = unsafe { (self.api.can_initialize)(self.handle, self.btr0btr1, 0, 0, 0) };
            if status == PCAN_ERROR_OK {
                *active = true;
                return Ok(());
            }
            thread::sleep(Duration::from_millis(RECONNECT_BACKOFF_MS[attempt]));
        }
        Err(MotorError::Io(format!(
            "pcan reconnect failed after {RECONNECT_MAX_ATTEMPTS} attempts"
        )))
    }
}

impl CanBus for PcanBus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        if frame.dlc > 8 {
            return Err(MotorError::InvalidArgument(format!(
                "invalid DLC {}, expected <= 8",
                frame.dlc
            )));
        }
        let msg = TPCANMsg {
            id: frame.arbitration_id,
            msg_type: if frame.is_extended {
                PCAN_MESSAGE_EXTENDED
            } else {
                PCAN_MESSAGE_STANDARD
            },
            len: frame.dlc,
            data: frame.data,
        };
        let _io = self
            .io_lock
            .lock()
            .map_err(|_| MotorError::Io("pcan io lock poisoned".to_string()))?;
        let mut active = self
            .active
            .lock()
            .map_err(|_| MotorError::Io("pcan active lock poisoned".to_string()))?;
        if !*active {
            return Err(MotorError::Io("pcan bus is already closed".to_string()));
        }

        for _ in 0..=RECONNECT_MAX_ATTEMPTS {
            let status = unsafe { (self.api.can_write)(self.handle, &msg) };
            if status == PCAN_ERROR_OK {
                return Ok(());
            }
            self.reconnect_locked(&mut active)?;
        }
        Err(MotorError::Io(
            "pcan write failed after reconnect retries".to_string(),
        ))
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>> {
        let deadline = Instant::now()
            .checked_add(timeout)
            .unwrap_or_else(|| Instant::now() + Duration::from_secs(3600));

        loop {
            let _io = self
                .io_lock
                .lock()
                .map_err(|_| MotorError::Io("pcan io lock poisoned".to_string()))?;
            let mut active = self
                .active
                .lock()
                .map_err(|_| MotorError::Io("pcan active lock poisoned".to_string()))?;
            if !*active {
                return Err(MotorError::Io("pcan bus is already closed".to_string()));
            }

            let mut msg = TPCANMsg {
                id: 0,
                msg_type: 0,
                len: 0,
                data: [0; 8],
            };
            let mut ts = TPCANTimestamp {
                millis: 0,
                millis_overflow: 0,
                micros: 0,
            };
            let status = unsafe { (self.api.can_read)(self.handle, &mut msg, &mut ts) };

            if status == PCAN_ERROR_OK {
                return Ok(Some(CanFrame {
                    arbitration_id: msg.id,
                    data: msg.data,
                    dlc: msg.len.min(8),
                    is_extended: (msg.msg_type & PCAN_MESSAGE_EXTENDED) != 0,
                    is_rx: true,
                }));
            }
            if status != PCAN_ERROR_QRCVEMPTY {
                if let Err(re_err) = self.reconnect_locked(&mut active) {
                    return Err(MotorError::Io(format!(
                        "pcan read failed ({}) and reconnect failed ({re_err})",
                        pcan_status_to_error("pcan read failed", status)
                    )));
                }
                continue;
            }
            drop(active);
            drop(_io);

            if timeout.is_zero() || Instant::now() >= deadline {
                return Ok(None);
            }
            thread::sleep(Duration::from_millis(1));
        }
    }

    fn shutdown(&self) -> Result<()> {
        let _io = self
            .io_lock
            .lock()
            .map_err(|_| MotorError::Io("pcan io lock poisoned".to_string()))?;
        let mut active = self
            .active
            .lock()
            .map_err(|_| MotorError::Io("pcan active lock poisoned".to_string()))?;
        if !*active {
            return Ok(());
        }

        let status = unsafe { (self.api.can_uninitialize)(self.handle) };
        if status != PCAN_ERROR_OK {
            return Err(pcan_status_to_error("pcan uninitialize failed", status));
        }
        *active = false;
        Ok(())
    }
}

impl Drop for PcanBus {
    fn drop(&mut self) {
        if let Ok(_io) = self.io_lock.lock() {
            if let Ok(mut active) = self.active.lock() {
                if *active {
                    let _ = unsafe { (self.api.can_uninitialize)(self.handle) };
                    *active = false;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn library_candidates_include_user_local_path() {
        let candidates = library_candidates(Some(Path::new("/Users/tester")));

        if cfg!(target_os = "macos") {
            assert_eq!(
                candidates,
                vec![
                    PathBuf::from("libPCBUSB.dylib"),
                    PathBuf::from("/Users/tester/.local/lib/libPCBUSB.dylib"),
                    PathBuf::from("PCBUSB"),
                ]
            );
        } else {
            assert_eq!(candidates, vec![PathBuf::from("PCANBasic.dll")]);
        }
    }

    #[test]
    fn library_load_error_includes_all_attempts() {
        let error = library_load_error(&[
            "libPCBUSB.dylib: first failure".to_string(),
            "/Users/tester/.local/lib/libPCBUSB.dylib: second failure".to_string(),
            "PCBUSB: third failure".to_string(),
        ]);
        let message = error.to_string();

        assert!(message.contains("libPCBUSB.dylib: first failure"));
        assert!(message.contains("/Users/tester/.local/lib/libPCBUSB.dylib: second failure"));
        assert!(message.contains("PCBUSB: third failure"));
    }

    #[test]
    fn bitrate_mapping_has_common_values() {
        assert_eq!(bitrate_to_btr0btr1(1_000_000), Some(0x0014));
        assert_eq!(bitrate_to_btr0btr1(500_000), Some(0x001C));
        assert_eq!(bitrate_to_btr0btr1(125_000), Some(0x031C));
        assert_eq!(bitrate_to_btr0btr1(123_456), None);
    }

    #[test]
    fn parse_channel_supports_can_aliases() {
        let (h0, b0) = parse_channel_and_bitrate("can0@1000000").expect("can0");
        let (h1, b1) = parse_channel_and_bitrate("can1@500000").expect("can1");
        assert_eq!(h0, 0x51);
        assert_eq!(b0, 0x0014);
        assert_eq!(h1, 0x52);
        assert_eq!(b1, 0x001C);
    }

    #[test]
    fn parse_channel_supports_pcan_and_hex_and_numeric() {
        let (a, _) = parse_channel_and_bitrate("PCAN_USBBUS1@1000000").expect("pcan name");
        let (b, _) = parse_channel_and_bitrate("0x51@1000000").expect("hex");
        let (c, _) = parse_channel_and_bitrate("81@1000000").expect("numeric");
        assert_eq!(a, 0x51);
        assert_eq!(b, 0x51);
        assert_eq!(c, 81);
    }

    #[test]
    fn parse_channel_rejects_invalid_inputs() {
        assert!(parse_channel_and_bitrate("canX@1000000").is_err());
        assert!(parse_channel_and_bitrate("PCAN_USBBUS0@1000000").is_err());
        assert!(parse_channel_and_bitrate("can0@123456").is_err());
    }
}
