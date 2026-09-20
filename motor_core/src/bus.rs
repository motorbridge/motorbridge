use crate::error::Result;
#[cfg(any(target_os = "windows", target_os = "macos"))]
use crate::pcan::PcanBus;
#[cfg(target_os = "linux")]
use crate::socketcan::SocketCanBus;
#[cfg(target_os = "linux")]
use crate::socketcanfd::SocketCanFdBus;
use std::sync::Arc;
use std::time::Duration;

#[derive(Debug, Clone, Copy)]
pub struct CanFrame {
    pub arbitration_id: u32,
    pub data: [u8; 8],
    pub dlc: u8,
    pub is_extended: bool,
    /// Direction marker used by routing and tests.
    ///
    /// Frames returned by `CanBus::recv` are receive frames and should set this
    /// to `true`. Frames passed to `CanBus::send` by vendors should set it to
    /// `false`.
    pub is_rx: bool,
}

pub trait CanBus: Send + Sync {
    fn send(&self, frame: CanFrame) -> Result<()>;
    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>>;
    fn shutdown(&self) -> Result<()>;
}

pub fn open_can_bus(channel: &str) -> Result<Arc<dyn CanBus>> {
    #[cfg(target_os = "linux")]
    {
        let bus: Arc<dyn CanBus> = Arc::new(SocketCanBus::open(channel)?);
        Ok(bus)
    }
    #[cfg(any(target_os = "windows", target_os = "macos"))]
    {
        let bus: Arc<dyn CanBus> = Arc::new(PcanBus::open(channel)?);
        Ok(bus)
    }
    #[cfg(not(any(target_os = "linux", target_os = "windows", target_os = "macos")))]
    {
        let _ = channel;
        Err(crate::error::MotorError::InvalidArgument(
            "No CAN backend for current platform".to_string(),
        ))
    }
}

/// Compatibility alias for older callers.
///
/// Prefer `open_can_bus` in new code: this function selects the platform
/// classic-CAN backend, not Linux SocketCAN on every OS.
pub fn open_socketcan(channel: &str) -> Result<Arc<dyn CanBus>> {
    open_can_bus(channel)
}

pub fn open_socketcanfd(channel: &str) -> Result<Arc<dyn CanBus>> {
    #[cfg(target_os = "linux")]
    {
        let bus: Arc<dyn CanBus> = Arc::new(SocketCanFdBus::open(channel)?);
        Ok(bus)
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = channel;
        Err(crate::error::MotorError::InvalidArgument(
            "socketcanfd transport is only available on Linux".to_string(),
        ))
    }
}

/// Vendor-agnostic transport selection. Names the platform `CanBus` driver
/// to open for a given transport; returns a trait object the vendor
/// controllers wrap via `new(bus)`. This is the single place a new
/// transport gets wired: every frontend (ws_gateway, motor_abi, motor_cli)
/// calls this instead of re-implementing the match, so adding a transport is
/// one edit here, not three.
///
/// `dm-serial` is routed through here (damiao uses it via `open_transport`);
/// `dm-device` is intentionally absent: it is damiao-only, feature-gated
/// (`motorbridge_dm_device_supported`), and needs `DmDeviceType` parsing, so
/// it stays on the vendor controller's `new_dm_device` path. `auto` is a
/// frontend concern (resolve to a concrete transport before calling this).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transport {
    SocketCan,
    SocketCanFd,
    McuSerial,
    DmSerial,
}

impl Transport {
    // Deliberate inherent from_str/as_str pair (kept co-located; not a
    // std::str::FromStr impl) so callers need no trait import.
    #[allow(clippy::should_implement_trait)]
    pub fn from_str(s: &str) -> Result<Self> {
        match s.to_lowercase().as_str() {
            "socketcan" => Ok(Self::SocketCan),
            "socketcanfd" => Ok(Self::SocketCanFd),
            "mcu-serial" => Ok(Self::McuSerial),
            "dm-serial" => Ok(Self::DmSerial),
            _ => Err(crate::error::MotorError::InvalidArgument(format!(
                "unsupported transport: {s} (expected socketcan|socketcanfd|mcu-serial|dm-serial)"
            ))),
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::SocketCan => "socketcan",
            Self::SocketCanFd => "socketcanfd",
            Self::McuSerial => "mcu-serial",
            Self::DmSerial => "dm-serial",
        }
    }
}

/// Parameters needed to open any transport. `channel` is used by the
/// SocketCAN family; `serial_port`/`serial_baud` by mcu-serial.
/// Unused fields for a given transport are ignored.
#[derive(Clone, Debug, Default)]
pub struct TransportParams<'a> {
    pub channel: &'a str,
    pub serial_port: &'a str,
    pub serial_baud: u32,
}

/// Open the platform `CanBus` for `transport` using `params`. Reuses the
/// granular `open_can_bus`/`open_socketcanfd` helpers for the SocketCAN
/// family and constructs the serial-backed bridges (`McuSerialBus`,
/// `DmSerialBus`) inline — all four impls live in `motor_core`, so this
/// introduces no cross-crate dependency. `dm-device` is not supported here
/// (see the `Transport` doc comment).
pub fn open_transport(t: Transport, p: &TransportParams<'_>) -> Result<Arc<dyn CanBus>> {
    match t {
        Transport::SocketCan => open_can_bus(p.channel),
        Transport::SocketCanFd => open_socketcanfd(p.channel),
        Transport::McuSerial => Ok(Arc::new(crate::mcu_serial::McuSerialBus::open(
            p.serial_port,
            p.serial_baud,
        )?)),
        Transport::DmSerial => Ok(Arc::new(crate::dm_serial::DmSerialBus::open(
            p.serial_port,
            p.serial_baud,
        )?)),
    }
}
