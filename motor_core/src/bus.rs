use crate::error::Result;
#[cfg(any(target_os = "windows", target_os = "macos"))]
use crate::pcan::PcanBus;
use crate::slcan::{SlcanBus, DEFAULT_BITRATE, DEFAULT_SERIAL_BAUD};
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

/// Prefix that selects the slcan (LAWICEL ASCII) serial transport.
pub const SLCAN_PREFIX: &str = "slcan:";

/// Parse `slcan:<port>[@<bitrate>]`, for example
/// `slcan:/dev/ttyACM0`, `slcan:/dev/cu.usbmodem1234@1000000` or `slcan:COM5@500000`.
fn open_slcan(spec: &str) -> Result<Arc<dyn CanBus>> {
    let spec = spec.trim();
    let (port, bitrate) = match spec.rsplit_once('@') {
        Some((port, rate)) => {
            let parsed = rate.parse::<u32>().map_err(|_| {
                crate::error::MotorError::InvalidArgument(format!(
                    "invalid slcan bitrate '{rate}', expected bit/s such as 1000000"
                ))
            })?;
            (port, parsed)
        }
        None => (spec, DEFAULT_BITRATE),
    };
    if port.is_empty() {
        return Err(crate::error::MotorError::InvalidArgument(
            "empty slcan port, expected slcan:<port>[@<bitrate>]".to_string(),
        ));
    }
    let bus: Arc<dyn CanBus> = Arc::new(SlcanBus::open(port, DEFAULT_SERIAL_BAUD, bitrate)?);
    Ok(bus)
}

pub fn open_can_bus(channel: &str) -> Result<Arc<dyn CanBus>> {
    // Platform independent: an slcan adapter is a plain serial device everywhere.
    if let Some(spec) = channel.strip_prefix(SLCAN_PREFIX) {
        return open_slcan(spec);
    }
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
