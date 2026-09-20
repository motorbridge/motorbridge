use std::fmt::{Display, Formatter};

#[derive(Debug)]
pub enum MotorError {
    InvalidArgument(String),
    Io(String),
    Timeout(String),
    Protocol(String),
    Unsupported(String),
    /// MCU link-status snapshot pushed by the bridge over a reserved
    /// mcu-serial id (see `McuSerialBus` / `STATUS_ID`). Carries a decoded,
    /// human-readable description so callers can distinguish bus-off /
    /// TX-ACK-failure / RX-drop / USB-stream-corruption instead of seeing
    /// every CAN-layer fault collapse to a recv timeout.
    BusStatus(String),
}

impl Display for MotorError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidArgument(msg)
            | Self::Io(msg)
            | Self::Timeout(msg)
            | Self::Protocol(msg)
            | Self::Unsupported(msg)
            | Self::BusStatus(msg) => f.write_str(msg),
        }
    }
}

impl std::error::Error for MotorError {}

pub type Result<T> = std::result::Result<T, MotorError>;

impl From<std::io::Error> for MotorError {
    fn from(value: std::io::Error) -> Self {
        Self::Io(value.to_string())
    }
}
