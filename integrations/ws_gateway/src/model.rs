use motor_core::bus::CanBus;
use motor_vendor_cyberbeast::{CyberBeastController, CyberBeastMotor};
use motor_vendor_damiao::{DamiaoController, DamiaoMotor};
use motor_vendor_hexfellow::{HexfellowController, HexfellowMotor};
use motor_vendor_myactuator::{MyActuatorController, MyActuatorMotor};
use motor_vendor_robstride::{RobstrideController, RobstrideMotor};
use std::sync::Arc;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Vendor {
    Damiao,
    Hexfellow,
    Hightorque,
    Myactuator,
    Robstride,
    CyberBeast,
}

impl Vendor {
    pub(crate) fn from_str(s: &str) -> Result<Self, String> {
        match s.to_lowercase().as_str() {
            "damiao" => Ok(Self::Damiao),
            "hexfellow" => Ok(Self::Hexfellow),
            "hightorque" => Ok(Self::Hightorque),
            "myactuator" => Ok(Self::Myactuator),
            "robstride" => Ok(Self::Robstride),
            "cyberbeast" => Ok(Self::CyberBeast),
            _ => Err(format!("unsupported vendor: {s}")),
        }
    }

    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Self::Damiao => "damiao",
            Self::Hexfellow => "hexfellow",
            Self::Hightorque => "hightorque",
            Self::Myactuator => "myactuator",
            Self::CyberBeast => "cyberbeast",
            Self::Robstride => "robstride",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Transport {
    Auto,
    SocketCan,
    SocketCanFd,
    DmSerial,
    DmDevice,
}

impl Transport {
    pub(crate) fn from_str(s: &str) -> Result<Self, String> {
        match s.to_lowercase().as_str() {
            "auto" => Ok(Self::Auto),
            "socketcan" => Ok(Self::SocketCan),
            "socketcanfd" => Ok(Self::SocketCanFd),
            "dm-serial" => Ok(Self::DmSerial),
            "dm-device" => Ok(Self::DmDevice),
            _ => Err(format!("unsupported transport: {s}")),
        }
    }

    pub(crate) fn as_str(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::SocketCan => "socketcan",
            Self::SocketCanFd => "socketcanfd",
            Self::DmSerial => "dm-serial",
            Self::DmDevice => "dm-device",
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct Target {
    pub(crate) vendor: Vendor,
    pub(crate) transport: Transport,
    pub(crate) channel: String,
    pub(crate) serial_port: String,
    pub(crate) serial_baud: u32,
    pub(crate) dm_device_type: String,
    pub(crate) dm_channel: String,
    pub(crate) model: String,
    pub(crate) motor_id: u16,
    pub(crate) feedback_id: u16,
}

#[derive(Clone, Debug)]
pub(crate) struct ServerConfig {
    pub(crate) bind: String,
    pub(crate) target: Target,
    pub(crate) dt_ms: u64,
}

#[derive(Clone, Debug)]
pub(crate) enum ActiveCommand {
    Mit {
        pos: f32,
        vel: f32,
        kp: f32,
        kd: f32,
        tau: f32,
    },
    PosVel {
        pos: f32,
        vlim: f32,
    },
    Vel {
        vel: f32,
    },
    ForcePos {
        pos: f32,
        vlim: f32,
        ratio: f32,
    },
}

pub(crate) enum ControllerHandle {
    Damiao(DamiaoController),
    Hexfellow(HexfellowController),
    Hightorque(Box<dyn CanBus>),
    Myactuator(MyActuatorController),
    Robstride(RobstrideController),
    CyberBeast(CyberBeastController),
}

pub(crate) enum MotorHandle {
    Damiao(Arc<DamiaoMotor>),
    Hexfellow(Arc<HexfellowMotor>),
    Hightorque(u16),
    Myactuator(Arc<MyActuatorMotor>),
    Robstride(Arc<RobstrideMotor>),
    CyberBeast(Arc<CyberBeastMotor>),
}
