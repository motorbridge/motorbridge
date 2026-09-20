use crate::model::{Target, Transport};
use motor_core::bus::CanBus;
use motor_core::dm_device::DmDeviceType;
use motor_vendor_damiao::DamiaoController;
use motor_vendor_hexfellow::HexfellowController;
use motor_vendor_myactuator::MyActuatorController;
use motor_vendor_robstride::RobstrideController;
use std::sync::Arc;

pub(crate) fn myactuator_feedback_default(motor_id: u16) -> u16 {
    0x240u16.saturating_add(motor_id)
}

pub(crate) fn open_damiao_controller(
    base: &Target,
    transport: Transport,
) -> Result<DamiaoController, String> {
    let p = motor_core::bus::TransportParams {
        channel: &base.channel,
        serial_port: &base.serial_port,
        serial_baud: base.serial_baud,
    };
    match transport {
        Transport::Auto | Transport::SocketCan => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                .map(DamiaoController::new)
                .map_err(|e| e.to_string())
        }
        Transport::SocketCanFd => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                .map(DamiaoController::new)
                .map_err(|e| e.to_string())
        }
        Transport::DmSerial => {
            motor_core::bus::open_transport(motor_core::bus::Transport::DmSerial, &p)
                .map(DamiaoController::new)
                .map_err(|e| e.to_string())
        }
        Transport::McuSerial => {
            motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                .map(DamiaoController::new)
                .map_err(|e| e.to_string())
        }
        Transport::DmDevice => DamiaoController::new_dm_device(
            DmDeviceType::parse(&base.dm_device_type).map_err(|e| e.to_string())?,
            &base.dm_channel,
        )
        .map_err(|e| e.to_string()),
    }
}

pub(crate) fn open_robstride_controller(
    base: &Target,
    transport: Transport,
) -> Result<RobstrideController, String> {
    let p = motor_core::bus::TransportParams {
        channel: &base.channel,
        serial_port: &base.serial_port,
        serial_baud: base.serial_baud,
    };
    match transport {
        Transport::Auto | Transport::SocketCan => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                .map(RobstrideController::new)
                .map_err(|e| e.to_string())
        }
        Transport::SocketCanFd => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                .map(RobstrideController::new)
                .map_err(|e| e.to_string())
        }
        Transport::McuSerial => {
            motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                .map(RobstrideController::new)
                .map_err(|e| e.to_string())
        }
        Transport::DmSerial => Err("transport dm-serial is damiao-only".to_string()),
        Transport::DmDevice => Err("transport dm-device is damiao-only".to_string()),
    }
}

pub(crate) fn open_myactuator_controller(
    base: &Target,
    transport: Transport,
) -> Result<MyActuatorController, String> {
    let p = motor_core::bus::TransportParams {
        channel: &base.channel,
        serial_port: &base.serial_port,
        serial_baud: base.serial_baud,
    };
    match transport {
        Transport::Auto | Transport::SocketCan => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                .map(MyActuatorController::new)
                .map_err(|e| e.to_string())
        }
        Transport::SocketCanFd => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                .map(MyActuatorController::new)
                .map_err(|e| e.to_string())
        }
        Transport::McuSerial => {
            motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                .map(MyActuatorController::new)
                .map_err(|e| e.to_string())
        }
        Transport::DmSerial | Transport::DmDevice => {
            Err("transport dm-serial/dm-device is damiao-only".to_string())
        }
    }
}

pub(crate) fn open_hexfellow_controller(
    base: &Target,
    transport: Transport,
) -> Result<HexfellowController, String> {
    let p = motor_core::bus::TransportParams {
        channel: &base.channel,
        serial_port: &base.serial_port,
        serial_baud: base.serial_baud,
    };
    match transport {
        Transport::Auto | Transport::SocketCanFd => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                .map(HexfellowController::new)
                .map_err(|e| e.to_string())
        }
        Transport::SocketCan => {
            Err("hexfellow requires transport socketcanfd (or auto)".to_string())
        }
        Transport::DmSerial | Transport::DmDevice => {
            Err("transport dm-serial/dm-device is damiao-only".to_string())
        }
        Transport::McuSerial => {
            Err("transport mcu-serial is classic-CAN only; hexfellow requires CAN-FD".to_string())
        }
    }
}

pub(crate) fn open_hightorque_bus(
    base: &Target,
    transport: Transport,
) -> Result<Arc<dyn CanBus>, String> {
    let p = motor_core::bus::TransportParams {
        channel: &base.channel,
        serial_port: &base.serial_port,
        serial_baud: base.serial_baud,
    };
    match transport {
        Transport::Auto | Transport::SocketCan => {
            motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                .map_err(|e| format!("open bus failed: {e}"))
        }
        Transport::SocketCanFd => {
            Err("hightorque currently uses standard CAN transport only".to_string())
        }
        Transport::DmSerial => Err("transport dm-serial is damiao-only".to_string()),
        Transport::DmDevice => Err("transport dm-device is damiao-only".to_string()),
        Transport::McuSerial => {
            motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                .map_err(|e| format!("open bus failed: {e}"))
        }
    }
}
