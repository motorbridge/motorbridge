use crate::model::{ControllerHandle, MotorHandle, Transport, Vendor};
use crate::vendors::hightorque_ws::open_hightorque_bus;
use motor_core::dm_device::DmDeviceType;
use motor_vendor_damiao::DamiaoController;
use motor_vendor_hexfellow::HexfellowController;
use motor_vendor_myactuator::MyActuatorController;
use motor_vendor_robstride::RobstrideController;

use super::{myactuator_feedback_default, SessionCtx};

impl SessionCtx {
    pub(crate) fn connect(&mut self) -> Result<(), String> {
        self.disconnect(false);
        match self.target.vendor {
            Vendor::Damiao => {
                let p = motor_core::bus::TransportParams {
                    channel: &self.target.channel,
                    serial_port: &self.target.serial_port,
                    serial_baud: self.target.serial_baud,
                };
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                            .map(DamiaoController::new)
                    }
                    Transport::SocketCanFd => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                            .map(DamiaoController::new)
                    }
                    Transport::DmSerial => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::DmSerial, &p)
                            .map(DamiaoController::new)
                    }
                    Transport::McuSerial => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                            .map(DamiaoController::new)
                    }
                    Transport::DmDevice => DamiaoController::new_dm_device(
                        DmDeviceType::parse(&self.target.dm_device_type)
                            .map_err(|e| e.to_string())?,
                        &self.target.dm_channel,
                    ),
                }
                .map_err(|e| format!("open bus failed: {e}"))?;
                self.controller = Some(ControllerHandle::Damiao(ctrl));
                if !Self::model_is_auto(&self.target.model) {
                    let motor = match self.controller.as_ref() {
                        Some(ControllerHandle::Damiao(c)) => c
                            .add_motor(
                                self.target.motor_id,
                                self.target.feedback_id,
                                &self.target.model,
                            )
                            .map_err(|e| format!("add motor failed: {e}"))?,
                        _ => return Err("damiao controller not connected".to_string()),
                    };
                    self.motor = Some(MotorHandle::Damiao(motor));
                } else {
                    self.motor = None;
                }
            }
            Vendor::Hexfellow => {
                let p = motor_core::bus::TransportParams {
                    channel: &self.target.channel,
                    serial_port: &self.target.serial_port,
                    serial_baud: self.target.serial_baud,
                };
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCanFd => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                            .map(HexfellowController::new)
                    }
                    Transport::SocketCan => Err(motor_core::error::MotorError::InvalidArgument(
                        "hexfellow is CAN-FD only, use socketcanfd".to_string(),
                    )),
                    Transport::McuSerial => Err(motor_core::error::MotorError::InvalidArgument(
                        "transport mcu-serial is classic CAN only; hexfellow requires CAN-FD"
                            .to_string(),
                    )),
                    Transport::DmSerial => Err(motor_core::error::MotorError::InvalidArgument(
                        "dm-serial transport is damiao-only".to_string(),
                    )),
                    Transport::DmDevice => Err(motor_core::error::MotorError::InvalidArgument(
                        "dm-device transport is damiao-only".to_string(),
                    )),
                }
                .map_err(|e| format!("open bus failed: {e}"))?;
                let motor = ctrl
                    .add_motor(
                        self.target.motor_id,
                        self.target.feedback_id,
                        &self.target.model,
                    )
                    .map_err(|e| format!("add motor failed: {e}"))?;
                self.controller = Some(ControllerHandle::Hexfellow(ctrl));
                self.motor = Some(MotorHandle::Hexfellow(motor));
            }
            Vendor::Hightorque => {
                let bus = open_hightorque_bus(&self.target)?;
                self.controller = Some(ControllerHandle::Hightorque(bus));
                self.motor = Some(MotorHandle::Hightorque(self.target.motor_id));
            }
            Vendor::Myactuator => {
                let p = motor_core::bus::TransportParams {
                    channel: &self.target.channel,
                    serial_port: &self.target.serial_port,
                    serial_baud: self.target.serial_baud,
                };
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                            .map(MyActuatorController::new)
                    }
                    Transport::SocketCanFd => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                            .map(MyActuatorController::new)
                    }
                    Transport::McuSerial => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                            .map(MyActuatorController::new)
                    }
                    Transport::DmSerial => Err(motor_core::error::MotorError::InvalidArgument(
                        "dm-serial transport is damiao-only".to_string(),
                    )),
                    Transport::DmDevice => Err(motor_core::error::MotorError::InvalidArgument(
                        "dm-device transport is damiao-only".to_string(),
                    )),
                }
                .map_err(|e| format!("open bus failed: {e}"))?;
                let fid = if self.target.feedback_id == 0 {
                    myactuator_feedback_default(self.target.motor_id)
                } else {
                    self.target.feedback_id
                };
                let motor = ctrl
                    .add_motor(self.target.motor_id, fid, &self.target.model)
                    .map_err(|e| format!("add motor failed: {e}"))?;
                self.controller = Some(ControllerHandle::Myactuator(ctrl));
                self.motor = Some(MotorHandle::Myactuator(motor));
            }
            Vendor::Robstride => {
                let p = motor_core::bus::TransportParams {
                    channel: &self.target.channel,
                    serial_port: &self.target.serial_port,
                    serial_baud: self.target.serial_baud,
                };
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCan, &p)
                            .map(RobstrideController::new)
                    }
                    Transport::SocketCanFd => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::SocketCanFd, &p)
                            .map(RobstrideController::new)
                    }
                    Transport::McuSerial => {
                        motor_core::bus::open_transport(motor_core::bus::Transport::McuSerial, &p)
                            .map(RobstrideController::new)
                    }
                    Transport::DmSerial => Err(motor_core::error::MotorError::InvalidArgument(
                        "dm-serial transport is damiao-only".to_string(),
                    )),
                    Transport::DmDevice => Err(motor_core::error::MotorError::InvalidArgument(
                        "dm-device transport is damiao-only".to_string(),
                    )),
                }
                .map_err(|e| format!("open bus failed: {e}"))?;
                let motor = ctrl
                    .add_motor(
                        self.target.motor_id,
                        self.target.feedback_id,
                        &self.target.model,
                    )
                    .map_err(|e| format!("add motor failed: {e}"))?;
                self.controller = Some(ControllerHandle::Robstride(ctrl));
                self.motor = Some(MotorHandle::Robstride(motor));
            }
        }
        Ok(())
    }

    pub(crate) fn ensure_connected(&mut self) -> Result<(), String> {
        if self.controller.is_none() {
            self.connect()?;
        }
        Ok(())
    }

    pub(crate) fn disconnect(&mut self, shutdown: bool) {
        self.active = None;
        self.motor = None;
        if let Some(ctrl) = self.controller.take() {
            match ctrl {
                ControllerHandle::Damiao(c) => {
                    if shutdown {
                        let _ = c.shutdown();
                    } else {
                        let _ = c.close_bus();
                    }
                }
                ControllerHandle::Hexfellow(c) => {
                    if shutdown {
                        let _ = c.shutdown();
                    } else {
                        let _ = c.close_bus();
                    }
                }
                ControllerHandle::Hightorque(bus) => {
                    let _ = bus.shutdown();
                }
                ControllerHandle::Myactuator(c) => {
                    if shutdown {
                        let _ = c.shutdown();
                    } else {
                        let _ = c.close_bus();
                    }
                }
                ControllerHandle::Robstride(c) => {
                    if shutdown {
                        let _ = c.shutdown();
                    } else {
                        let _ = c.close_bus();
                    }
                }
            }
        }
    }
}
