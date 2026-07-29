use crate::model::{ControllerHandle, MotorHandle, Transport, Vendor};
use crate::vendors::hightorque_ws::open_hightorque_bus;
use motor_core::dm_device::DmDeviceType;
use motor_vendor_damiao::DamiaoController;
use motor_vendor_hexfellow::HexfellowController;
use motor_vendor_myactuator::MyActuatorController;
use motor_vendor_cyberbeast::CyberBeastController;
use motor_vendor_robstride::RobstrideController;

use super::{myactuator_feedback_default, SessionCtx};

impl SessionCtx {
    pub(crate) fn connect(&mut self) -> Result<(), String> {
        self.disconnect(false);
        match self.target.vendor {
            Vendor::Damiao => {
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        DamiaoController::new_socketcan(&self.target.channel)
                    }
                    Transport::SocketCanFd => {
                        DamiaoController::new_socketcanfd(&self.target.channel)
                    }
                    Transport::DmSerial => DamiaoController::new_dm_serial(
                        &self.target.serial_port,
                        self.target.serial_baud,
                    ),
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
                if !matches!(
                    self.target.transport,
                    Transport::Auto | Transport::SocketCanFd
                ) {
                    return Err("hexfellow requires transport socketcanfd (or auto)".to_string());
                }
                let ctrl = HexfellowController::new_socketcanfd(&self.target.channel)
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
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        MyActuatorController::new_socketcan(&self.target.channel)
                    }
                    Transport::SocketCanFd => {
                        MyActuatorController::new_socketcanfd(&self.target.channel)
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
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        RobstrideController::new_socketcan(&self.target.channel)
                    }
                    Transport::SocketCanFd => {
                        RobstrideController::new_socketcanfd(&self.target.channel)
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
            Vendor::CyberBeast => {
                let ctrl = match self.target.transport {
                    Transport::Auto | Transport::SocketCan => {
                        CyberBeastController::new_socketcan(&self.target.channel)
                    }
                    Transport::SocketCanFd => {
                        CyberBeastController::new_socketcan(&self.target.channel)
                    }
                    _ => Err(motor_core::error::MotorError::InvalidArgument(
                        "unsupported transport for cyberbeast".to_string(),
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
                self.controller = Some(ControllerHandle::CyberBeast(ctrl));
                self.motor = Some(MotorHandle::CyberBeast(motor));
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
                ControllerHandle::CyberBeast(c) => {
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
