use crate::bus::CanBus;
use crate::controller::CoreController;
use crate::device::MotorDevice;
use crate::error::{MotorError, Result};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub struct VendorController<M: MotorDevice + 'static> {
    core: Arc<CoreController>,
    motors: Mutex<HashMap<u16, Arc<M>>>,
}

impl<M: MotorDevice + 'static> VendorController<M> {
    pub fn new(bus: Arc<dyn CanBus>) -> Self {
        Self {
            core: Arc::new(CoreController::new(bus)),
            motors: Mutex::new(HashMap::new()),
        }
    }

    /// Wrap an existing shared `CoreController` so multiple vendors can share
    /// one bus fd and one background receive thread (mixed-vendor bus).
    /// The caller keeps the `CoreController` alive through this `Arc`; motors
    /// added here go into that shared core's device table, and `enable_all` /
    /// `disable_all` / `poll_feedback_once` / `shutdown` / `close_bus` operate
    /// on every device in the shared core regardless of vendor. The vendor's
    /// own `motors` map is kept only for `get_motor` lookups.
    pub fn new_shared(core: Arc<CoreController>) -> Self {
        Self {
            core,
            motors: Mutex::new(HashMap::new()),
        }
    }

    pub fn bus(&self) -> Arc<dyn CanBus> {
        self.core.bus()
    }

    pub fn add_motor_with<F>(&self, motor_id: u16, builder: F) -> Result<Arc<M>>
    where
        F: FnOnce(Arc<dyn CanBus>) -> Result<M>,
    {
        let motor = Arc::new(builder(self.core.bus())?);
        let device: Arc<dyn MotorDevice> = motor.clone();
        self.core.add_device(device)?;
        self.motors
            .lock()
            .map_err(|_| MotorError::Io("motors lock poisoned".to_string()))?
            .insert(motor_id, Arc::clone(&motor));
        Ok(motor)
    }

    pub fn get_motor(&self, motor_id: u16) -> Result<Arc<M>> {
        self.motors
            .lock()
            .map_err(|_| MotorError::Io("motors lock poisoned".to_string()))?
            .get(&motor_id)
            .cloned()
            .ok_or_else(|| MotorError::InvalidArgument(format!("motor {motor_id} not found")))
    }

    pub fn motor_count(&self) -> Result<usize> {
        Ok(self
            .motors
            .lock()
            .map_err(|_| MotorError::Io("motors lock poisoned".to_string()))?
            .len())
    }

    pub fn poll_feedback_once(&self) -> Result<()> {
        self.core.poll_feedback_once()
    }

    pub fn enable_all(&self) -> Result<()> {
        self.core.enable_all()
    }

    pub fn disable_all(&self) -> Result<()> {
        self.core.disable_all()
    }

    pub fn shutdown(&self) -> Result<()> {
        self.core.shutdown()
    }

    pub fn close_bus(&self) -> Result<()> {
        self.core.close_bus()
    }
}
