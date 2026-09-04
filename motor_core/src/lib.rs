pub mod bus;
pub mod controller;
pub mod device;
pub mod dm_device;
pub mod dm_serial;
pub mod error;
pub mod mcu_serial;
pub mod model;
#[cfg(any(target_os = "windows", target_os = "macos"))]
pub mod pcan;
pub mod socketcan;
#[cfg(target_os = "linux")]
pub mod socketcanfd;
#[cfg(any(test, feature = "test-support"))]
pub mod test_support;
pub mod vendor_controller;

pub use bus::{open_can_bus, open_socketcan, open_socketcanfd, CanBus, CanFrame};
pub use controller::CoreController;
pub use device::MotorDevice;
pub use error::{MotorError, Result};
pub use model::{ModelCatalog, MotorModelSpec, PvTLimits, StaticModelCatalog};
pub use vendor_controller::VendorController;
