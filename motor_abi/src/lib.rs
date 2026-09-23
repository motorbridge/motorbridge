use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_core::dm_device::DmDeviceType;
use motor_core::CoreController;
use motor_vendor_damiao::{ControlMode as DamiaoControlMode, DamiaoController, DamiaoMotor};
use motor_vendor_hexfellow::{
    HexfellowController, HexfellowMotor, MitTarget as HexfellowMitTarget,
    PosVelTarget as HexfellowPosVelTarget,
};
use motor_vendor_hightorque::{HightorqueController, HightorqueMotor};
use motor_vendor_myactuator::{
    ControlMode as MyActuatorControlMode, MyActuatorController, MyActuatorMotor,
};
use motor_vendor_robstride::{
    ControlMode as RobstrideControlMode, ParameterValue, RobstrideController, RobstrideMotor,
};
use std::cell::RefCell;
use std::f32::consts::PI;
use std::ffi::{c_char, CStr, CString};
use std::ptr;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::new("ok").expect("static cstring"));
}

static ABI_CAPABILITIES_JSON: OnceLock<CString> = OnceLock::new();

const ABI_CAPABILITIES: &str = r#"{
  "schema": 1,
  "abi": {
    "name": "motor_abi",
    "version": "__MOTORBRIDGE_VERSION__"
  },
  "transports": ["socketcan", "socketcanfd", "dm-serial", "dm-device", "mcu-serial"],
  "vendors": ["damiao", "robstride", "myactuator", "hexfellow", "hightorque"],
  "features": {
    "state_cache": true,
    "controller_lifecycle": ["shutdown", "close_bus", "poll_feedback_once", "enable_all", "disable_all"],
    "control_modes": ["mit", "pos-vel", "vel", "force-pos", "robstride-pos-vel-pp", "robstride-pos-vel-csp"],
    "damiao": ["dm-serial", "dm-device", "register_u32", "register_f32", "param_u32", "param_f32", "set_can_timeout_ms"],
    "robstride": ["ping", "ping_host_id", "fault_report", "active_report", "device_id", "param_i8", "param_u8", "param_u16", "param_u32", "param_f32", "param_f32_host_id", "pos_vel_pp", "pos_vel_csp"],
    "myactuator": ["param_i8", "param_u8", "param_u16", "param_u32", "param_f32"],
    "hexfellow": ["socketcanfd", "mit", "pos_vel"],
    "hightorque": ["mit", "vel", "param_i8", "param_u8", "param_u16", "param_u32", "param_f32"]
  }
}"#;

fn abi_capabilities_json() -> &'static CString {
    ABI_CAPABILITIES_JSON.get_or_init(|| {
        CString::new(ABI_CAPABILITIES.replace("__MOTORBRIDGE_VERSION__", env!("CARGO_PKG_VERSION")))
            .expect("capabilities json has no nul bytes")
    })
}

fn set_last_error(msg: impl AsRef<str>) {
    let clean = msg.as_ref().replace('\0', " ");
    let cstr =
        CString::new(clean).unwrap_or_else(|_| CString::new("error").expect("fallback cstring"));
    LAST_ERROR.with(|slot| *slot.borrow_mut() = cstr);
}

fn ok_ptr() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

fn to_damiao_mode(mode: u32) -> Result<DamiaoControlMode, &'static str> {
    match mode {
        1 => Ok(DamiaoControlMode::Mit),
        2 => Ok(DamiaoControlMode::PosVel),
        3 => Ok(DamiaoControlMode::Vel),
        4 => Ok(DamiaoControlMode::ForcePos),
        _ => Err("Damiao mode must be 1(MIT) / 2(POS_VEL) / 3(VEL) / 4(FORCE_POS)"),
    }
}

fn to_robstride_mode(mode: u32) -> Result<RobstrideControlMode, &'static str> {
    match mode {
        1 => Ok(RobstrideControlMode::Mit),
        2 => Ok(RobstrideControlMode::Position),
        3 => Ok(RobstrideControlMode::Velocity),
        5 => Ok(RobstrideControlMode::PositionCsp),
        _ => Err("RobStride mode must be 1(MIT) / 2(POSITION-PP) / 3(VELOCITY) / 5(POSITION-CSP)"),
    }
}

fn to_myactuator_mode(mode: u32) -> Result<MyActuatorControlMode, &'static str> {
    match mode {
        1 => Ok(MyActuatorControlMode::Current),
        2 => Ok(MyActuatorControlMode::Position),
        3 => Ok(MyActuatorControlMode::Velocity),
        _ => Err("MyActuator mode must be 1(CURRENT) / 2(POSITION) / 3(VELOCITY)"),
    }
}

enum ControllerInner {
    // SocketCAN path — lazy: stores the channel name; the bus is re-opened on
    // first `add_*_motor`. Binding picks the vendor's own classic-vs-FD
    // `Transport` (hexfellow → CAN-FD, the rest → classic CAN) and routes
    // through core's `open_transport` so the driver constructor lives in one
    // place.
    Unbound(String),
    // mcu-serial path — a vendor-agnostic UART-to-CAN MCU bridge; store the
    // port spec, open lazily on first `add_*_motor`. Classic 8-byte CAN only:
    // hexfellow (CAN-FD) is rejected with a clear error rather than silently
    // broken; the other classic-CAN vendors share one bus.
    UnboundMcuSerial { port: String, baud: u32 },
    // Eager dm-serial / dm-device: Damiao-only USB dongle transports, opened at
    // `motor_controller_new_dm_serial` / `new_dm_device` time. These dongles
    // speak a Damiao-specific protocol, so only Damiao motors may be added.
    Damiao(DamiaoController),
    // Shared-core multi-vendor: ONE `CoreController` (one bus fd, one background
    // receive thread, one device table), with every vendor's motors added to it
    // and dispatched by CAN arbitration id (`accepts_frame`). Per-vendor
    // controllers are created lazily on first `add_*_motor` of that vendor, all
    // sharing this `Arc<CoreController>`. Used by the socketcan & mcu-serial
    // paths so mixed-vendor control (e.g. HighTorque + Damiao + RobStride on
    // one /dev/ttyACM0) works on a single controller instead of N fds stealing
    // bytes from one tty input buffer.
    Bound(BoundController),
}

/// One shared `CoreController` plus lazily-created per-vendor controllers, all
/// referencing that shared core. Created the first time any `add_*_motor` binds
/// an `Unbound`/`UnboundMcuSerial` controller. The eager dm-serial/dm-device
/// path keeps its own `DamiaoController` (own core) and never reaches here.
struct BoundController {
    core: Arc<CoreController>,
    transport: Transport,
    damiao: Option<DamiaoController>,
    hexfellow: Option<HexfellowController>,
    myactuator: Option<MyActuatorController>,
    robstride: Option<RobstrideController>,
    hightorque: Option<HightorqueController>,
}

impl BoundController {
    fn new(bus: Arc<dyn CanBus>, transport: Transport) -> Self {
        Self {
            core: Arc::new(CoreController::new(bus)),
            transport,
            damiao: None,
            hexfellow: None,
            myactuator: None,
            robstride: None,
            hightorque: None,
        }
    }
}

/// The vendor a motor belongs to, used for the transport-compatibility guard
/// and to name the lazily-created per-vendor controller field on `BoundController`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Vendor {
    Damiao,
    Hexfellow,
    MyActuator,
    Robstride,
    Hightorque,
}

impl Vendor {
    fn name(self) -> &'static str {
        match self {
            Vendor::Damiao => "Damiao",
            Vendor::Hexfellow => "Hexfellow",
            Vendor::MyActuator => "MyActuator",
            Vendor::Robstride => "RobStride",
            Vendor::Hightorque => "HighTorque",
        }
    }

    /// SocketCAN-family transport this vendor uses: hexfellow is CAN-FD, the
    /// rest are classic CAN. Determines what an `Unbound` socketcan controller
    /// opens on first bind, and what later same-class vendors may share.
    fn socketcan_transport(self) -> Transport {
        match self {
            Vendor::Hexfellow => Transport::SocketCanFd,
            _ => Transport::SocketCan,
        }
    }
}

/// Whether a motor of `vendor` may be added to a bus already opened as
/// `transport`. mcu-serial & socketcan accept any classic-CAN vendor (hexfellow
/// is CAN-FD); SocketCanFd is hexfellow-only; DmSerial is Damiao-only. (dm-device
/// has no `Transport` variant — it stays on the eager `Damiao` path, which
/// rejects every other vendor in `ensure_bound`.)
fn transport_supports_vendor(transport: Transport, vendor: Vendor) -> bool {
    match transport {
        Transport::McuSerial | Transport::SocketCan => vendor != Vendor::Hexfellow,
        Transport::SocketCanFd => vendor == Vendor::Hexfellow,
        Transport::DmSerial => vendor == Vendor::Damiao,
    }
}

/// Lazy-bind an `Unbound`/`UnboundMcuSerial` controller to a shared-core
/// `Bound` (opening the bus via `open_transport`), or return the existing
/// `Bound`. Errors when the requested `vendor` is incompatible with an
/// already-opened bus (mixed classic/FD, a non-Damiao on a Damiao dongle, or
/// any vendor on the eager `Damiao` variant).
fn ensure_bound(
    inner: &mut ControllerInner,
    vendor: Vendor,
) -> Result<&mut BoundController, String> {
    match inner {
        ControllerInner::Bound(b) => {
            if !transport_supports_vendor(b.transport, vendor) {
                return Err(format!(
                    "controller bus already opened as {:?}; {} is not supported on this transport — use a separate controller",
                    b.transport,
                    vendor.name()
                ));
            }
            Ok(b)
        }
        ControllerInner::Damiao(_) => Err(
            "controller already bound to Damiao (dm-serial/dm-device is damiao-only) — use a separate controller"
                .to_string(),
        ),
        ControllerInner::UnboundMcuSerial { port, baud } => {
            if vendor == Vendor::Hexfellow {
                return Err(
                    "hexfellow requires CAN-FD; mcu-serial is classic CAN only — use a separate socketcanfd controller"
                        .to_string(),
                );
            }
            let p = TransportParams {
                channel: "",
                serial_port: port,
                serial_baud: *baud,
            };
            let bus =
                open_transport(Transport::McuSerial, &p).map_err(|e| e.to_string())?;
            *inner = ControllerInner::Bound(BoundController::new(bus, Transport::McuSerial));
            match inner {
                ControllerInner::Bound(b) => Ok(b),
                _ => unreachable!("just set Bound"),
            }
        }
        ControllerInner::Unbound(channel) => {
            let t = vendor.socketcan_transport();
            let p = TransportParams {
                channel,
                serial_port: "",
                serial_baud: 0,
            };
            let bus = open_transport(t, &p).map_err(|e| e.to_string())?;
            *inner = ControllerInner::Bound(BoundController::new(bus, t));
            match inner {
                ControllerInner::Bound(b) => Ok(b),
                _ => unreachable!("just set Bound"),
            }
        }
    }
}

// Lazily fetch (or create) the per-vendor controller on a shared-core `Bound`,
// opening the bus first if still `Unbound`/`UnboundMcuSerial`. Each vendor
// controller wraps the SAME `Arc<CoreController>`, so all motors share one fd
// and one background receive thread. Damiao is special-cased below because the
// eager dm-serial/dm-device path keeps its own `DamiaoController` (own core).
macro_rules! ensure_vendor_controller {
    ($fn_name:ident, $vendor:expr, $ty:ty, $field:ident) => {
        fn $fn_name(inner: &mut ControllerInner) -> Result<&mut $ty, String> {
            let bound = ensure_bound(inner, $vendor)?;
            // Clone the Arc before the mutable borrow of the Option field so
            // the borrow checker sees no overlap between the shared borrow of
            // `bound.core` (captured by the closure) and `&mut bound.$field`.
            let core = Arc::clone(&bound.core);
            Ok(bound.$field.get_or_insert_with(move || <$ty>::new_shared(core)))
        }
    };
}

ensure_vendor_controller!(
    ensure_hexfellow_controller,
    Vendor::Hexfellow,
    HexfellowController,
    hexfellow
);
ensure_vendor_controller!(
    ensure_myactuator_controller,
    Vendor::MyActuator,
    MyActuatorController,
    myactuator
);
ensure_vendor_controller!(
    ensure_robstride_controller,
    Vendor::Robstride,
    RobstrideController,
    robstride
);
ensure_vendor_controller!(
    ensure_hightorque_controller,
    Vendor::Hightorque,
    HightorqueController,
    hightorque
);

// Damiao: the eager dm-serial/dm-device path keeps its own `DamiaoController`
// (own core) and must short-circuit before `ensure_bound` (which only handles
// Unbound/UnboundMcuSerial/Bound). On the shared-core path it lazily creates a
// damiao controller like the other vendors.
fn ensure_damiao_controller(inner: &mut ControllerInner) -> Result<&mut DamiaoController, String> {
    match inner {
        ControllerInner::Damiao(ctrl) => Ok(ctrl),
        _ => {
            let bound = ensure_bound(inner, Vendor::Damiao)?;
            let core = Arc::clone(&bound.core);
            Ok(bound
                .damiao
                .get_or_insert_with(move || DamiaoController::new_shared(core)))
        }
    }
}

enum MotorHandleInner {
    Damiao(Arc<DamiaoMotor>),
    Hexfellow(Arc<HexfellowMotor>),
    MyActuator(Arc<MyActuatorMotor>),
    Robstride(Arc<RobstrideMotor>),
    Hightorque(Arc<HightorqueMotor>),
}

#[repr(C)]
pub struct MotorController {
    inner: Mutex<ControllerInner>,
}

#[repr(C)]
pub struct MotorHandle {
    inner: Mutex<MotorHandleInner>,
}

#[repr(C)]
pub struct MotorState {
    pub has_value: i32,
    pub can_id: u8,
    pub arbitration_id: u32,
    pub status_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            has_value: 0,
            can_id: 0,
            arbitration_id: 0,
            status_code: 0,
            pos: 0.0,
            vel: 0.0,
            torq: 0.0,
            t_mos: 0.0,
            t_rotor: 0.0,
        }
    }
}

fn ffi_rc(result: Result<(), String>) -> i32 {
    match result {
        Ok(()) => 0,
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

macro_rules! lock_motor_inner {
    ($motor_ptr:expr, $null_message:expr) => {{
        if $motor_ptr.is_null() {
            set_last_error($null_message);
            return -1;
        }
        let motor = unsafe { &*$motor_ptr };
        match motor.inner.lock() {
            Ok(inner) => inner,
            Err(_) => {
                set_last_error("motor handle lock poisoned");
                return -1;
            }
        }
    }};
}

macro_rules! ffi_wrap_motor {
    ($motor_ptr:expr, $body:expr) => {{
        let inner = lock_motor_inner!($motor_ptr, "motor is null");
        ffi_rc($body(&*inner))
    }};
}

fn parse_cstr(ptr: *const c_char, name: &str) -> Result<String, String> {
    if ptr.is_null() {
        return Err(format!("{name} is null"));
    }
    let s = unsafe { CStr::from_ptr(ptr) };
    s.to_str()
        .map(|v| v.to_string())
        .map_err(|_| format!("{name} must be valid UTF-8"))
}

mod controller_add_motor_ffi;
mod controller_lifecycle_ffi;
mod motor_control_ffi;
mod motor_lifecycle_ffi;
mod motor_register_ffi;
mod param_ffi;
mod state_ffi;
mod vendor_params;

#[cfg(test)]
mod tests {
    use super::*;

    // The whole point of the shared-core refactor: a classic-CAN bus
    // (mcu-serial or socketcan) accepts EVERY classic-CAN vendor on one
    // controller — HighTorque + Damiao + RobStride + MyActuator all share one
    // fd / one receive thread. Hexfellow is CAN-FD and is rejected so it does
    // not silently break on a classic-only link.
    #[test]
    fn classic_can_transports_accept_all_classic_vendors_reject_hexfellow() {
        let classics = [
            Vendor::Damiao,
            Vendor::MyActuator,
            Vendor::Robstride,
            Vendor::Hightorque,
        ];
        for t in [Transport::McuSerial, Transport::SocketCan] {
            for v in classics {
                assert!(
                    transport_supports_vendor(t, v),
                    "{t:?} should accept {v:?}"
                );
            }
            assert!(
                !transport_supports_vendor(t, Vendor::Hexfellow),
                "{t:?} must reject Hexfellow (CAN-FD)"
            );
        }
    }

    #[test]
    fn socketcanfd_is_hexfellow_only() {
        assert!(transport_supports_vendor(
            Transport::SocketCanFd,
            Vendor::Hexfellow
        ));
        for v in [
            Vendor::Damiao,
            Vendor::MyActuator,
            Vendor::Robstride,
            Vendor::Hightorque,
        ] {
            assert!(
                !transport_supports_vendor(Transport::SocketCanFd, v),
                "SocketCanFd must reject {v:?} (classic-CAN vendor)"
            );
        }
    }

    #[test]
    fn dmserial_is_damiao_only() {
        assert!(transport_supports_vendor(Transport::DmSerial, Vendor::Damiao));
        for v in [
            Vendor::Hexfellow,
            Vendor::MyActuator,
            Vendor::Robstride,
            Vendor::Hightorque,
        ] {
            assert!(
                !transport_supports_vendor(Transport::DmSerial, v),
                "DmSerial (Damiao dongle) must reject {v:?}"
            );
        }
    }

    // SocketCAN first-bind picks classic for the classic vendors, FD only for
    // hexfellow — so a same-class mix shares one core, a classic/FD mix is
    // rejected by the guard above.
    #[test]
    fn socketcan_transport_picks_classic_for_all_but_hexfellow() {
        assert_eq!(Vendor::Hexfellow.socketcan_transport(), Transport::SocketCanFd);
        for v in [
            Vendor::Damiao,
            Vendor::MyActuator,
            Vendor::Robstride,
            Vendor::Hightorque,
        ] {
            assert_eq!(v.socketcan_transport(), Transport::SocketCan);
        }
    }
}

