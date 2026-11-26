use crate::bus::{CanBus, CanFrame};
use crate::error::{MotorError, Result};
use core::ffi::{c_char, c_int, c_short, c_uint, c_void};
use std::ffi::CString;
use std::mem::size_of;
use std::os::fd::RawFd;
use std::sync::Mutex;
use std::time::Duration;

const AF_CAN: c_int = 29;
const PF_CAN: c_int = AF_CAN;
const SOCK_RAW: c_int = 3;
const CAN_RAW: c_int = 1;
const SOL_CAN_BASE: c_int = 100;
const SOL_CAN_RAW: c_int = SOL_CAN_BASE + CAN_RAW;
const CAN_RAW_FD_FRAMES: c_int = 5;
const POLLIN: c_short = 0x0001;
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
const CAN_SFF_MASK: u32 = 0x0000_07FF;
const CANFD_BRS: u8 = 0x01;
const CAN_MTU: usize = 16;
const CANFD_MTU: usize = 72;

#[repr(C)]
struct SockAddrCan {
    can_family: u16,
    can_ifindex: c_int,
    can_addr: [u8; 8],
}

#[repr(C)]
struct PollFd {
    fd: c_int,
    events: c_short,
    revents: c_short,
}

#[repr(C)]
struct CanFrameRaw {
    can_id: u32,
    can_dlc: u8,
    __pad: u8,
    __res0: u8,
    __res1: u8,
    data: [u8; 8],
}

#[repr(C)]
struct CanFdFrameRaw {
    can_id: u32,
    len: u8,
    flags: u8,
    __res0: u8,
    __res1: u8,
    data: [u8; 64],
}

unsafe extern "C" {
    fn socket(domain: c_int, typ: c_int, protocol: c_int) -> c_int;
    fn bind(sockfd: c_int, addr: *const c_void, addrlen: u32) -> c_int;
    fn close(fd: c_int) -> c_int;
    fn setsockopt(
        fd: c_int,
        level: c_int,
        optname: c_int,
        optval: *const c_void,
        optlen: c_uint,
    ) -> c_int;
    fn write(fd: c_int, buf: *const c_void, count: usize) -> isize;
    fn read(fd: c_int, buf: *mut c_void, count: usize) -> isize;
    fn poll(fds: *mut PollFd, nfds: c_uint, timeout: c_int) -> c_int;
    fn if_nametoindex(ifname: *const c_char) -> c_uint;
}

fn last_os_error(prefix: &str, interface: Option<&str>) -> MotorError {
    let err = std::io::Error::last_os_error();
    let iface = interface.unwrap_or("can0");
    let hint = socketcanfd_hint(err.raw_os_error(), iface);
    MotorError::Io(format!("{prefix}: {err}{hint}"))
}

fn socketcanfd_hint(raw_os_error: Option<i32>, iface: &str) -> String {
    match raw_os_error {
        Some(100) => format!(
            " (hint: can interface is down; run `ip -details link show {iface}` then bring it up)"
        ),
        Some(6) => {
            " (hint: can device/address is unavailable; check USB-CAN adapter and `ip link show`)"
                .to_string()
        }
        Some(19) => format!(
            " (hint: interface not found; verify channel name like {iface} and adapter connection)"
        ),
        Some(22) => {
            " (hint: invalid argument; interface may not be in CAN-FD mode, run canfd_restart)"
                .to_string()
        }
        _ => String::new(),
    }
}

pub struct SocketCanFdBus {
    fd: Mutex<Option<RawFd>>,
    interface: String,
    enable_brs: bool,
}

impl SocketCanFdBus {
    pub fn open(interface: &str) -> Result<Self> {
        Self::open_with_brs(interface, crate::bus::canfd_brs_from_env())
    }

    pub fn open_with_brs(interface: &str, enable_brs: bool) -> Result<Self> {
        let iface = CString::new(interface)
            .map_err(|_| MotorError::InvalidArgument("interface contains NUL".to_string()))?;

        let index = unsafe { if_nametoindex(iface.as_ptr()) };
        if index == 0 {
            return Err(last_os_error(
                &format!("if_nametoindex failed for {interface}"),
                Some(interface),
            ));
        }

        let fd = unsafe { socket(PF_CAN, SOCK_RAW, CAN_RAW) };
        if fd < 0 {
            return Err(last_os_error(
                "socket(PF_CAN, SOCK_RAW, CAN_RAW) failed",
                Some(interface),
            ));
        }

        let enable_fd: c_int = 1;
        let sockopt_rc = unsafe {
            setsockopt(
                fd,
                SOL_CAN_RAW,
                CAN_RAW_FD_FRAMES,
                (&enable_fd as *const c_int).cast::<c_void>(),
                size_of::<c_int>() as c_uint,
            )
        };
        if sockopt_rc < 0 {
            let _ = unsafe { close(fd) };
            return Err(last_os_error(
                "setsockopt(CAN_RAW_FD_FRAMES) failed",
                Some(interface),
            ));
        }

        let addr = SockAddrCan {
            can_family: AF_CAN as u16,
            can_ifindex: index as c_int,
            can_addr: [0u8; 8],
        };

        let bind_rc = unsafe {
            bind(
                fd,
                (&addr as *const SockAddrCan).cast::<c_void>(),
                size_of::<SockAddrCan>() as u32,
            )
        };
        if bind_rc < 0 {
            let _ = unsafe { close(fd) };
            return Err(last_os_error(
                &format!("bind on {interface} failed"),
                Some(interface),
            ));
        }

        Ok(Self {
            fd: Mutex::new(Some(fd)),
            interface: interface.to_string(),
            enable_brs,
        })
    }

    fn with_fd<T>(&self, f: impl FnOnce(RawFd) -> Result<T>) -> Result<T> {
        let guard = self
            .fd
            .lock()
            .map_err(|_| MotorError::Io("socket fd lock poisoned".to_string()))?;
        let fd = guard
            .as_ref()
            .copied()
            .ok_or_else(|| MotorError::Io("socket already closed".to_string()))?;
        f(fd)
    }
}

impl CanBus for SocketCanFdBus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        self.with_fd(|fd| {
            if !frame.is_extended && frame.arbitration_id > CAN_SFF_MASK {
                return Err(MotorError::InvalidArgument(format!(
                    "invalid arbitration_id {:X}, expected 11-bit std id",
                    frame.arbitration_id
                )));
            }
            if frame.is_extended && frame.arbitration_id > CAN_EFF_MASK {
                return Err(MotorError::InvalidArgument(format!(
                    "invalid arbitration_id {:X}, expected 29-bit ext id",
                    frame.arbitration_id
                )));
            }
            if frame.dlc > 8 {
                return Err(MotorError::InvalidArgument(format!(
                    "invalid DLC {}, expected <= 8 for current frame model",
                    frame.dlc
                )));
            }

            let mut raw = CanFdFrameRaw {
                can_id: if frame.is_extended {
                    frame.arbitration_id | CAN_EFF_FLAG
                } else {
                    frame.arbitration_id
                },
                len: frame.dlc,
                flags: if self.enable_brs { CANFD_BRS } else { 0 },
                __res0: 0,
                __res1: 0,
                data: [0u8; 64],
            };
            raw.data[..8].copy_from_slice(&frame.data);

            let n = unsafe {
                write(
                    fd,
                    (&raw as *const CanFdFrameRaw).cast::<c_void>(),
                    size_of::<CanFdFrameRaw>(),
                )
            };
            if n != size_of::<CanFdFrameRaw>() as isize {
                return Err(last_os_error(
                    "socketcanfd write failed",
                    Some(&self.interface),
                ));
            }
            Ok(())
        })
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>> {
        self.with_fd(|fd| {
            let timeout_ms = if timeout.is_zero() {
                0
            } else {
                timeout.as_millis().min(c_int::MAX as u128) as c_int
            };

            let mut pfd = PollFd {
                fd,
                events: POLLIN,
                revents: 0,
            };

            let rc = unsafe { poll(&mut pfd as *mut PollFd, 1, timeout_ms) };
            if rc < 0 {
                return Err(last_os_error("poll failed", Some(&self.interface)));
            }
            if rc == 0 {
                return Ok(None);
            }

            let mut raw_fd = CanFdFrameRaw {
                can_id: 0,
                len: 0,
                flags: 0,
                __res0: 0,
                __res1: 0,
                data: [0u8; 64],
            };

            let n = unsafe {
                read(
                    fd,
                    (&mut raw_fd as *mut CanFdFrameRaw).cast::<c_void>(),
                    size_of::<CanFdFrameRaw>(),
                )
            };

            if n == CAN_MTU as isize {
                let raw = unsafe { &*((&raw_fd as *const CanFdFrameRaw).cast::<CanFrameRaw>()) };
                return Ok(Some(CanFrame {
                    arbitration_id: if (raw.can_id & CAN_EFF_FLAG) != 0 {
                        raw.can_id & CAN_EFF_MASK
                    } else {
                        raw.can_id & CAN_SFF_MASK
                    },
                    data: raw.data,
                    dlc: raw.can_dlc.min(8),
                    is_extended: (raw.can_id & CAN_EFF_FLAG) != 0,
                    is_rx: true,
                }));
            }

            if n != CANFD_MTU as isize {
                return Err(last_os_error(
                    "socketcanfd read failed",
                    Some(&self.interface),
                ));
            }

            if raw_fd.len > 8 {
                return Err(MotorError::Unsupported(format!(
                    "received CAN-FD payload length {} > 8 is not supported by current CanFrame",
                    raw_fd.len
                )));
            }
            let mut data = [0u8; 8];
            let dlc = raw_fd.len;
            if dlc > 0 {
                data[..dlc as usize].copy_from_slice(&raw_fd.data[..dlc as usize]);
            }
            Ok(Some(CanFrame {
                arbitration_id: if (raw_fd.can_id & CAN_EFF_FLAG) != 0 {
                    raw_fd.can_id & CAN_EFF_MASK
                } else {
                    raw_fd.can_id & CAN_SFF_MASK
                },
                data,
                dlc,
                is_extended: (raw_fd.can_id & CAN_EFF_FLAG) != 0,
                is_rx: true,
            }))
        })
    }

    fn shutdown(&self) -> Result<()> {
        let mut guard = self
            .fd
            .lock()
            .map_err(|_| MotorError::Io("socket fd lock poisoned".to_string()))?;
        if let Some(fd) = guard.take() {
            let rc = unsafe { close(fd) };
            if rc < 0 {
                return Err(last_os_error(
                    "close socketcanfd fd failed",
                    Some(&self.interface),
                ));
            }
        }
        Ok(())
    }
}

impl Drop for SocketCanFdBus {
    fn drop(&mut self) {
        if let Ok(mut guard) = self.fd.lock() {
            if let Some(fd) = guard.take() {
                let _ = unsafe { close(fd) };
            }
        }
    }
}
