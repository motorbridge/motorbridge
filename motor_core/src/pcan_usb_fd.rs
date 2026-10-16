//! Native userspace backend for PEAK "uCAN" USB adapters (PCAN-USB FD,
//! PCAN-USB Pro FD, PCAN-USB X6 and compatible clones) on top of libusb.
//!
//! Unlike the PCBUSB runtime on macOS, every channel of a multi-channel
//! adapter is reachable: all channels share one USB handle and one RX
//! endpoint, and `can0`/`can1` map to the first/second port of the same
//! device.
//!
//! The protocol follows the Linux kernel driver
//! `drivers/net/can/usb/peak_usb/pcan_usb_fd.c` (used as documentation only;
//! this is an independent implementation).
//!
//! Selected with the `peakusb:` channel prefix (see `bus::open_can_bus`).
//! Channel strings after the prefix: `can0`, `can1@500000`, `0`, `1@1000000`,
//! and for CAN FD `can0@1000000/5000000` (nominal/data bitrate). Default nominal bitrate is
//! 1 Mbit/s; [`PcanUsbFdBus::open_fd`] defaults the data bitrate to 5 Mbit/s.
//! The `CanBus` trait carries 8-byte frames, so CAN FD frames are limited to
//! 8 data bytes (enough for e.g. Damiao motors in CAN FD mode).
//!
//! Bit timing uses a 75 % nominal sample point (the Linux default at 1 Mbit/s).
//! This matters for CAN FD: with 80 %, a receiver switches to the data bitrate
//! ~50 ns later than a 75 % transmitter, which lost most 5 Mbit/s replies from
//! an 8-motor Damiao bus.

use crate::bus::{CanBus, CanFrame};
use crate::error::{MotorError, Result};
use rusb::{Context, DeviceHandle, UsbContext};
use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex, OnceLock, Weak};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const PEAK_VENDOR_ID: u16 = 0x0c72;
/// (product id, number of CAN channels)
const SUPPORTED_PRODUCTS: &[(u16, u8)] = &[
    (0x0012, 1), // PCAN-USB FD
    (0x0011, 2), // PCAN-USB Pro FD
    (0x0013, 1), // PCAN-Chip USB
    (0x0014, 2), // PCAN-USB X6 (per USB interface)
];
const MAX_CHANNELS: usize = 2;
const CLOCK_HZ: u32 = 80_000_000;

// vendor control requests (pcan_usb_pro.h)
const REQ_TYPE_VENDOR_OTHER: u8 = 0x43;
const REQ_INFO: u8 = 0;
const REQ_FCT: u8 = 2;
const INFO_FW: u16 = 1;
const FCT_DRVLD: u16 = 5;

// uCAN commands
const CMD_RESET_MODE: u16 = 0x001;
const CMD_NORMAL_MODE: u16 = 0x002;
const CMD_TIMING_SLOW: u16 = 0x004;
const CMD_TIMING_FAST: u16 = 0x005;
const CMD_FILTER_STD: u16 = 0x008;
const CMD_WR_ERR_CNT: u16 = 0x00a;
const CMD_SET_EN_OPTION: u16 = 0x00b;
const CMD_CLR_DIS_OPTION: u16 = 0x00c;
const CMD_CLK_SET: u16 = 0x080;
const OPTION_ERROR: u16 = 0x0001;
const OPTION_CANFD_ISO: u16 = 0x0004;
const FLTEXT_CALIBRATION: u16 = 0x8000;

// uCAN messages
const MSG_CAN_RX: u16 = 0x0001;
const MSG_STATUS: u16 = 0x0003;
const MSG_CAN_TX: u16 = 0x1000;
const FLAG_RTR: u16 = 0x01;
const FLAG_EXT_ID: u16 = 0x02;
const FLAG_LOOPED_BACK: u16 = 0x04;
const FLAG_EXT_DATA_LEN: u16 = 0x10;
const FLAG_BRS: u16 = 0x20;
const STATUS_BUSOFF: u8 = 0x80;

const USB_TIMEOUT: Duration = Duration::from_millis(1000);
const RX_POLL: Duration = Duration::from_millis(100);
const RX_QUEUE_LIMIT: usize = 4096;
const RECONNECT_BACKOFF_MS: [u64; 3] = [20, 100, 300];
const DEFAULT_DATA_BITRATE: u32 = 5_000_000;
const SAMPLE_POINT: f64 = 0.75;
/// Set to 0/false/off to send CAN FD frames without bit-rate switch
/// (same variable as the Linux SocketCAN FD backend; default here: on).
const BRS_ENV: &str = "MOTOR_SOCKETCANFD_BRS";

/// CAN bit timing in time quanta of the 80 MHz adapter clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BitTiming {
    pub brp: u16,
    pub tseg1: u16,
    pub tseg2: u16,
    pub sjw: u16,
}

impl BitTiming {
    /// Nominal (arbitration) bit timing: smallest exact prescaler, 75 % sample point.
    pub fn for_bitrate(bitrate: u32) -> Result<Self> {
        Self::search(bitrate, 256, 128)
    }

    /// CAN FD data-phase bit timing (narrower register fields), 75 % sample point.
    pub fn for_data_bitrate(bitrate: u32) -> Result<Self> {
        Self::search(bitrate, 32, 16)
    }

    fn search(bitrate: u32, max_tseg1: u32, max_tseg2: u32) -> Result<Self> {
        if bitrate == 0 {
            return Err(MotorError::InvalidArgument("bitrate must be > 0".into()));
        }
        for brp in 1u32..=1024 {
            if !CLOCK_HZ.is_multiple_of(brp * bitrate) {
                continue;
            }
            let ntq = CLOCK_HZ / (brp * bitrate);
            let tseg1 = ((ntq as f64) * SAMPLE_POINT).round() as u32 - 1;
            let tseg2 = ntq - 1 - tseg1;
            if (1..=max_tseg1).contains(&tseg1) && (1..=max_tseg2).contains(&tseg2) {
                return Ok(Self {
                    brp: brp as u16,
                    tseg1: tseg1 as u16,
                    tseg2: tseg2 as u16,
                    sjw: tseg2.min(16) as u16,
                });
            }
        }
        Err(MotorError::InvalidArgument(format!(
            "bitrate {bitrate} cannot be reached with an 80 MHz clock"
        )))
    }
}

fn opcode_channel(channel: u8, opcode: u16) -> u16 {
    (u16::from(channel & 0xf) << 12) | (opcode & 0x3ff)
}

/// One 8-byte uCAN command record.
fn cmd(channel: u8, opcode: u16, payload: &[u8]) -> [u8; 8] {
    let mut rec = [0u8; 8];
    rec[..2].copy_from_slice(&opcode_channel(channel, opcode).to_le_bytes());
    rec[2..2 + payload.len()].copy_from_slice(payload);
    rec
}

fn cmd_timing_slow(channel: u8, bt: BitTiming) -> [u8; 8] {
    let brp = ((bt.brp - 1) & 0x3ff).to_le_bytes();
    cmd(
        channel,
        CMD_TIMING_SLOW,
        &[
            96, // error warning limit (driver default)
            ((bt.sjw - 1) & 0x7f) as u8,
            ((bt.tseg2 - 1) & 0x7f) as u8,
            ((bt.tseg1 - 1) & 0xff) as u8,
            brp[0],
            brp[1],
        ],
    )
}

fn cmd_timing_fast(channel: u8, bt: BitTiming) -> [u8; 8] {
    let brp = ((bt.brp - 1) & 0x3ff).to_le_bytes();
    cmd(
        channel,
        CMD_TIMING_FAST,
        &[
            0,
            ((bt.sjw - 1) & 0xf) as u8,
            ((bt.tseg2 - 1) & 0xf) as u8,
            ((bt.tseg1 - 1) & 0x1f) as u8,
            brp[0],
            brp[1],
        ],
    )
}

fn cmd_options(channel: u8, enable: bool, ucan_mask: u16, usb_mask: u16) -> [u8; 8] {
    let op = if enable {
        CMD_SET_EN_OPTION
    } else {
        CMD_CLR_DIS_OPTION
    };
    let mut p = [0u8; 6];
    p[..2].copy_from_slice(&ucan_mask.to_le_bytes());
    p[4..].copy_from_slice(&usb_mask.to_le_bytes());
    cmd(channel, op, &p)
}

fn cmd_reset_err_counters(channel: u8) -> [u8; 8] {
    cmd(channel, CMD_WR_ERR_CNT, &[0x00, 0xc0, 0, 0])
}

/// Encode one frame (classic, or CAN FD with up to 8 bytes) as a uCAN TX
/// record, followed by the 4-byte null terminator the firmware expects.
pub fn encode_tx(channel: u8, frame: &CanFrame, fd: bool, brs: bool) -> Result<Vec<u8>> {
    if frame.dlc > 8 {
        return Err(MotorError::InvalidArgument(format!(
            "invalid DLC {}, expected <= 8",
            frame.dlc
        )));
    }
    let len = usize::from(frame.dlc);
    let size = (20 + len + 3) & !3;
    let (mut flags, id) = if frame.is_extended {
        (FLAG_EXT_ID, frame.arbitration_id & 0x1fff_ffff)
    } else {
        (0, frame.arbitration_id & 0x7ff)
    };
    if fd {
        flags |= FLAG_EXT_DATA_LEN; // DLC 0..8 encodes the same length in CAN FD
        if brs {
            flags |= FLAG_BRS;
        }
    }
    let mut out = Vec::with_capacity(size + 4);
    out.extend_from_slice(&(size as u16).to_le_bytes());
    out.extend_from_slice(&MSG_CAN_TX.to_le_bytes());
    out.extend_from_slice(&[0u8; 8]); // tag
    out.push((channel & 0xf) | (frame.dlc << 4));
    out.push(0); // client
    out.extend_from_slice(&flags.to_le_bytes());
    out.extend_from_slice(&id.to_le_bytes());
    out.extend_from_slice(&frame.data[..len]);
    out.resize(size + 4, 0);
    Ok(out)
}

/// A decoded record from the RX endpoint.
#[derive(Debug, Clone, Copy)]
pub enum RxEvent {
    Frame { channel: u8, frame: CanFrame },
    BusOff { channel: u8 },
}

/// Decode all records of one RX transfer. Unknown record types (timestamp
/// calibration, error counters, ...) are skipped; so are CAN FD frames longer
/// than 8 bytes and our own looped-back TX frames.
pub fn decode_rx(buf: &[u8]) -> Vec<RxEvent> {
    let mut events = Vec::new();
    let mut pos = 0;
    while pos + 4 <= buf.len() {
        let size = usize::from(u16::from_le_bytes([buf[pos], buf[pos + 1]]));
        let typ = u16::from_le_bytes([buf[pos + 2], buf[pos + 3]]);
        if size == 0 || pos + size > buf.len() {
            break;
        }
        let rec = &buf[pos..pos + size];
        match typ {
            MSG_CAN_RX if size >= 28 => {
                let channel = rec[20] & 0xf;
                let dlc = rec[20] >> 4;
                let flags = u16::from_le_bytes([rec[22], rec[23]]);
                let can_id = u32::from_le_bytes([rec[24], rec[25], rec[26], rec[27]]);
                let fd = flags & FLAG_EXT_DATA_LEN != 0;
                let len = usize::from(dlc.min(8));
                if flags & FLAG_LOOPED_BACK == 0 && !(fd && dlc > 8) {
                    let mut data = [0u8; 8];
                    if flags & FLAG_RTR == 0 && rec.len() >= 28 + len {
                        data[..len].copy_from_slice(&rec[28..28 + len]);
                    }
                    events.push(RxEvent::Frame {
                        channel,
                        frame: CanFrame {
                            arbitration_id: can_id,
                            data,
                            dlc: len as u8,
                            is_extended: flags & FLAG_EXT_ID != 0,
                            is_rx: true,
                        },
                    });
                }
            }
            MSG_STATUS if size >= 16 && rec[12] & STATUS_BUSOFF != 0 => {
                events.push(RxEvent::BusOff {
                    channel: rec[12] & 0xf,
                });
            }
            _ => {}
        }
        pos += size;
    }
    events
}

/// Parse `can1@500000`, `1`, `can0`, `can0@1000000/5000000` into
/// (channel index, nominal bitrate, optional CAN FD data bitrate).
pub fn parse_channel(spec: &str) -> Result<(u8, u32, Option<u32>)> {
    let parse = |s: &str| {
        s.parse::<u32>().map_err(|e| {
            MotorError::InvalidArgument(format!("invalid bitrate in channel '{spec}': {e}"))
        })
    };
    let (name, bitrate, data_bitrate) = match spec.split_once('@') {
        Some((n, rates)) => match rates.split_once('/') {
            Some((b, d)) => (n, parse(b)?, Some(parse(d)?)),
            None => (n, parse(rates)?, None),
        },
        None => (spec, 1_000_000, None),
    };
    let lower = name.trim().to_ascii_lowercase();
    let idx = lower.strip_prefix("can").unwrap_or(&lower);
    let channel = idx.parse::<u8>().map_err(|_| {
        MotorError::InvalidArgument(format!(
            "unsupported channel '{spec}', use can0/can1 (optionally @bitrate)"
        ))
    })?;
    if usize::from(channel) >= MAX_CHANNELS {
        return Err(MotorError::InvalidArgument(format!(
            "channel index {channel} out of range (0..{MAX_CHANNELS})"
        )));
    }
    Ok((channel, bitrate, data_bitrate))
}

fn usb_err(ctx: &str, e: rusb::Error) -> MotorError {
    MotorError::Io(format!("pcan-usb-fd {ctx}: {e}"))
}

#[derive(Default)]
struct ChannelState {
    open: bool,
    queue: VecDeque<CanFrame>,
}

/// State shared by all open channels of one adapter.
struct Device {
    handle: DeviceHandle<Context>,
    channel_count: u8,
    ep_cmd_out: u8,
    ep_data_in: u8,
    ep_data_out: [u8; MAX_CHANNELS],
    fw_major: u8,
    cmd_lock: Mutex<()>,
    channels: [(Mutex<ChannelState>, Condvar); MAX_CHANNELS],
    running: AtomicBool,
    rx_thread: Mutex<Option<JoinHandle<()>>>,
}

static DEVICE: OnceLock<Mutex<Weak<Device>>> = OnceLock::new();

impl Device {
    fn get_or_open() -> Result<Arc<Self>> {
        let slot = DEVICE.get_or_init(|| Mutex::new(Weak::new()));
        let mut slot = slot
            .lock()
            .map_err(|_| MotorError::Io("pcan-usb-fd registry lock poisoned".into()))?;
        if let Some(dev) = slot.upgrade() {
            return Ok(dev);
        }
        let dev = Arc::new(Self::open()?);
        let worker = Arc::downgrade(&dev);
        let handle = thread::Builder::new()
            .name("pcan-usb-fd-rx".into())
            .spawn(move || rx_loop(worker))
            .map_err(|e| MotorError::Io(format!("spawn rx thread: {e}")))?;
        *dev.rx_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
        *slot = Arc::downgrade(&dev);
        Ok(dev)
    }

    fn open() -> Result<Self> {
        let ctx = Context::new().map_err(|e| usb_err("libusb init", e))?;
        let devices = ctx.devices().map_err(|e| usb_err("list devices", e))?;
        let (device, channel_count) = devices
            .iter()
            .find_map(|d| {
                let desc = d.device_descriptor().ok()?;
                if desc.vendor_id() != PEAK_VENDOR_ID {
                    return None;
                }
                SUPPORTED_PRODUCTS
                    .iter()
                    .find(|(pid, _)| *pid == desc.product_id())
                    .map(|(_, n)| (d.clone(), *n))
            })
            .ok_or_else(|| MotorError::Io("no PEAK PCAN-USB FD family adapter found".into()))?;
        let handle = device.open().map_err(|e| usb_err("open", e))?;
        let _ = handle.set_auto_detach_kernel_driver(true);
        handle
            .claim_interface(0)
            .map_err(|e| usb_err("claim interface", e))?;

        // firmware info: versions and endpoint numbers (type >= 2)
        let mut info = [0u8; 36];
        let n = handle
            .read_control(
                REQ_TYPE_VENDOR_OTHER | 0x80,
                REQ_INFO,
                INFO_FW,
                0,
                &mut info,
                USB_TIMEOUT,
            )
            .map_err(|e| usb_err("read firmware info", e))?;
        let info_type = u16::from_le_bytes([info[2], info[3]]);
        let fw_major = info[9];
        let (ep_cmd_out, ep_data_in, ep_data_out) = if info_type >= 2 && n >= 33 {
            (info[28], info[32], [info[30], info[31]])
        } else {
            (0x01, 0x82, [0x02, 0x03])
        };

        // tell the firmware a driver is running
        let mut drvld = [0u8; 16];
        drvld[1] = 1;
        handle
            .write_control(
                REQ_TYPE_VENDOR_OTHER,
                REQ_FCT,
                FCT_DRVLD,
                0,
                &drvld,
                USB_TIMEOUT,
            )
            .map_err(|e| usb_err("driver-loaded request", e))?;

        Ok(Self {
            handle,
            channel_count,
            ep_cmd_out,
            ep_data_in,
            ep_data_out,
            fw_major,
            cmd_lock: Mutex::new(()),
            channels: Default::default(),
            running: AtomicBool::new(true),
            rx_thread: Mutex::new(None),
        })
    }

    /// Send a list of command records, terminated by an end-of-collection record.
    fn send_cmds(&self, records: &[[u8; 8]]) -> Result<()> {
        let mut buf: Vec<u8> = records.iter().flatten().copied().collect();
        if buf.len() <= 512 - 8 {
            buf.extend_from_slice(&[0xff; 8]);
        }
        let _g = self.cmd_lock.lock().unwrap_or_else(|p| p.into_inner());
        for chunk in buf.chunks(512) {
            self.handle
                .write_bulk(self.ep_cmd_out, chunk, USB_TIMEOUT)
                .map_err(|e| usb_err("send command", e))?;
        }
        Ok(())
    }

    fn start_channel(
        &self,
        channel: u8,
        bt: BitTiming,
        dbt: Option<BitTiming>,
        first: bool,
    ) -> Result<()> {
        let mut setup = vec![
            cmd(channel, CMD_RESET_MODE, &[]),
            cmd(channel, CMD_CLK_SET, &[0]), // 80 MHz
            cmd_timing_slow(channel, bt),
        ];
        if let Some(dbt) = dbt {
            setup.push(cmd_timing_fast(channel, dbt));
        }
        for row in 0u16..64 {
            let mut p = [0u8; 6];
            p[..2].copy_from_slice(&row.to_le_bytes());
            p[2..].copy_from_slice(&u32::MAX.to_le_bytes());
            setup.push(cmd(channel, CMD_FILTER_STD, &p)); // accept all 11-bit ids
        }
        self.send_cmds(&setup)?;

        let usb_mask = if first { FLTEXT_CALIBRATION } else { 0 };
        let mut run = vec![
            cmd_options(channel, true, OPTION_ERROR, usb_mask),
            cmd_reset_err_counters(channel),
        ];
        if self.fw_major >= 2 {
            run.push(cmd_options(channel, true, OPTION_CANFD_ISO, 0));
        }
        run.push(cmd(channel, CMD_NORMAL_MODE, &[]));
        self.send_cmds(&run)
    }

    fn restart_channel(&self, channel: u8) {
        let _ = self.send_cmds(&[
            cmd_reset_err_counters(channel),
            cmd(channel, CMD_NORMAL_MODE, &[]),
        ]);
    }

    fn any_open(&self) -> bool {
        self.channels
            .iter()
            .any(|(s, _)| s.lock().map(|s| s.open).unwrap_or(false))
    }
}

impl Drop for Device {
    fn drop(&mut self) {
        self.running.store(false, Ordering::SeqCst);
        let _ = self.send_cmds(&[cmd_options(0, false, OPTION_ERROR, FLTEXT_CALIBRATION)]);
        let _ = self.handle.write_control(
            REQ_TYPE_VENDOR_OTHER,
            REQ_FCT,
            FCT_DRVLD,
            0,
            &[0u8; 16],
            USB_TIMEOUT,
        );
        let _ = self.handle.release_interface(0);
    }
}

fn rx_loop(dev: Weak<Device>) {
    let mut buf = vec![0u8; 2048];
    loop {
        let Some(d) = dev.upgrade() else { return };
        if !d.running.load(Ordering::SeqCst) {
            return;
        }
        match d.handle.read_bulk(d.ep_data_in, &mut buf, RX_POLL) {
            Ok(n) => {
                for ev in decode_rx(&buf[..n]) {
                    match ev {
                        RxEvent::Frame { channel, frame } => {
                            if let Some((state, cv)) = d.channels.get(usize::from(channel)) {
                                let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
                                if s.open {
                                    if s.queue.len() >= RX_QUEUE_LIMIT {
                                        s.queue.pop_front();
                                    }
                                    s.queue.push_back(frame);
                                    cv.notify_all();
                                }
                            }
                        }
                        RxEvent::BusOff { channel } => {
                            if usize::from(channel) < MAX_CHANNELS {
                                d.restart_channel(channel);
                            }
                        }
                    }
                }
            }
            Err(rusb::Error::Timeout) => {}
            Err(rusb::Error::NoDevice) => {
                d.running.store(false, Ordering::SeqCst);
                for (_, cv) in &d.channels {
                    cv.notify_all();
                }
                return;
            }
            Err(_) => thread::sleep(Duration::from_millis(10)),
        }
        // drop the strong ref before the next blocking read
    }
}

/// One CAN channel of a PEAK uCAN adapter.
pub struct PcanUsbFdBus {
    dev: Arc<Device>,
    channel: u8,
    bt: BitTiming,
    dbt: Option<BitTiming>,
    fd: bool,
    brs: bool,
    active: AtomicBool,
}

fn brs_from_env() -> bool {
    std::env::var(BRS_ENV)
        .map(|v| {
            !matches!(
                v.trim().to_ascii_lowercase().as_str(),
                "0" | "false" | "off" | "no"
            )
        })
        .unwrap_or(true)
}

impl PcanUsbFdBus {
    /// Classic CAN, or CAN FD if the spec carries a data bitrate (`can0@1000000/5000000`).
    pub fn open(spec: &str) -> Result<Self> {
        let (_, _, data) = parse_channel(spec)?;
        Self::open_with(spec, data.is_some())
    }

    /// CAN FD (data bitrate from the spec, default 5 Mbit/s). FD frames are sent
    /// with bit-rate switch unless `MOTOR_SOCKETCANFD_BRS=0`.
    pub fn open_fd(spec: &str) -> Result<Self> {
        Self::open_with(spec, true)
    }

    fn open_with(spec: &str, fd: bool) -> Result<Self> {
        let (channel, bitrate, data_bitrate) = parse_channel(spec)?;
        let bt = BitTiming::for_bitrate(bitrate)?;
        let dbt = if fd {
            Some(BitTiming::for_data_bitrate(
                data_bitrate.unwrap_or(DEFAULT_DATA_BITRATE),
            )?)
        } else {
            None
        };
        let dev = Device::get_or_open()?;
        if channel >= dev.channel_count {
            return Err(MotorError::InvalidArgument(format!(
                "adapter has {} channel(s), can{channel} requested",
                dev.channel_count
            )));
        }
        let first = !dev.any_open();
        {
            let (state, _) = &dev.channels[usize::from(channel)];
            let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
            if s.open {
                return Err(MotorError::InvalidArgument(format!(
                    "can{channel} is already open"
                )));
            }
            s.open = true;
            s.queue.clear();
        }
        if let Err(e) = dev.start_channel(channel, bt, dbt, first) {
            dev.channels[usize::from(channel)]
                .0
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .open = false;
            return Err(e);
        }
        Ok(Self {
            dev,
            channel,
            bt,
            dbt,
            fd,
            brs: fd && brs_from_env(),
            active: AtomicBool::new(true),
        })
    }

    /// Re-initialise this channel (reset, bit timing, normal mode) with backoff.
    /// An unplugged adapter cannot be recovered here: reopen the bus instead.
    fn reconnect(&self) -> Result<()> {
        for backoff in RECONNECT_BACKOFF_MS {
            if self.dev.running.load(Ordering::SeqCst)
                && self
                    .dev
                    .start_channel(self.channel, self.bt, self.dbt, false)
                    .is_ok()
            {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(backoff));
        }
        Err(MotorError::Io(format!(
            "pcan-usb-fd can{} reconnect failed after {} attempts",
            self.channel,
            RECONNECT_BACKOFF_MS.len()
        )))
    }
}

impl CanBus for PcanUsbFdBus {
    fn send(&self, frame: CanFrame) -> Result<()> {
        if !self.active.load(Ordering::SeqCst) {
            return Err(MotorError::Io("pcan-usb-fd bus is already closed".into()));
        }
        let pkt = encode_tx(self.channel, &frame, self.fd, self.brs)?;
        let ep = self.dev.ep_data_out[usize::from(self.channel)];
        let mut last_err = None;
        for _ in 0..=RECONNECT_BACKOFF_MS.len() {
            match self.dev.handle.write_bulk(ep, &pkt, USB_TIMEOUT) {
                Ok(_) => return Ok(()),
                Err(e) => last_err = Some(e),
            }
            self.reconnect()?;
        }
        Err(usb_err(
            "send frame after reconnect retries",
            last_err.unwrap_or(rusb::Error::Other),
        ))
    }

    fn recv(&self, timeout: Duration) -> Result<Option<CanFrame>> {
        if !self.active.load(Ordering::SeqCst) {
            return Err(MotorError::Io("pcan-usb-fd bus is already closed".into()));
        }
        let (state, cv) = &self.dev.channels[usize::from(self.channel)];
        let deadline = Instant::now().checked_add(timeout);
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        loop {
            if let Some(f) = s.queue.pop_front() {
                return Ok(Some(f));
            }
            if !self.dev.running.load(Ordering::SeqCst) {
                return Err(MotorError::Io("pcan-usb-fd adapter disconnected".into()));
            }
            let remaining = match deadline {
                Some(d) => d.saturating_duration_since(Instant::now()),
                None => Duration::from_secs(3600),
            };
            if remaining.is_zero() {
                return Ok(None);
            }
            s = cv
                .wait_timeout(s, remaining)
                .unwrap_or_else(|p| p.into_inner())
                .0;
        }
    }

    fn shutdown(&self) -> Result<()> {
        if !self.active.swap(false, Ordering::SeqCst) {
            return Ok(());
        }
        let _ = self
            .dev
            .send_cmds(&[cmd(self.channel, CMD_RESET_MODE, &[])]);
        let (state, cv) = &self.dev.channels[usize::from(self.channel)];
        let mut s = state.lock().unwrap_or_else(|p| p.into_inner());
        s.open = false;
        s.queue.clear();
        cv.notify_all();
        Ok(())
    }
}

impl Drop for PcanUsbFdBus {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bit_timing_common_bitrates() {
        let bt = BitTiming::for_bitrate(1_000_000).unwrap();
        assert_eq!(
            bt,
            BitTiming {
                brp: 1,
                tseg1: 59,
                tseg2: 20,
                sjw: 16
            }
        ); // 75 % sample point
        let dbt = BitTiming::for_data_bitrate(5_000_000).unwrap();
        assert_eq!(
            dbt,
            BitTiming {
                brp: 1,
                tseg1: 11,
                tseg2: 4,
                sjw: 4
            }
        );
        for br in [500_000, 250_000, 125_000, 800_000, 50_000] {
            let bt = BitTiming::for_bitrate(br).unwrap();
            let ntq = 1 + u32::from(bt.tseg1) + u32::from(bt.tseg2);
            assert_eq!(CLOCK_HZ / (u32::from(bt.brp) * ntq), br);
        }
    }

    #[test]
    fn timing_slow_record_layout() {
        let bt = BitTiming::for_bitrate(1_000_000).unwrap();
        // opcode 0x004 on channel 1 -> 0x1004 LE; ewl, sjw-1, tseg2-1, tseg1-1, brp-1 LE
        assert_eq!(cmd_timing_slow(1, bt), [0x04, 0x10, 96, 15, 19, 58, 0, 0]);
        let dbt = BitTiming::for_data_bitrate(5_000_000).unwrap();
        // opcode 0x005 on channel 0; unused, sjw-1, tseg2-1, tseg1-1, brp-1 LE
        assert_eq!(cmd_timing_fast(0, dbt), [0x05, 0x00, 0, 3, 3, 10, 0, 0]);
    }

    #[test]
    fn encode_classic_frame() {
        let frame = CanFrame {
            arbitration_id: 0x7ff,
            data: [0x07, 0x00, 0xcc, 0, 0, 0, 0, 0],
            dlc: 8,
            is_extended: false,
            is_rx: false,
        };
        let pkt = encode_tx(1, &frame, false, false).unwrap();
        assert_eq!(pkt.len(), 28 + 4);
        assert_eq!(&pkt[14..16], &[0, 0]); // classic: no flags
        let fd = encode_tx(1, &frame, true, true).unwrap();
        assert_eq!(&fd[14..16], &(FLAG_EXT_DATA_LEN | FLAG_BRS).to_le_bytes());
        assert_eq!(fd[12], 0x81); // same DLC code for 8 bytes
        let fd_nobrs = encode_tx(1, &frame, true, false).unwrap();
        assert_eq!(&fd_nobrs[14..16], &FLAG_EXT_DATA_LEN.to_le_bytes());
        assert_eq!(&pkt[..4], &[28, 0, 0x00, 0x10]);
        assert_eq!(pkt[12], 0x81); // channel 1, dlc 8
        assert_eq!(&pkt[16..20], &0x7ffu32.to_le_bytes());
        assert_eq!(&pkt[20..23], &[0x07, 0x00, 0xcc]);
        assert_eq!(&pkt[28..], &[0, 0, 0, 0]);
    }

    fn rx_record(channel: u8, id: u32, data: &[u8], flags: u16) -> Vec<u8> {
        let size = (28 + data.len() + 3) & !3;
        let mut r = vec![0u8; size];
        r[..2].copy_from_slice(&(size as u16).to_le_bytes());
        r[2..4].copy_from_slice(&MSG_CAN_RX.to_le_bytes());
        r[20] = channel | ((data.len() as u8) << 4);
        r[22..24].copy_from_slice(&flags.to_le_bytes());
        r[24..28].copy_from_slice(&id.to_le_bytes());
        r[28..28 + data.len()].copy_from_slice(data);
        r
    }

    #[test]
    fn decode_frames_status_and_skips() {
        let mut buf = rx_record(1, 0x17, &[7, 0x7e, 0x3b, 0x7f, 0xf8, 0, 0x1c, 0x1b], 0);
        buf.extend(rx_record(0, 0x18, &[1, 2], FLAG_LOOPED_BACK)); // own echo: skipped
        let mut status = vec![0u8; 16];
        status[..2].copy_from_slice(&16u16.to_le_bytes());
        status[2..4].copy_from_slice(&MSG_STATUS.to_le_bytes());
        status[12] = STATUS_BUSOFF | 1;
        buf.extend(status);
        let mut calib = vec![0u8; 16];
        calib[..2].copy_from_slice(&16u16.to_le_bytes());
        calib[2..4].copy_from_slice(&0x0100u16.to_le_bytes());
        buf.extend(calib);
        buf.extend([0u8; 4]); // end of list

        let ev = decode_rx(&buf);
        assert_eq!(ev.len(), 2);
        match ev[0] {
            RxEvent::Frame { channel, frame } => {
                assert_eq!(channel, 1);
                assert_eq!(frame.arbitration_id, 0x17);
                assert_eq!(frame.dlc, 8);
                assert_eq!(frame.data[0], 7);
                assert!(frame.is_rx);
            }
            _ => panic!("expected frame"),
        }
        assert!(matches!(ev[1], RxEvent::BusOff { channel: 1 }));
    }

    #[test]
    fn parse_channel_forms() {
        assert_eq!(parse_channel("can0").unwrap(), (0, 1_000_000, None));
        assert_eq!(parse_channel("can1@500000").unwrap(), (1, 500_000, None));
        assert_eq!(parse_channel("1").unwrap(), (1, 1_000_000, None));
        assert_eq!(
            parse_channel("can1@1000000/5000000").unwrap(),
            (1, 1_000_000, Some(5_000_000))
        );
        assert!(parse_channel("can0@1000000/x").is_err());
        assert!(parse_channel("can2").is_err());
        assert!(parse_channel("canX").is_err());
        assert!(parse_channel("can0@fast").is_err());
    }
}
