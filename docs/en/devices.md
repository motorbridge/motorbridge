# Supported Devices

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


## Support Landscape

```mermaid
mindmap
  root((motorbridge devices))
    Production
      Damiao
        3507
        4310 / 4310P
        4340 / 4340P
        6006 / 8006 / 8009
        10010 / 10010L
        H3510 / G6215 / H6220 / JH11 / 6248P
        Modes
          MIT / POS_VEL / VEL / FORCE_POS
      RobStride
        rs-00 / rs-01 / rs-02
        rs-03 / rs-04 / rs-05 / rs-06
        Modes
          MIT / POS_VEL / VEL / ping / enable-disable / parameter read-write / set-id / zero
      MyActuator
        X-series (ID based)
        Modes
          enable / disable / stop / status / current / vel / pos
      HighTorque
        hightorque (native ht_can v1.5.5)
        Modes
          scan / read / mit / pos-vel / vel / stop
    Template
      template_vendor
        model_a
```

## Production Support

| Brand | Models | Control Modes | Register R/W | ABI Coverage | Notes |
|---|---|---|---|---|---|
| Damiao | 3507, 4310, 4310P, 4340, 4340P, 6006, 8006, 8009, 10010L, 10010, H3510, G6215, H6220, JH11, 6248P | scan, enable, disable, MIT, POS_VEL, VEL, FORCE_POS, set-id, set-zero | Yes (f32/u32) | Yes | Production baseline; use `--store 1 --verify-id 1` for ID updates |
| RobStride | rs-00, rs-01, rs-02, rs-03, rs-04, rs-05, rs-05el, rs-06 | scan, ping, enable, disable, MIT, POS_VEL, VEL, parameter read/write, set-id, zero | Yes (i8/u8/u16/u32/f32) | Yes | Uses 29-bit extended CAN IDs; default host/feedback ID `0xFD`; set-id aligned to upper-tool frame layout |
| MyActuator | X-series (runtime model string, default `X8`) | enable, disable, stop, status, current, vel, pos, version, mode-query | No (CLI command-level support) | Yes | Uses standard 11-bit IDs `0x140+id` / `0x240+id`; practical ID range 1..32 |
| HighTorque | hightorque (runtime model string; native `ht_can v1.5.5`) | scan, read, MIT, POS_VEL, VEL, stop, brake, rezero | No (vendor command-level support) | Yes | Unified `rad/rad/s/Nm` interface; native payload scaling handled internally |

## Template (Not Production)

| Brand | Models | Control Modes | Register R/W | ABI Coverage | Notes |
|---|---|---|---|---|---|
| template_vendor | model_a (placeholder) | Placeholder only | Placeholder only | No | Scaffolding for new vendor integration |

## Mode Legend

- MIT: position + velocity + stiffness + damping + torque feedforward
- POS_VEL: position + velocity limit
- VEL: velocity control
- FORCE_POS: position + velocity limit + torque ratio
