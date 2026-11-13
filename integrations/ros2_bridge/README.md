# ros2_bridge

<!-- channel-compat-note -->
## Channel Compatibility (PCAN + CANable candleLight/gs_usb + MCU Serial Bridge + Damiao Serial Bridge + DM_Device)

- Linux SocketCAN uses prepared interfaces directly: `can0`, `can1`. For CANable, use candleLight/gs_usb firmware so it appears as a SocketCAN interface such as `can0`.
- Use PCAN or CANable candleLight/gs_usb for standard CAN.
- Damiao-only adapter transports are available in CLI: serial bridge (`--transport dm-serial --serial-port /dev/ttyACM0 --serial-baud 921600`) and DM_Device SDK (`--transport dm-device --dm-device-type usb2canfd|usb2canfd-dual|linkx4c --dm-channel 0|1|2|3`; Damiao motors only; adapter must be in USB mode).
- Generic MCU UART-to-CAN bridge: `--transport mcu-serial --serial-port /dev/ttyACM0 --serial-baud 921600`. Note this is a different protocol from the Damiao-only `dm-serial` bridge: `mcu-serial` relays classic CAN frames through the bridge firmware, while `dm-serial` carries the Damiao serial protocol. It covers all classic-CAN vendors (damiao / robstride / myactuator / hightorque / huayi; hexfellow is CAN-FD-only and not supported); the bridge firmware proactively reports structured link status on bus-off / TX failure / RX drop, and all four frontends (motor_abi / Python / motor_cli / ws_gateway) support it.
- Full Damiao serial-bridge interface list and command patterns are documented in `motor_cli/README.md` (section `3.6` in `motor_cli/README.zh-CN.md`).
- On Linux SocketCAN, do not append bitrate in `--channel` (for example `can0@1000000` is invalid).
- On Windows (PCAN backend), `can0/can1` map to `PCAN_USBBUS1/2`; optional `@bitrate` suffix is supported.


ROS2 bridge for `motorbridge` using Python (`rclpy`) + existing `motorbridge` Python SDK bindings.

```mermaid
flowchart LR
  CMD["/motorbridge/cmd (JSON)"] --> NODE["ros2_bridge node"]
  NODE --> SDK["motorbridge Python SDK"]
  SDK --> MOTOR["Motor over CAN"]
  MOTOR --> SDK
  SDK --> NODE
  NODE --> STATE["/motorbridge/state"]
  NODE --> EVT["/motorbridge/event"]
```

## Features

- Subscribe command topic and execute motor commands
- Publish motor state periodically
- Expose operational commands via same command topic:
  - `scan`
  - `set_id`
  - `verify`

## Topics

- Subscribed: `/motorbridge/cmd` (`std_msgs/String`, JSON payload)
- Published:
  - `/motorbridge/state` (`std_msgs/String`, JSON payload)
  - `/motorbridge/event` (`std_msgs/String`, JSON payload)

## Command JSON

Control:

```json
{"op":"enable"}
{"op":"disable"}
{"op":"mit","pos":0.0,"vel":0.0,"kp":20.0,"kd":1.0,"tau":0.0,"continuous":true}
{"op":"pos_vel","pos":3.10,"vlim":1.50,"continuous":true}
{"op":"vel","vel":0.5,"continuous":true}
{"op":"force_pos","pos":0.8,"vlim":2.0,"ratio":0.3,"continuous":true}
```

Calibration:

```json
{"op":"scan","start_id":1,"end_id":16,"feedback_base":16,"timeout_ms":100}
{"op":"set_id","old_motor_id":2,"old_feedback_id":18,"new_motor_id":5,"new_feedback_id":21,"store":true,"verify":true}
{"op":"verify","motor_id":5,"feedback_id":21,"timeout_ms":1000}
```

## Run

Prerequisites:

- ROS2 installed (`rclpy`, `std_msgs`)
- `motorbridge` Python package importable

From repo root:

```bash
PYTHONPATH=bindings/python/src:$PYTHONPATH \
python3 integrations/ros2_bridge/python/motorbridge_ros2_bridge.py \
  --channel can0 --model 4340P --motor-id 0x01 --feedback-id 0x11 --dt-ms 20
```

## Publish command examples

```bash
ros2 topic pub --once /motorbridge/cmd std_msgs/msg/String '{data: "{\"op\":\"enable\"}"}'
ros2 topic pub --once /motorbridge/cmd std_msgs/msg/String '{data: "{\"op\":\"vel\",\"vel\":0.5,\"continuous\":true}"}'
ros2 topic pub --once /motorbridge/cmd std_msgs/msg/String '{data: "{\"op\":\"scan\",\"start_id\":1,\"end_id\":16}"}'
```

## Notes

- `continuous=true` means command is resent at each timer tick (`--dt-ms`).
- `enable/disable/scan/set_id/verify` are one-shot operations.
