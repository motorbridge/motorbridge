#ifndef MOTOR_ABI_H
#define MOTOR_ABI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MotorController MotorController;
typedef struct MotorHandle MotorHandle;

typedef struct MotorState {
  int32_t has_value;
  uint8_t can_id;
  uint32_t arbitration_id;
  uint8_t status_code;
  float pos;
  float vel;
  float torq;
  float t_mos;
  float t_rotor;
  // HighTorque: table-3 fault code (0 = no fault). Other vendors: always 0.
  uint8_t fault_code;
} MotorState;

typedef struct MotorStateMetadata {
  uint64_t feedback_age_ns;
  uint64_t feedback_sequence;
} MotorStateMetadata;

// Unified units across all vendors:
// - position: rad
// - velocity: rad/s
// - torque: Nm
//
// Unified mode IDs for motor_handle_ensure_mode:
// - 1: MIT
// - 2: POS_VEL
// - 3: VEL
// - 4: FORCE_POS
//
// Vendor support in current ABI:
// - Damiao
// - Hexfellow (CAN-FD transport via motor_controller_new_socketcanfd)
// - RobStride
// - MyActuator
// - HighTorque (native ht_can v1.5.5)
// - Huayi (CANopen over classic CAN; 6060h mode numbering: 1=PP, 3=PV, 4=torque,
//   5=MIT — unified modes 1/2/3 are mapped by the ABI, 4/FORCE_POS is rejected)

// Returns the last error message for the calling thread.
//
// Lifetime: the returned pointer is owned by the ABI and remains valid only
// until the next ABI call on the same thread changes the thread-local error
// slot. Copy the string immediately if it must be kept.
const char* motor_last_error_message(void);
const char* motor_abi_version(void);
const char* motor_abi_capabilities_json(void);

// Thread-safety:
// - Calls using the same MotorController or MotorHandle are serialized inside
//   the ABI.
// - Free functions are exclusive ownership operations: do not call
//   motor_controller_free or motor_handle_free concurrently with any other
//   operation using the same pointer, and do not use a pointer after free.
// - Different handles may be used from different threads.

MotorController* motor_controller_new_socketcan(const char* channel);
MotorController* motor_controller_new_socketcanfd(const char* channel);
MotorController* motor_controller_new_dm_serial(const char* serial_port, uint32_t baud);
MotorController* motor_controller_new_dm_device(const char* dm_device_type, const char* dm_channel);
MotorController* motor_controller_new_mcu_serial(const char* serial_port, uint32_t baud);
void motor_controller_free(MotorController* controller);
int32_t motor_controller_poll_feedback_once(MotorController* controller);
int32_t motor_controller_enable_all(MotorController* controller);
int32_t motor_controller_disable_all(MotorController* controller);
int32_t motor_controller_shutdown(MotorController* controller);
int32_t motor_controller_close_bus(MotorController* controller);

MotorHandle* motor_controller_add_damiao_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_hexfellow_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_myactuator_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_robstride_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_hightorque_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_huayi_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
void motor_handle_free(MotorHandle* motor);

int32_t motor_handle_enable(MotorHandle* motor);
int32_t motor_handle_disable(MotorHandle* motor);
int32_t motor_handle_clear_error(MotorHandle* motor);
int32_t motor_handle_set_zero_position(MotorHandle* motor);
int32_t motor_handle_ensure_mode(MotorHandle* motor, uint32_t mode, uint32_t timeout_ms);
int32_t motor_handle_stop(MotorHandle* motor);

int32_t motor_handle_send_mit(MotorHandle* motor, float target_position, float target_velocity, float stiffness, float damping, float feedforward_torque);
int32_t motor_handle_send_pos_vel(MotorHandle* motor, float target_position, float velocity_limit);
int32_t motor_handle_robstride_send_pos_vel_pp(MotorHandle* motor, float target_position, float velocity_max, float acceleration);
int32_t motor_handle_robstride_send_pos_vel_csp(MotorHandle* motor, float target_position, float velocity_limit);
int32_t motor_handle_send_vel(MotorHandle* motor, float target_velocity);
int32_t motor_handle_send_force_pos(MotorHandle* motor, float target_position, float velocity_limit, float torque_limit_ratio);

int32_t motor_handle_store_parameters(MotorHandle* motor);
// Request a feedback frame (unified op, per-vendor semantics). Hightorque: non-blocking
// since 2026-10-23 (DM semantics, aligned with the gateway op and Damiao) — sends two
// 2-byte query frames (since 2026-10-24, real-bus verified): 17 01 (pos/vel/torque) and
// 16 0E (temperature 0x0E + fault code 0x0F, consecutive addresses in one count-2 segment)
// and returns immediately; the background RX thread caches the replies — the 27 01 reply
// anchors one state count (sequence +1), the 26 0E reply patches t_mos/fault_code in place
// without bumping the count. Read it back via motor_handle_get_state /
// motor_handle_get_state_snapshot. Offline motors surface as stale/absent cached state,
// not as an error here; use motor_handle_hightorque_get_register(0x01, 3) for
// send-and-block-wait semantics.
int32_t motor_handle_request_feedback(MotorHandle* motor);
int32_t motor_handle_set_can_timeout_ms(MotorHandle* motor, uint32_t timeout_ms);

int32_t motor_handle_write_register_f32(MotorHandle* motor, uint8_t rid, float value);
int32_t motor_handle_write_register_u32(MotorHandle* motor, uint8_t rid, uint32_t value);
int32_t motor_handle_get_register_f32(MotorHandle* motor, uint8_t rid, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_get_register_u32(MotorHandle* motor, uint8_t rid, uint32_t timeout_ms, uint32_t* out_value);

int32_t motor_handle_robstride_ping(MotorHandle* motor, uint8_t* out_device_id, uint8_t* out_responder_id);
int32_t motor_handle_robstride_ping_host_id(MotorHandle* motor, uint16_t host_id, uint32_t timeout_ms, uint8_t* out_device_id, uint8_t* out_responder_id);
int32_t motor_handle_robstride_get_param_f32_host_id(MotorHandle* motor, uint16_t param_id, uint16_t host_id, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_robstride_get_fault_report(MotorHandle* motor, uint32_t* out_fault_raw, uint32_t* out_warning_raw);
int32_t motor_handle_robstride_set_device_id(MotorHandle* motor, uint8_t new_device_id);
int32_t motor_handle_robstride_set_active_report(MotorHandle* motor, uint8_t enabled);
int32_t motor_handle_robstride_write_param_i8(MotorHandle* motor, uint16_t param_id, int8_t value);
int32_t motor_handle_robstride_write_param_u8(MotorHandle* motor, uint16_t param_id, uint8_t value);
int32_t motor_handle_robstride_write_param_u16(MotorHandle* motor, uint16_t param_id, uint16_t value);
int32_t motor_handle_robstride_write_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t value);
int32_t motor_handle_robstride_write_param_f32(MotorHandle* motor, uint16_t param_id, float value);
int32_t motor_handle_robstride_get_param_i8(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, int8_t* out_value);
int32_t motor_handle_robstride_get_param_u8(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint8_t* out_value);
int32_t motor_handle_robstride_get_param_u16(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint16_t* out_value);
int32_t motor_handle_robstride_get_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint32_t* out_value);
int32_t motor_handle_robstride_get_param_f32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, float* out_value);

int32_t motor_handle_get_state(MotorHandle* motor, MotorState* out_state);
int32_t motor_handle_get_state_snapshot(MotorHandle* motor, MotorState* out_state, MotorStateMetadata* out_metadata);

int32_t motor_handle_damiao_get_param_f32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_damiao_get_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint32_t* out_value);
int32_t motor_handle_damiao_write_param_f32(MotorHandle* motor, uint16_t param_id, float value);
int32_t motor_handle_damiao_write_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t value);

// HighTorque-only extensions (reject non-HighTorque motors):
// Trapezoid control (07 2d, firmware v4.6.0+).
int32_t motor_handle_hightorque_send_pos_vel_acc(MotorHandle* motor, float target_position, float velocity_limit, float acceleration_limit);
// Classic position mode (07 07): torque limit in Nm; pass NaN for unlimited.
int32_t motor_handle_hightorque_send_pos_classic(MotorHandle* motor, float target_position, float torque_limit_nm);
// Generic register read (17 family), fixed Int16 since 2026-10-22 (ty removed):
// count 1-3 consecutive registers; every address in addr..addr+count-1 must be in
// the HighTorque parameter table (the 43 xlsx-listed registers within 0x00-0x45 —
// missing/out-of-table addresses are rejected before any frame is sent); writes
// `count` f32 physical values (pos/vel/acc in rad-family, torque via model k/d,
// current/voltage/temperature/PWM/PID per scale; unclassified registers 1:1).
int32_t motor_handle_hightorque_get_register(MotorHandle* motor, uint8_t addr, uint8_t count, uint32_t timeout_ms, float* out_values);
// Non-blocking register query (17 family, fixed Int16): send the read request and
// return immediately; the background RX thread caches the reply. Same parameter-
// table address validation as hightorque_get_register. Read it back via
// hightorque_get_cached_register (latest value, converted to physical units).
int32_t motor_handle_hightorque_request_register(MotorHandle* motor, uint8_t addr, uint8_t count);
// Read the latest cached register value (no frame sent, staleness not checked).
// Writes out_has_value=1 and the converted physical value, or out_has_value=0 when
// no reply was ever received for that address or the address is outside the
// parameter table.
int32_t motor_handle_hightorque_get_cached_register(MotorHandle* motor, uint8_t addr, uint8_t* out_has_value, float* out_value);
// Torque mode (05 13): direct torque target in Nm (model coeff compensated).
int32_t motor_handle_hightorque_send_torque(MotorHandle* motor, float torque_nm);
// DQ voltage mode (01 00 08 05 1B): Q-axis voltage in V (0.1 V/LSB).
int32_t motor_handle_hightorque_send_dq_voltage(MotorHandle* motor, float voltage_v);
// DQ current mode (01 00 09 05 1C): Q-axis current in A (0.1 A/LSB).
int32_t motor_handle_hightorque_send_dq_current(MotorHandle* motor, float current_a);
// Brake (run mode 15, 01 00 0F): all phases shorted damping brake.
int32_t motor_handle_hightorque_brake(MotorHandle* motor);
// Run mode switch (01 00 <mode>, register 0x000): mode 0-17 per protocol table 2
// (0 = stop & clear error). Sends a real frame, unlike motor_handle_ensure_mode.
int32_t motor_handle_hightorque_set_run_mode(MotorHandle* motor, uint8_t mode);
// Generic register write (05 <addr> <val16>): config_class != 0 sends via extended
// 0x8000|id frame (config registers), 0 via the control channel (control targets).
// addr must be in the HighTorque parameter table (missing/out-of-table addresses
// are rejected before any frame is sent).
int32_t motor_handle_hightorque_write_register(MotorHandle* motor, uint8_t addr, int16_t value, uint8_t config_class);
// Periodic status report (05 B4 02 <t_ms16>, extended frame): period_ms >= 1 ms;
// pass 0 to stop the motor-initiated 27 01 status stream.
int32_t motor_handle_hightorque_set_periodic_report(MotorHandle* motor, uint16_t period_ms);

// Huayi-only extensions (reject non-Huayi motors). motor_id = CANopen node id
// (1..=127). The generic motor_handle_*_register u8-rid interface cannot
// address 16-bit CANopen indices — the typed SDO access below is the
// canonical register interface for Huayi (the generic functions reject
// Huayi motors). Unified motor_handle_get_state maps: status_code = 6061h
// mode display, fault_code = 603Fh/EMCY low byte (full u16 via
// motor_handle_huayi_get_error_code).
// SDO typed access: 16-bit index + 8-bit sub, write timeout fixed at 300 ms.
int32_t motor_handle_huayi_sdo_read_u16(MotorHandle* motor, uint16_t index, uint8_t sub, uint32_t timeout_ms, uint16_t* out_value);
int32_t motor_handle_huayi_sdo_read_i16(MotorHandle* motor, uint16_t index, uint8_t sub, uint32_t timeout_ms, int16_t* out_value);
int32_t motor_handle_huayi_sdo_read_i32(MotorHandle* motor, uint16_t index, uint8_t sub, uint32_t timeout_ms, int32_t* out_value);
int32_t motor_handle_huayi_sdo_read_f32(MotorHandle* motor, uint16_t index, uint8_t sub, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_huayi_sdo_write_u16(MotorHandle* motor, uint16_t index, uint8_t sub, uint16_t value);
int32_t motor_handle_huayi_sdo_write_i16(MotorHandle* motor, uint16_t index, uint8_t sub, int16_t value);
int32_t motor_handle_huayi_sdo_write_i32(MotorHandle* motor, uint16_t index, uint8_t sub, int32_t value);
int32_t motor_handle_huayi_sdo_write_f32(MotorHandle* motor, uint16_t index, uint8_t sub, float value);
// Torque mode (6060h=4): target torque in Nm (6071h permille of the 6076h peak;
// conversion and peak caching in the vendor layer).
int32_t motor_handle_huayi_send_torque(MotorHandle* motor, float torque_nm);
// Compressed MIT fast path (manual 5.2): one 8-byte classic frame to 0x500+node
// per command (first call remaps RPDO4 and reads the range objects). All
// arguments in unified SI units. The slow 2003h SDO path is motor_handle_send_mit.
int32_t motor_handle_huayi_send_mit_compressed(MotorHandle* motor, float pos, float vel, float kp, float kd, float tau);
// Position preset (3001h "pres"): requires Switch On Disabled (disable first);
// |position_rev| <= 0.5 output-shaft revolutions.
int32_t motor_handle_huayi_position_preset(MotorHandle* motor, float position_rev);
// TPDO stream control (RobStride set_active_report role): period_ms 1..=65535;
// 0 stops reporting. TPDO2 (pos/vel) anchors the sequence, TPDO1/TPDO3 patch in place.
int32_t motor_handle_huayi_configure_periodic_report(MotorHandle* motor, uint32_t period_ms);
// Last cached 603Fh/EMCY error code (u16, 0 = no error); no bus traffic, fails
// when no feedback source has reported yet. EMCY fault bursts are recorded
// even before a TPDO2 anchor (they only anchor the full state).
int32_t motor_handle_huayi_get_error_code(MotorHandle* motor, uint16_t* out_code);
// Error history (1003h walk): writes up to 32 codes into out_codes (oldest
// first) and the count into out_count. Pass out_codes with room for 32 u16.
int32_t motor_handle_huayi_get_error_history(MotorHandle* motor, uint16_t* out_codes, uint32_t* out_count);
// Change the CANopen node id (2001h_1); effective after a node reset.
int32_t motor_handle_huayi_set_node_id(MotorHandle* motor, uint8_t new_id);
// Raw NMT master command on this motor's bus: command 0x01 start / 0x02 stop /
// 0x80 enter pre-operational / 0x81 reset node / 0x82 reset communication;
// node 0 broadcasts to every CANopen node on the bus.
int32_t motor_handle_huayi_nmt(MotorHandle* motor, uint8_t command, uint8_t node);
// Scan CANopen nodes: NMT start broadcast, then 6041h/6061h/603Fh SDO probes
// for each id in start_id..=end_id (1..=127, start <= end). out_nodes must have
// room for end_id - start_id + 1 u16 entries; out_count receives the hit count.
// Returns an error if any motor is already registered on the controller (the
// core receive thread would claim the probe replies) — call before
// add_huayi_motor or from a separate controller on the same bus. The scan
// holds the controller for its whole duration, so keep timeout_ms modest.
// Huayi controllers only.
int32_t motor_controller_huayi_scan_ids(MotorController* controller, uint16_t start_id, uint16_t end_id, uint32_t timeout_ms, uint16_t* out_nodes, uint32_t* out_count);

#ifdef __cplusplus
}
#endif

#endif
