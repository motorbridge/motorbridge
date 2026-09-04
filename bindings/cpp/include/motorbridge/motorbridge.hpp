#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

extern "C" {
#include "motor_abi.h"
}

namespace motorbridge {

class Error : public std::runtime_error {
 public:
  explicit Error(const std::string& message) : std::runtime_error(message) {}
};

inline std::string last_error_message() {
  const char* msg = motor_last_error_message();
  if (!msg) return "unknown error";
  return std::string(msg);
}

inline std::string abi_version() {
  const char* msg = motor_abi_version();
  if (!msg) return "";
  return std::string(msg);
}

inline std::string abi_capabilities_json() {
  const char* msg = motor_abi_capabilities_json();
  if (!msg) return "{}";
  return std::string(msg);
}

inline void check_rc(int32_t rc, const char* what) {
  if (rc == 0) return;
  throw Error(std::string(what) + " failed: " + last_error_message());
}

enum class Mode : uint32_t {
  MIT = 1,
  POS_VEL = 2,
  VEL = 3,
  FORCE_POS = 4,
};

struct State {
  uint8_t can_id;
  uint32_t arbitration_id;
  uint8_t status_code;
  float pos;
  float vel;
  float torq;
  float t_mos;
  float t_rotor;
};

struct RegisterSpec {
  uint8_t rid;
  const char* variable;
  const char* description;
  const char* data_type;
  const char* access;
  const char* range;
};

inline constexpr uint8_t RID_UV_VALUE = 0;
inline constexpr uint8_t RID_KT_VALUE = 1;
inline constexpr uint8_t RID_OT_VALUE = 2;
inline constexpr uint8_t RID_OC_VALUE = 3;
inline constexpr uint8_t RID_ACC = 4;
inline constexpr uint8_t RID_DEC = 5;
inline constexpr uint8_t RID_MAX_SPD = 6;
inline constexpr uint8_t RID_MST_ID = 7;
inline constexpr uint8_t RID_ESC_ID = 8;
inline constexpr uint8_t RID_TIMEOUT = 9;
inline constexpr uint8_t RID_CTRL_MODE = 10;
inline constexpr uint8_t RID_PMAX = 21;
inline constexpr uint8_t RID_VMAX = 22;
inline constexpr uint8_t RID_TMAX = 23;

inline constexpr std::array<RegisterSpec, 26> DAMIAO_RW_REGISTERS = {{
    {0, "UV_Value", "Under-voltage protection value", "f32", "RW", "(10.0, 3.4E38]"},
    {1, "KT_Value", "Torque coefficient", "f32", "RW", "[0.0, 3.4E38]"},
    {2, "OT_Value", "Over-temperature protection value", "f32", "RW", "[80.0, 200)"},
    {3, "OC_Value", "Over-current protection value", "f32", "RW", "(0.0, 1.0)"},
    {4, "ACC", "Acceleration", "f32", "RW", "(0.0, 3.4E38)"},
    {5, "DEC", "Deceleration", "f32", "RW", "[-3.4E38, 0.0)"},
    {6, "MAX_SPD", "Maximum speed", "f32", "RW", "(0.0, 3.4E38]"},
    {7, "MST_ID", "Feedback ID", "u32", "RW", "[0, 0x7FF]"},
    {8, "ESC_ID", "Receive ID", "u32", "RW", "[0, 0x7FF]"},
    {9, "TIMEOUT", "Timeout alarm time", "u32", "RW", "[0, 2^32-1]"},
    {10, "CTRL_MODE", "Control mode", "u32", "RW", "[1, 4]"},
    {21, "PMAX", "Position mapping range", "f32", "RW", "(0.0, 3.4E38]"},
    {22, "VMAX", "Speed mapping range", "f32", "RW", "(0.0, 3.4E38]"},
    {23, "TMAX", "Torque mapping range", "f32", "RW", "(0.0, 3.4E38]"},
    {24, "I_BW", "Current loop control bandwidth", "f32", "RW", "[100.0, 10000.0]"},
    {25, "KP_ASR", "Speed loop Kp", "f32", "RW", "[0.0, 3.4E38]"},
    {26, "KI_ASR", "Speed loop Ki", "f32", "RW", "[0.0, 3.4E38]"},
    {27, "KP_APR", "Position loop Kp", "f32", "RW", "[0.0, 3.4E38]"},
    {28, "KI_APR", "Position loop Ki", "f32", "RW", "[0.0, 3.4E38]"},
    {29, "OV_Value", "Over-voltage protection value", "f32", "RW", "TBD"},
    {30, "GREF", "Gear torque efficiency", "f32", "RW", "(0.0, 1.0]"},
    {31, "Deta", "Speed loop damping coefficient", "f32", "RW", "[1.0, 30.0]"},
    {32, "V_BW", "Speed loop filter bandwidth", "f32", "RW", "(0.0, 500.0)"},
    {33, "IQ_c1", "Current loop enhancement coefficient", "f32", "RW", "[100.0, 10000.0]"},
    {34, "VL_c1", "Speed loop enhancement coefficient", "f32", "RW", "(0.0, 10000.0]"},
    {35, "can_br", "CAN baud rate code", "u32", "RW", "[0, 4]"},
}};

inline constexpr std::array<uint8_t, 11> DAMIAO_HIGH_IMPACT_RIDS = {
    21, 22, 23, 25, 26, 27, 28, 4, 5, 6, 9};
inline constexpr std::array<uint8_t, 4> DAMIAO_PROTECTION_RIDS = {0, 2, 3, 29};

inline const RegisterSpec* get_damiao_register_spec(uint8_t rid) {
  for (const auto& spec : DAMIAO_RW_REGISTERS) {
    if (spec.rid == rid) return &spec;
  }
  return nullptr;
}

class Controller;

namespace detail {
struct ControllerHandle {
  MotorController* ptr;

  explicit ControllerHandle(MotorController* p) : ptr(p) {}

  ~ControllerHandle() {
    if (ptr) {
      motor_controller_shutdown(ptr);
      motor_controller_free(ptr);
    }
  }
};
}  // namespace detail

class Motor {
 public:
  Motor() = delete;
  ~Motor() {
    if (ptr_) motor_handle_free(ptr_);
  }

  Motor(Motor&& other) noexcept : controller_(std::move(other.controller_)), ptr_(other.ptr_) {
    other.ptr_ = nullptr;
  }

  Motor& operator=(Motor&& other) noexcept {
    if (this == &other) return *this;
    if (ptr_) motor_handle_free(ptr_);
    controller_ = std::move(other.controller_);
    ptr_ = other.ptr_;
    other.ptr_ = nullptr;
    return *this;
  }

  Motor(const Motor&) = delete;
  Motor& operator=(const Motor&) = delete;

  void close() {
    if (ptr_) {
      motor_handle_free(ptr_);
      ptr_ = nullptr;
    }
  }

  void enable() { require_open(); check_rc(motor_handle_enable(ptr_), "enable"); }
  void disable() { require_open(); check_rc(motor_handle_disable(ptr_), "disable"); }
  void clear_error() { require_open(); check_rc(motor_handle_clear_error(ptr_), "clear_error"); }
  void set_zero_position() {
    require_open();
    check_rc(motor_handle_set_zero_position(ptr_), "set_zero_position");
  }
  void ensure_mode(Mode mode, uint32_t timeout_ms = 1000) {
    require_open();
    check_rc(motor_handle_ensure_mode(ptr_, static_cast<uint32_t>(mode), timeout_ms), "ensure_mode");
  }
  void send_mit(float pos, float vel, float kp, float kd, float tau) {
    require_open();
    check_rc(motor_handle_send_mit(ptr_, pos, vel, kp, kd, tau), "send_mit");
  }
  void send_pos_vel(float pos, float vlim) {
    require_open();
    check_rc(motor_handle_send_pos_vel(ptr_, pos, vlim), "send_pos_vel");
  }
  void send_vel(float vel) { require_open(); check_rc(motor_handle_send_vel(ptr_, vel), "send_vel"); }
  void send_force_pos(float pos, float vlim, float ratio) {
    require_open();
    check_rc(motor_handle_send_force_pos(ptr_, pos, vlim, ratio), "send_force_pos");
  }
  void request_feedback() { require_open(); check_rc(motor_handle_request_feedback(ptr_), "request_feedback"); }
  void set_can_timeout_ms(uint32_t timeout_ms) {
    require_open();
    check_rc(motor_handle_set_can_timeout_ms(ptr_, timeout_ms), "set_can_timeout_ms");
  }
  void store_parameters() {
    require_open();
    check_rc(motor_handle_store_parameters(ptr_), "store_parameters");
  }

  void write_register_f32(uint8_t rid, float value) {
    require_open();
    check_rc(motor_handle_write_register_f32(ptr_, rid, value), "write_register_f32");
  }
  void write_register_u32(uint8_t rid, uint32_t value) {
    require_open();
    check_rc(motor_handle_write_register_u32(ptr_, rid, value), "write_register_u32");
  }
  float get_register_f32(uint8_t rid, uint32_t timeout_ms = 1000) {
    require_open();
    float out = 0.0f;
    check_rc(motor_handle_get_register_f32(ptr_, rid, timeout_ms, &out), "get_register_f32");
    return out;
  }
  uint32_t get_register_u32(uint8_t rid, uint32_t timeout_ms = 1000) {
    require_open();
    uint32_t out = 0;
    check_rc(motor_handle_get_register_u32(ptr_, rid, timeout_ms, &out), "get_register_u32");
    return out;
  }

  std::pair<uint8_t, uint8_t> robstride_ping() {
    require_open();
    uint8_t device_id = 0;
    uint8_t responder_id = 0;
    check_rc(motor_handle_robstride_ping(ptr_, &device_id, &responder_id), "robstride_ping");
    return {device_id, responder_id};
  }

  std::pair<uint8_t, uint8_t> robstride_ping_host_id(uint16_t host_id,
                                                     uint32_t timeout_ms = 500) {
    require_open();
    if (host_id > 255) {
      throw Error("RobStride host_id must be in 0..255");
    }
    uint8_t device_id = 0;
    uint8_t responder_id = 0;
    check_rc(
        motor_handle_robstride_ping_host_id(ptr_, host_id, timeout_ms, &device_id, &responder_id),
        "robstride_ping_host_id");
    return {device_id, responder_id};
  }

  float robstride_get_param_f32_host_id(uint16_t param_id, uint16_t host_id,
                                        uint32_t timeout_ms = 1000) {
    require_open();
    if (host_id > 255) {
      throw Error("RobStride host_id must be in 0..255");
    }
    float out = 0.0f;
    check_rc(
        motor_handle_robstride_get_param_f32_host_id(ptr_, param_id, host_id, timeout_ms, &out),
        "robstride_get_param_f32_host_id");
    return out;
  }

  std::pair<uint32_t, uint32_t> robstride_get_fault_report() {
    require_open();
    uint32_t fault_raw = 0;
    uint32_t warning_raw = 0;
    check_rc(motor_handle_robstride_get_fault_report(ptr_, &fault_raw, &warning_raw),
             "robstride_get_fault_report");
    return {fault_raw, warning_raw};
  }

  void robstride_set_device_id(uint8_t new_device_id) {
    require_open();
    if (new_device_id < 1) {
      throw Error("RobStride new_device_id must be in 1..255");
    }
    check_rc(motor_handle_robstride_set_device_id(ptr_, new_device_id), "robstride_set_device_id");
  }

  void robstride_set_active_report(bool enabled) {
    require_open();
    check_rc(motor_handle_robstride_set_active_report(ptr_, enabled ? 1 : 0),
             "robstride_set_active_report");
  }

  void robstride_write_param_i8(uint16_t param_id, int8_t value) {
    require_open();
    check_rc(motor_handle_robstride_write_param_i8(ptr_, param_id, value), "robstride_write_param_i8");
  }
  void robstride_write_param_u8(uint16_t param_id, uint8_t value) {
    require_open();
    check_rc(motor_handle_robstride_write_param_u8(ptr_, param_id, value), "robstride_write_param_u8");
  }
  void robstride_write_param_u16(uint16_t param_id, uint16_t value) {
    require_open();
    check_rc(motor_handle_robstride_write_param_u16(ptr_, param_id, value), "robstride_write_param_u16");
  }
  void robstride_write_param_u32(uint16_t param_id, uint32_t value) {
    require_open();
    check_rc(motor_handle_robstride_write_param_u32(ptr_, param_id, value), "robstride_write_param_u32");
  }
  void robstride_write_param_f32(uint16_t param_id, float value) {
    require_open();
    check_rc(motor_handle_robstride_write_param_f32(ptr_, param_id, value), "robstride_write_param_f32");
  }

  int8_t robstride_get_param_i8(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    int8_t out = 0;
    check_rc(motor_handle_robstride_get_param_i8(ptr_, param_id, timeout_ms, &out), "robstride_get_param_i8");
    return out;
  }
  uint8_t robstride_get_param_u8(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    uint8_t out = 0;
    check_rc(motor_handle_robstride_get_param_u8(ptr_, param_id, timeout_ms, &out), "robstride_get_param_u8");
    return out;
  }
  uint16_t robstride_get_param_u16(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    uint16_t out = 0;
    check_rc(motor_handle_robstride_get_param_u16(ptr_, param_id, timeout_ms, &out), "robstride_get_param_u16");
    return out;
  }
  uint32_t robstride_get_param_u32(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    uint32_t out = 0;
    check_rc(motor_handle_robstride_get_param_u32(ptr_, param_id, timeout_ms, &out), "robstride_get_param_u32");
    return out;
  }
  float robstride_get_param_f32(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    float out = 0.0f;
    check_rc(motor_handle_robstride_get_param_f32(ptr_, param_id, timeout_ms, &out), "robstride_get_param_f32");
    return out;
  }

  float damiao_get_param_f32(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    float out = 0.0f;
    check_rc(motor_handle_damiao_get_param_f32(ptr_, param_id, timeout_ms, &out), "damiao_get_param_f32");
    return out;
  }
  uint32_t damiao_get_param_u32(uint16_t param_id, uint32_t timeout_ms = 1000) {
    require_open();
    uint32_t out = 0;
    check_rc(motor_handle_damiao_get_param_u32(ptr_, param_id, timeout_ms, &out), "damiao_get_param_u32");
    return out;
  }
  void damiao_write_param_f32(uint16_t param_id, float value) {
    require_open();
    check_rc(motor_handle_damiao_write_param_f32(ptr_, param_id, value), "damiao_write_param_f32");
  }
  void damiao_write_param_u32(uint16_t param_id, uint32_t value) {
    require_open();
    check_rc(motor_handle_damiao_write_param_u32(ptr_, param_id, value), "damiao_write_param_u32");
  }

  std::optional<State> get_state() const {
    require_open();
    MotorState st{};
    check_rc(motor_handle_get_state(ptr_, &st), "get_state");
    if (!st.has_value) return std::nullopt;
    return State{
        st.can_id, st.arbitration_id, st.status_code, st.pos, st.vel, st.torq, st.t_mos, st.t_rotor};
  }

 private:
  friend class Controller;

  Motor(std::shared_ptr<detail::ControllerHandle> controller, MotorHandle* ptr)
      : controller_(std::move(controller)), ptr_(ptr) {}

  void require_open() const {
    if (!ptr_) throw Error("motor handle is closed");
  }

  std::shared_ptr<detail::ControllerHandle> controller_;
  MotorHandle* ptr_;
};

class Controller {
 public:
  explicit Controller(const std::string& channel = "can0") {
    MotorController* raw = motor_controller_new_socketcan(channel.c_str());
    if (!raw) {
      throw Error("new_socketcan failed: " + last_error_message());
    }
    handle_ = std::make_shared<detail::ControllerHandle>(raw);
  }

  static Controller from_socketcanfd(const std::string& channel = "can0") {
    MotorController* raw = motor_controller_new_socketcanfd(channel.c_str());
    if (!raw) {
      throw Error("new_socketcanfd failed: " + last_error_message());
    }
    return Controller(raw);
  }

  static Controller from_dm_serial(const std::string& serial_port = "/dev/ttyACM0",
                                   uint32_t baud = 921600) {
    MotorController* raw = motor_controller_new_dm_serial(serial_port.c_str(), baud);
    if (!raw) {
      throw Error("new_dm_serial failed: " + last_error_message());
    }
    return Controller(raw);
  }

  // UART-to-CAN MCU bridge: vendor-agnostic, classic 8-byte CAN only
  // (hexfellow/CAN-FD is not supported on this transport).
  static Controller from_mcu_serial(const std::string& serial_port = "/dev/ttyUSB0",
                                    uint32_t baud = 921600) {
    MotorController* raw = motor_controller_new_mcu_serial(serial_port.c_str(), baud);
    if (!raw) {
      throw Error("new_mcu_serial failed: " + last_error_message());
    }
    return Controller(raw);
  }

  Controller(Controller&&) noexcept = default;
  Controller& operator=(Controller&&) noexcept = default;
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  ~Controller() = default;

  void close() {
    if (handle_ && handle_->ptr) {
      motor_controller_free(handle_->ptr);
      handle_->ptr = nullptr;
    }
  }

  void enable_all() { require_open(); check_rc(motor_controller_enable_all(handle_->ptr), "enable_all"); }
  void disable_all() { require_open(); check_rc(motor_controller_disable_all(handle_->ptr), "disable_all"); }
  void poll_feedback_once() {
    require_open();
    check_rc(motor_controller_poll_feedback_once(handle_->ptr), "poll_feedback_once");
  }
  void shutdown() { require_open(); check_rc(motor_controller_shutdown(handle_->ptr), "shutdown"); }
  void close_bus() { require_open(); check_rc(motor_controller_close_bus(handle_->ptr), "close_bus"); }

  Motor add_damiao_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model) {
    MotorHandle* m = motor_controller_add_damiao_motor(handle_->ptr, motor_id, feedback_id, model.c_str());
    if (!m) {
      throw Error("add_damiao_motor failed: " + last_error_message());
    }
    return Motor(handle_, m);
  }

  Motor add_hexfellow_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model) {
    MotorHandle* m =
        motor_controller_add_hexfellow_motor(handle_->ptr, motor_id, feedback_id, model.c_str());
    if (!m) {
      throw Error("add_hexfellow_motor failed: " + last_error_message());
    }
    return Motor(handle_, m);
  }

  Motor add_myactuator_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model) {
    MotorHandle* m =
        motor_controller_add_myactuator_motor(handle_->ptr, motor_id, feedback_id, model.c_str());
    if (!m) {
      throw Error("add_myactuator_motor failed: " + last_error_message());
    }
    return Motor(handle_, m);
  }

  Motor add_robstride_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model) {
    if (motor_id < 1 || motor_id > 255) {
      throw Error("RobStride motor_id must be in 1..255");
    }
    if (feedback_id > 255) {
      throw Error("RobStride feedback_id/host_id must be in 0..255");
    }
    MotorHandle* m =
        motor_controller_add_robstride_motor(handle_->ptr, motor_id, feedback_id, model.c_str());
    if (!m) {
      throw Error("add_robstride_motor failed: " + last_error_message());
    }
    return Motor(handle_, m);
  }

  Motor add_hightorque_motor(uint16_t motor_id, uint16_t feedback_id, const std::string& model) {
    MotorHandle* m =
        motor_controller_add_hightorque_motor(handle_->ptr, motor_id, feedback_id, model.c_str());
    if (!m) {
      throw Error("add_hightorque_motor failed: " + last_error_message());
    }
    return Motor(handle_, m);
  }

 private:
  explicit Controller(MotorController* raw) {
    handle_ = std::make_shared<detail::ControllerHandle>(raw);
  }

  void require_open() const {
    if (!handle_ || !handle_->ptr) throw Error("controller is closed");
  }

  std::shared_ptr<detail::ControllerHandle> handle_;
};

}  // namespace motorbridge
