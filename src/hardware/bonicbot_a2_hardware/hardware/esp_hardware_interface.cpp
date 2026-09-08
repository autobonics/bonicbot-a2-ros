#include "bonicbot_a2_hardware/esp_hardware_interface.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bonicbot_a2_hardware
{

namespace
{
constexpr const char * kLogger = "EspHardwareInterface";
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

/// Read a URDF hardware_parameter, falling back to a default when absent.
double paramOr(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, double fallback)
{
  auto it = params.find(key);
  if (it == params.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger), "Parameter '%s' = '%s' is not a number; using %f",
      key.c_str(), it->second.c_str(), fallback);
    return fallback;
  }
}
}  // namespace

// ════════════════════════════════════════════════════════════════════
//  Lifecycle
// ════════════════════════════════════════════════════════════════════

hardware_interface::CallbackReturn EspHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;

  auto port_it = p.find("serial_port");
  serial_port_ = (port_it != p.end()) ? port_it->second : "/dev/esp";

  serial_baud_   = static_cast<int>(paramOr(p, "serial_baud", 115200));
  encoder_cpr_   = paramOr(p, "encoder_cpr", 14040.0);
  wheel_radius_  = paramOr(p, "wheel_radius", 0.06);
  max_velocity_  = paramOr(p, "max_velocity", 10.47);
  motor_accel_   = paramOr(p, "motor_accel", 1.5);
  servo_speed_   = static_cast<int>(paramOr(p, "servo_speed", 500));
  servo_accel_   = static_cast<int>(paramOr(p, "servo_accel", 50));
  servo_position_threshold_   = paramOr(p, "servo_position_threshold", 0.01);
  servo_feedback_interval_ms_ = static_cast<int>(paramOr(p, "servo_feedback_interval_ms", 100));
  // CMD_SERVO_FEEDBACK_REQUEST has no streaming mode (see cdc_protocol.hpp), so
  // read() re-requests it every Nth cycle at the controller_manager's fixed
  // 50 Hz (20 ms) rate to approximate the configured interval.
  servo_feedback_decimation_ = std::max(
    1, static_cast<int>(std::lround(servo_feedback_interval_ms_ / 20.0)));

  auto frame_it = p.find("imu_frame_id");
  if (frame_it != p.end()) {
    imu_frame_id_ = frame_it->second;
  }

  if (encoder_cpr_ <= 0.0 || wheel_radius_ <= 0.0) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger),
      "encoder_cpr (%.1f) and wheel_radius (%.3f) must both be > 0",
      encoder_cpr_, wheel_radius_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "USB CDC port: %s | encoder_cpr %.0f | wheel_radius %.3f m | max_vel %.2f rad/s",
    serial_port_.c_str(), encoder_cpr_, wheel_radius_, max_velocity_);

  const size_t n = info_.joints.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);
  prev_positions_.assign(n, 0.0);
  last_servo_commands_.assign(n, std::numeric_limits<double>::quiet_NaN());

  initializeServoMapping();

  bool found_left = false;
  bool found_right = false;

  for (size_t i = 0; i < n; i++) {
    const auto & joint = info_.joints[i];

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "Joint '%s' has %zu command interfaces; 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger), "Joint '%s' has %zu state interfaces; 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string & cmd_type = joint.command_interfaces[0].name;
    if (cmd_type != hardware_interface::HW_IF_VELOCITY &&
      cmd_type != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(kLogger),
        "Joint '%s' has unsupported command interface '%s'; expected velocity or position.",
        joint.name.c_str(), cmd_type.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (cmd_type == hardware_interface::HW_IF_VELOCITY) {
      // Resolve wheels BY NAME rather than assuming indices 0/1 — joint order in
      // the URDF is not guaranteed, and getting this wrong swaps the wheels.
      if (joint.name == "left_wheel_joint") {
        left_wheel_idx_ = i;
        found_left = true;
      } else if (joint.name == "right_wheel_joint") {
        right_wheel_idx_ = i;
        found_right = true;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(kLogger),
          "Velocity joint '%s' is neither left_wheel_joint nor right_wheel_joint; ignored.",
          joint.name.c_str());
      }
    } else {
      auto it = joint_to_servo_id_.find(joint.name);
      if (it == joint_to_servo_id_.end()) {
        RCLCPP_FATAL(
          rclcpp::get_logger(kLogger),
          "Position joint '%s' has no servo ID mapping. Add it to "
          "initializeServoMapping() or remove it from ros2_control.xacro.",
          joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      servo_joint_indices_.push_back(i);
      servo_id_to_joint_index_[it->second] = i;
      RCLCPP_INFO(
        rclcpp::get_logger(kLogger), "Servo joint '%s' -> registry ID %u",
        joint.name.c_str(), it->second);
    }
  }

  if (!found_left || !found_right) {
    RCLCPP_FATAL(
      rclcpp::get_logger(kLogger),
      "Missing wheel joints (left found: %d, right found: %d). Both "
      "left_wheel_joint and right_wheel_joint are required.", found_left, found_right);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "Initialised %zu joints (2 wheels, %zu servos)",
    n, servo_joint_indices_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EspHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!openSerialPort()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "Failed to open %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // SystemInterface has no node handle of its own, so spin one here. It carries
  // the IMU/battery publishers AND the Wi-Fi relay topics — the relay cannot be
  // a separate process because that would mean two owners of one CDC stream.
  node_ = std::make_shared<rclcpp::Node>("bonicbot_a2_esp_interface");

  imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  battery_publisher_ =
    node_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  wifi_credentials_publisher_ =
    node_->create_publisher<std_msgs::msg::String>("/esp/wifi_credentials", 10);

  // Power button -> robot_app. The ESP hard-cuts the latch 25 s after it asks,
  // so this must reach a subscriber that can actually halt the machine;
  // robot_app's PowerManager is that subscriber.
  shutdown_publisher_ =
    node_->create_publisher<std_msgs::msg::Empty>("/esp/shutdown", 10);

  // robot_app -> ESP: "run your shutdown sequence and cut power". Needed
  // because with this stack up robot_app cannot reach /dev/esp itself — the
  // direct CDC lane it uses when the stack is down is locked out, by design.
  shutdown_request_subscription_ = node_->create_subscription<std_msgs::msg::Empty>(
    "/esp/shutdown_request", 10,
    [this](const std_msgs::msg::Empty::SharedPtr) {
      // Flag only: the serial fd belongs to the control thread. write() sends it.
      shutdown_push_pending_ = true;
    });

  wifi_status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/esp/wifi_status", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      // "connected,ssid,rssi,ip" from robot_app (the nmcli owner), packed into
      // the 50-byte CDC payload now so the reply path stays allocation-free.
      std::vector<uint8_t> packed(cdc_protocol::WIFI_STATUS_PAYLOAD_SIZE, 0);

      std::string data = msg->data;
      std::vector<std::string> fields;
      size_t start = 0;
      for (int i = 0; i < 3; i++) {
        size_t comma = data.find(',', start);
        if (comma == std::string::npos) {
          break;
        }
        fields.push_back(data.substr(start, comma - start));
        start = comma + 1;
      }
      fields.push_back(data.substr(start));

      if (fields.size() != 4) {
        RCLCPP_WARN(
          rclcpp::get_logger(kLogger),
          "Malformed /esp/wifi_status '%s'; expected 'connected,ssid,rssi,ip'",
          msg->data.c_str());
        return;
      }

      // Tri-state, not a bool: WifiStatusPayload.status is 0 = idle/failed,
      // 1 = connected, 2 = connecting (firmware include/protocol_defs.h).
      // Collapsing it to 0/1 here would drop the "connecting" state robot_app
      // sends the moment credentials arrive, which is what lets the tablet show
      // progress during the ~15 s join instead of looking frozen. Legacy
      // "true"/"True" still map to connected.
      const std::string & state = fields[0];
      if (state == "true" || state == "True") {
        packed[0] = 1;
      } else {
        int parsed = 0;
        try {
          parsed = std::stoi(state);
        } catch (const std::exception &) {
          parsed = 0;
        }
        packed[0] = static_cast<uint8_t>(std::clamp(parsed, 0, 2));
      }

      const std::string & ssid = fields[1];
      std::memcpy(&packed[1], ssid.data(), std::min<size_t>(ssid.size(), 32));

      int rssi = 0;
      try {
        rssi = std::stoi(fields[2]);
      } catch (const std::exception &) {
        rssi = 0;
      }
      packed[33] = static_cast<uint8_t>(std::clamp(rssi, 0, 100));

      const std::string & ip = fields[3];
      std::memcpy(&packed[34], ip.data(), std::min<size_t>(ip.size(), 16));

      {
        std::lock_guard<std::mutex> lock(wifi_status_mutex_);
        wifi_status_payload_ = std::move(packed);
      }
      // Push rather than wait to be polled. robot_app publishes here right
      // after every successful nmcli join, and the ESP relays any payload-
      // carrying CMD_WIFI_STATUS straight to the phone as a BLE notify — so
      // this is what delivers "connected, and here is the new IP" without the
      // app having to guess when to ask. The serial fd belongs to the control
      // thread, so only flag it here; write() does the send.
      wifi_status_push_pending_ = true;
    });

  {
    std::lock_guard<std::mutex> lock(wifi_status_mutex_);
    wifi_status_payload_.assign(cdc_protocol::WIFI_STATUS_PAYLOAD_SIZE, 0);
  }

  face_matrix_subscription_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/face/matrix_action", 10,
    [this](const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
      // Raw pass-through — byte 0 is the action code, rest is that action's
      // own payload (spec §4). No interpretation here: expression/animation
      // choices belong to whatever publishes this (robot_app / bonicOS), not
      // this repo. Just buffer it for write() to send on the control thread.
      if (msg->data.empty()) {
        return;
      }
      std::lock_guard<std::mutex> lock(matrix_action_mutex_);
      pending_matrix_action_ = msg->data;
      matrix_action_pending_ = true;
    });

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  spin_thread_ = std::thread([this]() {executor_->spin();});

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "Publishing /imu/data, /battery_state, /esp/wifi_credentials; "
    "subscribed /esp/wifi_status, /face/matrix_action");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void EspHardwareInterface::stopInternalNode()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  executor_.reset();
  imu_publisher_.reset();
  battery_publisher_.reset();
  wifi_credentials_publisher_.reset();
  wifi_status_subscription_.reset();
  shutdown_publisher_.reset();
  shutdown_request_subscription_.reset();
  face_matrix_subscription_.reset();
  node_.reset();
}

hardware_interface::CallbackReturn EspHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stopInternalNode();
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EspHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Process termination (SIGINT/SIGTERM, e.g. stop_session.sh) drives the
  // lifecycle through deactivate() -> shutdown() — cleanup() is NOT part of
  // that path. Without this override, spin_thread_ was still joinable when
  // the destructor ran, and destroying a joinable std::thread is an
  // unconditional std::terminate()/abort (SIGABRT) regardless of connection
  // state — confirmed via a real crash during bench testing (2026-08-24).
  stopInternalNode();
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EspHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  clearBuffer();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Liveness first — distinguishes "cable/port wrong" from "encoders unhappy".
  bool alive = false;
  for (int attempt = 0; attempt < 3 && !alive; attempt++) {
    if (sendPacket(cdc_protocol::CMD_PING, nullptr, 0)) {
      // RESP_ACK sets no dedicated flag, so just pump and accept silence as a
      // soft failure; the encoder handshake below is the real gate.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      pumpSerial();
      alive = true;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  for (int attempt = 0; attempt < 3; attempt++) {
    clearBuffer();
    if (sendPacket(cdc_protocol::CMD_RESET_ENCODERS, nullptr, 0)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      pumpSerial();
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  bool encoders_ok = false;
  for (int attempt = 0; attempt < 3 && !encoders_ok; attempt++) {
    clearBuffer();
    encoder_data_ready_ = false;
    if (!sendPacket(cdc_protocol::CMD_ENCODER_REQUEST, nullptr, 0)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    encoders_ok = waitFor(encoder_data_ready_, std::chrono::milliseconds(500));
  }

  if (!encoders_ok) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "No encoder response from ESP32 after 3 attempts");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "Encoders responding: %d, %d",
    encoder_left_, encoder_right_);

  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(prev_positions_.begin(), prev_positions_.end(), 0.0);
  std::fill(
    last_servo_commands_.begin(), last_servo_commands_.end(),
    std::numeric_limits<double>::quiet_NaN());

  encoder_left_ = 0;
  encoder_right_ = 0;
  encoder_data_ready_ = false;

  // No explicit torque-on: CMD_SERVO_MULTI (0x0A) auto-engages torque on the
  // first position command, so there's nothing to do here. Servo feedback
  // polling starts on its own decimation inside read() — no setup call needed.
  servo_feedback_decimator_ = 0;

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EspHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // No stream to stop — servo feedback is per-request, not persistent.
  sendPacket(cdc_protocol::CMD_STOP, nullptr, 0);
  // Release torque so the arms don't stay powered (and warm) while idle — the
  // old implementation left them energised after deactivation.
  setAllServoTorque(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════════
//  read / write
// ════════════════════════════════════════════════════════════════════

hardware_interface::return_type EspHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!connected_) {
    // Returning ERROR here would trigger ros2_control's on_error() transition
    // and deactivate this component permanently — controller_manager would
    // stop calling read()/write() altogether, and no amount of replugging
    // would ever be seen again. Returning OK keeps this cycle firing so
    // attemptReconnect()'s own backoff can actually do its job.
    attemptReconnect();
    return hardware_interface::return_type::OK;
  }

  // Consume everything the ESP has sent since the last cycle: encoder replies,
  // streamed servo feedback, IMU samples, battery, Wi-Fi relay frames.
  //
  // This is pipelined rather than blocking: the request issued at the end of
  // this function is answered by the time the NEXT cycle pumps. The old code
  // blocked up to 100 ms waiting for each reply inside a 20 ms control period,
  // which could not meet its own deadline.
  pumpSerial();

  if (encoder_data_ready_) {
    hw_positions_[left_wheel_idx_]  = (encoder_left_  / encoder_cpr_) * 2.0 * M_PI;
    hw_positions_[right_wheel_idx_] = (encoder_right_ / encoder_cpr_) * 2.0 * M_PI;
    encoder_data_ready_ = false;
  }

  const double dt = period.seconds();
  if (dt > 0.0) {
    for (size_t i = 0; i < hw_positions_.size(); i++) {
      hw_velocities_[i] = (hw_positions_[i] - prev_positions_[i]) / dt;
      prev_positions_[i] = hw_positions_[i];
    }
  }

  if (!sendPacket(cdc_protocol::CMD_ENCODER_REQUEST, nullptr, 0)) {
    markDisconnected("encoder request write failed");
    return hardware_interface::return_type::OK;
  }

  // IMU is decimated: the control loop runs at 50 Hz, the EKF only needs ~25 Hz
  // of yaw rate, and every request costs a CDC round trip.
  if (++imu_decimator_ >= 2) {
    imu_decimator_ = 0;
    if (!sendPacket(cdc_protocol::CMD_IMU_REQUEST, nullptr, 0)) {
      markDisconnected("IMU request write failed");
      return hardware_interface::return_type::OK;
    }
  }

  // Servo feedback has no streaming mode (see cdc_protocol.hpp) — re-request
  // on the configured decimation, same pattern as the IMU above.
  if (++servo_feedback_decimator_ >= servo_feedback_decimation_) {
    servo_feedback_decimator_ = 0;
    if (!requestServoFeedback()) {
      markDisconnected("servo feedback request write failed");
      return hardware_interface::return_type::OK;
    }
  }

  // Battery telemetry: request/response like everything else on CDC, decimated
  // hard since voltage/SOC only need ~1 Hz.
  if (++battery_decimator_ >= kBatteryDecimation) {
    battery_decimator_ = 0;
    if (!sendPacket(cdc_protocol::CMD_BATTERY_REQUEST, nullptr, 0)) {
      markDisconnected("battery request write failed");
      return hardware_interface::return_type::OK;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EspHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // See read() above: never return ERROR for a recoverable disconnect — that
  // would deactivate this component permanently instead of letting the
  // reconnect backoff run.
  if (!connected_) {
    return hardware_interface::return_type::OK;
  }

  // Wheels: hw_commands_ is rad/s (velocity interface); CMD_MOTOR_MOVE wants m/s.
  // The old protocol took a raw PWM duty (±1022) scaled by max_velocity_, so
  // that conversion is gone entirely — the ESP closes the loop on real units now.
  const double max_mps = max_velocity_ * wheel_radius_;
  double left_mps  = hw_commands_[left_wheel_idx_]  * wheel_radius_;
  double right_mps = hw_commands_[right_wheel_idx_] * wheel_radius_;
  left_mps  = std::clamp(left_mps,  -max_mps, max_mps);
  right_mps = std::clamp(right_mps, -max_mps, max_mps);

  if (!sendMotorMove(left_mps, right_mps, motor_accel_)) {
    markDisconnected("motor command write failed");
    return hardware_interface::return_type::OK;
  }

  if (!sendServoPositions()) {
    markDisconnected("servo command write failed");
    return hardware_interface::return_type::OK;
  }

  // Face LED matrix: fire-and-forget, only when /face/matrix_action actually
  // publishes something — no per-cycle traffic like the sensors/servos above.
  if (matrix_action_pending_) {
    std::vector<uint8_t> action;
    {
      std::lock_guard<std::mutex> lock(matrix_action_mutex_);
      action = std::move(pending_matrix_action_);
      matrix_action_pending_ = false;
    }
    if (!sendPacket(cdc_protocol::CMD_MATRIX_ACTION, action.data(), action.size())) {
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "Face matrix command write failed");
    }
  }

  // Unprompted Wi-Fi status push — see wifi_status_push_pending_. Same
  // fire-and-forget shape as the matrix action above: only on a fresh publish,
  // never per-cycle.
  // Outbound shutdown request from robot_app. With the latch idle the ESP
  // reads this as "start the shutdown sequence", which ends in a real power
  // cut rather than a halted-but-powered robot.
  if (shutdown_push_pending_) {
    shutdown_push_pending_ = false;
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "robot_app requested shutdown — asking the ESP to cut power");
    sendPacket(cdc_protocol::CMD_SHUTDOWN, nullptr, 0);
  }

  if (wifi_status_push_pending_) {
    wifi_status_push_pending_ = false;
    replyWifiStatus();
  }

  return hardware_interface::return_type::OK;
}

// ════════════════════════════════════════════════════════════════════
//  Motion commands
// ════════════════════════════════════════════════════════════════════

bool EspHardwareInterface::sendMotorMove(double left_mps, double right_mps, double accel_mps2)
{
  // Only resend when something actually changed — at 50 Hz an idle robot would
  // otherwise flood the link with identical frames.
  static double prev_left = std::numeric_limits<double>::quiet_NaN();
  static double prev_right = std::numeric_limits<double>::quiet_NaN();
  constexpr double kEpsilon = 1e-4;

  const bool unchanged =
    std::abs(left_mps - prev_left) < kEpsilon &&
    std::abs(right_mps - prev_right) < kEpsilon;
  if (unchanged) {
    return true;
  }
  prev_left = left_mps;
  prev_right = right_mps;

  // CMD_MOTOR_MOVE: float left (m/s), float right (m/s), float accel (m/s^2)
  uint8_t payload[12];
  const float l = static_cast<float>(left_mps);
  const float r = static_cast<float>(right_mps);
  const float a = static_cast<float>(accel_mps2);
  std::memcpy(&payload[0], &l, sizeof(float));
  std::memcpy(&payload[4], &r, sizeof(float));
  std::memcpy(&payload[8], &a, sizeof(float));

  return sendPacket(cdc_protocol::CMD_MOTOR_MOVE, payload, sizeof(payload));
}

bool EspHardwareInterface::sendServoPositions()
{
  // CMD_SERVO_MULTI (0x0A): Count + N x [id, float angle(deg), uint16 speed, uint8 accel]
  //
  // NOT CMD_SERVO_CONTROL (0x27) — that command's payload is only two bytes
  // ([id][action]) and cannot carry an angle at all.
  uint8_t payload[1 + 18 * cdc_protocol::SERVO_MULTI_ENTRY_SIZE];
  uint16_t idx = 1;
  uint8_t count = 0;

  for (size_t joint_idx : servo_joint_indices_) {
    const std::string & joint_name = info_.joints[joint_idx].name;
    auto it = joint_to_servo_id_.find(joint_name);
    if (it == joint_to_servo_id_.end()) {
      continue;
    }

    const double cmd = hw_commands_[joint_idx];
    if (std::isnan(cmd)) {
      continue;
    }

    // NaN on the first pass means "never sent", so the first command always goes.
    const double last = last_servo_commands_[joint_idx];
    if (!std::isnan(last) && std::abs(cmd - last) <= servo_position_threshold_) {
      continue;
    }

    const uint8_t servo_id = it->second;
    double angle_deg = cmd * kRadToDeg;
    if (isServoInverted(servo_id)) {
      angle_deg = -angle_deg;
    }

    const float angle_f = static_cast<float>(angle_deg);
    const uint16_t speed = static_cast<uint16_t>(std::clamp(servo_speed_, 0, 1000));
    const uint8_t accel = static_cast<uint8_t>(std::clamp(servo_accel_, 0, 255));

    payload[idx++] = servo_id;
    std::memcpy(&payload[idx], &angle_f, sizeof(float));
    idx += sizeof(float);
    payload[idx++] = static_cast<uint8_t>(speed & 0xFF);
    payload[idx++] = static_cast<uint8_t>((speed >> 8) & 0xFF);
    payload[idx++] = accel;

    last_servo_commands_[joint_idx] = cmd;
    count++;
  }

  if (count == 0) {
    return true;
  }

  payload[0] = count;
  return sendPacket(cdc_protocol::CMD_SERVO_MULTI, payload, idx);
}

bool EspHardwareInterface::setAllServoTorque(bool enabled)
{
  // CMD_SERVO_CONTROL (0x27): N x [id, action], no count prefix — one packet
  // for every servo instead of one packet per servo.
  const auto action =
    enabled ? cdc_protocol::ServoAction::TORQUE_ON : cdc_protocol::ServoAction::TORQUE_OFF;
  uint8_t payload[18 * 2];
  uint16_t idx = 0;
  for (const auto & [joint_name, servo_id] : joint_to_servo_id_) {
    (void)joint_name;
    payload[idx++] = servo_id;
    payload[idx++] = static_cast<uint8_t>(action);
  }
  const bool ok = sendPacket(cdc_protocol::CMD_SERVO_CONTROL, payload, idx);
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "Servo torque %s", enabled ? "enabled" : "released");
  return ok;
}

bool EspHardwareInterface::requestServoFeedback()
{
  // CMD_SERVO_FEEDBACK_REQUEST (0x2A): [Count][IDs...]. One request triggers
  // one async poll + one RESP_SERVO_FEEDBACK reply — no mode/interval fields.
  uint8_t payload[1 + 18];
  uint16_t idx = 0;
  const uint8_t count = static_cast<uint8_t>(joint_to_servo_id_.size());
  payload[idx++] = count;
  for (const auto & [joint_name, servo_id] : joint_to_servo_id_) {
    (void)joint_name;
    payload[idx++] = servo_id;
  }

  return sendPacket(cdc_protocol::CMD_SERVO_FEEDBACK_REQUEST, payload, idx);
}

// ════════════════════════════════════════════════════════════════════
//  Servo registry
// ════════════════════════════════════════════════════════════════════

void EspHardwareInterface::initializeServoMapping()
{
  // Canonical actuator-registry IDs (bonicbot_actuator_naming.md). A2 fits 7 of
  // the 18: both grippers, both elbows, both shoulder pitches, neck yaw.
  //
  // These IDs go on the wire verbatim. The pre-migration firmware used a
  // different, A2-only numbering (1/3/6/7/9/12/13, sent as id-1); that scheme
  // and its conversion are gone. Note only right_gripper keeps the same wire
  // value across the change, so a partial port silently drives wrong joints.
  joint_to_servo_id_["right_gripper_finger1_joint"] = 0;   // rightGripper
  joint_to_servo_id_["right_elbow_joint"]           = 4;   // rightElbow
  joint_to_servo_id_["right_shoulder_pitch_joint"]  = 7;   // rightShoulderPitch
  joint_to_servo_id_["left_shoulder_pitch_joint"]   = 8;   // leftShoulderPitch
  joint_to_servo_id_["left_elbow_joint"]            = 11;  // leftElbow
  joint_to_servo_id_["left_gripper_finger1_joint"]  = 15;  // leftGripper
  joint_to_servo_id_["neck_yaw_joint"]              = 16;  // neckYaw
}

bool EspHardwareInterface::isServoInverted(uint8_t servo_id)
{
  // Mirrored joints, carried over from the pre-migration mapping (old IDs
  // 1/3/7/9/13 = grippers, elbows, neck yaw) translated to registry IDs.
  //
  // 2026-08-24: elbows (4, 11) removed after bench testing showed a
  // double-inversion (this table negating a sign the unified firmware
  // already flips internally, per the CDC spec's own note that
  // processServoMulti() handles mirrored-joint inversion natively) pushed
  // their one-sided range (-50..0 deg) out of bounds entirely.
  //
  // 2026-08-25: gripper (0, 15) and neck (16) removed too, for the same
  // reason — confirmed reversed vs simulation during robot_app teleop
  // testing (sim has no inversion table at all, so it shows the "true"
  // direction; real hardware was applying a redundant second flip on top of
  // the firmware's own). Bidirectional range meant the double-inversion
  // still landed in-range and looked plausible in isolated bench testing,
  // just mirrored — unlike the elbows, this didn't clamp to a dead stop, so
  // it wasn't caught until compared side-by-side against sim.
  //
  // Every registry ID the firmware fits is now uninverted on the ROS side.
  // If a joint is later found to need inversion again, that means the
  // firmware's own internal handling changed — re-verify against the CDC
  // spec's current wording before re-adding an entry here.
  (void)servo_id;
  return false;
}

// ════════════════════════════════════════════════════════════════════
//  Framing
// ════════════════════════════════════════════════════════════════════

bool EspHardwareInterface::sendPacket(
  uint8_t packet_type, const uint8_t * payload, uint16_t length)
{
  if (serial_fd_ < 0 || !connected_) {
    return false;
  }
  if (length > cdc_protocol::MAX_PAYLOAD_SIZE) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "Payload too large: %u (max %u)",
      length, cdc_protocol::MAX_PAYLOAD_SIZE);
    return false;
  }

  // 0xAA 0x55 | type | length (2B LE) | payload   — no checksum
  uint8_t packet[cdc_protocol::HEADER_SIZE + cdc_protocol::MAX_PAYLOAD_SIZE];
  packet[0] = cdc_protocol::MAGIC1;
  packet[1] = cdc_protocol::MAGIC2;
  packet[2] = packet_type;
  packet[3] = static_cast<uint8_t>(length & 0xFF);
  packet[4] = static_cast<uint8_t>((length >> 8) & 0xFF);
  if (length > 0 && payload != nullptr) {
    std::memcpy(&packet[cdc_protocol::HEADER_SIZE], payload, length);
  }

  const size_t packet_size = cdc_protocol::HEADER_SIZE + length;
  size_t written_total = 0;
  while (written_total < packet_size) {
    const ssize_t written =
      ::write(serial_fd_, packet + written_total, packet_size - written_total);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Kernel buffer full: rare on CDC-ACM, and a partially written frame
        // would desync the receiver, so give the port a moment and retry.
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        continue;
      }
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "Serial write failed: %s", strerror(errno));
      return false;
    }
    written_total += static_cast<size_t>(written);
  }

  return true;
}

void EspHardwareInterface::pumpSerial()
{
  if (serial_fd_ < 0 || !connected_) {
    return;
  }

  uint8_t buffer[512];
  while (true) {
    int bytes_available = 0;
    if (ioctl(serial_fd_, FIONREAD, &bytes_available) < 0) {
      markDisconnected("FIONREAD failed");
      return;
    }
    if (bytes_available <= 0) {
      return;
    }

    const ssize_t n = ::read(
      serial_fd_, buffer,
      std::min<size_t>(sizeof(buffer), static_cast<size_t>(bytes_available)));
    if (n < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        return;
      }
      markDisconnected(strerror(errno));
      return;
    }
    if (n == 0) {
      return;
    }

    for (ssize_t i = 0; i < n; i++) {
      const uint8_t byte = buffer[i];
      switch (rx_state_) {
        case cdc_protocol::RxState::WAIT_MAGIC1:
          if (byte == cdc_protocol::MAGIC1) {
            rx_state_ = cdc_protocol::RxState::WAIT_MAGIC2;
          }
          break;

        case cdc_protocol::RxState::WAIT_MAGIC2:
          if (byte == cdc_protocol::MAGIC2) {
            rx_state_ = cdc_protocol::RxState::WAIT_TYPE;
          } else if (byte == cdc_protocol::MAGIC1) {
            // Stay put: this may be the real frame start.
            rx_state_ = cdc_protocol::RxState::WAIT_MAGIC2;
          } else {
            rx_state_ = cdc_protocol::RxState::WAIT_MAGIC1;
          }
          break;

        case cdc_protocol::RxState::WAIT_TYPE:
          rx_packet_type_ = byte;
          rx_state_ = cdc_protocol::RxState::WAIT_LENGTH_LOW;
          break;

        case cdc_protocol::RxState::WAIT_LENGTH_LOW:
          rx_length_ = byte;
          rx_state_ = cdc_protocol::RxState::WAIT_LENGTH_HIGH;
          break;

        case cdc_protocol::RxState::WAIT_LENGTH_HIGH:
          rx_length_ |= static_cast<uint16_t>(byte) << 8;
          rx_payload_index_ = 0;
          if (rx_length_ > cdc_protocol::MAX_PAYLOAD_SIZE) {
            RCLCPP_WARN(
              rclcpp::get_logger(kLogger),
              "Oversized frame (type 0x%02X, len %u); resyncing",
              rx_packet_type_, rx_length_);
            rx_state_ = cdc_protocol::RxState::WAIT_MAGIC1;
          } else if (rx_length_ == 0) {
            processPacket(rx_packet_type_, nullptr, 0);
            rx_state_ = cdc_protocol::RxState::WAIT_MAGIC1;
          } else {
            rx_state_ = cdc_protocol::RxState::WAIT_PAYLOAD;
          }
          break;

        case cdc_protocol::RxState::WAIT_PAYLOAD:
          rx_payload_[rx_payload_index_++] = byte;
          if (rx_payload_index_ >= rx_length_) {
            processPacket(rx_packet_type_, rx_payload_, rx_length_);
            rx_state_ = cdc_protocol::RxState::WAIT_MAGIC1;
          }
          break;
      }
    }
  }
}

bool EspHardwareInterface::waitFor(const bool & flag, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!flag && std::chrono::steady_clock::now() < deadline) {
    pumpSerial();
    if (flag) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return flag;
}

void EspHardwareInterface::processPacket(
  uint8_t packet_type, const uint8_t * payload, uint16_t length)
{
  switch (packet_type) {
    case cdc_protocol::RESP_ENCODERS:
      if (length == cdc_protocol::ENCODER_PAYLOAD_SIZE) {
        std::memcpy(&encoder_left_, &payload[0], sizeof(int32_t));
        std::memcpy(&encoder_right_, &payload[4], sizeof(int32_t));
        encoder_data_ready_ = true;
      } else {
        RCLCPP_WARN(
          rclcpp::get_logger(kLogger), "RESP_ENCODERS length %u (expected %u)",
          length, cdc_protocol::ENCODER_PAYLOAD_SIZE);
      }
      break;

    case cdc_protocol::RESP_IMU:
      processImu(payload, length);
      break;

    case cdc_protocol::RESP_SERVO_FEEDBACK:
      processServoFeedback(payload, length);
      break;

    case cdc_protocol::RESP_BATTERY:
      processBattery(payload, length);
      break;

    case cdc_protocol::CMD_SHUTDOWN:
      handleShutdown();
      break;

    case cdc_protocol::CMD_WIFI_CONFIG:
      handleWifiConfig(payload, length);
      break;

    case cdc_protocol::CMD_WIFI_STATUS:
      replyWifiStatus();
      break;

    case cdc_protocol::RESP_ACK:
      break;

    case cdc_protocol::RESP_NACK:
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "ESP NACKed the last command");
      break;

    default:
      RCLCPP_DEBUG(
        rclcpp::get_logger(kLogger), "Unhandled packet type 0x%02X (len %u)",
        packet_type, length);
      break;
  }
}

// ════════════════════════════════════════════════════════════════════
//  Response handlers
// ════════════════════════════════════════════════════════════════════

void EspHardwareInterface::processImu(const uint8_t * payload, uint16_t length)
{
  if (length != cdc_protocol::IMU_PAYLOAD_SIZE) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger), "RESP_IMU length %u (expected %u)",
      length, cdc_protocol::IMU_PAYLOAD_SIZE);
    return;
  }

  float gx_dps, gy_dps, gz_dps;
  std::memcpy(&imu_ax_, &payload[0], sizeof(float));
  std::memcpy(&imu_ay_, &payload[4], sizeof(float));
  std::memcpy(&imu_az_, &payload[8], sizeof(float));
  std::memcpy(&gx_dps, &payload[12], sizeof(float));
  std::memcpy(&gy_dps, &payload[16], sizeof(float));
  std::memcpy(&gz_dps, &payload[20], sizeof(float));

  // Spec states gyro is deg/s; sensor_msgs/Imu requires rad/s.
  imu_gx_ = static_cast<float>(gx_dps * kDegToRad);
  imu_gy_ = static_cast<float>(gy_dps * kDegToRad);
  imu_gz_ = static_cast<float>(gz_dps * kDegToRad);
  imu_data_ready_ = true;

  if (!imu_publisher_) {
    return;
  }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = imu_frame_id_;

  msg.linear_acceleration.x = imu_ax_;
  msg.linear_acceleration.y = imu_ay_;
  msg.linear_acceleration.z = imu_az_;
  msg.angular_velocity.x = imu_gx_;
  msg.angular_velocity.y = imu_gy_;
  msg.angular_velocity.z = imu_gz_;

  msg.orientation_covariance[0] = -1.0;   // no orientation estimate

  msg.linear_acceleration_covariance[0] = 0.01;
  msg.linear_acceleration_covariance[4] = 0.01;
  msg.linear_acceleration_covariance[8] = 0.01;
  msg.angular_velocity_covariance[0] = 0.0005;
  msg.angular_velocity_covariance[4] = 0.0005;
  // gz is tight on purpose: ekf.yaml fuses only this axis, and diff_cont's yaw
  // covariance is inflated so the EKF prefers the gyro over wheel-derived yaw
  // (slip rejection). Changing this rebalances that trade-off.
  msg.angular_velocity_covariance[8] = 0.0001;

  imu_publisher_->publish(msg);
}

void EspHardwareInterface::processServoFeedback(const uint8_t * payload, uint16_t length)
{
  if (length < 1) {
    return;
  }

  const uint8_t count = payload[0];
  uint16_t idx = 1;

  for (uint8_t i = 0; i < count; i++) {
    if (idx + cdc_protocol::SERVO_FEEDBACK_ENTRY_SIZE > length) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLogger), "Truncated servo feedback at entry %u/%u", i + 1, count);
      break;
    }

    const uint8_t servo_id = payload[idx++];
    float angle_deg;
    std::memcpy(&angle_deg, &payload[idx], sizeof(float));
    idx += sizeof(float);

    double angle_rad = angle_deg * kDegToRad;
    if (isServoInverted(servo_id)) {
      angle_rad = -angle_rad;   // mirror of the inversion applied when sending
    }

    auto it = servo_id_to_joint_index_.find(servo_id);
    if (it != servo_id_to_joint_index_.end()) {
      hw_positions_[it->second] = angle_rad;
    }
  }
}

void EspHardwareInterface::processBattery(const uint8_t * payload, uint16_t length)
{
  if (length < cdc_protocol::BATTERY_PAYLOAD_SIZE) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger), "RESP_BATTERY length %u (expected %u)",
      length, cdc_protocol::BATTERY_PAYLOAD_SIZE);
    return;
  }
  if (!battery_publisher_) {
    return;
  }

  float voltage, current, soc_percent;
  std::memcpy(&voltage, &payload[0], sizeof(float));
  std::memcpy(&current, &payload[4], sizeof(float));
  std::memcpy(&soc_percent, &payload[8], sizeof(float));

  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = node_->now();
  msg.voltage = voltage;
  msg.current = current;
  msg.percentage = soc_percent / 100.0f;    // BatteryState wants 0..1
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.present = true;
  battery_publisher_->publish(msg);

  // No servo census on CDC's RESP_BATTERY (12B: voltage/current/SOC% only) —
  // that trailing active-count + online-IDs field only exists on the BLE
  // RESP_BATTERY payload, not this one.
}

// ════════════════════════════════════════════════════════════════════
//  Wi-Fi relay
// ════════════════════════════════════════════════════════════════════

void EspHardwareInterface::handleWifiConfig(const uint8_t * payload, uint16_t length)
{
  // The ESP forwards its fixed `WifiConfigPayload` struct verbatim
  // (firmware include/protocol_defs.h): char ssid[32] then char password[64],
  // each null-PADDED at a fixed offset — NOT null-SEPARATED. Scanning for a
  // separator instead read the ssid correctly and then hit the zero padding
  // immediately after it, yielding an empty password and an nmcli join that
  // could never authenticate.
  constexpr size_t kSsidOffset = 0;
  constexpr size_t kSsidLen = 32;
  constexpr size_t kPasswordOffset = kSsidOffset + kSsidLen;
  constexpr size_t kPasswordLen = 64;

  if (!wifi_credentials_publisher_) {
    return;
  }

  if (length < kPasswordOffset + kPasswordLen) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "CMD_WIFI_CONFIG payload is %u B; expected %zu B — ignoring",
      length, kPasswordOffset + kPasswordLen);
    return;
  }

  const auto field = [payload](size_t offset, size_t max) {
    const char * start = reinterpret_cast<const char *>(payload) + offset;
    return std::string(start, strnlen(start, max));
  };

  const std::string ssid = field(kSsidOffset, kSsidLen);
  const std::string password = field(kPasswordOffset, kPasswordLen);

  if (ssid.empty()) {
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "CMD_WIFI_CONFIG has an empty SSID — ignoring");
    return;
  }

  std_msgs::msg::String msg;
  msg.data = ssid + "\n" + password;
  wifi_credentials_publisher_->publish(msg);

  // SSID only — the password must not reach the logs.
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "Wi-Fi credentials relayed from phone (ssid=%s)",
    ssid.c_str());
}

void EspHardwareInterface::handleShutdown()
{
  // ACK FIRST. The ESP retries CMD_SHUTDOWN three times at 1.5 s and cuts the
  // power latch at 25 s regardless (firmware src/latch_switch.cpp), so this
  // ack is what stops the retries and buys the Pi its unmount window. Sending
  // it after the publish would still be correct; sending it after anything
  // that can block would not.
  sendPacket(cdc_protocol::RESP_ACK, nullptr, 0);

  if (!shutdown_publisher_) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger),
      "CMD_SHUTDOWN arrived but /esp/shutdown has no publisher — the Pi will "
      "NOT halt and the ESP will cut power in ~25 s");
    return;
  }

  RCLCPP_WARN(
    rclcpp::get_logger(kLogger),
    "CMD_SHUTDOWN from the ESP (power button) — acked, relaying to robot_app");
  shutdown_publisher_->publish(std_msgs::msg::Empty());
}

void EspHardwareInterface::replyWifiStatus()
{
  std::vector<uint8_t> payload;
  {
    std::lock_guard<std::mutex> lock(wifi_status_mutex_);
    payload = wifi_status_payload_;
  }
  if (payload.size() != cdc_protocol::WIFI_STATUS_PAYLOAD_SIZE) {
    payload.assign(cdc_protocol::WIFI_STATUS_PAYLOAD_SIZE, 0);
  }
  sendPacket(cdc_protocol::CMD_WIFI_STATUS, payload.data(),
    static_cast<uint16_t>(payload.size()));
}

// ════════════════════════════════════════════════════════════════════
//  Serial port
// ════════════════════════════════════════════════════════════════════

bool EspHardwareInterface::openSerialPort()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "open(%s) failed: %s",
      serial_port_.c_str(), strerror(errno));
    return false;
  }

  struct termios options;
  if (tcgetattr(serial_fd_, &options) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "tcgetattr failed: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Baud is cosmetic on CDC-ACM (the USB link runs at its own rate), but termios
  // still wants a valid value.
  speed_t baud;
  switch (serial_baud_) {
    case 9600:   baud = B9600;   break;
    case 19200:  baud = B19200;  break;
    case 38400:  baud = B38400;  break;
    case 57600:  baud = B57600;  break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 921600: baud = B921600; break;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger(kLogger),
        "Unsupported baud %d; using 115200 (ignored by CDC-ACM anyway)", serial_baud_);
      baud = B115200;
      break;
  }
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_cflag &= ~PARENB;          // no parity
  options.c_cflag &= ~CSTOPB;          // 1 stop bit
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;              // 8 data bits
  options.c_cflag &= ~CRTSCTS;         // no hardware flow control
  options.c_cflag |= CREAD | CLOCAL;

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;   // fully non-blocking; pumpSerial gates on FIONREAD

  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &options) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "tcsetattr failed: %s", strerror(errno));
    ::close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));   // let the port settle
  connected_ = true;
  clearBuffer();

  // A frame half-read before a disconnect must not merge with the first frame
  // after reconnecting.
  rx_state_ = cdc_protocol::RxState::WAIT_MAGIC1;
  rx_payload_index_ = 0;

  reconnect_backoff_ = std::chrono::milliseconds(1000);

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Serial port open: %s", serial_port_.c_str());
  return true;
}

void EspHardwareInterface::closeSerialPort()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  connected_ = false;
}

void EspHardwareInterface::markDisconnected(const char * reason)
{
  if (!connected_) {
    return;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger(kLogger), "ESP32 link lost (%s); will retry %s",
    reason, serial_port_.c_str());
  closeSerialPort();
  next_reconnect_attempt_ = std::chrono::steady_clock::now() + reconnect_backoff_;
}

void EspHardwareInterface::attemptReconnect()
{
  // USB CDC re-enumerates on replug — unlike the fixed UART pins this used to
  // run on, where a dropped link meant the node was dead anyway. Retry on a
  // backoff instead of requiring a restart.
  const auto now = std::chrono::steady_clock::now();
  if (now < next_reconnect_attempt_) {
    return;
  }

  if (!openSerialPort()) {
    reconnect_backoff_ = std::min(reconnect_backoff_ * 2, kReconnectBackoffMax);
    next_reconnect_attempt_ = now + reconnect_backoff_;
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Reconnected; re-running handshake");

  // The ESP rebooted or was replugged, so its encoder origin is unknown —
  // redo what on_activate() established. Servo torque needs no re-enable
  // (CMD_SERVO_MULTI auto-engages it) and feedback polling resumes on its own
  // next decimated read() cycle — neither needs a setup call here.
  clearBuffer();
  sendPacket(cdc_protocol::CMD_RESET_ENCODERS, nullptr, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pumpSerial();

  // Force a fresh command on the next write() — the cached "last sent" values
  // are meaningless to a freshly booted ESP.
  std::fill(
    last_servo_commands_.begin(), last_servo_commands_.end(),
    std::numeric_limits<double>::quiet_NaN());
}

void EspHardwareInterface::clearBuffer()
{
  if (serial_fd_ < 0) {
    return;
  }
  tcflush(serial_fd_, TCIFLUSH);

  uint8_t scratch[256];
  while (true) {
    int bytes_available = 0;
    if (ioctl(serial_fd_, FIONREAD, &bytes_available) < 0 || bytes_available <= 0) {
      break;
    }
    if (::read(serial_fd_, scratch, sizeof(scratch)) <= 0) {
      break;
    }
  }
}

// ════════════════════════════════════════════════════════════════════
//  ros2_control interface export
// ════════════════════════════════════════════════════════════════════

std::vector<hardware_interface::StateInterface> EspHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EspHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &hw_commands_[i]));
  }
  return command_interfaces;
}

}  // namespace bonicbot_a2_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  bonicbot_a2_hardware::EspHardwareInterface, hardware_interface::SystemInterface)
