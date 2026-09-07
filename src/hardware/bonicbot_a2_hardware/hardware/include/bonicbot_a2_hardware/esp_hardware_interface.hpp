// BonicBot A2 — ros2_control SystemInterface for the ESP32-S3 over USB CDC.
//
// On A2 the ESP32 is the sole motion controller: wheels (PWM + encoders), all
// seven serial servos, the IMU and the battery monitor sit behind it, reached
// over one CDC-ACM link. This plugin is therefore an ACTIVE CDC master, not a
// relay (contrast M1, where the Jetson drives its actuators directly and the
// ESP bridge only forwards IMU/Wi-Fi traffic).
//
// It also serves the Wi-Fi relay topics. That is deliberate: SystemInterface
// has no node of its own, so an internal rclcpp::Node is created and spun for
// IMU/battery publishing, and the relay rides on that same node. A separate
// process cannot be used — two processes sharing one CDC byte stream would
// interleave and split frames.

#ifndef BONICBOT_A2_HARDWARE__ESP_HARDWARE_INTERFACE_HPP_
#define BONICBOT_A2_HARDWARE__ESP_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "bonicbot_a2_hardware/cdc_protocol.hpp"

// POSIX serial
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace bonicbot_a2_hardware
{

class EspHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EspHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Cancels the executor and joins spin_thread_, then resets the internal
  /// node and its publishers/subscriptions. Shared by on_cleanup() and
  /// on_shutdown() — process termination (SIGINT/SIGTERM) goes through
  /// deactivate() -> shutdown(), NEVER cleanup(), so this can't only live in
  /// on_cleanup() or a plain kill destroys a still-joinable std::thread,
  /// which is an unconditional std::terminate()/abort, no exception involved.
  void stopInternalNode();

  // ── serial port ──────────────────────────────────────────────
  bool openSerialPort();
  void closeSerialPort();
  void clearBuffer();
  /// Close and retry open() on a backoff. USB CDC re-enumerates on replug,
  /// unlike the fixed UART pins this used to run on, so a dropped link must be
  /// recoverable without restarting the node.
  void attemptReconnect();
  void markDisconnected(const char * reason);

  // ── framing ──────────────────────────────────────────────────
  bool sendPacket(uint8_t packet_type, const uint8_t * payload, uint16_t length);
  /// Drains everything currently buffered, feeding the RX state machine.
  void pumpSerial();
  void processPacket(uint8_t packet_type, const uint8_t * payload, uint16_t length);
  /// Pumps until `flag` goes true or `timeout` elapses. Returns flag state.
  bool waitFor(const bool & flag, std::chrono::milliseconds timeout);

  // ── motion ───────────────────────────────────────────────────
  bool sendMotorMove(double left_mps, double right_mps, double accel_mps2);
  bool sendServoPositions();
  bool setAllServoTorque(bool enabled);
  /// Requests one feedback poll for all mapped servos. No persistent stream —
  /// must be called on whatever interval the caller wants updates.
  bool requestServoFeedback();
  void processServoFeedback(const uint8_t * payload, uint16_t length);
  void processBattery(const uint8_t * payload, uint16_t length);
  void processImu(const uint8_t * payload, uint16_t length);
  void initializeServoMapping();

  /// Registry servo IDs whose sign is mirrored relative to the URDF joint axis.
  static bool isServoInverted(uint8_t servo_id);

  // ── Wi-Fi relay (rides the internal node; see file header) ───
  void handleWifiConfig(const uint8_t * payload, uint16_t length);
  void replyWifiStatus();
  void handleShutdown();

  // ── URDF parameters ──────────────────────────────────────────
  std::string serial_port_;
  int serial_baud_ = 115200;      // cosmetic on CDC-ACM, kept for termios
  double encoder_cpr_ = 14040.0;
  double wheel_radius_ = 0.06;
  double max_velocity_ = 10.47;   // rad/s
  double motor_accel_ = 1.5;      // m/s^2, CMD_MOTOR_MOVE accel field
  int servo_speed_ = 500;         // 0-1000, 0 = unrestricted
  int servo_accel_ = 50;          // 0-255
  double servo_position_threshold_ = 0.01;   // rad, ~0.57 deg
  int servo_feedback_interval_ms_ = 100;

  // ── connection state ─────────────────────────────────────────
  int serial_fd_ = -1;
  bool connected_ = false;
  std::chrono::steady_clock::time_point next_reconnect_attempt_{};
  std::chrono::milliseconds reconnect_backoff_{1000};
  static constexpr std::chrono::milliseconds kReconnectBackoffMax{5000};

  // ── RX state machine ─────────────────────────────────────────
  cdc_protocol::RxState rx_state_ = cdc_protocol::RxState::WAIT_MAGIC1;
  uint8_t  rx_packet_type_ = 0;
  uint16_t rx_length_ = 0;
  uint16_t rx_payload_index_ = 0;
  uint8_t  rx_payload_[cdc_protocol::MAX_PAYLOAD_SIZE];

  // ── joint state ──────────────────────────────────────────────
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> prev_positions_;
  std::vector<double> last_servo_commands_;

  size_t left_wheel_idx_ = 0;
  size_t right_wheel_idx_ = 1;

  int32_t encoder_left_ = 0;
  int32_t encoder_right_ = 0;
  bool encoder_data_ready_ = false;

  // ── servos ───────────────────────────────────────────────────
  std::map<std::string, uint8_t> joint_to_servo_id_;   // joint name -> registry ID
  std::map<uint8_t, size_t> servo_id_to_joint_index_;  // registry ID -> hw_* index
  std::vector<size_t> servo_joint_indices_;

  // ── IMU ──────────────────────────────────────────────────────
  float imu_ax_ = 0.0f, imu_ay_ = 0.0f, imu_az_ = 0.0f;   // m/s^2
  float imu_gx_ = 0.0f, imu_gy_ = 0.0f, imu_gz_ = 0.0f;   // rad/s (converted on parse)
  bool imu_data_ready_ = false;
  int imu_decimator_ = 0;
  std::string imu_frame_id_ = "imu_link";

  // ── battery ────────────────────────────────────────────────────
  // Voltage/SOC change slowly — 1 Hz is plenty at the 50 Hz control rate.
  int battery_decimator_ = 0;
  static constexpr int kBatteryDecimation = 50;

  // ── servo feedback polling ────────────────────────────────────
  // CMD_SERVO_FEEDBACK_REQUEST has no streaming mode: read() re-requests on
  // this decimation, derived from servo_feedback_interval_ms_ at the
  // controller_manager's fixed 50 Hz (20 ms) update rate.
  int servo_feedback_decimator_ = 0;
  int servo_feedback_decimation_ = 1;

  // ── internal node: publishers, Wi-Fi relay, spin thread ──────
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wifi_credentials_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wifi_status_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr face_matrix_subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr shutdown_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr shutdown_request_subscription_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;

  /// Latest /esp/wifi_status, packed into the 50-byte CDC payload ready to send.
  /// Written by the subscription callback on the spin thread, read by the
  /// control thread when the ESP asks — hence the mutex.
  std::vector<uint8_t> wifi_status_payload_;
  std::mutex wifi_status_mutex_;

  /// Set when a FRESH /esp/wifi_status arrives, so write() pushes it to the ESP
  /// unprompted instead of waiting to be polled. The ESP forwards any
  /// CMD_WIFI_STATUS carrying a payload straight out as a BLE RESP_WIFI_STATUS
  /// notify (firmware src/uart_ros.cpp), so this is what makes a successful
  /// join — and the new IP — reach the phone on its own.
  std::atomic<bool> wifi_status_push_pending_{false};

  /// Set when robot_app publishes /esp/shutdown_request, so write() sends
  /// CMD_SHUTDOWN to the ESP. Same push pattern as wifi_status_push_pending_
  /// and for the same reason: the serial fd belongs to the control thread, so
  /// a subscription callback may only raise the flag.
  std::atomic<bool> shutdown_push_pending_{false};

  /// Raw CMD_MATRIX_ACTION payload from /face/matrix_action — byte 0 is the
  /// action code, the rest is that action's own layout (spec §4). This is a
  /// dumb pipe: this repo doesn't interpret expressions, just forwards bytes.
  /// Written by the subscription callback on the spin thread, sent from
  /// write() on the control thread — same split as the Wi-Fi payload above,
  /// because only the control thread may touch the serial fd.
  std::vector<uint8_t> pending_matrix_action_;
  std::atomic<bool> matrix_action_pending_{false};
  std::mutex matrix_action_mutex_;
};

}  // namespace bonicbot_a2_hardware

#endif  // BONICBOT_A2_HARDWARE__ESP_HARDWARE_INTERFACE_HPP_
