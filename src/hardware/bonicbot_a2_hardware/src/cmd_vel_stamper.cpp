// Twist -> TwistStamped bridge between twist_mux and diff_cont.
//
// On Jazzy diff_drive_controller accepts ONLY geometry_msgs/TwistStamped on
// ~/cmd_vel: `use_stamped_vel` and ~/cmd_vel_unstamped are gone. Everything
// that drives this base still publishes plain Twist into twist_mux —
// robot_app, Nav2's velocity_smoother and teleop_twist_joy — so twist_mux runs
// unstamped (use_stamped: false in twist_mux.yaml) and this node stamps its
// output on the way into the controller:
//
//     twist_mux --/cmd_vel_muxed (Twist)--> cmd_vel_stamper
//               --/diff_cont/cmd_vel (TwistStamped)--> diff_cont
//
// The stamp is the receive time on THIS node's clock. diff_cont drops any
// command older than its cmd_vel_timeout (0.5 s) against its own clock, so in
// simulation this node must run with use_sim_time:=true like everything else.
//
// C++ rather than a rclpy script: it sits on every drive command, and on the
// Pi a Python callback per message is the cost hardware.launch.py's
// joint_states throttle exists to avoid.

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bonicbot_a2_hardware
{

class CmdVelStamper : public rclcpp::Node
{
public:
  CmdVelStamper()
  : Node("cmd_vel_stamper")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    // Depth 1, matching twist_mux's own output: only the newest command is
    // worth anything, and a backlog would replay stale motion.
    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_out", rclcpp::QoS(1));
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_in", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        geometry_msgs::msg::TwistStamped out;
        out.header.stamp = now();
        out.header.frame_id = frame_id_;
        out.twist = *msg;
        pub_->publish(out);
      });
  }

private:
  std::string frame_id_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

}  // namespace bonicbot_a2_hardware

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bonicbot_a2_hardware::CmdVelStamper>());
  rclcpp::shutdown();
  return 0;
}
