#include "ugv/control_node.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <utility>

namespace ugv {
namespace {
constexpr std::uint32_t kSerialPollPeriodMs = 20;
}

ControlNode::ControlNode(const rclcpp::NodeOptions& options)
    : Node("ugv_control_node", options),
      cmd_vel_topic_(declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel")),
      mcu_state_topic_(
          declare_parameter<std::string>("mcu_state_topic", "/ugv/mcu_state")),
      enable_mcu_commands_(declare_parameter<bool>("enable_mcu_commands", false)),
      cmd_vel_timeout_ms_(
          static_cast<std::uint32_t>(declare_parameter<int>("cmd_vel_timeout_ms", 500))),
      kinematics_(declare_parameter<double>("track_width_m", 0.0),
                  declare_parameter<double>("drive_sprocket_radius_m", 0.0)),
      serial_bridge_(make_usb_cdc_serial_bridge(SerialBridgeConfig{
          declare_parameter<std::string>("serial_device", "/dev/ttyACM0"),
          static_cast<std::uint32_t>(declare_parameter<int>("serial_baud_rate", 115200)),
          static_cast<std::uint32_t>(
              declare_parameter<int>("serial_reconnect_interval_ms", 1000)),
          true})),
      last_cmd_vel_time_(get_clock()->now()) {
  if (cmd_vel_timeout_ms_ == 0) {
    throw std::invalid_argument("cmd_vel_timeout_ms must be greater than zero");
  }
  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&ControlNode::on_cmd_vel, this, std::placeholders::_1));
  mcu_state_publisher_ =
      create_publisher<std_msgs::msg::String>(mcu_state_topic_, rclcpp::QoS(10));
  serial_poll_timer_ = create_wall_timer(
      std::chrono::milliseconds(kSerialPollPeriodMs),
      std::bind(&ControlNode::on_serial_poll, this));
  command_timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ControlNode::on_command_timeout, this));

  if (!kinematics_.configured()) {
    RCLCPP_WARN(get_logger(),
                "Track dimensions are unset. Set positive track_width_m and "
                "drive_sprocket_radius_m before wheel targets can be produced.");
  }
  if (!enable_mcu_commands_) {
    RCLCPP_INFO(get_logger(),
                "MCU drive enable is false; CDC status reception remains active.");
  }
}

ControlNode::~ControlNode() {
  if (command_active_) {
    send_disable_command();
  }
}

void ControlNode::on_cmd_vel(const geometry_msgs::msg::Twist::ConstSharedPtr message) {
  const auto targets = kinematics_.calculate(message->linear.x, message->angular.z);
  if (!targets) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejected cmd_vel: track dimensions are unset or input is non-finite.");
    return;
  }

  WheelVelocityCommand command{};
  command.sequence = ++sequence_;
  command.enable = enable_mcu_commands_;
  command.left_target_rad_s = command.enable ? targets->left_rad_s : 0.0;
  command.right_target_rad_s = command.enable ? targets->right_rad_s : 0.0;

  if (!serial_bridge_->send_command(command)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MCU command not sent: USB CDC device is not connected.");
    command_active_ = false;
    return;
  }
  last_cmd_vel_time_ = get_clock()->now();
  command_active_ = command.enable;
}

void ControlNode::on_serial_poll() {
  while (const auto status = serial_bridge_->poll_status()) {
    std_msgs::msg::String message;
    message.data = serialize_status_message(*status);
    mcu_state_publisher_->publish(message);
  }
}

void ControlNode::on_command_timeout() {
  if (!command_active_) {
    return;
  }
  const auto elapsed_ms = (get_clock()->now() - last_cmd_vel_time_).nanoseconds() / 1000000;
  if (elapsed_ms >= static_cast<std::int64_t>(cmd_vel_timeout_ms_)) {
    send_disable_command();
    command_active_ = false;
    RCLCPP_WARN(get_logger(), "cmd_vel timeout: sent MCU disable command.");
  }
}

void ControlNode::send_disable_command() {
  WheelVelocityCommand command{};
  command.sequence = ++sequence_;
  command.enable = false;
  command.left_target_rad_s = 0.0;
  command.right_target_rad_s = 0.0;
  static_cast<void>(serial_bridge_->send_command(command));
}

}  // namespace ugv

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ugv::ControlNode>());
  rclcpp::shutdown();
  return 0;
}
