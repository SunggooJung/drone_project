#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <memory>

namespace drone_mpc_control {

class PX4BridgeNode : public rclcpp::Node {
public:
  PX4BridgeNode();
  ~PX4BridgeNode() override = default;

private:
  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void timerCallback();

  void arm();
  void disarm();
  void enableOffboard();

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  uint8_t nav_state_{0};
  uint8_t arming_state_{0};
  bool offboard_enabled_{false};
};

PX4BridgeNode::PX4BridgeNode()
  : Node("px4_bridge_node") {

  status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status", 10,
    std::bind(&PX4BridgeNode::statusCallback, this, std::placeholders::_1));

  control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);

  command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PX4BridgeNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "PX4 Bridge node initialized");
}

void PX4BridgeNode::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  nav_state_ = msg->nav_state;
  arming_state_ = msg->arming_state;

  if (!offboard_enabled_) {
    enableOffboard();
  }
}

void PX4BridgeNode::timerCallback() {
  if (!offboard_enabled_) {
    return;
  }

  auto control_mode = px4_msgs::msg::OffboardControlMode();
  control_mode.position = false;
  control_mode.velocity = false;
  control_mode.acceleration = false;
  control_mode.attitude = true;
  control_mode.body_rate = false;
  control_mode.thrust_and_torque = true;
  control_mode.timestamp = this->now().nanoseconds() / 1000;
  control_mode_pub_->publish(control_mode);
}

void PX4BridgeNode::arm() {
  auto command = px4_msgs::msg::VehicleCommand();
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  command.param1 = 1.0;
  command.param2 = 21196.0;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 1;
  command.from_external = true;
  command.timestamp = this->now().nanoseconds() / 1000;
  command_pub_->publish(command);

  RCLCPP_INFO(this->get_logger(), "Arming command sent");
}

void PX4BridgeNode::disarm() {
  auto command = px4_msgs::msg::VehicleCommand();
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
  command.param1 = 0.0;
  command.param2 = 21196.0;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 1;
  command.from_external = true;
  command.timestamp = this->now().nanoseconds() / 1000;
  command_pub_->publish(command);

  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void PX4BridgeNode::enableOffboard() {
  auto control_mode = px4_msgs::msg::OffboardControlMode();
  control_mode.position = false;
  control_mode.velocity = false;
  control_mode.acceleration = false;
  control_mode.attitude = true;
  control_mode.body_rate = false;
  control_mode.thrust_and_torque = true;
  control_mode.timestamp = this->now().nanoseconds() / 1000;
  control_mode_pub_->publish(control_mode);

  auto command = px4_msgs::msg::VehicleCommand();
  command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
  command.param1 = 1.0;
  command.param2 = 6.0;
  command.target_system = 1;
  command.target_component = 1;
  command.source_system = 1;
  command.source_component = 1;
  command.from_external = true;
  command.timestamp = this->now().nanoseconds() / 1000;
  command_pub_->publish(command);

  offboard_enabled_ = true;
  RCLCPP_INFO(this->get_logger(), "Offboard mode enabled");
}

} 

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_mpc_control::PX4BridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
