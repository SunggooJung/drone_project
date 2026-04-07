#include "drone_mpc_control/mpc_controller.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <memory>
#include <cmath>

namespace drone_mpc_control {

class MPCControllerNode : public rclcpp::Node {
public:
  MPCControllerNode();
  ~MPCControllerNode() override = default;

private:
  void stateCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void timerCallback();

  void publishControl();

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<MPCController> mpc_controller_;
  MPCController::State current_state_;
  MPCController::TrajectoryPoint desired_state_;
  bool has_trajectory_{false};

  double control_rate_{20.0};
  double drone_mass_{1.5};
  int mpc_horizon_{20};
  double mpc_dt_{0.05};
};

MPCControllerNode::MPCControllerNode()
  : Node("mpc_controller_node") {

  this->declare_parameter("control_rate", control_rate_);
  this->declare_parameter("drone_mass", drone_mass_);
  this->declare_parameter("mpc_horizon", mpc_horizon_);
  this->declare_parameter("mpc_dt", mpc_dt_);

  control_rate_ = this->get_parameter("control_rate").as_double();
  drone_mass_ = this->get_parameter("drone_mass").as_double();
  mpc_horizon_ = this->get_parameter("mpc_horizon").as_int();
  mpc_dt_ = this->get_parameter("mpc_dt").as_double();

  mpc_controller_ = std::make_unique<MPCController>();
  mpc_controller_->configure(mpc_horizon_, mpc_dt_, drone_mass_);

  state_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_odometry", 10,
    std::bind(&MPCControllerNode::stateCallback, this, std::placeholders::_1));

  trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/reference_trajectory", 10,
    std::bind(&MPCControllerNode::trajectoryCallback, this, std::placeholders::_1));

  control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);

  setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&MPCControllerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "MPC Controller node initialized");
}

void MPCControllerNode::stateCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  current_state_.position.x() = msg->position[0];
  current_state_.position.y() = msg->position[1];
  current_state_.position.z() = msg->position[2];

  current_state_.velocity.x() = msg->velocity[0];
  current_state_.velocity.y() = msg->velocity[1];
  current_state_.velocity.z() = msg->velocity[2];

  current_state_.orientation.w() = msg->q[0];
  current_state_.orientation.x() = msg->q[1];
  current_state_.orientation.y() = msg->q[2];
  current_state_.orientation.z() = msg->q[3];

  current_state_.angular_velocity.x() = msg->angular_velocity[0];
  current_state_.angular_velocity.y() = msg->angular_velocity[1];
  current_state_.angular_velocity.z() = msg->angular_velocity[2];
}

void MPCControllerNode::trajectoryCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty trajectory");
    return;
  }

  desired_state_.position.x() = msg->poses[0].pose.position.x;
  desired_state_.position.y() = msg->poses[0].pose.position.y;
  desired_state_.position.z() = msg->poses[0].pose.position.z;

  desired_state_.velocity.x() = 0.0;
  desired_state_.velocity.y() = 0.0;
  desired_state_.velocity.z() = 0.0;

  desired_state_.acceleration.x() = 0.0;
  desired_state_.acceleration.y() = 0.0;
  desired_state_.acceleration.z() = 0.0;

  has_trajectory_ = true;
  RCLCPP_INFO(this->get_logger(), "Updated reference trajectory");
}

void MPCControllerNode::timerCallback() {
  if (!has_trajectory_) {
    return;
  }

  publishControl();
}

void MPCControllerNode::publishControl() {
  auto control = mpc_controller_->computeControl(current_state_, desired_state_);

  auto control_mode = px4_msgs::msg::OffboardControlMode();
  control_mode.position = false;
  control_mode.velocity = false;
  control_mode.acceleration = false;
  control_mode.attitude = true;
  control_mode.body_rate = false;
  control_mode.thrust_and_torque = true;
  control_mode.timestamp = this->now().nanoseconds() / 1000;
  control_mode_pub_->publish(control_mode);

  auto setpoint = px4_msgs::msg::TrajectorySetpoint();
  
  double total_thrust = control.thrust.sum();
  Eigen::Vector3d thrust_vec;
  thrust_vec.x() = (control.thrust[0] - control.thrust[2]) / total_thrust;
  thrust_vec.y() = (control.thrust[1] - control.thrust[3]) / total_thrust;
  thrust_vec.z() = total_thrust;

  setpoint.position[0] = desired_state_.position.x();
  setpoint.position[1] = desired_state_.position.y();
  setpoint.position[2] = desired_state_.position.z();

  setpoint.velocity[0] = desired_state_.velocity.x();
  setpoint.velocity[1] = desired_state_.velocity.y();
  setpoint.velocity[2] = desired_state_.velocity.z();

  setpoint.acceleration[0] = thrust_vec.x();
  setpoint.acceleration[1] = thrust_vec.y();
  setpoint.acceleration[2] = thrust_vec.z();

  setpoint.yaw = std::atan2(2.0 * current_state_.orientation.y() * current_state_.orientation.w() +
                            2.0 * current_state_.orientation.z() * current_state_.orientation.x(),
                            1.0 - 2.0 * current_state_.orientation.y() * current_state_.orientation.y() -
                            2.0 * current_state_.orientation.z() * current_state_.orientation.z());

  setpoint.yawspeed = 0.0;

  setpoint.timestamp = this->now().nanoseconds() / 1000;
  setpoint_pub_->publish(setpoint);
}

} 

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_mpc_control::MPCControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
