#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/empty.hpp>

#include <memory>
#include <vector>
#include <cmath>

namespace drone_mpc_control {

class TrajectoryGeneratorNode : public rclcpp::Node {
public:
  TrajectoryGeneratorNode();
  ~TrajectoryGeneratorNode() override = default;

private:
  struct Waypoint {
    Eigen::Vector3d position;
    double velocity;
  };

  void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg);
  void timerCallback();

  nav_msgs::msg::Path generateCircularTrajectory(double radius, double height, double num_points);
  nav_msgs::msg::Path generateWaypointTrajectory(const std::vector<Waypoint>& waypoints, double dt);
  nav_msgs::msg::Path generateHoverTrajectory(const Eigen::Vector3d& position, double duration);

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr hover_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<Waypoint> waypoints_;
  bool has_waypoints_{false};
  int current_waypoint_index_{0};
  double trajectory_publish_rate_{10.0};
  double waypoint_reached_threshold_{0.2};
};

TrajectoryGeneratorNode::TrajectoryGeneratorNode()
  : Node("trajectory_generator_node") {

  this->declare_parameter("trajectory_publish_rate", trajectory_publish_rate_);
  this->declare_parameter("waypoint_reached_threshold", waypoint_reached_threshold_);

  trajectory_publish_rate_ = this->get_parameter("trajectory_publish_rate").as_double();
  waypoint_reached_threshold_ = this->get_parameter("waypoint_reached_threshold").as_double();

  goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/goal_position", 10,
    std::bind(&TrajectoryGeneratorNode::goalCallback, this, std::placeholders::_1));

  hover_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/hover_command", 10,
    [this](const std_msgs::msg::Empty::SharedPtr) {
      auto trajectory = generateHoverTrajectory(Eigen::Vector3d(0, 0, 5.0), 10.0);
      trajectory_pub_->publish(trajectory);
    });

  trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/reference_trajectory", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / trajectory_publish_rate_),
    std::bind(&TrajectoryGeneratorNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Trajectory Generator node initialized");
}

void TrajectoryGeneratorNode::goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
  Waypoint wp;
  wp.position.x() = msg->x;
  wp.position.y() = msg->y;
  wp.position.z() = msg->z;
  wp.velocity = 1.0;

  waypoints_.push_back(wp);
  has_waypoints_ = true;

  RCLCPP_INFO(this->get_logger(), "Added waypoint: [%.2f, %.2f, %.2f]",
              msg->x, msg->y, msg->z);
}

void TrajectoryGeneratorNode::timerCallback() {
  if (waypoints_.empty()) {
    auto trajectory = generateHoverTrajectory(Eigen::Vector3d(0, 0, 0.0), 1.0);
    trajectory_pub_->publish(trajectory);
    return;
  }

  auto trajectory = generateWaypointTrajectory(waypoints_, 0.5);
  trajectory_pub_->publish(trajectory);
}

nav_msgs::msg::Path TrajectoryGeneratorNode::generateCircularTrajectory(
    double radius, double height, double num_points) {

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "map";

  for (int i = 0; i < num_points; ++i) {
    double theta = 2.0 * M_PI * i / num_points;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now() + rclcpp::Duration::from_seconds(i * 0.1);
    pose.header.frame_id = "map";

    pose.pose.position.x = radius * std::cos(theta);
    pose.pose.position.y = radius * std::sin(theta);
    pose.pose.position.z = height;

    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

nav_msgs::msg::Path TrajectoryGeneratorNode::generateWaypointTrajectory(
    const std::vector<Waypoint>& waypoints, double dt) {

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "map";

  if (waypoints.empty()) {
    return path;
  }

  for (size_t i = 0; i < waypoints.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now() + rclcpp::Duration::from_seconds(i * dt);
    pose.header.frame_id = "map";

    pose.pose.position.x = waypoints[i].position.x();
    pose.pose.position.y = waypoints[i].position.y();
    pose.pose.position.z = waypoints[i].position.z();

    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

nav_msgs::msg::Path TrajectoryGeneratorNode::generateHoverTrajectory(
    const Eigen::Vector3d& position, double duration) {

  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "map";

  double dt = 0.1;
  int num_points = static_cast<int>(duration / dt);

  for (int i = 0; i < num_points; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now() + rclcpp::Duration::from_seconds(i * dt);
    pose.header.frame_id = "map";

    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = position.z();

    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;

    path.poses.push_back(pose);
  }

  return path;
}

} 

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drone_mpc_control::TrajectoryGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
