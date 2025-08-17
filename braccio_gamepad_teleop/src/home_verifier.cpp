#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "tf2_ros/buffer.h"
#include <chrono>
#include <cmath>

class HomeVerifier : public rclcpp::Node {
public:
  HomeVerifier() : Node("home_verifier") {
    RCLCPP_INFO(this->get_logger(), "=== POSITION READER - ACTUAL ===");
    RCLCPP_INFO(this->get_logger(), "Mostrando la posición actual del brazo en radianes y grados");
    controller_state_sub = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
        "/position_trajectory_controller/controller_state", 10,
        std::bind(&HomeVerifier::controllerStateCallback, this, std::placeholders::_1));
  }
private:
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr controller_state_sub;
  std::vector<std::string> controller_joint_names;
  std::vector<double> current_position;

  void controllerStateCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg) {
    controller_joint_names = msg->joint_names;
    current_position = msg->actual.positions;
    if (current_position.size() >= 5) {
      RCLCPP_INFO(this->get_logger(), "\n[JOINTS] %s", jointsToString(controller_joint_names).c_str());
      RCLCPP_INFO(this->get_logger(), "[RADIANES] %s", positionsToString(current_position).c_str());
      RCLCPP_INFO(this->get_logger(), "[GRADOS]   %s", positionsToStringDegrees(current_position).c_str());
    }
  }
  std::string jointsToString(const std::vector<std::string>& joints) {
    std::string result = "[";
    for (size_t i = 0; i < joints.size(); ++i) {
      result += joints[i];
      if (i + 1 < joints.size()) result += ", ";
    }
    result += "]";
    return result;
  }
  std::string positionsToString(const std::vector<double>& positions) {
    std::string result = "[";
    for (size_t i = 0; i < std::min(positions.size(), size_t(5)); ++i) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%.4f", positions[i]);
      result += buf;
      if (i + 1 < std::min(positions.size(), size_t(5))) result += ", ";
    }
    result += "]";
    return result;
  }
  std::string positionsToStringDegrees(const std::vector<double>& positions) {
    std::string result = "[";
    for (size_t i = 0; i < std::min(positions.size(), size_t(5)); ++i) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%.1f°", positions[i] * 180.0/M_PI);
      result += buf;
      if (i + 1 < std::min(positions.size(), size_t(5))) result += ", ";
    }
    result += "]";
    return result;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HomeVerifier>();
  RCLCPP_INFO(node->get_logger(), "Presiona Ctrl+C para salir");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
