#include <chrono>
#include <vector>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class ArmTrajectoryControllerNode : public rclcpp::Node
{
public:
  ArmTrajectoryControllerNode()
  : Node("arm_trajectory_controller_node")
  {
    // 1) subscribe to incoming joint trajectories from MoveIt (radians)
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "kinova_arm/joint_trajectory", 10,
      std::bind(&ArmTrajectoryControllerNode::traj_callback, this, _1)
    );

    // 2) publish desired joint positions to the Kinova controller (expects degrees)
    joint_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "kinova_arm/joints/setPositions", 10
    );

    // 3) subscribe to actual feedback from the arm (degrees)
    positions_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "kinova_arm/joints/positions", 10,
      std::bind(&ArmTrajectoryControllerNode::positions_callback, this, _1)
    );

    // 4) publish a proper JointState stream (radians) for MoveIt/Servo
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10
    );

    // 5) define your joint names: 4 flippers + 6 arm joints
    joint_names_ = {
      "RoveCore_FlipperBLJoint",
      "RoveCore_FlipperBRJoint",
      "RoveCore_FlipperFRJoint",
      "RoveCore_FlipperFLJoint",
      "RoveCore_Joint1",
      "ArmSectionA_Joint3",
      "ArmSectionB_Joint4",
      "ArmSectionC_Joint5",
      "ArmSectionD_Joint6",
      "ArmBase_Revolute-51"
    };
  }

private:
  // callback: convert MoveIt trajectory (radians) → Kinova setPositions (degrees)
  void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received trajectory with %zu point(s)", msg->points.size());
    if (msg->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty trajectory received, ignoring.");
      return;
    }
    const auto& point = msg->points.front();
    if (point.positions.size() < 6) {
      RCLCPP_WARN(this->get_logger(), "Trajectory point has insufficient positions.");
      return;
    }

    std_msgs::msg::Float64MultiArray out_msg;
    // We always send 10 values: 4 flipper zeros + 6 arm angles
    out_msg.data.resize(10);
    // Zero flipper commands
    for (size_t i = 0; i < 4; ++i) {
      out_msg.data[i] = 0.0;
    }
    // Fill arm joint commands (radians→degrees)
    for (size_t i = 0; i < 6; ++i) {
      out_msg.data[4 + i] = point.positions[i] * 180.0 / M_PI;
    }

    RCLCPP_INFO(this->get_logger(), "Publishing %zu joint positions (deg)", out_msg.data.size());
    joint_position_pub_->publish(out_msg);
  }

  // callback: republish arm feedback (degrees) → JointState (radians) with 10 joints
  void positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 6) {
      RCLCPP_ERROR(this->get_logger(),
                   "Expected 6 feedback values but got %zu", msg->data.size());
      return;
    }
    sensor_msgs::msg::JointState js;
    // Manually assign header stamp
    auto now = this->get_clock()->now();
    uint64_t ns = now.nanoseconds();
    js.header.stamp.sec = static_cast<int32_t>(ns / 1000000000ULL);
    js.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);

    js.name = joint_names_;
    js.position.resize(joint_names_.size());
    // Flippers are always zero
    for (size_t i = 0; i < 4; ++i) {
      js.position[i] = 0.0;
    }
    // Fill arm joint positions (degrees→radians)
    for (size_t i = 0; i < 6; ++i) {
      js.position[4 + i] = msg->data[i] * M_PI / 360.0;
    }

    joint_state_pub_->publish(js);
  }

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr positions_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::vector<std::string> joint_names_;
};
