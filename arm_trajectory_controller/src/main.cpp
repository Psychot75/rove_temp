#include "rclcpp/rclcpp.hpp"
#include "arm_trajectory_controller_node.cpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTrajectoryControllerNode>());
  rclcpp::shutdown();
  return 0;
}