#include <memory>

#include "object_tracking_2d/obstacle_detector.hpp"
// #include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<state_estimation::ObstacleDetector>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}