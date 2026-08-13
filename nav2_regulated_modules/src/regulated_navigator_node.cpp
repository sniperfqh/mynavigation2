#include <memory>

#include "nav2_regulated_modules/regulated_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  SpdlogWrapper::init("nav2_regulated_modules", "regulated_navigator");
  auto node = std::make_shared<nav2_regulated_modules::RegulatedNavigator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  SpdlogWrapper::shutdown();
  return 0;
}
