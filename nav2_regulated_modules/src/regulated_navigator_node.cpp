#include <memory>

#include "nav2_regulated_modules/regulated_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

// 中文注释：进程入口初始化 ROS 2，创建 Lifecycle 导航器并交给多线程执行器运行。
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_regulated_modules::RegulatedNavigator>();
  // 中文注释：多线程执行器允许 Action 结果、反馈和定位监控并发推进。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
