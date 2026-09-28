// 规控节点可执行入口。初始化 ROS 与日志，使用多线程执行器运行生命周期节点并有序退出。

#include <memory>

#include "nav2_regulated_modules/regulated_navigator.hpp"
#include "rclcpp/rclcpp.hpp"

// 初始化 ROS 和日志系统，以多线程执行器运行节点并在退出时释放资源。
int main(int argc, char ** argv)
{
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
