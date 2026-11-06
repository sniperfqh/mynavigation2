#ifndef NAV2_REGULATED_MODULES__TF_DIAGNOSTICS_HPP_
#define NAV2_REGULATED_MODULES__TF_DIAGNOSTICS_HPP_

#include <map>
#include <mutex>
#include <string>

#include "nav2_regulated_modules/detail/tf_timing.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_regulated_modules
{
// fixed_path 主程序内的被动诊断；独立于任务与速度日志开关，不发布 TF 或改变控制判据。
class TfDiagnostics
{
public:
  TfDiagnostics(nav2_util::LifecycleNode & node, tf2_ros::Buffer & buffer, std::string global_frame, std::string base_frame, double frequency, double map_base_frequency);

private:
  struct Edge
  {
    std::string topic;
    std::string parent;
    std::string child;
    std::string gid;
    bool is_static = false;
    detail::TfTiming timing;
    geometry_msgs::msg::Transform transform;
    std::uint64_t reported_samples = 0;
    std::int64_t dds_source_ns = 0;
    std::int64_t dds_received_ns = 0;
  };

  void receive(tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info, bool is_static);
  void report();
  void sampleMapBase();

  nav2_util::LifecycleNode & node_;
  tf2_ros::Buffer & buffer_;
  std::string global_frame_;
  std::string base_frame_;
  std::mutex mutex_;
  std::map<std::string, Edge> edges_;
  std::uint64_t dropped_edges_ = 0;
  std::int64_t last_ros_ns_ = 0;
  std::int64_t last_wall_ns_ = 0;
  std::int64_t last_steady_ns_ = 0;
  std::uint64_t sample_sequence_ = 0;
  std::int64_t last_sample_steady_ns_ = 0;
  std::int64_t last_sample_source_ns_ = 0;
  bool has_sample_transform_ = false;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr dynamic_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr map_base_timer_;
};
}

#endif
