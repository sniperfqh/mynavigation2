#include "nav2_regulated_modules/tf_diagnostics.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "spdlog_wrapper.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{
namespace
{
std::int64_t steadyNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t wallNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

template<typename Iterator>
std::string gidText(Iterator begin, Iterator end)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (auto it = begin; it != end; ++it)
  {
    stream << std::setw(2) << static_cast<unsigned int>(*it);
  }
  return stream.str();
}
}

TfDiagnostics::TfDiagnostics(nav2_util::LifecycleNode & node, tf2_ros::Buffer & buffer, std::string global_frame, std::string base_frame, const double frequency, const double map_base_frequency)
: node_(node), buffer_(buffer), global_frame_(std::move(global_frame)), base_frame_(std::move(base_frame))
{
  if (!std::isfinite(frequency) || frequency <= 0.0 || frequency > 10.0)
  {
    throw std::invalid_argument("tf_diagnostics_log_frequency must be in (0, 10] Hz");
  }
  if (!std::isfinite(map_base_frequency) || map_base_frequency <= 0.0 || map_base_frequency > 100.0)
  {
    throw std::invalid_argument("tf_diagnostics_map_base_frequency must be in (0, 100] Hz");
  }
  // /tf 用 best_effort 同时兼容 best_effort 与 reliable 发布者；静态 TF 保留 transient_local。
  dynamic_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>("tf", rclcpp::QoS(100).best_effort(), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info)
  {
    receive(std::move(message), info, false);
  });
  static_sub_ = node_.create_subscription<tf2_msgs::msg::TFMessage>("tf_static", rclcpp::QoS(100).reliable().transient_local(), [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info)
  {
    receive(std::move(message), info, true);
  });
  last_ros_ns_ = node_.now().nanoseconds();
  last_wall_ns_ = wallNanoseconds();
  last_steady_ns_ = steadyNanoseconds();
  timer_ = node_.create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / frequency)), [this]()
  {
    report();
  });
  map_base_timer_ = node_.create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / map_base_frequency)), [this]()
  {
    sampleMapBase();
  });
  LOG_INFO("TF高频打印已启用：parent={} child={} sample_frequency={}Hz file_level=DEBUG；TF_MAP_BASE为Buffer采样，TF_RX_MAP_BASE逐条记录直接边接收，source_changed区分重复采样与新源帧", global_frame_, base_frame_, map_base_frequency);
  LOG_INFO("TF诊断已启用：global={}，base={}，dynamic_topic={}，static_topic={}，frequency={}Hz；age为源时间戳差，不能独立区分发布延迟和时钟偏差，static不判过期", global_frame_, base_frame_, dynamic_sub_->get_topic_name(), static_sub_->get_topic_name(), frequency);
}

void TfDiagnostics::receive(tf2_msgs::msg::TFMessage::ConstSharedPtr message, const rclcpp::MessageInfo & info, const bool is_static)
{
  const auto ros_ns = node_.now().nanoseconds();
  const auto wall_ns = wallNanoseconds();
  const auto steady_ns = steadyNanoseconds();
  const auto & metadata = info.get_rmw_message_info();
  const auto gid = gidText(std::begin(metadata.publisher_gid.data), std::end(metadata.publisher_gid.data));
  const std::string topic = is_static ? static_sub_->get_topic_name() : dynamic_sub_->get_topic_name();
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & transform : message->transforms)
  {
    const auto key = topic + "|" + transform.header.frame_id + "|" + transform.child_frame_id + "|" + gid;
    if (edges_.count(key) == 0 && edges_.size() >= 256)
    {
      ++dropped_edges_;
      continue;
    }
    auto & edge = edges_[key];
    edge.topic = topic;
    edge.parent = transform.header.frame_id;
    edge.child = transform.child_frame_id;
    edge.gid = gid;
    edge.is_static = is_static;
    const auto stamp = static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000LL + transform.header.stamp.nanosec;
    edge.timing.observe(stamp, ros_ns, wall_ns, steady_ns);
    edge.transform = transform.transform;
    edge.dds_source_ns = metadata.source_timestamp;
    edge.dds_received_ns = metadata.received_timestamp;
    if (!is_static && edge.parent == global_frame_ && edge.child == base_frame_)
    {
      LOG_DEBUG("TF_RX_MAP_BASE parent={} child={} gid={} rx_sequence={} source_ns={} rx_ros_ns={} rx_wall_ns={} rx_steady_ns={} arrival_age_s={:.9f} dds_source_ns={} dds_received_ns={} xyz=({:.9f},{:.9f},{:.9f}) quat=({:.9f},{:.9f},{:.9f},{:.9f}) yaw={:.9f}", edge.parent, edge.child, edge.gid, edge.timing.samples, stamp, ros_ns, wall_ns, steady_ns, static_cast<double>(ros_ns - stamp) * 1e-9, edge.dds_source_ns, edge.dds_received_ns, edge.transform.translation.x, edge.transform.translation.y, edge.transform.translation.z, edge.transform.rotation.x, edge.transform.rotation.y, edge.transform.rotation.z, edge.transform.rotation.w, tf2::getYaw(edge.transform.rotation));
    }
  }
}

void TfDiagnostics::sampleMapBase()
{
  const auto ros_ns = node_.now().nanoseconds();
  const auto wall_ns = wallNanoseconds();
  const auto steady_ns = steadyNanoseconds();
  const double sample_dt = last_sample_steady_ns_ == 0 ? 0.0 : static_cast<double>(steady_ns - last_sample_steady_ns_) * 1e-9;
  last_sample_steady_ns_ = steady_ns;
  ++sample_sequence_;
  try
  {
    const auto transform = buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    const auto stamp = static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000LL + transform.header.stamp.nanosec;
    const bool changed = !has_sample_transform_ || stamp != last_sample_source_ns_;
    const double source_step = has_sample_transform_ ? static_cast<double>(stamp - last_sample_source_ns_) * 1e-9 : 0.0;
    const auto & value = transform.transform;
    LOG_DEBUG("TF_MAP_BASE parent={} child={} available=true sequence={} source_ns={} ros_ns={} wall_ns={} steady_ns={} age_s={:.9f} sample_dt_s={:.9f} source_changed={} source_step_s={:.9f} xyz=({:.9f},{:.9f},{:.9f}) quat=({:.9f},{:.9f},{:.9f},{:.9f}) yaw={:.9f}", transform.header.frame_id, transform.child_frame_id, sample_sequence_, stamp, ros_ns, wall_ns, steady_ns, static_cast<double>(ros_ns - stamp) * 1e-9, sample_dt, changed, source_step, value.translation.x, value.translation.y, value.translation.z, value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w, tf2::getYaw(value.rotation));
    last_sample_source_ns_ = stamp;
    has_sample_transform_ = true;
  }
  catch (const tf2::TransformException & error)
  {
    LOG_DEBUG("TF_MAP_BASE parent={} child={} available=false sequence={} ros_ns={} wall_ns={} steady_ns={} sample_dt_s={:.9f} error={}", global_frame_, base_frame_, sample_sequence_, ros_ns, wall_ns, steady_ns, sample_dt, error.what());
  }
}

void TfDiagnostics::report()
{
  const auto ros_ns = node_.now().nanoseconds();
  const auto wall_ns = wallNanoseconds();
  const auto steady_ns = steadyNanoseconds();
  const double window_s = static_cast<double>(steady_ns - last_steady_ns_) * 1e-9;
  const double ros_step_s = static_cast<double>(ros_ns - last_ros_ns_) * 1e-9;
  const double wall_step_s = static_cast<double>(wall_ns - last_wall_ns_) * 1e-9;
  LOG_INFO("TF_CLOCK ros_ns={} wall_ns={} steady_ns={} use_sim_time={} ros_step_s={:.6f} wall_step_s={:.6f} steady_step_s={:.6f} ros_minus_wall_s={:.6f}", ros_ns, wall_ns, steady_ns, node_.get_parameter("use_sim_time").as_bool(), ros_step_s, wall_step_s, window_s, static_cast<double>(ros_ns - wall_ns) * 1e-9);
  last_ros_ns_ = ros_ns;
  last_wall_ns_ = wall_ns;
  last_steady_ns_ = steady_ns;
  std::map<std::string, Edge> snapshot;
  std::uint64_t dropped = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = edges_;
    dropped = dropped_edges_;
    for (auto & item : edges_)
    {
      item.second.reported_samples = item.second.timing.samples;
      item.second.timing.resetWindow();
    }
  }
  // 每个摘要窗口查询一次图；按 GID 精确匹配，不能把同话题第一个节点当作每条边的发布者。
  std::map<std::string, std::string> publishers;
  const std::vector<std::string> topics =
  {
    dynamic_sub_->get_topic_name(), static_sub_->get_topic_name()
  };
  for (const auto & topic : topics)
  {
    try
    {
      for (const auto & endpoint : node_.get_publishers_info_by_topic(topic))
      {
        const auto & gid = endpoint.endpoint_gid();
        const auto prefix = endpoint.node_namespace() == "/" ? "/" : endpoint.node_namespace() + "/";
        publishers[gidText(gid.begin(), gid.end())] = prefix + endpoint.node_name();
      }
    }
    catch (const std::exception & error)
    {
      LOG_WARN("TF_PUBLISHER_GRAPH topic={} error={}", topic, error.what());
    }
  }
  for (const auto & item : snapshot)
  {
    const auto & edge = item.second;
    const auto & timing = edge.timing;
    const auto received = timing.samples - edge.reported_samples;
    const auto publisher = publishers.find(edge.gid);
    const double source_age = static_cast<double>(ros_ns - timing.stamp_ns) * 1e-9;
    const double silence = static_cast<double>(steady_ns - timing.steady_ns) * 1e-9;
    const double last_arrival_age = static_cast<double>(timing.ros_ns - timing.stamp_ns) * 1e-9;
    const bool dds_valid = edge.dds_source_ns > 0 && edge.dds_received_ns > 0;
    LOG_INFO("TF_EDGE topic={} parent={} child={} static={} publisher={} gid={} source_ns={} rx_ros_ns={} rx_wall_ns={} rx_steady_ns={} age_now_s={:.6f} arrival_age_s={:.6f} window_samples={} rx_hz={:.3f} silence_s={:.6f} rx_gap_max_s={:.6f} arrival_min_s={:.6f} arrival_max_s={:.6f} stamp_step_s={:.6f} repeats_total={} backwards_total={} zero_total={} dds_valid={} dds_source_ns={} dds_received_ns={} dds_delta_s={:.6f} callback_after_dds_s={:.6f} xyz=({:.6f},{:.6f},{:.6f}) yaw={:.6f}", edge.topic, edge.parent, edge.child, edge.is_static, publisher == publishers.end() ? "unknown" : publisher->second, edge.gid, timing.stamp_ns, timing.ros_ns, timing.wall_ns, timing.steady_ns, source_age, last_arrival_age, received, window_s > 0.0 ? received / window_s : 0.0, silence, timing.max_gap_s, received > 0 ? timing.min_arrival_age_s : last_arrival_age, received > 0 ? timing.max_arrival_age_s : last_arrival_age, timing.stamp_step_s, timing.repeats, timing.backwards, timing.zero_stamps, dds_valid, edge.dds_source_ns, edge.dds_received_ns, dds_valid ? static_cast<double>(edge.dds_received_ns - edge.dds_source_ns) * 1e-9 : 0.0, edge.dds_received_ns > 0 ? static_cast<double>(timing.wall_ns - edge.dds_received_ns) * 1e-9 : 0.0, edge.transform.translation.x, edge.transform.translation.y, edge.transform.translation.z, tf2::getYaw(edge.transform.rotation));
  }
  if (snapshot.empty() || dropped > 0)
  {
    LOG_INFO("TF_RX_STATUS observed_sources={} dropped_transforms_total={}", snapshot.size(), dropped);
  }
  // 这是导航器自身 Buffer 的最新合成结果；控制器真正使用的位姿由 TF_CONTROL 单独打印。
  try
  {
    const auto transform = buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    const auto stamp = static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000LL + transform.header.stamp.nanosec;
    LOG_INFO("TF_CHAIN observer=regulated_navigator parent={} child={} source_ns={} ros_ns={} age_s={:.6f} xy=({:.6f},{:.6f}) yaw={:.6f}", transform.header.frame_id, transform.child_frame_id, stamp, ros_ns, static_cast<double>(ros_ns - stamp) * 1e-9, transform.transform.translation.x, transform.transform.translation.y, tf2::getYaw(transform.transform.rotation));
  }
  catch (const tf2::TransformException & error)
  {
    LOG_WARN("TF_CHAIN observer=regulated_navigator parent={} child={} available=false error={}", global_frame_, base_frame_, error.what());
  }
}
}
