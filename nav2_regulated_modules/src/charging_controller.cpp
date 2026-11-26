// 充电专用 Controller 实现（方案一：固定里程参考 + 纯 odometry 位移）。
// 起点方向／切线解析、前视跟踪、终点制动复用基类；机器人位姿喂入改为里程折算——
// 首个同帧位姿到达时采集一次 odometry→map 参考，此后每帧用最新里程话题采样按该
// 固定参考折算回 map，整段对接与外部定位抖动隔离。前视点等跨帧变换仍走基类 TF。

#include "nav2_regulated_modules/charging_controller.hpp"

#include <cmath>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 锁定生命周期父节点，声明并读取充电参数，订阅里程话题后委托基类完成剩余配置。
void ChargingController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  // 生命周期父节点已销毁时无法读取参数或创建订阅，直接终止插件配置。
  if (!node)
  {
    throw nav2_core::PlannerException("ChargingController cannot lock lifecycle node");
  }
  // 充电特有参数先声明：基类按自己的参数表逐一读取，多出的键不影响它，缺省值由本类保证。
  const std::string param_prefix = name + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "charge_odometry_topic", rclcpp::ParameterValue("/odometry"));
  nav2_util::declare_parameter_if_not_declared(
    node, param_prefix + "charge_odom_max_age", rclcpp::ParameterValue(0.1));
  FixedPathController::configure(parent, std::move(name), std::move(tf), std::move(costmap_ros));
  node->get_parameter(param_prefix + "charge_odometry_topic", charge_odometry_topic_);
  node->get_parameter(param_prefix + "charge_odom_max_age", charge_odom_max_age_);
  // 保留负值或非有限的上限只会让新鲜度门控失效，里程过旧时可能带着陈旧位姿继续控制。
  if (charge_odom_max_age_ <= 0.0 || !std::isfinite(charge_odom_max_age_))
  {
    throw nav2_core::PlannerException("ChargingController charge_odom_max_age must be finite and positive");
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    odom_has_sample_ = false;
    reference_valid_ = false;
    odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    odom_sub_.reset();
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      charge_odometry_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdometry(msg); });
  }
}

// 清空路径、里程订阅和参考状态，供后续重新 configure。
void ChargingController::cleanup()
{
  FixedPathController::cleanup();
  std::lock_guard<std::mutex> lock(state_mutex_);
  odom_sub_.reset();
  odom_has_sample_ = false;
  reference_valid_ = false;
  odom_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

// 接收充电路径：基类负责解析起点方向、首末切线并复位起步／制动历史。
// 里程参考在首个同帧位姿（开始控制的第一帧）采集；新路径先失效旧参考，防止残留偏移。
void ChargingController::setPlan(const nav_msgs::msg::Path & path)
{
  FixedPathController::setPlan(path);
  std::lock_guard<std::mutex> lock(state_mutex_);
  reference_valid_ = false;
}

// 同帧调用即控制器服务器喂入的机器人位姿（本部署 global 帧与 path 帧同为 map，基类
// 此时也只做同帧复制）：改由里程位移折算 map 位姿；其余跨帧转换委托基类 TF 变换。
bool ChargingController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & input,
  geometry_msgs::msg::PoseStamped & output) const
{
  if (input.header.frame_id == frame)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    // 里程话题尚无采样，或采样超过新鲜度上限时不产出位姿，上层按控制失败策略处置。
    if (!odom_has_sample_)
    {
      return false;
    }
    const double age_s = (ros_clock_.now() - odom_stamp_).seconds();
    if (age_s < 0.0 || age_s > charge_odom_max_age_)
    {
      return false;
    }
    // 首个同帧位姿同时作为参考锚点：map 位姿与当前里程采样对齐，保证第一帧折算结果
    // 与服务器位姿完全一致，后续只随里程位移增量变化。
    if (!reference_valid_ && !getTransformMatrixFromAToB(input))
    {
      return false;
    }
    const double cos_t = std::cos(ref_theta_);
    const double sin_t = std::sin(ref_theta_);
    output.pose.position.x = cos_t * odom_x_ - sin_t * odom_y_ + ref_tx_;
    output.pose.position.y = sin_t * odom_x_ + cos_t * odom_y_ + ref_ty_;
    output.pose.position.z = 0.0;
    output.pose.orientation = tf2::toMsg(
      tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), normalizeAngle(odom_yaw_ + ref_theta_)));
    output.header.frame_id = frame;
    output.header.stamp = odom_stamp_;
    // 充电专用修正（对接横偏／航向折算等）在此原位追加；未实现时原样采纳里程位姿。
    return true;
  }
  // 其余跨帧转换（前视点→base 帧等）仍走基类 TF 变换。
  return FixedPathController::transformPose(frame, input, output);
}

// 从 odometry 帧（A）到 map 帧（B）构造二维刚体变换：p_map = R(θ)·p_odom + t。
// 调用方已持有 state_mutex_，以当前里程采样与传入的 map 位姿同拍对齐。
bool ChargingController::getTransformMatrixFromAToB(
  const geometry_msgs::msg::PoseStamped & pose_in_map) const
{
  const double map_yaw = tf2::getYaw(pose_in_map.pose.orientation);
  ref_theta_ = normalizeAngle(map_yaw - odom_yaw_);
  const double cos_t = std::cos(ref_theta_);
  const double sin_t = std::sin(ref_theta_);
  ref_tx_ = pose_in_map.pose.position.x - (cos_t * odom_x_ - sin_t * odom_y_);
  ref_ty_ = pose_in_map.pose.position.y - (sin_t * odom_x_ + cos_t * odom_y_);
  reference_valid_ =
    std::isfinite(ref_theta_) && std::isfinite(ref_tx_) && std::isfinite(ref_ty_);
  if (reference_valid_)
  {
    LOG_INFO(
      "ChargingController 里程参考已采集：θ={:.6f}rad，平移=({:.6f},{:.6f})m，里程话题={}",
      ref_theta_, ref_tx_, ref_ty_, charge_odometry_topic_);
  }
  return reference_valid_;
}

// 里程订阅回调：高频到达，只缓存最新采样，折算与新鲜度判断在消费时进行。
void ChargingController::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  odom_yaw_ = tf2::getYaw(msg->pose.pose.orientation);
  odom_stamp_ = rclcpp::Time(msg->header.stamp);
  odom_has_sample_ = true;
}

// 用 atan2(sin θ, cos θ) 把任意角差折回主值区间，θ 的单位为 rad。
double ChargingController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}
// namespace nav2_regulated_modules

PLUGINLIB_EXPORT_CLASS(nav2_regulated_modules::ChargingController, nav2_core::Controller)