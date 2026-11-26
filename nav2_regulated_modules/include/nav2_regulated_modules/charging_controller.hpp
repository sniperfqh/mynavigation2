// 充电专用 Controller 插件声明：继承 FixedPathController，复用其方向解析、前视跟踪、
// 终点制动与零速锁存控制循环，只在位姿喂入链路接入里程位移折算。
// 里程位移链路（方案一）：控制起点处采集一次 odometry→map 参考，之后每帧的机器人
// 位姿不再用外部定位 TF，而由最新里程话题采样按固定参考折算回 map，整段对接与定位
// 抖动隔离；前视点等跨帧变换仍走基类 TF。

#ifndef NAV2_REGULATED_MODULES__CHARGING_CONTROLLER_HPP_
#define NAV2_REGULATED_MODULES__CHARGING_CONTROLLER_HPP_

#include <cmath>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_regulated_modules/fixed_path_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

// 充电对接控制器：终点锁定与检查器容差复用基类，机器人位姿喂入改为里程位移折算。
class ChargingController : public FixedPathController
{
public:
  // 默认构造仅使用成员初值；插件资源与参数在 configure 和 setPlan 阶段建立。
  ChargingController() = default;
  // 与基类一致的析构语义：成员由基类管理，cleanup／deactivate 业务职责仍由宿主按约定调用。
  ~ChargingController() override = default;

  // 先声明充电参数并订阅里程话题，其余依赖与参数表委托基类 configure。
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  // 释放里程订阅并复位里程／参考状态，供再次 configure。
  void cleanup() override;

  // 先调基类完整解析（方向／切线／起点策略／制动复位），再失效待重新采集的里程参考。
  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  // 同帧调用即控制器服务器喂入的机器人位姿（path 帧与 global 帧同为 map，基类此时
  // 也只做同帧复制）：改用里程位移折算的 map 位姿；首个同帧位姿同时作为里程参考锚点。
  // 其余跨帧转换（前视点→base 帧等）仍委托基类 TF 变换。
  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output) const override;
  // 充电专用修正不另设钩子：直接在 transformPose 里程分支末段原位追加，维持单点扩展。
  // 从 odometry 帧（A）到 map 帧（B）的二维刚体变换参数：以传入的 map 位姿为锚点，
  // 与当前里程采样同拍对齐，得到旋转角与平移（p_map = R·p_odom + t）。
  // 调用方需持有 state_mutex_；成功写入 reference_* 并置位后返回 true。
  bool getTransformMatrixFromAToB(const geometry_msgs::msg::PoseStamped & pose_in_map) const;

  // —— 你准备新增的函数（protected virtual 占位，签名由实现阶段补充）——

private:
  // 里程订阅回调：只缓存最新采样，不做任何计算（高频到达保持轻量）。
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  // 用 atan2(sin θ, cos θ) 把角差折回主值区间，角度单位 rad。
  static double normalizeAngle(double angle);

  std::string charge_odometry_topic_{"/odometry"};
  double charge_odom_max_age_{0.1};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  // 里程参考与最新采样共用互斥锁：回调写、控制循环读，均在锁内完成。
  mutable std::mutex state_mutex_;
  // now() 非 const，const transformPose 内取当前时间需要 mutable。
  mutable rclcpp::Clock ros_clock_{RCL_ROS_TIME};
  bool odom_has_sample_{false};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  // 参考在 const transformPose 内惰性采集，写侧与读侧均由 state_mutex_ 串行化。
  mutable bool reference_valid_{false};
  mutable double ref_theta_{0.0};  // odometry→map 旋转角
  mutable double ref_tx_{0.0};     // odometry→map 平移 x
  mutable double ref_ty_{0.0};     // odometry→map 平移 y
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CHARGING_CONTROLLER_HPP_