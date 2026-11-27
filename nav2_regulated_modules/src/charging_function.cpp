// 充电动作的三段回调 + 异步执行状态机。
// 接受后进入 AWAIT_QR：消费定时器非阻塞轮询二维码缓存，出现新鲜观测即认领一次，
// 用二维码坐标修正 map 帧终点、生成充电路径并固定经 fixed 通道（follow_client_）下发
// FollowPath——充电不占 task_，followClient() 的任务分派不适用；在途请求计入
// fixed_path_outstanding_follows_，交接监控的 unifiedDrained 据此不会放行并发任务。
// 随后由 FollowPath 的 feedback 同步上报进度、result 回调收口。任何回调都不得 sleep 阻塞，
// 否则二维码订阅与动作应答送不进来；重计算（折算/插值）因此放在消费定时器回调里，
// 而不放在可能高频到达的 qrCodeMsgCallback 里。
// 状态机由 charging_mutex_ 保护：截止/消费定时器、FollowPath 回调与取消应答在多线程
// 执行器上可能并发，只有一次持锁把目标从 active 摘除的收口有效，动作 Goal 不会被二次终结。
// 本文件同时承载充电相关的二维码观测缓存，充电对接与扫码观察共享同一协作文件。

#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

// 二维位姿结构，仅承载平面坐标与航向，theta 单位 rad。
// 与外部充电折算代码的 Point3D 用途一致，这里直接用 theta 字段而非其 speed 字段。
struct Point3D
{
  double x;
  double y;
  double theta;
};

// 构造“把位姿 a 对齐到位姿 b”的刚体变换 T，使 P_b = T * P_a。
// 公式逐行对应外部充电代码 get_transform_matrix_form_a_to_b，仅字段名适配。
Eigen::Matrix3d getTransformMatrixFromAToB(const Point3D & a, const Point3D & b)
{
  const double dtheta = b.theta - a.theta;
  const double cos_theta = std::cos(dtheta);
  const double sin_theta = std::sin(dtheta);
  const double dx = b.x - (a.x * cos_theta - a.y * sin_theta);
  const double dy = b.y - (a.x * sin_theta + a.y * cos_theta);

  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  T(0, 0) = cos_theta;
  T(0, 1) = -sin_theta;
  T(1, 0) = sin_theta;
  T(1, 1) = cos_theta;
  T(0, 2) = dx;
  T(1, 2) = dy;
  return T;
}

// 二维码等待的窗口与最大年龄不再用编译期常量，改为 yaml 参数
// charging_qr_wait_timeout_／charging_qr_max_age_（见 on_configure），由 launch 传入。
// 消费定时器的轮询间隔：二维码缓存检查频率，只做轻量判断，不与消息发布频率耦合。
constexpr std::chrono::milliseconds kChargingQrPollInterval { 50 };

}  // namespace

namespace nav2_regulated_modules
{

// 仅接受节点激活、固定通道就绪、底盘准入、task_id 非空、起点终点坐标有限且当前无其他任务的充电请求。
// 已有导航任务（含待执行 Goal）时拒绝，与 admitUnified 的充电检查形成双向互斥，避免共用车辆。
// 底盘准入与通道就绪沿用导航任务的统一纪律：遥控占用或未停稳、执行节点未激活时不接单。
rclcpp_action::GoalResponse RegulatedNavigator::handleChargingGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const Charging::Goal> goal)
{
  if (!active_)
  {
    LOG_WARN("拒绝 Charging Goal：节点未激活");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (task_.type != TaskType::NONE)
  {
    LOG_WARN("拒绝 Charging Goal：当前已有其他导航任务在执行");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (pending_pose_ || pending_fixed_)
  {
    LOG_WARN("拒绝 Charging Goal：已有导航任务待执行");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!executionReady(false))
  {
    LOG_WARN("拒绝 Charging Goal：固定执行通道未就绪");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!chassis_control_subscriber_->canStartNavigation())
  {
    LOG_WARN("拒绝 Charging Goal：底盘控制权未就绪（遥控占用或未停稳）");
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (active_charging_goal_)
    {
      LOG_WARN("拒绝 Charging Goal：已有充电任务在执行");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  if (goal->task_id.empty())
  {
    LOG_WARN("拒绝 Charging Goal：task_id 为空");
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto valid_point = [](const geometry_msgs::msg::Point & point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  };
  if (!valid_point(goal->start_point) || !valid_point(goal->target_point))
  {
    LOG_WARN("拒绝 Charging Goal：起点或终点包含非有限坐标");
    return rclcpp_action::GoalResponse::REJECT;
  }
  LOG_INFO("接受 Charging Goal，task_id={}，start=({:.3f},{:.3f},{:.3f})，target=({:.3f},{:.3f},{:.3f})", goal->task_id, goal->start_point.x, goal->start_point.y, goal->start_point.z, goal->target_point.x, goal->target_point.y, goal->target_point.z);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 取消直接把充电目标摘回 IDLE、停充电侧定时器、作废在途 FollowPath 并向控制器请求停止，
// 再以 canceled 终结充电 Goal。任何阶段都适用：AWAIT_QR 取消即停等，FOLLOWING 取消会中断跟踪。
rclcpp_action::CancelResponse RegulatedNavigator::handleChargingCancel(const std::shared_ptr<ChargingHandle> goal)
{
  std::shared_ptr<FollowHandle> follow_to_cancel;
  bool should_finalize = false;
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (goal != active_charging_goal_)
    {
      return rclcpp_action::CancelResponse::ACCEPT;  // 非当前任务，无需响应通行的取消。
    }
    should_finalize = true;
    follow_to_cancel = charging_follow_goal_;
    charging_follow_goal_.reset();
    charging_follow_sequence_++;  // 在途 FollowPath 的迟到结果与反馈全部作废。
    charging_phase_ = ChargingPhase::IDLE;
    charging_qr_timeout_timer_.reset();
    charging_qr_poll_timer_.reset();
    active_charging_goal_.reset();
  }
  if (should_finalize)
  {
    if (follow_to_cancel)
    {
      follow_client_->async_cancel_goal(follow_to_cancel);
    }
    // 与 cancelTask 同款收口：取消子 Goal 后再向速度链入口请求停车（task_ 为 NONE，
    // stopRobot 自动选择固定通道），不单靠控制器的取消握手兜底。
    stopRobot();
    publishChargingSpeedLimit(0.0);
    auto result = std::make_shared<Charging::Result>();
    result->success = false;
    result->message = "充电任务被取消";
    LOG_WARN("Charging 取消，task_id={}", goal->get_goal()->task_id);
    goal->canceled(result);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 保存句柄并进入 AWAIT_QR：启动截止定时器与二维码消费定时器。二者只在各自回调里做有限工作，
// 不 sleep；重计算（折算/插值/下发）由消费定时器在认领到新鲜观测后转入 startChargingFollow。
// 接受后先复核占用：Goal 回调与 Accepted 回调之间可能插进导航任务（与 handlePoseAccepted
// 的 admitUnified 复核互为镜像），冲突时立即 abort，保证同时最多一个上层任务占用车辆。
void RegulatedNavigator::handleChargingAccepted(const std::shared_ptr<ChargingHandle> goal)
{
  if (!active_ || task_.type != TaskType::NONE || pending_pose_ || pending_fixed_)
  {
    LOG_WARN("Charging 接受后复核失败：车辆已被导航任务占用或节点未激活");
    auto result = std::make_shared<Charging::Result>();
    result->success = false;
    result->message = "充电任务被导航任务抢占";
    goal->abort(result);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    active_charging_goal_ = goal;
    charging_phase_ = ChargingPhase::AWAIT_QR;
  }
  charging_qr_timeout_timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<std::int64_t>(charging_qr_wait_timeout_ * 1000.0)),
    [this, goal]() { handleChargingQrTimeout(goal); });
  charging_qr_poll_timer_ = create_wall_timer(kChargingQrPollInterval, [this, goal]() { handleChargingQrPoll(goal); });
}

// AWAIT_QR 截止：等待窗口内未认领到新鲜二维码即失败收口。一次性定时器，触发后复位自身。
// 检查与摘除在同一把锁内完成：若二维码认领已先转入 FOLLOWING，本回调直接退出，绝不误伤新阶段。
void RegulatedNavigator::handleChargingQrTimeout(const std::shared_ptr<ChargingHandle> goal)
{
  bool should_finalize = false;
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (active_charging_goal_ == goal && charging_phase_ == ChargingPhase::AWAIT_QR)
    {
      should_finalize = true;
      charging_phase_ = ChargingPhase::IDLE;
      charging_qr_timeout_timer_.reset();
      charging_qr_poll_timer_.reset();
      active_charging_goal_.reset();
    }
  }
  if (should_finalize)
  {
    auto result = std::make_shared<Charging::Result>();
    result->success = false;
    result->message = "充电任务等待二维码超时";
    LOG_WARN("Charging 等待二维码超时 {}s，task_id={}", charging_qr_wait_timeout_, goal->get_goal()->task_id);
    goal->abort(result);
  }
}

// AWAIT_QR 消费（非阻塞轮询）：每拍检查缓存，出现新鲜二维码即认领一次并转入路径下发；
// 没有则返回等待下一拍，本定时器保持周期运行直到认领／收口时才停。认领前先做新鲜性闸门，
// 避免对过期缓存立即认领并失败。
void RegulatedNavigator::handleChargingQrPoll(const std::shared_ptr<ChargingHandle> goal)
{
  std::shared_ptr<ChargingHandle> claimed;
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (charging_phase_ != ChargingPhase::AWAIT_QR || active_charging_goal_ != goal)
    {
      return;  // 已超时／取消／收口，本定时器由收口方一并复位。
    }
    bool fresh = false;
    {
      // 新鲜性闸门：只认领 ≤charging_qr_max_age_ 的观测；消费前的复核在快照锁内再做一次。
      std::lock_guard<std::mutex> qr_lock(qr_code_mutex_);
      const double age = (now() - current_qr_code_.header.stamp).seconds();
      fresh = std::isfinite(age) && age <= charging_qr_max_age_;
    }
    if (!fresh)
    {
      return;
    }
    claimed = goal;
    charging_phase_ = ChargingPhase::FOLLOWING;
    // 认领成功：一次性停掉两个充电定时器，后续推进改由 FollowPath 回调驱动。
    charging_qr_timeout_timer_.reset();
    charging_qr_poll_timer_.reset();
  }
  startChargingFollow(claimed);
}

// 用二维码坐标修正 map 帧终点、生成充电路径并异步下发 FollowPath。
// 起点与修正后的终点都留在 map 帧；进度由 feedback 回调转成 Charging 反馈实时上报，
// 结果回调调用 finalizeChargingGoal 收口。下发对齐 navigator_control.cpp sendFollowPath 的
// 通道纪律：充电固定走 fixed 通道（follow_client_，不设 task_ 故不经 followClient() 分派），
// 发送前校验 executionReady，在途请求计入 fixed_path_outstanding_follows_，让
// unifiedDrained／canStartNavigation 把充电跟随视作占用中的子任务。
// controller_server 不重投影，按 charging_controller_id_ 交给充电专用控制器
// （继承 FixedPathController，只在位姿变换环节插充电修正）。
// 充电对接固定使用倒车模式：下发前把生成路径的首点车头航向旋转 π，让控制器判定为 backward。
void RegulatedNavigator::startChargingFollow(const std::shared_ptr<ChargingHandle> goal)
{
  // 认领后、下发前复核固定通道：与导航任务的 executionReady 门槛同源，
  // 执行节点掉活或 Action 不可发现时立即失败收口，不把路径发给失效通道。
  if (!executionReady(false))
  {
    LOG_WARN("充电跟随下发前固定执行通道未就绪，task_id={}", goal->get_goal()->task_id);
    finalizeChargingGoal(goal, false, "固定执行通道未就绪");
    return;
  }
  const auto & request = *goal->get_goal();
  geometry_msgs::msg::Point start = request.start_point;
  geometry_msgs::msg::Point target = request.target_point;
  if (!correctChargingTargetToMap(target))
  {
    finalizeChargingGoal(goal, false, "二维码观测修正终点失败");
    return;
  }
  auto path = generateChargingPath(start, target);
  if (!path)
  {
    finalizeChargingGoal(goal, false, "map 帧充电路径生成失败");
    return;
  }
  // 充电对接全部采用倒车进桩：任务下发前将首点车头航向反向（绕 z 旋转 π）。
  // FixedPathController::setPlan 以「首点朝向 vs 起点切线」的夹角判定 direction_sign_，
  // 反向即判为 -1（backward），对齐运动航向（+π）、负线速度与曲率符号由基类自动适配，
  // 中间点/末点航向不参与判定与转角律，无需改动。
  if (!path->poses.empty())
  {
    const auto & front_ori = path->poses.front().pose.orientation;
    tf2::Quaternion front_q(front_ori.x, front_ori.y, front_ori.z, front_ori.w);
    front_q *= tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), M_PI);
    front_q.normalize();
    path->poses.front().pose.orientation = tf2::toMsg(front_q);
  }
  double total_length = 0.0;
  for (std::size_t index = 1; index < path->poses.size(); ++index)
  {
    total_length += std::hypot(
      path->poses[index].pose.position.x - path->poses[index - 1].pose.position.x,
      path->poses[index].pose.position.y - path->poses[index - 1].pose.position.y);
  }

  uint64_t sequence = 0;
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    // 发送前复核：取消可能在认领与下发之间到达，已摘除的任务不得再真正下发。
    if (active_charging_goal_ != goal || charging_phase_ != ChargingPhase::FOLLOWING)
    {
      return;
    }
    sequence = ++charging_follow_sequence_;
  }

  FollowPath::Goal follow_goal;
  follow_goal.path = *path;
  follow_goal.controller_id = charging_controller_id_;
  follow_goal.goal_checker_id = charging_checker_id_;
  LOG_INFO("Charging 下发 FollowPath，task_id={}，点数={}，总长={:.3f}m，controller_id={}",
    request.task_id, path->poses.size(), total_length, follow_goal.controller_id);

  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  // 发送时尚未持有已接受句柄；响应到达时先核对跟踪轮次与活动任务，过期则取消迟到接管。
  // 被拒的请求在此处递减在途计数（不会有 result 回调），被接受的请求统一在 result 递减。
  options.goal_response_callback = [this, sequence, goal](const FollowHandle::SharedPtr & handle)
  {
    if (!handle)
    {
      --fixed_path_outstanding_follows_;
      finalizeChargingGoal(goal, false, "FollowPath 目标被控制器拒绝");
      return;
    }
    bool current = false;
    {
      std::lock_guard<std::mutex> lock(charging_mutex_);
      current = active_charging_goal_ == goal && charging_follow_sequence_ == sequence;
      if (current)
      {
        charging_follow_goal_ = handle;
      }
    }
    if (!current)
    {
      // 任务已取消／作废后迟到的接管不能继续控制底盘，向控制器请求取消。
      follow_client_->async_cancel_goal(handle);
    }
  };
  options.feedback_callback = [this, sequence, goal, total_length](auto, const std::shared_ptr<const FollowPath::Feedback> controller_feedback)
  {
    if (sequence != charging_follow_sequence_)
    {
      return;  // 已作废的跟踪不再更新进度。
    }
    auto feedback = std::make_shared<Charging::Feedback>();
    feedback->status = "对接执行中";
    if (total_length > 0.0)
    {
      feedback->progress = std::clamp(
        static_cast<float>((total_length - controller_feedback->distance_to_goal) / total_length),
        0.0F, 1.0F);
    }
    goal->publish_feedback(feedback);
  };
  options.result_callback = [this, sequence, goal](const auto & result)
  {
    // 无论是否当前轮次，本请求已到终态，固定通道在途计数先行归零。
    --fixed_path_outstanding_follows_;
    if (sequence != charging_follow_sequence_)
    {
      return;  // 已取消或新跟踪接管，迟到结果不终结当前任务。
    }
    const bool ok = result.code == rclcpp_action::ResultCode::SUCCEEDED;
    finalizeChargingGoal(goal, ok,
      ok ? "充电对接路径执行完成" :
        (result.code == rclcpp_action::ResultCode::ABORTED ? "充电 FollowPath 执行失败" : "充电 FollowPath 被取消"));
  };
  // 与 sendFollowPath 同款在途计数：交接监控据此不会在充电跟随时放行新的导航任务。
  ++fixed_path_outstanding_follows_;
  // 下发前声明充电速度基准：固定通道 smoother 的 collision 门控否则以任务速度 0
  // 把控制器输出清零（对齐阶段角速度也会被清掉，车完全不动）。
  publishChargingSpeedLimit(charging_task_speed_);
  try
  {
    follow_client_->async_send_goal(follow_goal, options);
  }
  catch (...)
  {
    --fixed_path_outstanding_follows_;
    LOG_ERROR("Charging 下发 FollowPath 异常，task_id={}", request.task_id);
    finalizeChargingGoal(goal, false, "FollowPath 下发异常");
  }
}

// 临界收口：只在 goal 仍是活动充电任务时执行一次，摘回 IDLE、停充电侧定时器、作废在途跟随，
// 再在锁外终结动作 Goal；迟到的反馈/结果/重复收口调用在这里被自然过滤。
void RegulatedNavigator::finalizeChargingGoal(const std::shared_ptr<ChargingHandle> goal, bool success, const std::string & message)
{
  std::shared_ptr<FollowHandle> follow_to_cancel;
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (active_charging_goal_ != goal)
    {
      return;  // 已被取消或超时等其他收口方先摘下，迟到结果不再打扰。
    }
    follow_to_cancel = charging_follow_goal_;
    charging_follow_goal_.reset();
    charging_follow_sequence_++;
    charging_phase_ = ChargingPhase::IDLE;
    charging_qr_timeout_timer_.reset();
    charging_qr_poll_timer_.reset();
    active_charging_goal_.reset();
  }
  // 收口复位固定通道速度基准，避免 collision 门控残留充电速度影响后续任务。
  publishChargingSpeedLimit(0.0);
  auto result = std::make_shared<Charging::Result>();
  if (success)
  {
    result->success = true;
    result->message = message;
    LOG_INFO("Charging 完成，task_id={}，{}", goal->get_goal()->task_id, message);
    goal->succeed(result);
  }
  else
  {
    // 失败收口时若仍有在途跟随，先请控制器停止并向速度链入口请求停车；
    // 成功收口的结果已到，无需再取消。
    if (follow_to_cancel)
    {
      follow_client_->async_cancel_goal(follow_to_cancel);
      stopRobot();
    }
    result->success = false;
    result->message = message;
    LOG_WARN("Charging 失败，task_id={}：{}", goal->get_goal()->task_id, message);
    goal->abort(result);
  }
}

// 内部取消充电任务：统一出口接管保护、生命周期停用／清理与导航任务收口共用。
// 与 handleChargingCancel 共享临界摘除逻辑，但以 abort 终结（外部取消才报 canceled）；
// 无活动充电目标时静默返回，调用方无需自行判空。
void RegulatedNavigator::cancelChargingTask(const std::string & reason)
{
  std::shared_ptr<ChargingHandle> goal;
  std::shared_ptr<FollowHandle> follow_to_cancel;
  {
    std::lock_guard<std::mutex> lock(charging_mutex_);
    if (!active_charging_goal_)
    {
      return;
    }
    goal = active_charging_goal_;
    follow_to_cancel = charging_follow_goal_;
    charging_follow_goal_.reset();
    charging_follow_sequence_++;
    charging_phase_ = ChargingPhase::IDLE;
    charging_qr_timeout_timer_.reset();
    charging_qr_poll_timer_.reset();
    active_charging_goal_.reset();
  }
  if (follow_to_cancel)
  {
    follow_client_->async_cancel_goal(follow_to_cancel);
  }
  // 充电期间 task_ 为 NONE，stopRobot 自动选择固定通道的停车入口。
  stopRobot();
  publishChargingSpeedLimit(0.0);
  auto result = std::make_shared<Charging::Result>();
  result->success = false;
  result->message = reason;
  LOG_WARN("Charging 被内部取消，task_id={}：{}", goal->get_goal()->task_id, reason);
  goal->abort(result);
}

// 充电任务处于任意在途阶段（含等待二维码）：导航 Goal 准入据此拒绝，保证车辆独占。
bool RegulatedNavigator::chargingTaskActive() const
{
  std::lock_guard<std::mutex> lock(charging_mutex_);
  return active_charging_goal_ != nullptr;
}

// 充电 FollowPath 执行中：底盘统一出口需把固定通道当作当前导航速度源放行。
bool RegulatedNavigator::chargingFollowActive() const
{
  std::lock_guard<std::mutex> lock(charging_mutex_);
  return active_charging_goal_ != nullptr && charging_phase_ == ChargingPhase::FOLLOWING;
}

// 充电跟随的跟踪轮次；充电期间该值稳定，供底盘出口的速度缓存做代次匹配。
uint64_t RegulatedNavigator::chargingFollowGeneration() const
{
  std::lock_guard<std::mutex> lock(charging_mutex_);
  return charging_follow_sequence_;
}

// 固定通道 velocity_smoother 的 collision 门控以最近一次 SpeedLimit 为任务速度基准：
// 基准为 0 时 collisionVelocityTarget 会把控制器输出整包清零，车不会动。
// 导航任务在 goal_response 里发布 requested_speed、在收口时归零（navigator_control.cpp /
// navigator_task_manager.cpp）；充电不设 task_，走不到那两处，必须在此对齐同一纪律——
// 下发跟随时置 charging_task_speed_，任何收口路径复位为 0。
void RegulatedNavigator::publishChargingSpeedLimit(const double speed)
{
  if (!speed_limit_pub_)
  {
    return;
  }
  nav2_msgs::msg::SpeedLimit speed_limit;
  speed_limit.percentage = false;
  speed_limit.speed_limit = speed;
  speed_limit_pub_->publish(speed_limit);
}

// 只做观测缓存：把原始单位 mm／度归一化为 m／弧度后写入 current_qr_code_。
// 二维码可能高频到达，这里刻意保持轻量——不调用 generateChargingPath／correctChargingTargetToMap，
// 重计算与路径下发由 handleChargingAccepted 起的消费定时器在 AWAIT_QR 认领时承担一次。
void RegulatedNavigator::qrCodeMsgCallback(const byd_custom_msgs::msg::QrCode::SharedPtr msg)
{
  LOG_INFO("qrcode tag:{}, x:{}, y:{}, theta:{}", msg->tag, msg->x, msg->y, msg->theta);

  std::lock_guard<std::mutex> lock(qr_code_mutex_);
  current_qr_code_ = *msg;
  // 原始数据为 mm 和角度，需要转换为 m 和弧度
  current_qr_code_.x = current_qr_code_.x / 1000.0;
  current_qr_code_.y = current_qr_code_.y / 1000.0;
  current_qr_code_.theta = (current_qr_code_.theta / 180.0) * M_PI;
  if (current_qr_code_.theta > M_PI)
  {
    current_qr_code_.theta -= 2 * M_PI;
  }
  else if (current_qr_code_.theta < -M_PI)
  {
    current_qr_code_.theta += 2 * M_PI;
  }
}

// 从起点到终点沿直线按步幅均匀插值，并在终点方向额外延长 extension 距离。
// 输出 z=0，车头航向取行进方向，不进行碰撞可行性验证。
// 两个输入点约定为同一帧（充电起终点都在 map 帧），输出路径按 global_frame 自动盖章；
// 插值本身与坐标系无关，frame_id 只是描述文件所处坐标系的标识。
std::optional<nav_msgs::msg::Path> RegulatedNavigator::generateChargingPath(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & target) const
{
  if (!std::isfinite(charging_path_step_) || charging_path_step_ <= 0.0 || !std::isfinite(charging_path_extension_) || charging_path_extension_ < 0.0)
  {
    LOG_ERROR("充电路径参数非法，step={}，extension={}", charging_path_step_, charging_path_extension_);
    return std::nullopt;
  }
  const double direction_x = target.x - start.x;
  const double direction_y = target.y - start.y;
  const double segment_length = std::hypot(direction_x, direction_y);
  if (segment_length <= 1e-9)
  {
    LOG_ERROR("充电路径起终点重合，无法生成方向向量");
    return std::nullopt;
  }
  const double unit_x = direction_x / segment_length;
  const double unit_y = direction_y / segment_length;
  // 延长目标方向：总长 = 起终点距离 + 延长量，末点落在延长后的端点位置。
  const double total_length = segment_length + charging_path_extension_;
  const std::size_t point_count = static_cast<std::size_t>(std::ceil(total_length / charging_path_step_)) + 1;

  nav_msgs::msg::Path output;
  output.header.frame_id = global_frame_;
  output.header.stamp = now();
  output.poses.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index)
  {
    const double arc = std::min(static_cast<double>(index) * charging_path_step_, total_length);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = output.header;
    pose.pose.position.x = start.x + unit_x * arc;
    pose.pose.position.y = start.y + unit_y * arc;
    pose.pose.position.z = 0.0;
    // 车头朝向沿行进方向；末点保留与倒数第二点一致的方向。
    const double yaw = std::atan2(unit_y, unit_x);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    orientation.normalize();
    pose.pose.orientation = tf2::toMsg(orientation);
    output.poses.push_back(pose);
  }
  LOG_DEBUG("生成充电路径，start=({:.3f},{:.3f})，target=({:.3f},{:.3f})，延长={:.3f}m，步幅={:.3f}m，点数量={}", start.x, start.y, target.x, target.y, charging_path_extension_, charging_path_step_, output.poses.size());
  return output;
}

// 无阻塞读取二维码快照并沿 map→QR→map 链路只修正 map 帧终点。
// 起点不做处理，仍按 goal 的 start_point 原样使用；二维码位姿与终点都在 map 帧。
// 变换锚定 map 原点（二维码位姿即 QR 系在 map 的摆放），不引入机器人位姿；
// 快照锁内复核消息年龄（≤charging_qr_max_age_），过期或定位缺失返回 false，由调用方决定收口。
bool RegulatedNavigator::correctChargingTargetToMap(geometry_msgs::msg::Point & target)
{
  // 1) 取两路位姿快照：二维码相对位姿、当前 map 位姿。
  // 不同来源在各自锁内复制并复核新鲜度，计算阶段在锁外进行。
  byd_custom_msgs::msg::QrCode qr_snapshot;
  {
    std::lock_guard<std::mutex> lock(qr_code_mutex_);
    const double age = (now() - current_qr_code_.header.stamp).seconds();
    if (!std::isfinite(age) || age > charging_qr_max_age_)
    {
      LOG_WARN("充电二维码过期：最近消息年龄超过 {}s", charging_qr_max_age_);
      return false;
    }
    qr_snapshot = current_qr_code_;
  }
  {
    // 定位健康检查：修正链路本身只用二维码位姿（map 帧），但定位缺失时不应继续对接。
    geometry_msgs::msg::PoseStamped pose;
    if (!lookupCurrentPose(pose))
    {
      LOG_WARN("充电二维码修正终点前定位不可用");
      return false;
    }
  }
  const Point3D qr_point { qr_snapshot.x, qr_snapshot.y, qr_snapshot.theta };

  // 2) 把终点对齐二维码横纵轴再写回 map 帧：二维码位姿按 map 帧解释，
  //    world→QR 取「QR 位姿→map 原点」的坐标变换 R(-θ)(P-q)，把绝对值较小的分量置零
  //    （只保留沿二维码主轴的偏差），再沿「map 原点→QR 位姿」转回 map。
  //    旧实现以机器人位姿做 world 锚点且两变换参数颠倒：QR 远离 map 原点时
  //    投影混入平移量、终点被甩到数米外；锚定 map 原点后 QR 上的点投影为 (0,0)，
  //    几何上才是真正的二维码系投影。
  const Point3D map_origin { 0.0, 0.0, 0.0 };
  const Eigen::Matrix3d tf_qr_w = getTransformMatrixFromAToB(qr_point, map_origin);
  const Eigen::Matrix3d tf_w_qr = getTransformMatrixFromAToB(map_origin, qr_point);

  Eigen::Vector3d tp_in_world(target.x, target.y, 1.0);
  Eigen::Vector3d tp_in_qr = tf_qr_w * tp_in_world;
  // 把充电点对齐二维码横纵轴：绝对值小的分量置零，只保留沿充电方向的主偏差。
  if (std::abs(tp_in_qr(0)) > std::abs(tp_in_qr(1)))
  {
    tp_in_qr(1) = 0.0;
  }
  else
  {
    tp_in_qr(0) = 0.0;
  }
  tp_in_qr(2) = 1.0;
  const Eigen::Vector3d tp_in_map = tf_w_qr * tp_in_qr;
  target.x = tp_in_map(0);
  target.y = tp_in_map(1);

  if (!std::isfinite(target.x) || !std::isfinite(target.y))
  {
    LOG_ERROR("充电二维码修正出的终点含非有限坐标");
    return false;
  }
  LOG_DEBUG("充电二维码修正终点完成：target=({:.3f},{:.3f})", target.x, target.y);
  return true;
}

}
// namespace nav2_regulated_modules