#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <memory>

namespace nav2_regulated_modules
{

// 中文注释：把当前路径封装为 FollowPath Goal，并用任务代次和序号过滤过期反馈／结果。
void RegulatedNavigator::sendFollowPath(const nav_msgs::msg::Path & path) {
  if (path.poses.empty() || task_.type == TaskType::NONE) {
    handlePlanningFailure("控制路径为空");
    return;
  }
  task_.active_path = path;
  task_.state = NavigationState::CONTROLLING;
  const auto generation = task_.generation;
  const auto sequence = ++follow_sequence_;
  const bool replacing_path = active_follow_goal_ != nullptr;
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = control_module_.controllerId();
  goal.goal_checker_id = control_module_.goalCheckerId();
  if (replacing_path) {
    LOG_DEBUG("更新 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}", generation, sequence, path.poses.size(), control_module_.controllerId());
  } else {
    LOG_INFO("下发 FollowPath，generation={}，follow_sequence={}，路径点数={}，controller_id={}，goal_checker_id={}", generation, sequence, path.poses.size(), control_module_.controllerId(), control_module_.goalCheckerId());
  }
  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  options.goal_response_callback = [this, generation, sequence](auto handle) {
      if (isCurrentFollow(generation, sequence)) {
        active_follow_goal_ = handle;
        if (!handle) {
          startRecovery("控制 Goal 被拒绝");
        }
      }
    };
  options.feedback_callback = [this, generation, sequence](auto, const std::shared_ptr<const FollowPath::Feedback> controller_feedback) {
      if (isCurrentFollow(generation, sequence)) {
        task_.distance_remaining = static_cast<double>(controller_feedback->distance_to_goal);
        current_speed_ = controller_feedback->speed;
        if (active_fixed_path_goal_) {
          auto feedback = std::make_shared<FollowFixedPath::Feedback>();
          feedback->distance_remaining = task_.distance_remaining;
          active_fixed_path_goal_->publish_feedback(feedback);
        }
      }
    };
  options.result_callback = [this, generation, sequence](const auto & result) {
      if (!isCurrentFollow(generation, sequence)) {return;}
      active_follow_goal_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        succeedTask();
      } else if (task_.state != NavigationState::CANCELING) {
        startRecovery("控制器执行路径失败");
      }
    };
  follow_client_->async_send_goal(goal, options);
}

// 中文注释：只有外层任务代次和 FollowPath 序号同时匹配时，异步回调才允许修改状态。
bool RegulatedNavigator::isCurrentFollow(const uint64_t generation, const uint64_t sequence) const {
  return generation == task_.generation && sequence == follow_sequence_;
}

// 中文注释：向控制链入口连续发送零速度，覆盖取消过程中可能残留的非零命令。
void RegulatedNavigator::stopRobot() {
  // 中文注释：连续发布三次零速度，降低异步取消期间最后一帧非零速度残留的风险。
  if (!stop_cmd_pub_) {return;}
  geometry_msgs::msg::Twist stop;
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  LOG_DEBUG("已向速度链入口连续发布 3 帧零速度");
}

}  // namespace nav2_regulated_modules
