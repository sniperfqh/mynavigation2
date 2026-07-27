#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <memory>

namespace nav2_regulated_modules
{

void RegulatedNavigator::sendFollowPath(const nav_msgs::msg::Path & path) {
  if (path.poses.empty() || task_.type == TaskType::NONE) {
    handlePlanningFailure("控制路径为空");
    return;
  }
  task_.active_path = path;
  task_.state = NavigationState::CONTROLLING;
  const auto generation = task_.generation;
  const auto sequence = ++follow_sequence_;
  FollowPath::Goal goal;
  goal.path = path;
  goal.controller_id = control_module_.controllerId();
  goal.goal_checker_id = control_module_.goalCheckerId();
  auto options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  options.goal_response_callback = [this, generation, sequence](auto handle) {
      if (isCurrentFollow(generation, sequence)) {
        active_follow_goal_ = handle;
        if (!handle) {
          startRecovery("控制 Goal 被拒绝");
        }
      }
    };
  options.feedback_callback = [this, generation, sequence](auto, const std::shared_ptr<const FollowPath::Feedback> feedback) {
      if (isCurrentFollow(generation, sequence)) {
        task_.distance_remaining = feedback->distance_to_goal;
        current_speed_ = feedback->speed;
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

bool RegulatedNavigator::isCurrentFollow(const uint64_t generation, const uint64_t sequence) const {
  return generation == task_.generation && sequence == follow_sequence_;
}

void RegulatedNavigator::stopRobot() {
  // 中文注释：连续发布三次零速度，降低异步取消期间最后一帧非零速度残留的风险。
  if (!stop_cmd_pub_) {return;}
  geometry_msgs::msg::Twist stop;
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
  stop_cmd_pub_->publish(stop);
}

}  // namespace nav2_regulated_modules
