// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CORE__PROGRESS_CHECKER_HPP_
#define NAV2_CORE__PROGRESS_CHECKER_HPP_

// 中文：本文件定义 Controller Server 用于检测机器人是否持续取得运动进展的有状态插件接口。

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

namespace nav2_core
{
/**
 * @class nav2_core::ProgressChecker
 * @brief This class defines the plugin interface used to check the
 * position of the robot to make sure that it is actually progressing
 * towards a goal.
 * 中文：ProgressChecker 不判断是否到达目标，而是比较一段时间内的位置／姿态变化，识别被困、打滑
 * 中文：或控制无效。Controller Server 在控制循环中调用它，失败时通常终止当前控制并进入恢复链。
 */
class ProgressChecker
{
public:
  // 中文：Controller Server 以共享指针保存当前进度检查器插件。
  typedef std::shared_ptr<nav2_core::ProgressChecker> Ptr;

  // 中文：虚析构用于安全释放具体的状态化检查器实现。
  virtual ~ProgressChecker() {}

  /**
   * @brief Initialize parameters for ProgressChecker
   * @param parent Node pointer
   * 中文：配置阶段读取 `<plugin_name>.*` 下的移动半径、允许停滞时间等实现参数，并初始化内部状态。
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;
  /**
   * @brief Checks if the robot has moved compare to previous
   * pose
   * @param current_pose Current pose of the robot
   * @return True if progress is made
   * 中文：控制循环传入当前机器人 Pose。实现可更新基准 Pose 和时间戳；返回 false 表示在配置窗口内
   * 中文：未达到最低进展要求。引用参数属于调用方，插件不应长期保存其地址。
   */
  virtual bool check(geometry_msgs::msg::PoseStamped & current_pose) = 0;
  /**
   * @brief Reset class state upon calling
   * 中文：新路径、恢复完成或任务重启时清除基准 Pose／计时状态，避免上一任务污染当前判断。
   */
  virtual void reset() = 0;
};
}  // namespace nav2_core

#endif  // NAV2_CORE__PROGRESS_CHECKER_HPP_
