// Copyright (c) 2020 Fetullah Atas
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


#ifndef NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#define NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
#pragma once

// 中文：本文件定义 Waypoint Follower 到达每个航点后执行附加业务任务的插件接口。

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_core
{
/**
 * @brief Base class for creating a plugin in order to perform a specific task at waypoint arrivals.
 *
 * 中文：WaypointTaskExecutor 将导航移动与到点业务解耦，可实现等待、拍照或人工确认等任务。
 * 中文：Waypoint Follower 负责 FollowWaypoints Action、航点索引和导航结果；插件只处理当前到达事件。
 */
class WaypointTaskExecutor
{
public:
  /**
   * @brief Construct a new Simple Task Execution At Waypoint Base object
   *
   * 中文：默认构造不创建 ROS 资源，具体发布器、订阅器和参数应在 initialize() 中准备。
   */
  WaypointTaskExecutor() {}

  /**
   * @brief Destroy the Simple Task Execution At Waypoint Base object
   *
   * 中文：虚析构保证 Waypoint Follower 通过基类指针释放具体任务插件时执行完整析构链。
   */
  virtual ~WaypointTaskExecutor() {}

  /**
   * @brief Override this to setup your pub, sub or any ros services that you will use in the plugin.
   *
   * @param parent parent node that plugin will be created within(for an example see nav_waypoint_follower)
   * @param plugin_name plugin name comes from parameters in yaml file
   * 中文：Waypoint Follower 配置阶段传入父 Lifecycle Node 和插件参数命名空间。实现可在此读取配置并
   * 中文：创建所需 ROS 接口；parent 为弱引用，必须检查 lock() 结果。
   */
  virtual void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) = 0;

  /**
   * @brief Override this to define the body of your task that you would like to execute once the robot arrived to waypoint
   *
   * @param curr_pose current pose of the robot
   * @param curr_waypoint_index current waypoint, that robot just arrived
   * @return true if task execution was successful
   * @return false if task execution failed
   * 中文：机器人到达航点后同步调用。curr_pose 是当前到达 Pose，curr_waypoint_index 是原航点序号；
   * 中文：true 表示该点任务完成，false 表示失败，后续是否停止整个 FollowWaypoints 由上层策略决定。
   */
  virtual bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) = 0;
};
}  // namespace nav2_core
#endif  // NAV2_CORE__WAYPOINT_TASK_EXECUTOR_HPP_
