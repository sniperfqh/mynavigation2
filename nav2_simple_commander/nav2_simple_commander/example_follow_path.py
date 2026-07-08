#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


"""
Basic navigation demo to follow a given path after smoothing

中文注解：路径跟随示例，先规划路径，再调用 smoother 平滑，最后交给 controller 执行。
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # 中文注解：设置机器人在 map 坐标系下的初始定位。
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    # 中文注解：等待 Nav2 全部启动完成，避免 action server 尚未可用。
    navigator.waitUntilNav2Active()

    # Go to our demos first goal pose
    # 中文注解：构造目标点，只用于规划路径，不直接触发导航。
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -3.0
    goal_pose.pose.position.y = -2.0
    goal_pose.pose.orientation.w = 1.0

    # Get the path, smooth it
    # 中文注解：调用 planner_server 生成路径，再调用 smoother_server 平滑路径。
    path = navigator.getPath(initial_pose, goal_pose)
    smoothed_path = navigator.smoothPath(path)

    # Follow path
    # 中文注解：把平滑后的路径发送给 controller_server 执行。
    navigator.followPath(smoothed_path, controller_id='DWB')

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        # 中文注解：周期性读取 action feedback，可用于 UI、日志或任务调度。
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated distance remaining to goal position: ' +
                  '{0:.3f}'.format(feedback.distance_to_goal) +
                  '\nCurrent speed of the robot: ' +
                  '{0:.3f}'.format(feedback.speed))

    # Do something depending on the return code
    # 中文注解：根据 Nav2 action 结束状态做业务分支处理。
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
