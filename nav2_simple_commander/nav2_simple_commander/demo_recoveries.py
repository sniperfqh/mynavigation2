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
from rclpy.duration import Duration


"""
Basic recoveries demo. In this demonstration, the robot navigates
to a dead-end where recoveries such as backup and spin are used
to get out of it.

中文注解：恢复行为 demo，机器人进入死胡同后依次执行后退和原地旋转。
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # 中文注解：设置 AMCL 初始位姿。
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    # 中文注解：等待 Nav2 active。
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 6.13
    goal_pose.pose.position.y = 1.90
    goal_pose.pose.orientation.w = 1.0

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                f'Estimated time of arrival to destination is: \
                {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9}'
            )

    # Robot hit a dead end, back it up
    # 中文注解：模拟进入死胡同后的后退恢复行为。
    print('Robot hit a dead end, backing up...')
    navigator.backup(backup_dist=0.5, backup_speed=0.1)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Distance traveled: {feedback.distance_traveled}')

    # Turn it around
    # 中文注解：后退后原地旋转，尝试脱困。
    print('Spinning robot around...')
    navigator.spin(spin_dist=3.14)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f'Spin angle traveled: {feedback.angular_distance_traveled}')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Dead end confirmed! Returning to start...')
    elif result == TaskResult.CANCELED:
        print('Recovery was canceled. Returning to start...')
    elif result == TaskResult.FAILED:
        print('Recovering from dead end failed! Returning to start...')

    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()
