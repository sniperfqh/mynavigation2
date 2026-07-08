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
Basic navigation demo to go to pose.

中文注解：单目标点导航示例，演示 goToPose、feedback、取消和抢占。
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # 中文注解：设置 AMCL 初始位姿，必须在等待 Nav2 active 前完成。
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.0
    initial_pose.pose.position.y = -0.5
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # 中文注解：如果 Nav2 没有 autostart，可以手动启动生命周期，但必须在 setInitialPose() 后调用。
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # 中文注解：否则 AMCL 会以地图原点初始化，代价地图可能被错误传感器数据污染。
    # If autostart, you should `waitUntilNav2Active()` instead.
    # 中文注解：当前 launch 默认 autostart 时，只需要等待 Nav2 active。
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    # 中文注解：阻塞直到 AMCL 和 bt_navigator 都处于 active 状态。
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # 中文注解：需要切换地图时可调用 map_server/load_map 服务。
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # 中文注解：调试局部/全局代价地图时可清理或读取 costmap。
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    # 中文注解：构造 map 坐标系下的导航目标位姿。
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0

    # sanity check a valid path exists
    # 中文注解：可选的路径可达性检查，只规划不执行。
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        # 中文注解：读取 ETA、导航耗时等反馈，演示超时取消和目标抢占。
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            # 中文注解：超过设定时间后取消当前导航任务。
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            # 中文注解：发送新目标可抢占当前目标。
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                goal_pose.pose.position.x = -3.0
                navigator.goToPose(goal_pose)

    # Do something depending on the return code
    # 中文注解：任务结束后根据 action 状态输出结果。
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
