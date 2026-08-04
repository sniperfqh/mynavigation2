// Copyright (c) 2022 Samsung R&D Institute Russia
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

#ifndef NAV2_COLLISION_MONITOR__TYPES_HPP_
#define NAV2_COLLISION_MONITOR__TYPES_HPP_

namespace nav2_collision_monitor
{

/// @brief Velocity for 2D model of motion
// 中文：用三个标量描述平面速度，x/y 是机器人基座坐标系下的线速度，tw 是绕 z 轴的角速度。
// 中文：该类型同时作为输入速度、限速后的输出速度和 APPROACH 预测过程中的状态量。
struct Velocity
{
  double x;  // x-component of linear velocity
  double y;  // y-component of linear velocity
  double tw;  // z-component of angular twist

  inline bool operator<(const Velocity & second) const
  {
    // 中文：比较三维速度向量的平方范数，用于在多个安全区域给出的候选速度中选择更保守者。
    // 中文：平方范数避免开平方；同时纳入原地旋转，使线速度为零时仍能比较角速度大小。
    const double first_vel = x * x + y * y + tw * tw;
    const double second_vel = second.x * second.x + second.y * second.y + second.tw * second.tw;
    // This comparison includes rotations in place, where linear velocities are equal to zero
    return first_vel < second_vel;
  }

  inline Velocity operator*(const double & mul) const
  {
    // 中文：按比例缩放线速度和角速度，SLOWDOWN 与 APPROACH 都通过该运算生成安全速度。
    return {x * mul, y * mul, tw * mul};
  }

  inline bool isZero() const
  {
    // 中文：只接受三个分量都精确为零的速度作为“停车”状态，供停止消息超时发布策略使用。
    return x == 0.0 && y == 0.0 && tw == 0.0;
  }
};

/// @brief 2D point
// 中文：所有传感器数据都会被变换并压缩为机器人基座坐标系下的二维障碍点。
struct Point
{
  double x;  // x-coordinate of point
  double y;  // y-coordinate of point
};

/// @brief 2D Pose
// 中文：碰撞预测中的平面位姿；x/y 是模拟机器人位置，theta 是其朝向。
struct Pose
{
  double x;  // x-coordinate of pose
  double y;  // y-coordinate of pose
  double theta;  // rotation angle of pose
};

/// @brief Action type for robot
// 中文：Collision Monitor 对输入速度施加的安全动作，枚举值也用于参数字符串解析后的内部状态。
enum ActionType
{
  DO_NOTHING = 0,  // No action
  STOP = 1,  // Stop the robot
  SLOWDOWN = 2,  // Slowdown in percentage from current operating speed
  APPROACH = 3  // Keep constant time interval before collision
};

/// @brief Action for robot
// 中文：保存一次碰撞判定的动作类型和最终要发布的请求速度。
struct Action
{
  ActionType action_type;
  Velocity req_vel;
};

}  // namespace nav2_collision_monitor

#endif  // NAV2_COLLISION_MONITOR__TYPES_HPP_
