# Copyright (c) 2023 LG Electronics.
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

# 中文：本模块从 ros2 launch 的命令行参数中解析多个机器人的命名空间与初始六自由度位姿。
import yaml
import sys
from typing import Text, Dict


class ParseMultiRobotPose():
    """
    Parsing argument using sys module
    """
    # 中文：该类直接读取进程参数，不继承 launch.Substitution，因此解析时机由调用方主动控制。

    def __init__(self, target_argument: Text):
        """
        Parse arguments for multi-robot's pose

        for example,
        `ros2 launch nav2_bringup bringup_multirobot_launch.py
            robots:="robot1={x: 1.0, y: 1.0, yaw: 0.0};
                     robot2={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"`

        `target_argument` shall be 'robots'.
        Then, this will parse a string value for `robots` argument.

        Each robot name which is corresponding to namespace and pose of it will be separted by `;`.
        The pose consists of x, y and yaw with YAML format.

        :param: target argument name to parse
        """
        # 中文：构造函数立即提取目标参数原始字符串，并把结果保存供 value() 后续拆分。
        self.__args: Text = self.__parse_argument(target_argument)

    def __parse_argument(self, target_argument: Text) -> Text:
        """
        get value of target argument
        """
        # 中文：只扫描 sys.argv 的第 5 项及之后，跳过 ros2 launch 自身的固定前缀参数。
        if len(sys.argv) > 4:
            argv = sys.argv[4:]
            # 中文：每个候选项应为 <参数名>:=<参数值>；只返回第一个名称匹配项。
            for arg in argv:
                if arg.startswith(target_argument + ":="):
                    # 中文：删除一次参数名前缀，保留后续的分号分隔机器人描述原文。
                    return arg.replace(target_argument + ":=", "")
        # 中文：未找到目标参数时返回空字符串，value() 会把它解释成空机器人集合。
        return ""

    def value(self) -> Dict:
        """
        get value of target argument
        """
        # 中文：把每个机器人描述拆成名称和 YAML 位姿，并补齐缺省的六个位姿字段。
        args = self.__args
        # 中文：分号分隔不同机器人；没有参数时使用空列表，避免制造一个空名称条目。
        parsed_args = list() if len(args) == 0 else args.split(';')
        multirobots = dict()
        # 中文：每次循环处理一个 namespace=pose 片段，格式不正确的片段会被静默跳过。
        for arg in parsed_args:
            key_val = arg.strip().split('=')
            if len(key_val) != 2:
                # 中文：等号数量不符合预期时无法区分名称和 YAML 值，因此不加入结果。
                continue
            key = key_val[0].strip()
            val = key_val[1].strip()
            # 中文：YAML 负责把字符串位姿转换为字典；非映射值会在后续键访问处暴露格式错误。
            robot_pose = yaml.safe_load(val)
            if 'x' not in robot_pose:
                # 中文：平移 x 缺省为 0，表示机器人从参考坐标原点开始。
                robot_pose['x'] = 0.0
            if 'y' not in robot_pose:
                # 中文：平移 y 缺省为 0。
                robot_pose['y'] = 0.0
            if 'z' not in robot_pose:
                # 中文：平移 z 缺省为 0，适用于二维地面仿真。
                robot_pose['z'] = 0.0
            if 'roll' not in robot_pose:
                # 中文：横滚角缺省为 0。
                robot_pose['roll'] = 0.0
            if 'pitch' not in robot_pose:
                # 中文：俯仰角缺省为 0。
                robot_pose['pitch'] = 0.0
            if 'yaw' not in robot_pose:
                # 中文：偏航角缺省为 0；这是二维机器人最常用的朝向默认值。
                robot_pose['yaw'] = 0.0
            # 中文：以机器人名称作为键保存完整位姿，后续 Launch 通常用该键构造 namespace。
            multirobots[key] = robot_pose
        # 中文：返回名称到六自由度位姿字典的映射；无有效输入时返回空字典。
        return multirobots
