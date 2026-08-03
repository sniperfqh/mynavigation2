# Copyright (c) 2021 PAL Robotics S.L.
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

# 中文：本模块提供一个 Launch Substitution，在启动展开期检查参数 YAML 顶层是否存在指定节点名。
from typing import List
from typing import Text

import yaml
import launch

import sys # delete this

class HasNodeParams(launch.Substitution):
  """
  Substitution that checks if a param file contains parameters for a node

  Used in launch system
  """
  # 中文：该类不创建 ROS 节点，只把“参数文件是否含有节点配置”转换成 Launch 可消费的文本结果。

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    node_name: Text) -> None:
    super().__init__()
    """
    Construct the substitution

    :param: source_file the parameter YAML file
    :param: node_name the name of the node to check
    """
    # 中文：构造阶段只保存尚未解析的 Substitution；真正的文件路径要等 LaunchContext 可用时再解析。

    from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
    # 中文：延迟导入避免模块初始化时形成 launch 工具之间的循环依赖。
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    # 中文：规范化后即使调用方传入字符串或 Substitution，内部都统一成列表表示。
    self.__node_name = node_name
    # 中文：记录待查询的 YAML 顶层键；本实现只检查顶层，不递归搜索嵌套节点。

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    # 中文：返回源文件 Substitution 列表，Launch 会在 perform() 中结合上下文求值。
    return self.__source_file

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    # 中文：当前实现不提供额外描述，因此返回空字符串；这不影响实际求值。
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    # 中文：Launch 展开回调：先解析路径，再读取 YAML，最后返回可被条件表达式继续消费的文本。
    yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    # 中文：source_file 可能包含 LaunchConfiguration 等延迟替换，此处才得到最终文件名。
    data = yaml.safe_load(open(yaml_filename, 'r'))
    # 中文：safe_load 将 YAML 转为 Python 对象；代码假定顶层是映射，因此直接检查其键集合。

    if self.__node_name in data.keys():
        # 中文：节点名存在时返回字符串 "True"，而不是 Python bool，以匹配 Substitution 接口。
        return "True"
    # 中文：节点名不存在时返回字符串 "False"；调用方可在 Launch 条件中继续比较该结果。
    return "False"
