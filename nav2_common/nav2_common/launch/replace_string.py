# Copyright (c) 2019 Intel Corporation
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

# 中文：本模块把 Launch 中的替换值解析为字符串，并将源文件内容写入新的临时文件。
from typing import Dict
from typing import List
from typing import Text
from typing import Optional
import tempfile
import launch

class ReplaceString(launch.Substitution):
  """
  Substitution that replaces strings on a given file.

  Used in launch system
  """
  # 中文：它适合替换 RViz 或参数文件中的占位文本，不修改原始文件，而是返回临时文件路径。

  def __init__(self,
    source_file: launch.SomeSubstitutionsType,
    replacements: Dict,
    condition: Optional[launch.Condition] = None) -> None:
    super().__init__()

    from launch.utilities import normalize_to_list_of_substitutions  # import here to avoid loop
    # 中文：延迟导入 Launch 工具，避免模块初始化阶段形成循环依赖。
    self.__source_file = normalize_to_list_of_substitutions(source_file)
    # 中文：源文件统一保存为 Substitution 列表，待 perform() 使用 LaunchContext 求值。
    self.__replacements = {}
    # 中文：每个替换值也提前规范化，但真正的文本内容要在当前上下文中解析。
    for key in replacements:
        # 中文：保留调用方字典的键顺序，并把 value 包装成可延迟求值的列表。
        self.__replacements[key] = normalize_to_list_of_substitutions(replacements[key])
    self.__condition = condition
    # 中文：可选条件决定是否生成新文件；条件不满足时直接沿用原始路径。

  @property
  def name(self) -> List[launch.Substitution]:
    """Getter for name."""
    # 中文：返回源文件的 Substitution 表示，供 Launch 在 perform() 中解析。
    return self.__source_file

  @property
  def condition(self) -> Optional[launch.Condition]:
    """Getter for condition."""
    # 中文：暴露构造时保存的条件对象，便于 Launch 或调用方检查当前替换是否有条件约束。
    return self.__condition

  def describe(self) -> Text:
    """Return a description of this substitution as a string."""
    # 中文：当前实现没有生成描述文本，因此返回空字符串。
    return ''

  def perform(self, context: launch.LaunchContext) -> Text:
    # 中文：Launch 展开回调负责解析源路径、评估条件、解析替换值并生成最终临时文件。
    yaml_filename = launch.utilities.perform_substitutions(context, self.name)
    # 中文：只有条件为空或评估为真时才执行文本替换，否则返回原始文件路径。
    if self.__condition is None or self.__condition.evaluate(context):
      # 中文：delete=False 保留文件名给下游节点使用；临时文件生命周期不在本类内清理。
      output_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
      replacements = self.resolve_replacements(context)
      # 中文：替换值在打开输入文件前解析，失败时仍进入统一异常处理和文件关闭流程。
      try:
        input_file = open(yaml_filename, 'r')
        # 中文：replace() 逐行处理输入，并把每个键值替换写入输出文件。
        self.replace(input_file, output_file, replacements)
      except Exception as err:  # noqa: B902
        # 中文：现有实现只打印错误并继续返回临时路径；调用方需自行关注生成内容是否完整。
        print('ReplaceString substitution error: ', err)
      finally:
        # 中文：正常打开输入文件时这里关闭两个句柄；若 open() 失败，现有代码会因 input_file 未绑定而再次抛异常。
        input_file.close()
        output_file.close()
      # 中文：返回新文件名，让下游参数或 RViz 节点读取替换后的副本。
      return output_file.name
    else:
      # 中文：条件不满足时不创建副本，返回原始文件名以保持输入内容不变。
      return yaml_filename

  def resolve_replacements(self, context):
    # 中文：把每个替换值从 Substitution 列表解析成普通字符串，形成实际替换表。
    resolved_replacements = {}
    for key in self.__replacements:
      # 中文：键本身保持原样，只有值需要结合 LaunchContext 延迟求值。
      resolved_replacements[key] = launch.utilities.perform_substitutions(context, self.__replacements[key])
    # 中文：返回键到最终文本值的映射，供 replace() 执行逐行替换。
    return resolved_replacements

  def replace(self, input_file, output_file, replacements):
    # 中文：按输入文件原始行顺序处理，每行依次应用所有字符串替换规则。
    for line in input_file:
      for key, value in replacements.items():
        if isinstance(key, str) and isinstance(value, str):
          # 中文：只有键和值都是字符串时才执行替换；同一行可被多条规则连续改写。
          if key in line:
            line = line.replace(key, value)
        else:
          # 中文：非字符串替换对属于调用契约错误，立即抛出类型异常交给 perform() 记录。
          raise TypeError('A provided replacement pair is not a string. Both key and value should be strings.')
      # 中文：写出当前行，未命中的行保持原样，换行符也随输入一并保留。
      output_file.write(line)
