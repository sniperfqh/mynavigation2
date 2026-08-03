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

# 中文：本模块在 Launch 展开期读取 YAML，按参数名、完整路径、键和值四种维度生成临时改写文件。
from collections.abc import Generator
import tempfile
from typing import Optional, TypeAlias, Union

import launch
import yaml

YamlValue: TypeAlias = Union[str, int, float, bool]
# 中文：类型别名描述本模块主动处理的 YAML 标量；实际递归数据还可能包含字典和列表容器。


class DictItemReference:
    # 中文：该轻量引用把字典对象与键绑定起来，允许遍历器在不复制树的情况下原地修改叶值。

    def __init__(self, dictionary: dict[str, YamlValue], key: str):
        # 中文：保存目标字典本身和其中的键；setValue() 会直接写回这个字典。
        self.dictionary = dictionary
        self.dictKey = key

    def key(self) -> str:
        # 中文：返回当前引用对应的键名，供参数同名匹配使用。
        return self.dictKey

    def setValue(self, value: YamlValue) -> None:
        # 中文：原地覆盖键值，使 getYamlLeafKeys() 产生的引用能够修改原 YAML 树。
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):  # type: ignore[misc]
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """
    # 中文：该类是 Nav2 启动配置的核心入口，最终返回一个下游节点可读取的临时 YAML 路径。

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: dict[str, launch.SomeSubstitutionsType],
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        value_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        convert_types: bool = False,
    ) -> None:
        super().__init__()
        """
        Construct the substitution

        :param: source_file the original YAML file to modify
        :param: param_rewrites mappings to replace
        :param: root_key if provided, the contents are placed under this key
        :param: key_rewrites keys of mappings to replace
        :param: value_rewrites values to replace
        :param: convert_types whether to attempt converting the string to a number or boolean
        """
        # 中文：构造阶段只规范化替换规则，不读取文件；文件路径和 LaunchConfiguration 要延迟到 perform()。

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions
        # 中文：延迟导入可避免 launch 工具初始化顺序造成循环依赖。

        self.__source_file: list[launch.Substitution] = \
            normalize_to_list_of_substitutions(source_file)
        # 中文：源文件统一存为 Substitution 列表，以便后续按 LaunchContext 求值。
        self.__param_rewrites = {}
        # 中文：param_rewrites 保存参数名或点分绝对路径到新值的规则。
        self.__key_rewrites = {}
        # 中文：key_rewrites 保存 YAML 映射键名替换规则。
        self.__value_rewrites = {}
        # 中文：value_rewrites 保存全树标量值替换规则。
        self.__convert_types = convert_types
        # 中文：该开关只控制数字转换；布尔文本在 convert() 中始终会尝试识别。
        self.__root_key = None
        # 中文：根键默认为空；设置后 perform() 会把完整结果包在该键下。
        for key in param_rewrites:
            # 中文：逐项规范化参数替换值，保留键名以便后续在上下文中解析值。
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(
                param_rewrites[key]
            )
        if key_rewrites is not None:
            # 中文：只有调用方提供键替换表时才建立对应的延迟求值列表。
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(
                    key_rewrites[key]
                )
        if value_rewrites is not None:
            # 中文：值替换的键是原始 YAML 标量的字符串表示，值可由 Launch Substitution 组成。
            for value in value_rewrites:
                self.__value_rewrites[value] = normalize_to_list_of_substitutions(
                    value_rewrites[value]
                )
        if root_key is not None:
            # 中文：根键也延迟求值，支持由 LaunchConfiguration 动态决定输出 YAML 的顶层名称。
            self.__root_key = normalize_to_list_of_substitutions(root_key)

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        # 中文：返回源 YAML 的 Substitution 列表，perform() 会结合 LaunchContext 得到真实路径。
        return self.__source_file

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        # 中文：当前实现不输出额外描述，因此返回空字符串，不影响改写流程。
        return ''

    def perform(self, context: launch.LaunchContext) -> str:
        # 中文：按固定顺序完成路径解析、读取、参数改写、键值改写、根键包装和临时文件输出。
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)
        # 中文：源路径在 Launch 展开期才确定，支持 LaunchConfiguration 等动态替换。
        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False)
        # 中文：输出文件使用 delete=False，以便把持久化路径返回给下游 ROS 节点；本类不负责删除。
        param_rewrites, keys_rewrites, value_rewrites = self.resolve_rewrites(context)
        # 中文：先解析三组规则的值，后续树遍历只处理普通字符串，不再触碰 Launch Substitution。

        with open(yaml_filename, 'r') as yaml_file:
            # 中文：safe_load 将 YAML 文本转换为可递归处理的字典／列表／标量对象。
            data = yaml.safe_load(yaml_file)

        # 中文：参数改写顺序是先替换已有参数，再新增缺失参数，随后处理键和值，保证路径规则可覆盖同名结果。
        self.substitute_params(data, param_rewrites)
        self.add_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        self.substitute_values(data, value_rewrites)
        if self.__root_key is not None:
            # 中文：根键为空字符串时不包装；非空时把原树整体放到新的顶层映射下。
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        # 中文：使用 PyYAML 序列化最终树并关闭文件，返回临时文件的绝对路径。
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context: launch.LaunchContext) -> \
            tuple[dict[str, str], dict[str, str], dict[str, str]]:
        # 中文：分别解析参数、键和值三张规则表，返回普通字符串字典供树处理函数使用。
        resolved_params = {}
        for key in self.__param_rewrites:
            # 中文：参数替换值可由 LaunchConfiguration 等对象组成，此处绑定当前上下文。
            resolved_params[key] = launch.utilities.perform_substitutions(
                context, self.__param_rewrites[key]
            )
        resolved_keys = {}
        for key in self.__key_rewrites:
            # 中文：键替换值解析后用于映射键重命名，不改变对应值对象本身。
            resolved_keys[key] = launch.utilities.perform_substitutions(
                context, self.__key_rewrites[key]
            )
        resolved_values = {}
        for value in self.__value_rewrites:
            # 中文：值替换值解析后用于全树标量匹配，匹配依据是原值的字符串形式。
            resolved_values[value] = launch.utilities.perform_substitutions(
                context, self.__value_rewrites[value]
            )
        # 中文：三张表的返回顺序与 perform() 的后续调用顺序一致。
        return resolved_params, resolved_keys, resolved_values

    def substitute_params(self, yaml: dict[str, YamlValue],
                          param_rewrites: dict[str, str]) -> None:
        # 中文：先处理已有叶键的同名替换，再处理点分绝对路径，后者可以覆盖前者的结果。
        # substitute leaf-only parameters
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                # 中文：把同名参数的新字符串转换成目标 YAML 标量，并通过引用原地写回。
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        # 中文：pathify() 将所有叶值展平为 key.keyA.keyB.val 形式，支持精确定位嵌套参数。
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                # 中文：完整路径规则只命中同名路径，不会误改其他层级的同名键。
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)

    def add_params(self, yaml: dict[str, YamlValue],
                   param_rewrites: dict[str, str]) -> None:
        # 中文：只为尚不存在且路径包含 ros__parameters 的规则创建新参数，避免随意扩张 YAML 结构。
        # add new total path parameters
        yaml_paths = self.pathify(yaml)
        for path in param_rewrites:
            if not path in yaml_paths:  # noqa: E713
                # 中文：缺失路径先转换值，再沿点分路径按需创建中间字典节点。
                new_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                if 'ros__parameters' in yaml_keys:
                    # 中文：ROS 参数约定要求新增参数位于 ros__parameters 分支下，其他路径被忽略。
                    yaml = self.updateYamlPathVals(yaml, yaml_keys, new_val)

    def substitute_values(
            self, yaml: dict[str, YamlValue],
            value_rewrites: dict[str, str]) -> None:
        # 中文：启动全树值替换；键名不参与匹配，字典、列表和标量都通过递归处理。

        def process_value(value: YamlValue) -> YamlValue:
            # 中文：递归函数保持容器结构，只替换标量并返回新的列表或原字典对象。
            if isinstance(value, dict):
                # 中文：字典逐键递归，修改其值但保留原有键名。
                for k, v in list(value.items()):
                    value[k] = process_value(v)
                return value
            elif isinstance(value, list):
                # 中文：列表生成新列表，确保列表内嵌套字典和标量也能参与匹配。
                return [process_value(v) for v in value]
            elif str(value) in value_rewrites:
                # 中文：标量按字符串形式查找替换规则，再按 convert_types 及布尔规则转换结果。
                return self.convert(value_rewrites[str(value)])
            # 中文：没有命中规则的标量保持原类型和原值。
            return value

        # 中文：从根映射的每个值开始递归，覆盖整个 YAML 数据树。
        for key in list(yaml.keys()):
            yaml[key] = process_value(yaml[key])

    def updateYamlPathVals(
            self, yaml: dict[str, YamlValue],
            yaml_key_list: list[str], rewrite_val: YamlValue) -> dict[str, YamlValue]:
        # 中文：沿点分路径递归下降；遇到列表索引按整数访问，遇到字典键则按需创建中间映射。

        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                # 中文：到达路径末端时写入新值并结束当前递归层。
                yaml[key] = rewrite_val
                break
            # 中文：逐层弹出已消费的键，剩余列表传给下一层继续定位。
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                # 中文：列表路径片段必须转换成整数索引，随后递归更新对应元素。
                yaml[int(key)] = self.updateYamlPathVals(
                    yaml[int(key)], yaml_key_list, rewrite_val
                )
            else:
                # 中文：字典缺少中间键时以空字典作为默认值，支持 add_params() 创建新路径。
                yaml[key] = self.updateYamlPathVals(  # type: ignore[assignment]
                    yaml.get(key, {}),  # type: ignore[arg-type]
                    yaml_key_list,
                    rewrite_val
                )
        # 中文：返回更新后的当前容器，供上一层递归把结果重新挂回原树。
        return yaml

    def substitute_keys(
            self, yaml: dict[str, YamlValue], key_rewrites: dict[str, str]) -> None:
        # 中文：递归重命名字典键；值对象保持不变，键冲突时由后写入的键覆盖原有映射关系。
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    # 中文：先复制到新键，再删除旧键，完成当前映射层的重命名。
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                if isinstance(val, dict):
                    # 中文：只对嵌套字典继续递归；列表中的字典不会由此函数继续展开。
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData: dict[str, YamlValue]) -> \
            Generator[DictItemReference, None, None]:
        # 中文：深度优先遍历映射并产出键引用，调用方可通过引用直接修改原 YAML 对象。
        if not isinstance(yamlData, dict):
            # 中文：遇到非字典输入时停止当前分支，不产生任何引用。
            return

        for key in yamlData.keys():
            child = yamlData[key]

            if isinstance(child, dict):
                # Recursively process nested dictionaries
                # 中文：先递归产出子映射引用，再产出当前键对应的父级映射引用。
                yield from self.getYamlLeafKeys(child)

            # 中文：该实现不仅产出真正叶键，也会产出指向字典值的父级键引用。
            yield DictItemReference(yamlData, key)

    def pathify(
            self, d: Union[dict[str, YamlValue], list[YamlValue], YamlValue],
            p: Optional[str] = None,
            paths: Optional[dict[str, YamlValue]] = None,
            joinchar: str = '.') -> dict[str, YamlValue]:
        # 中文：把嵌套字典和列表的叶值扁平化为点分路径，供绝对路径参数替换匹配。
        if p is None:
            # 中文：顶层调用负责创建结果字典并从空路径开始递归。
            paths = {}
            self.pathify(d, '', paths, joinchar=joinchar)
            return paths

        assert paths is not None
        pn = p
        if p != '':
            # 中文：非根层路径追加分隔符，保证后续键名或列表索引直接拼接即可。
            pn += joinchar
        if isinstance(d, dict):
            # 中文：字典把键名追加到当前路径后继续递归。
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            # 中文：列表把整数索引转换为路径片段，支持参数路径定位列表元素。
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            # 中文：到达标量叶值时记录完整路径和值，递归结果通过共享 paths 汇总。
            paths[p] = d
        # 中文：递归分支返回同一张路径字典，便于上层继续追加结果。
        return paths

    def convert(self, text_value: str) -> YamlValue:
        # 中文：把 Launch 传入的字符串转换为 YAML 标量；数字是否转换由 convert_types 控制。
        if self.__convert_types:
            # try converting to int or float
            # 中文：包含小数点时优先尝试 float，否则尝试 int；科学计数法没有小数点时会保持字符串。
            try:
                return float(text_value) if '.' in text_value else int(text_value)
            except ValueError:
                # 中文：数字解析失败后继续尝试布尔值和普通字符串，不在此处抛出异常。
                pass

        # try converting to bool
        # 中文：布尔文本始终转换且大小写不敏感，与 convert_types 开关无关。
        if text_value.lower() == 'true':
            return True
        if text_value.lower() == 'false':
            return False

        # nothing else worked so fall through and return text
        # 中文：其他文本保持字符串类型，包含 null、科学计数法等未被当前规则识别的值。
        return text_value
