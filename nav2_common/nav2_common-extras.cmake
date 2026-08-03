# Copyright 2019 Intel Corporation
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

set(AMENT_BUILD_CONFIGURATION_KEYWORD_SEPARATOR ":")
# 中文：让 ament 的构建配置关键字使用冒号分隔，保持导出的配置与 ament 约定一致。

include("${nav2_common_DIR}/nav2_package.cmake")
# 中文：下游 find_package(nav2_common) 时加载公开的 nav2_package() 宏，后续 CMakeLists 可直接调用。
