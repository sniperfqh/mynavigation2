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

#
# Standard Nav2 project setup
#
# @public
#
macro(nav2_package)
  # 中文：该宏是 Nav2 CMake 工程的统一入口，所有下游包调用它来获得一致的编译默认值。
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    # 中文：单配置生成器没有显式指定构建类型时，统一采用 Release，避免不同包使用不同默认值。
    message(STATUS "Setting build type to Release as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE
        STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    # 中文：把常见构建类型写入缓存属性，便于 CMake GUI 和命令行选择。
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # Default to C++14
  # 中文：英文历史注释写的是 C++14，但当前代码实际将默认标准设置为 C++17。
  if(NOT CMAKE_CXX_STANDARD)
    # 中文：只有下游没有主动指定标准时才设置 C++17，尊重下游更高层的显式配置。
    set(CMAKE_CXX_STANDARD 17)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # 中文：对 GCC 和 Clang 开启告警、将告警视为错误，并要求位置无关代码以兼容共享库。
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
  endif()

  option(COVERAGE_ENABLED "Enable code coverage" FALSE)
  # 中文：覆盖率默认关闭；开启后同时给编译阶段和可执行文件／共享库链接阶段追加覆盖率参数。
  if(COVERAGE_ENABLED)
    add_compile_options(--coverage)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
  endif()

  # Defaults for Microsoft C++ compiler
  # 中文：MSVC 分支只处理 Windows 编译器特有的符号导出和数学常量宏，不影响 GCC／Clang 分支。
  if(MSVC)
    # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
    # 中文：让 CMake 自动导出 DLL 符号，减少下游手写 __declspec 导出的需要。
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

    # Enable Math Constants
    # 中文：允许 Windows CRT 暴露 M_PI 等数学常量，保持与非 MSVC 平台的源码可移植性。
    # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
    add_compile_definitions(
      _USE_MATH_DEFINES
    )
  endif()
  # 中文：宏结束后，调用方继续执行自己的目标定义；本宏不创建可执行文件或库目标。
endmacro()
