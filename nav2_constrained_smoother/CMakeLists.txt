cmake_minimum_required(VERSION 3.5)
project(nav2_constrained_smoother)
# 中文：本包编译为 nav2_core::Smoother Pluginlib 共享库，并在测试阶段构建两个 GTest 可执行文件。

set(CMAKE_BUILD_TYPE Release) # significant Ceres optimization speedup
# 中文：Release 优化能显著降低 Ceres 数值优化的运行开销；下游仍可显式覆盖构建类型。

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
# 中文：Ceres 提供非线性最小二乘求解，SuiteSparse 用于稀疏线性方程求解器。

set(CMAKE_CXX_STANDARD 17)

if(${CERES_VERSION} VERSION_LESS_EQUAL 2.0.0)
  # 中文：旧版 Ceres 使用不同的无穷值 API，通过宏选择兼容实现。
  add_definitions(-DUSE_OLD_CERES_API)
endif()

nav2_package()
# 中文：继承 Nav2 的统一 C++17、告警、位置无关代码和覆盖率构建约定。

set(library_name nav2_constrained_smoother)
# 中文：共享库名称同时用于目标安装、导出和插件 XML 的动态加载。

include_directories(
  include
  ${CERES_INCLUDES}
)
# 中文：公开头文件和 Ceres 头文件路径供生产源文件及测试目标使用。

set(dependencies
  angles
  rclcpp
  rclcpp_action
  nav2_msgs
  nav2_costmap_2d
  nav2_util
  nav2_core
  pluginlib
)
# 中文：这些依赖同时用于生产插件的链接、公开头文件的接口解析和测试目标的编译。

add_library(${library_name} SHARED src/constrained_smoother.cpp)
# 中文：只把插件入口实现编译进共享库，算法模板主要位于头文件中。
target_link_libraries(${library_name} ${CERES_LIBRARIES})
# prevent pluginlib from using boost
# 中文：沿用 Nav2 的 Pluginlib 配置，避免插件导出宏依赖 Boost 函数封装。
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(${library_name} ${dependencies})

if(BUILD_TESTING)
  # 中文：测试构建打开时加载 Ament Lint 和 GTest 依赖，并递归构建 test/ 子目录。
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(
  TARGETS
    ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# 中文：安装共享库到 lib，供 Pluginlib 按 XML 描述动态加载。

install(DIRECTORY include/
  DESTINATION include/
)
# 中文：安装公开头文件，使下游包可以复用插件接口和内部算法类型。

if(BUILD_TESTING)
  # 中文：再次确保测试环境执行统一的 Ament 静态检查依赖发现。
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
# 中文：向下游导出头文件目录、库目标和依赖包，支持 find_package 后链接插件。
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_constrained_smoother.xml)
# 中文：把 XML 中的 nav2_core 插件描述导出到安装空间，供 Pluginlib 运行时发现插件。

# 中文：生成 Ament 包配置，使下游能够发现本包的头文件、共享库、依赖和插件索引。
ament_package()
