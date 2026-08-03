ament_add_gtest(test_constrained_smoother
  test_constrained_smoother.cpp
)
# 中文：构建包含 Lifecycle、Costmap 和完整插件链路的集成式平滑测试。

target_link_libraries(test_constrained_smoother
  ${library_name}
)
# 中文：测试直接链接待测共享库，验证 Plugin 实现而不是复制测试算法。
ament_target_dependencies(test_constrained_smoother
  ${dependencies}
)

ament_add_gtest(test_smoother_cost_function
  test_smoother_cost_function.cpp
)
# 中文：构建只覆盖几何工具和四项代价残差的轻量单元测试。
target_link_libraries(test_smoother_cost_function
  ${library_name}
)
# 中文：复用生产库中的模板实现，确保单元测试与实际插件使用同一套代码。
ament_target_dependencies(test_smoother_cost_function
  ${dependencies}
)
