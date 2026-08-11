# 中文说明：构建进展检查器测试，并链接两个被测插件动态库；pctest 覆盖平移、旋转和超时逻辑。
ament_add_gtest(pctest progress_checker.cpp)
target_link_libraries(pctest simple_progress_checker pose_progress_checker)
# 中文说明：构建目标检查器测试，并链接普通与停稳 GoalChecker；gctest 覆盖位姿、速度及动态参数行为。
ament_add_gtest(gctest goal_checker.cpp)
target_link_libraries(gctest simple_goal_checker stopped_goal_checker)
