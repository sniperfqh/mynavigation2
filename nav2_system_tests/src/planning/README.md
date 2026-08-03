# Global Planner Component Testing

A PlannerTester node provides the world representation in the form of a costmap, sends a request to generate a path, and receives and checks the quality of the generated path.

As mentioned above, currently the world is represented as a costmap. Simplified versions of the world model and costmap are used for testing.

PlannerTester can sequentially pass random starting and goal poses and check the returned path for possible collision along the path.

Below is an example of the output from randomized testing. Blue spheres represent the starting locations, green, the goals. Red lines are the computed paths. Grey cells represent obstacles.

![alt text](example_result.png "Output Example")

*Note: Currently robot size is 1x1 cells, no obstacle inflation is done on the costmap*

*Note: The Navfn algorithm sometimes fails to generate a path as you can see from the 'orphan' spheres.*

## 中文翻译

# 全局规划器组件测试

该测试在示例地图中启动全局规划器组件，生成不同起点和目标点的规划结果，并检查路径是否有效。Navfn 有时可能无法生成路径，结果图中会出现“孤立”球体，这是测试场景和算法边界的一部分。
