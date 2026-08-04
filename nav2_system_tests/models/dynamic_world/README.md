# Dynamic World Model

A world model with a ground plane, 20x20 meter empty room, and 9 dynamic obstacles.

## How to Add a Dynamic Obstacle

dynamic_world/world.model  

    <model name="DynamicObstacle">
          -- Position of the new obstacle
           <pose>-4 4 0.15 0 0 0</pose>
           <include>
            <uri>model://dynamic_obstacle</uri>
          </include>
    </model>

## 中文翻译

# 动态世界模型

该模型提供包含动态障碍物的测试世界。要添加动态障碍，应在世界或模型文件中加入对应 SDF 模型和运动插件，并在测试 Launch 中加载它；保持模型名称、Topic 和插件参数与测试用例一致。
