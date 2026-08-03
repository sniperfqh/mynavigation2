# Dynamic Obstacle

Dynamic Obstacle. 30x30x30 cm box.

Updates its velocity direction and amount every 3 seconds. Max velocity is 0.5. 

## How to Configure a Dynamic Obstacle

dynamic_obstacle/model.sdf

      <plugin name="random" filename="libRandomVelocityPlugin.so">
      
        -- Name of the link in this model that receives the velocity
        <link>link</link>
        
        -- Initial velocity that is applied to the link
        <initial_velocity>0 0.05 0</initial_velocity>
        
        -- Scaling factor that is used to compute a new velocity
        <velocity_factor>0.5</velocity_factor>
        
        -- Time delay between new velocities
        <update_period>3</update_period>
        
        -- X velocity treshold values
        <min_x>-0.5</min_x>
        <max_x>0.5</max_x>
        
        -- Y velocity treshold values
        <min_y>-0.5</min_y>
        <max_y>0.5</max_y>
        
        -- Z velocity treshold values
        <min_z>-10</min_z>
        <max_z>-5</max_z>
        
      </plugin>

## 中文翻译

# 动态障碍物

该模型用于系统测试中的动态障碍物场景。按照原文的 SDF 和插件配置，为障碍物设置运动轨迹、速度和更新周期；修改模型文件后重新启动测试 Launch，即可验证 Costmap 和控制器对移动障碍的响应。
