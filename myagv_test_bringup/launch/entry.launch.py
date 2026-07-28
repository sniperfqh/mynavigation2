# Copyright (c) 2018 Intel Corporation
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

"""myAGV 实车导航总入口。

本文件负责声明统一的 launch 参数，并按需启动地图服务器、Nav2 导航栈、RViz、
控制指令转发节点，以及兼容保留的 Gazebo Classic 服务端和客户端。

当前默认配置面向实车：不启动仿真器，不在此处启动 AMCL，也不发布机器人模型 TF。
定位系统应在外部提供 Nav2 所需的 TF；导航栈产生的 ``/cmd_vel`` 由
``controlpub`` 转发到 ``/control_to_uart``，交给底盘通信链路。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取本包安装后的 share 目录，用于定位默认地图、参数和 RViz 配置文件。
    bringup_dir = get_package_share_directory('myagv_test_bringup')
    # 当前 launch 文件所在目录，用于 include 同目录下的子 launch 文件。
    launch_dir = os.path.dirname(__file__)

    # LaunchConfiguration 是运行期替换对象。这里仅建立参数引用，实际值会在
    # LaunchDescription 执行 DeclareLaunchArgument 后确定，也可由命令行覆盖。
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    log_level = LaunchConfiguration('log_level')

    # 以下机器人描述读取逻辑暂时停用。当前入口不启动 robot_state_publisher，
    # 机器人本体 TF 应由外部定位、驱动或专门的模型发布节点提供。
    # robot_description_file = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle.urdf')
    # with open(robot_description_file, 'r', encoding='utf-8') as urdf_file:
    #     robot_description = urdf_file.read()

    # 将绝对 TF 话题重映射为相对话题，使 namespace 生效时 TF 也能进入对应命名空间。
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # 在启动时覆盖参数文件中的公共字段：所有节点统一使用同一个时钟源，
    # map_server 的 yaml_filename 则使用本 launch 的 map 参数。
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    # RewrittenYaml 生成运行期临时参数视图，不直接修改磁盘上的 nav2_params.yaml。
    # root_key=namespace 可在启用命名空间时把参数放到对应层级；convert_types=True
    # 会把命令行字符串转换为 bool、int、float 等 ROS 参数类型。
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # ------------------------- 通用 Nav2 参数 -------------------------
    # 顶层命名空间。多机器人部署时可设置不同 namespace 隔离节点和话题。
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # 控制导航子 launch 是否真正应用上面的 namespace。
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    # 兼容上层启动接口的 SLAM 开关；本文件当前没有根据它启动 SLAM Toolbox。
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    # map_server 加载的静态地图 YAML；命令行传入 map:=... 可替换默认地图。
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            bringup_dir, 'maps', 'out.yaml'),
        description='Full path to map file to load')

    # 实车默认使用系统时间；仅在仿真或 rosbag 使用 /clock 时设为 True。
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    # Nav2 全栈共用参数文件，navigation_launch.py 和 map_server 都从这里取参数。
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # 为 true 时 lifecycle manager 会自动把受管节点切换到 active 状态。
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # 是否使用组件容器承载 Nav2 节点；具体行为由 navigation_launch.py 实现。
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    # 非组件模式下，节点异常退出后是否由 launch 自动重新拉起。
    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    # ------------------------- 定位兼容参数 -------------------------
    # 这些初始位姿参数保留给上层或外部定位 launch 使用；本文件不启动 AMCL，
    # 也没有在本地节点中直接消费它们。
    declare_set_initial_pose_cmd = DeclareLaunchArgument(
        'set_initial_pose',
        default_value='True',
        description='Whether localization should seed its pose from the initial_pose parameters')

    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.569',
        description='Initial localization pose x coordinate in map frame')

    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.541',
        description='Initial localization pose y coordinate in map frame')

    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Initial localization pose yaw in map frame')

    # ------------------------- 可视化和仿真参数 -------------------------
    # RViz 配置文件。use_rviz=False 时即使提供该路径也不会启动 RViz。
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # Gazebo Classic 总开关。默认 False，符合实车入口的运行方式。
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='False',
        description='Whether to start the simulator')

    # 兼容保留参数；当前 robot_state_publisher 节点代码被注释，不会实际启动。
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='False',
        description='Whether to start the robot state publisher')

    # RViz 启动开关，默认开启，方便观察地图、代价地图、规划路径和机器人状态。
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # headless=True 时只启动 gzserver，不启动有界面的 gzclient。
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    # Gazebo Classic 世界文件，仅在 use_simulator=True 时由 gzserver 使用。
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'world_only.model'),
        description='Full path to world model file to load')

    # 以下 robot_name 和 robot_sdf 是历史兼容接口；当前文件没有启用机器人生成节点。
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='odom',
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    # 传给 map_server 等节点的 ROS 日志级别。
    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    # 可选启动 Gazebo Classic 服务端。两个 ROS 插件分别完成 ROS 初始化和模型生成服务。
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    # 仅当 use_simulator=True 且 headless=False 时启动 Gazebo 图形客户端。
    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression(
            [use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    # ------------------------- 已停用的实车辅助节点 -------------------------

    # laserpub 原计划生成或重整 LaserScan。当前实车雷达驱动直接提供扫描数据，
    # 因此不在入口中额外发布，避免与真实雷达话题冲突。
    # laserpub_cmd = Node(
    #     package='laserpub',
    #     executable='laserpub',
    #     name='laserpub',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'topic': '/c200_lidar_node1/scan'},
    #                 {'frame_id': 'base_link'},
    #                 {'angle_min': -3.141592653589793},
    #                 {'angle_max': 3.141592653589793},
    #                 {'range_min': 0.12},
    #                 {'range_max': 3.5},
    #                 {'default_range': 3.5},
    #                 {'sample_count': 360},
    #                 {'publish_rate': 10.0}])

    # map2base_tf 原计划直接发布 map->base_link。若定位链已经提供该变换，
    # 再启动此节点会形成 TF 冲突，所以当前保持停用。
    # map2base_tf_cmd = Node(
    #     package='map2base_tf',
    #     executable='map2baseTF',
    #     name='map2baseTF',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time},
    #                 {'parent_frame': 'map'},
    #                 {'child_frame': 'base_link'},
    #                 {'x': 0.569},
    #                 {'y': 0.541},
    #                 {'z': 0.0},
    #                 {'yaw': 0.0},
    #                 {'publish_rate': 30.0}])

    # 在 map->base_link 定位模式中，车辆定位系统直接发布 map->base_link。
    # 因此不能再静态发布 odom->base_link，否则 TF 树会出现重复或矛盾的父子关系。
    # static_robot_to_base_link_cmd = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_to_base_link_tf',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'])

    # robot_state_publisher 和 joint_state_publisher 依赖上方已停用的 URDF 读取逻辑。
    # 当前实车 TF 由外部系统负责，因此两者均不加入 LaunchDescription。
    # robot_state_publisher_cmd = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': robot_description}])

    # joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': robot_description}])

    # ------------------------- 当前实际启动的功能 -------------------------
    # 条件包含 RViz 子 launch；关闭 use_rviz 后不会创建 RViz 进程。
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    # 静态地图服务器发布 /map 和地图元数据。configured_params 已把 map 参数
    # 写入 yaml_filename，并统一注入 use_sim_time。
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)

    # 单独管理 map_server 的 lifecycle。autostart=True 时自动执行 configure 和 activate。
    # 节点名沿用 localization，是因为地图服务器通常属于 Nav2 定位侧受管节点。
    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server']}])

    # 启动 Nav2 导航主链，具体包含 planner、controller、behavior、BT navigator、
    # waypoint follower、velocity smoother 及其 lifecycle manager 等节点。
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    # 将 Nav2 最终输出的 /cmd_vel 转发为底盘串口桥接使用的 /control_to_uart。
    # 该节点只做控制接口衔接，不参与路径规划、局部控制或速度平滑。
    controlpub_cmd = Node(
        package='controlpub',
        executable='controlpub_node',
        name='controlpub',
        output='screen',
        parameters=[{'input_topic': '/cmd_vel'},
                    {'output_topic': '/control_to_uart'}])

    # ------------------------- 组装启动描述 -------------------------
    ld = LaunchDescription()

    # 先注册全部参数声明，使命令行覆盖值能够被后续 Action 正确解析。
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_set_initial_pose_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 仿真进程带有条件判断；默认实车配置下这两项都会跳过。
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    # ld.add_action(start_gazebo_spawner_cmd)

    # ld.add_action(laserpub_cmd)
    # ld.add_action(map2base_tf_cmd)
    # ld.add_action(static_robot_to_base_link_cmd)
    # ld.add_action(robot_state_publisher_cmd)
    # ld.add_action(joint_state_publisher_cmd)
    # ld.add_action(Node(
    #     package='myagv_test_bringup',
    #     executable='robot_description_publisher.py',
    #     name='robot_description_publisher',
    #     output='screen'))
    # ld.add_action(start_robot_state_publisher_cmd)
    # 实际功能节点的注册顺序：可视化、地图、地图 lifecycle、Nav2 主链、底盘转发。
    # launch 注册顺序不等于所有进程完全串行启动；生命周期依赖由对应 manager 协调。
    ld.add_action(rviz_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_map_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(controlpub_cmd)

    return ld
