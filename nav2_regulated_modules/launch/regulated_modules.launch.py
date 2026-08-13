import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_regulated_modules')
    launch_dir = os.path.dirname(__file__)

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    operation_mode = LaunchConfiguration('operation_mode')
    fixed_path_progress_timeout = LaunchConfiguration(
        'fixed_path_progress_timeout')
    default_keyboard_input_device = (
        os.ttyname(sys.stdin.fileno()) if sys.stdin.isatty() else '/dev/tty')
    keyboard_input_device = LaunchConfiguration('keyboard_input_device')
    is_remote = PythonExpression(["'", operation_mode, "' == 'remote'"])
    is_navigation = PythonExpression(["'", operation_mode, "' != 'remote'"])
    smoothed_cmd_vel_topic = 'cmd_vel'
    # collision_monitor_output_topic = PythonExpression([
    #     "'cmd_vel' if '", operation_mode,
    #     "' == 'fixed_path' else 'cmd_vel_collision_unused'"])
    effective_progress_timeout = ParameterValue(
        PythonExpression([
            fixed_path_progress_timeout, " if '", operation_mode,
            "' == 'fixed_path' else 10.0"]),
        value_type=float)

    lifecycle_nodes = [
        'planner_server',
        'controller_server',
        'smoother_server',
        'velocity_smoother',
        # 'collision_monitor',
        'regulated_navigator',
    ]

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'yaml_filename': map_yaml_file,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    spdlog_log_dir_envvar = SetEnvironmentVariable(
        'SPDLOG_WRAPPER_LOG_DIR', '/tmp/nav2_logs')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='False',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'out.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'regulated_modules.yaml'),
        description='Full path to the ROS 2 parameters file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        # default_value=os.path.join(bringup_dir, 'rviz', 'myrviz2.rviz'),
        description='Full path to the RViz config file')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup lifecycle nodes')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if true')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_regulated_container',
        description='Container name used when composition is enabled')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Respawn nodes if they crash when composition is disabled')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level')

    declare_operation_mode_cmd = DeclareLaunchArgument(
        'operation_mode',
        default_value='autonomous',
        choices=['remote', 'autonomous', 'fixed_path'],
        description='Robot operation mode')

    declare_fixed_path_progress_timeout_cmd = DeclareLaunchArgument(
        'fixed_path_progress_timeout',
        default_value='120.0',
        description='Maximum stationary time allowed in fixed_path mode')

    declare_keyboard_input_device_cmd = DeclareLaunchArgument(
        'keyboard_input_device',
        default_value=default_keyboard_input_device,
        description='Terminal device used by remote keyboard control')

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

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

    lifecycle_manager_map_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['map_server']}])

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {'progress_checker.movement_time_allowance':
                     effective_progress_timeout}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', smoothed_cmd_vel_topic)]),
            # Node(
            #     package='nav2_collision_monitor',
            #     executable='collision_monitor',
            #     name='collision_monitor',
            #     output='screen',
            #     respawn=use_respawn,
            #     respawn_delay=2.0,
            #     parameters=[
            #         configured_params,
            #         {
            #             'cmd_vel_in_topic': 'cmd_vel_collision_in',
            #             'cmd_vel_out_topic': ParameterValue(
            #                 collision_monitor_output_topic,
            #                 value_type=str),
            #         },
            #     ],
            #     arguments=['--ros-args', '--log-level', log_level],
            #     remappings=remappings),
            Node(
                package='nav2_regulated_modules',
                executable='regulated_navigator_node',
                name='regulated_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {
                        'operation_mode': operation_mode,
                        'progress_timeout': effective_progress_timeout,
                    },
                ],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='controlpub',
                executable='controlpub_node',
                name='controlpub',
                output='screen',
                parameters=[{'input_topic': '/cmd_vel'},
                            {'output_topic': '/control_to_uart'}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_regulated_modules',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ])

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[
                    configured_params,
                    {'progress_checker.movement_time_allowance':
                     effective_progress_timeout}],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                [('cmd_vel', 'cmd_vel_nav'),
                 ('cmd_vel_smoothed', smoothed_cmd_vel_topic)]),
            # ComposableNode(
            #     package='nav2_collision_monitor',
            #     plugin='nav2_collision_monitor::CollisionMonitor',
            #     name='collision_monitor',
            #     parameters=[
            #         configured_params,
            #         {
            #             'cmd_vel_in_topic': 'cmd_vel_collision_in',
            #             'cmd_vel_out_topic': ParameterValue(
            #                 collision_monitor_output_topic,
            #                 value_type=str),
            #         },
            #     ],
            #     remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_regulated_modules',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ])

    start_regulated_navigator_cmd = Node(
        condition=IfCondition(use_composition),
        package='nav2_regulated_modules',
        executable='regulated_navigator_node',
        name='regulated_navigator',
        output='screen',
        parameters=[
            configured_params,
            {
                'operation_mode': operation_mode,
                'progress_timeout': effective_progress_timeout,
            },
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings)

    start_controlpub_cmd = Node(
        condition=IfCondition(use_composition),
        package='controlpub',
        executable='controlpub_node',
        name='controlpub',
        output='screen',
        parameters=[{'input_topic': '/cmd_vel'},
                    {'output_topic': '/control_to_uart'}])

    # collision_boundary_visualizer_cmd = Node(
    #     package='nav2_regulated_modules',
    #     executable='collision_boundary_visualizer_node',
    #     name='collision_boundary_visualizer',
    #     output='screen')

    remote_control_cmd = Node(
        condition=IfCondition(is_remote),
        package='myagv_keyboard_control',
        executable='myagv_keyboard_control_node',
        name='myagv_keyboard_control',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params, {'input_device': keyboard_input_device,
                     'output_topic': '/control_to_uart'}])

    navigation_group = GroupAction(
        condition=IfCondition(is_navigation),
        actions=[
            rviz_cmd,
            map_server_cmd,
            lifecycle_manager_map_cmd,
            load_nodes,
            load_composable_nodes,
            start_regulated_navigator_cmd,
            start_controlpub_cmd,
            # collision_boundary_visualizer_cmd,
        ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(spdlog_log_dir_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_operation_mode_cmd)
    ld.add_action(declare_fixed_path_progress_timeout_cmd)
    ld.add_action(declare_keyboard_input_device_cmd)
    ld.add_action(remote_control_cmd)
    ld.add_action(navigation_group)

    return ld
