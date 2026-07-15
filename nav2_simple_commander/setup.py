from glob import glob
import os

from setuptools import setup


# Python package name installed into the ROS 2 environment.
# 中文注解：安装到 ROS 2 环境中的 Python 包名，需要与目录名保持一致。
package_name = 'nav2_simple_commander'
launch_files = [path for path in glob('launch/*') if os.path.isfile(path)]
param_files = [path for path in glob('params/*') if os.path.isfile(path)]
rviz_files = [path for path in glob('rviz/*') if os.path.isfile(path)]
urdf_files = [path for path in glob('urdf/*') if os.path.isfile(path)]
model_data_files = []
for root, _, files in os.walk('models'):
    model_files = [
        os.path.join(root, file_name)
        for file_name in files
        if os.path.isfile(os.path.join(root, file_name))
    ]
    if model_files:
        model_data_files.append(
            (os.path.join('share', package_name, root), model_files))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Register the package in the ament resource index.
        # 中文注解：把包注册到 ament 资源索引，ros2 pkg 才能发现它。
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml into share/<package>.
        # 中文注解：安装 package.xml，供 ROS 2 包管理和依赖查询使用。
        ('share/' + package_name, ['package.xml']),
        # Install launch and world files into share/<package>, excluding cache directories.
        # 中文注解：只安装 launch 目录下的普通文件，排除 __pycache__ 等缓存目录。
        (os.path.join('share', package_name), launch_files),
        (os.path.join('share', package_name, 'params'), param_files),
        (os.path.join('share', package_name, 'rviz'), rviz_files),
        (os.path.join('share', package_name, 'urdf'), urdf_files),
    ] + model_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stevenmacenski@gmail.com',
    description='An importable library for writing mobile robot applications in python3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                # Each console script maps a ROS executable name to a Python main().
                # 中文注解：每个 console script 把 ROS 可执行名映射到对应 Python main()。
                'example_nav_to_pose = nav2_simple_commander.example_nav_to_pose:main',
                'example_nav_through_poses = nav2_simple_commander.example_nav_through_poses:main',
                'example_waypoint_follower = nav2_simple_commander.example_waypoint_follower:main',
                'example_follow_path = nav2_simple_commander.example_follow_path:main',
                'demo_picking = nav2_simple_commander.demo_picking:main',
                'demo_inspection = nav2_simple_commander.demo_inspection:main',
                'demo_security = nav2_simple_commander.demo_security:main',
                'demo_recoveries = nav2_simple_commander.demo_recoveries:main',
                'example_assisted_teleop = nav2_simple_commander.example_assisted_teleop:main',
                'cpu_lidar = nav2_simple_commander.cpu_lidar:main',
        ],
    },
)
