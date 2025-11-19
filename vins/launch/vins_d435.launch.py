#!/usr/bin/env python3
# ============================================================================
# VINS-Fusion ROS2 Launch File - RealSense D435
# ============================================================================
# 硬件: Intel RealSense D435 (双目红外相机) + PX4 ICM-42688-P IMU
# 配置: S 级标定参数 (2025-11-17)
# 用途: 启动 VINS-Fusion 视觉惯性里程计节点
# ============================================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    生成 VINS-Fusion RealSense D435 的 launch 描述
    """

    # 配置文件绝对路径 (在源码目录，不在安装目录)
    config_file = '/home/fsuav/fast-drone-250-humble-6.1/src/realflight_modules/VINS-Fusion-ROS2-humble/config/realsense_d435/realsense_stereo_imu_config.yaml'

    # 声明 launch 参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='VINS 配置文件的完整路径'
    )

    # VINS 估计器节点
    vins_node = Node(
        package='vins_fusion_ros2_humble',
        executable='vins_fusion_ros2_humble_node',
        namespace='vins_estimator',
        name='vins_fusion_ros2_humble_node',
        output='screen',
        emulate_tty=True,
        arguments=[LaunchConfiguration('config_file')],
        parameters=[{
            'use_sim_time': False
        }],
        # remappings=[
        #     # Topic 重映射格式: (节点内部订阅的topic, 外部实际发布的topic)
        #     # VINS配置文件中订阅 /camera/imu，数据包中发布 /vins/imu
        #     ('/camera/imu', '/vins/imu'),
        #     # VINS配置文件中订阅 /camera/infra1/image_rect_raw，数据包中发布 /camera/camera/infra1/image_rect_raw
        #     ('/camera/infra1/image_rect_raw', '/camera/camera/infra1/image_rect_raw'),
        #     # VINS配置文件中订阅 /camera/infra2/image_rect_raw，数据包中发布 /camera/camera/infra2/image_rect_raw
        #     ('/camera/infra2/image_rect_raw', '/camera/camera/infra2/image_rect_raw'),
        # ]
    )

    return LaunchDescription([
        config_file_arg,
        vins_node
    ])
