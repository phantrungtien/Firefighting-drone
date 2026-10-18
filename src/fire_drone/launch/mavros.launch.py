"""MAVROS cho ArduPilot = `mavros apm.launch` + 2 sửa lỗi đã gặp thật.

1. config/mavros_pluginlists.yaml cấm plugin param/waypoint/...: sim chạy chậm
   làm chúng fulfil một std::promise hai lần -> MAVROS abort() giữa chuyến bay.
2. config/mavros_tf.yaml bật TF map->base_link (apm_config tắt sẵn).
Chi tiết lý do nằm trong chính hai file đó.

  SITL :  ros2 launch fire_drone mavros.launch.py fcu_url:=tcp://127.0.0.1:5760
  Thật :  ros2 launch fire_drone mavros.launch.py fcu_url:=/dev/ttyAMA0:921600
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('fire_drone'), 'config')
    apm_config = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value='tcp://127.0.0.1:5760'),
        # GCS (Mission Planner/QGC) nối ké qua MAVROS. Rỗng = không bridge.
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='mavros',
            executable='mavros_node',
            namespace='mavros',
            output='screen',
            parameters=[
                os.path.join(cfg, 'mavros_pluginlists.yaml'),
                apm_config,
                os.path.join(cfg, 'mavros_tf.yaml'),  # phải SAU apm_config
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                    'tgt_system': 1,
                    'tgt_component': 1,
                    'fcu_protocol': 'v2.0',
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
