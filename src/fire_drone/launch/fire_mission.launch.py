"""Cầu nối camera gz -> ROS + marker_detector + fire_mission.

Gazebo, SITL, MAVROS chạy ở terminal riêng (xem README) để khi hỏng còn đọc
được log của đúng cái hỏng.

Kiểm cầu nối:  ros2 topic hz /rgb/image   (phải ~15 Hz nhân RTF)
Chỉ phát hiện, không bay:  ros2 launch fire_drone fire_mission.launch.py mission:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='true'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
            # tên topic khớp <topic>rgb/image</topic> của rgb_cam_link trong firehawk_x4
            arguments=['/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
                       '/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            output='screen',
        ),
        Node(package='fire_drone', executable='marker_detector', output='screen'),
        # khoảng cách tới mặt đất bằng camera 2D (thay lidar) - chạy được cả trên drone thật
        Node(package='fire_drone', executable='ground_range', output='screen'),
        Node(package='fire_drone', executable='fire_mission', output='screen',
             condition=IfCondition(LaunchConfiguration('mission'))),
    ])
