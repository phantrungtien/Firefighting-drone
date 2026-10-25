"""Cầu nối camera gz -> ROS + marker_detector + fire_mission.

Gazebo, SITL, MAVROS chạy ở terminal riêng (xem README) để khi hỏng còn đọc
được log của đúng cái hỏng.

Kiểm cầu nối:  ros2 topic hz /rgb/image   (phải ~15 Hz nhân RTF)
Chỉ phát hiện, không bay:  ros2 launch fire_drone fire_mission.launch.py mission:=false
Drone thật (Jetson):        ros2 launch fire_drone fire_mission.launch.py real:=true
  camera + YOLO lửa + ground_range nằm trong run_gui.py (mjpeg-stream.service), ở đây chỉ còn
  fire_locator (hộp lửa -> /fire/target) + fire_mission. Cùng RMW_IMPLEMENTATION/CYCLONEDDS_URI
  với service (jetson/cyclonedds_tailscale.xml), không thì không thấy topic của nhau.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    sim = UnlessCondition(LaunchConfiguration('real'))
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('real', default_value='false'),
        # Drone thật 23/10/2026: vòng tìm 2 m (sim 24 m) - nhỏ hơn bước vòng nên ở x1..x3 chỉ treo
        # ở điểm chấm rồi zoom dần; x4/x5 bay một vòng nhỏ 2 / 1.6 m (ở 20 m).
        DeclareLaunchArgument('search_radius', default_value=PythonExpression(
            ["2.0 if '", LaunchConfiguration('real'), "' == 'true' else 24.0"])),
        # Drone thật: x6 = vùng cắt 640 px của khung 4K, bằng ảnh vào YOLO -> vẫn điểm ảnh thật.
        # Đo 23/10/2026 (ảnh lửa điện thoại): x5 điểm 0.59-0.63, x6 0.77-0.78. Sim 1080p: x6 là phóng ảnh.
        DeclareLaunchArgument('max_zoom', default_value=PythonExpression(
            ["6.0 if '", LaunchConfiguration('real'), "' == 'true' else 5.0"])),
        # Thử tấm lửa A0 (print/, README): max_alt:=5 chặn alt GUI > 5 m, check_radius:=1.5 để
        # vòng kiểm tra ở 5 m còn thấy tấm (camera chỉ phủ ~7.7x4.3 m). float(): gõ "5" (int) thì node
        # khai báo double sẽ từ chối.
        DeclareLaunchArgument('max_alt', default_value='120.0'),
        DeclareLaunchArgument('check_radius', default_value='6.0'),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
            # tên topic khớp <topic>rgb/image</topic> của rgb_cam_link trong firehawk_x4
            arguments=['/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
                       '/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            output='screen', condition=sim,
        ),
        Node(package='fire_drone', executable='marker_detector', output='screen', condition=sim),
        # khoảng cách tới mặt đất bằng camera 2D (thay lidar). Drone thật: chạy trong run_gui.py
        Node(package='fire_drone', executable='ground_range', output='screen', condition=sim),
        # min_score 0.2 = --fire-conf cua run_gui: hop da qua Gemini (fire_verify.py, chi YOLO thi van
        # doi 0.35) nen khong loc lai o day. 24/10/2026: 0.35 lam rot hop lua 0.21-0.32 Gemini da xac nhan.
        Node(package='fire_drone', executable='fire_locator', output='screen',
             parameters=[{'min_score': 0.2}],
             condition=IfCondition(LaunchConfiguration('real'))),
        Node(package='fire_drone', executable='fire_mission', output='screen',
             parameters=[{'search_radius': LaunchConfiguration('search_radius'),
                          'max_zoom': LaunchConfiguration('max_zoom'),
                          'max_alt': PythonExpression(['float(', LaunchConfiguration('max_alt'), ')']),
                          'check_radius': PythonExpression(['float(', LaunchConfiguration('check_radius'), ')'])}],
             condition=IfCondition(LaunchConfiguration('mission'))),
    ])
