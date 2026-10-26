"""Cầu nối camera gz -> ROS + marker_detector + fire_mission.

Gazebo, SITL, MAVROS chạy ở terminal riêng (xem README) để khi hỏng còn đọc
được log của đúng cái hỏng.

Kiểm cầu nối:  ros2 topic hz /rgb/image   (phải ~15 Hz nhân RTF)
Chỉ phát hiện, không bay:  ros2 launch fire_drone fire_mission.launch.py mission:=false
Drone thật (Jetson):        ros2 launch fire_drone fire_mission.launch.py real:=true
  camera + YOLO lửa + ground_range nằm trong run_gui.py (mjpeg-stream.service), ở đây chỉ còn
  fire_locator (hộp lửa -> /fire/target) + fire_mission. Cùng RMW_IMPLEMENTATION/CYCLONEDDS_URI
  với service (jetson/cyclonedds_tailscale.xml), không thì không thấy topic của nhau.
Sim bằng chuỗi của drone thật:  yolo:=true  (run_sim.sh ANH_A0=1) - run_gui.py của ~/jetson đọc
  /rgb/image (YOLO lửa + ground_range trong tiến trình), bỏ marker_detector/ground_range sim,
  fire_locator dùng calib camera sim.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = UnlessCondition(LaunchConfiguration('real'))
    aruco = IfCondition(PythonExpression(["'", LaunchConfiguration('real'), "' != 'true' and '",
                                          LaunchConfiguration('yolo'), "' != 'true'"]))
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('real', default_value='false'),
        DeclareLaunchArgument('yolo', default_value='false'),
        # Drone thật 23/10/2026: vòng tìm 2 m (sim 24 m) - nhỏ hơn bước vòng nên ở x1..x3 chỉ treo
        # ở điểm chấm rồi zoom dần; x4/x5 bay một vòng nhỏ 2 / 1.6 m (ở 20 m).
        # yolo:=true (sim bằng chuỗi drone thật) cũng 2 m: 26/10/2026 vòng 24 m ở 5 m đâm toà nhà 100 m.
        DeclareLaunchArgument('search_radius', default_value=PythonExpression(
            ["2.0 if 'true' in ('", LaunchConfiguration('real'), "', '", LaunchConfiguration('yolo'),
             "') else 24.0"])),
        # Vòng kiểm tra sau khi thả: drone thật / yolo cũng 2 m (user 26/10/2026: chỉ bay quanh 2 m;
        # sim 6 m ra xa tâm 5.6 m, cách toà nhà ~3 m ở độ cao 5 m).
        DeclareLaunchArgument('check_radius', default_value=PythonExpression(
            ["2.0 if 'true' in ('", LaunchConfiguration('real'), "', '", LaunchConfiguration('yolo'),
             "') else 6.0"])),
        # Drone thật: x6 = vùng cắt 640 px của khung 4K, bằng ảnh vào YOLO -> vẫn điểm ảnh thật.
        # Đo 23/10/2026 (ảnh lửa điện thoại): x5 điểm 0.59-0.63, x6 0.77-0.78. Sim 1080p: x6 là phóng ảnh.
        DeclareLaunchArgument('max_zoom', default_value=PythonExpression(
            ["6.0 if '", LaunchConfiguration('real'), "' == 'true' else 5.0"])),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
            # tên topic khớp <topic>rgb/image</topic> của rgb_cam_link trong firehawk_x4
            arguments=['/rgb/image@sensor_msgs/msg/Image[gz.msgs.Image',
                       '/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            output='screen', condition=sim,
        ),
        Node(package='fire_drone', executable='marker_detector', output='screen', condition=aruco),
        # khoảng cách tới mặt đất bằng camera 2D (thay lidar). Drone thật: chạy trong run_gui.py
        Node(package='fire_drone', executable='ground_range', output='screen', condition=aruco),
        # min_score 0.2 = --fire-conf cua run_gui: hop da qua Gemini (fire_verify.py, chi YOLO thi van
        # doi 0.35) nen khong loc lai o day. 24/10/2026: 0.35 lam rot hop lua 0.21-0.32 Gemini da xac nhan.
        Node(package='fire_drone', executable='fire_locator', output='screen',
             parameters=[{'min_score': 0.2}],
             condition=IfCondition(LaunchConfiguration('real'))),
        Node(package='fire_drone', executable='fire_locator', output='screen',
             parameters=[{'min_score': 0.2, 'calib': PathJoinSubstitution(
                 [FindPackageShare('fire_drone'), 'config', 'sim_cam_1920x1080.yaml'])}],
             condition=IfCondition(LaunchConfiguration('yolo'))),
        Node(package='fire_drone', executable='fire_mission', output='screen',
             parameters=[{'search_radius': LaunchConfiguration('search_radius'),
                          'check_radius': LaunchConfiguration('check_radius'),
                          'max_zoom': LaunchConfiguration('max_zoom')}],
             condition=IfCondition(LaunchConfiguration('mission'))),
    ])
