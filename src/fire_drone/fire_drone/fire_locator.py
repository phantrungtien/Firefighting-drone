"""Hộp lửa YOLO (run_gui.py trên Jetson) -> vị trí ENU -> /fire/target. Drone THẬT thay marker_detector.

    run_gui.py --fire-model ... -> /fire/detections (vision_msgs/Detection2DArray, tâm hộp tính
    theo điểm ảnh của KHUNG GỐC 4K đã calib - run_gui đổi ngược phép cắt zoom trước khi gửi)

Lửa thật không có cỡ chuẩn như mã ArUco nên không PnP được: chiếu tia từ camera qua tâm hộp,
cắt mặt phẳng z = mặt đất quanh đám cháy. z đó lấy từ /fire/ground_z (ground_range, chỉ có
lúc hạ để thả); trước đó coi ngang chỗ cất cánh (z = 0). Sai z chỉ làm lệch NGANG theo độ lệch
tia khỏi phương thẳng đứng: lửa trên mái 30 m thấy ở mép ảnh (lệch trục 30°) là lệch ~17 m,
nhưng fire_mission bay tới ngay trên rồi mới hạ - tia gần thẳng đứng, sai còn ~0. Tự hội tụ.

Nhiều đám cháy: mỗi khung chỉ lấy hộp điểm cao nhất, trung vị nuốt phần lẻ (CHECKLIST 3.5).

    python3 fire_locator.py --selfcheck
"""
import bisect
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, Float32
from vision_msgs.msg import Detection2DArray

from fire_drone.fire_mission import ground_filter
from fire_drone.marker_detector import R_OPT_TO_BODY, load_calib, quat_to_rot


def pixel_ray(u, v, K, D):
    """Điểm ảnh (khung calib) -> tia đơn vị trong hệ quang (x phải, y xuống, z ra trước)."""
    x, y = cv2.undistortPoints(np.array([[[u, v]]], np.float64), K, D).ravel()
    r = np.array([x, y, 1.0])
    return r / np.linalg.norm(r)


def ray_ground(ray_opt, drone_pos, R_body, cam_off, ground_z):
    """Tia hệ quang -> điểm cắt mặt z = ground_z trong ENU. None nếu tia không chúc xuống."""
    cam = drone_pos + R_body @ cam_off
    d = R_body @ (R_OPT_TO_BODY @ ray_opt)
    if d[2] > -0.05:  # nằm ngang trên 87° lệch trục: chạm đất ở vô cực, số vô nghĩa
        return None
    t = (ground_z - cam[2]) / d[2]
    return cam + t * d if t > 0 else None


def pose_at(hist, t, tol=0.5):
    """Lịch sử [(giờ, pose)] tăng dần -> pose gần giờ t nhất trong tol s; None nếu không có."""
    i = bisect.bisect_left(hist, t, key=lambda h: h[0])
    near = [h for h in hist[max(i - 1, 0):i + 1] if abs(h[0] - t) <= tol]
    return min(near, key=lambda h: abs(h[0] - t))[1] if near else None


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class FireLocator(Node):
    def __init__(self):
        super().__init__('fire_locator')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        # Mặc định = rgb_cam_link của firehawk_x4 (sim). Drone thật: số đo 27/10/2026 đặt trong launch real:=true.
        self.cam_off = np.array(p('camera_offset', [0.26, 0.0, -0.18]))
        self.confirm = p('confirm_frames', 5)       # YOLO 5 Hz: 1 s
        self.window = p('window', 20)
        self.min_score = p('min_score', 0.35)
        path = p('calib', '') or os.path.join(get_package_share_directory('fire_drone'),
                                              'config', 'drone_cam_3840x2160.yaml')
        self.K, self.D, *_ = load_calib(path)
        self.pose = None
        # ~20 s pose gần nhất: hộp Gemini (run_gui --fire-scan, YOLO sót) đến trễ 1-7 s - tia phải đi từ
        # chỗ drone LÚC CHỤP khung (stamp của Detection2DArray), không phải chỗ drone bây giờ.
        self.hist = []
        self.ground = []
        self.hits = []
        self.pub = self.create_publisher(PointStamped, '/fire/target', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/fire/ground_enable', self.on_enable, latched)
        self.create_subscription(Float32, '/fire/ground_z', lambda m: self.ground.append(m.data), 10)
        self.create_subscription(Detection2DArray, '/fire/detections', self.on_dets, 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos_profile_sensor_data)
        self.get_logger().info(f'lua YOLO -> /fire/target, K tu {path}')

    def on_pose(self, msg):
        self.pose = msg.pose
        self.hist.append((stamp_s(msg.header.stamp), msg.pose))
        if len(self.hist) > 1200:
            del self.hist[:200]

    def on_enable(self, msg):
        if msg.data:
            self.ground = []  # đợt hạ mới, số đo đợt trước (chỗ khác) không dùng

    def on_dets(self, msg):
        best = max(msg.detections, default=None,
                   key=lambda d: d.results[0].hypothesis.score if d.results else 0.0)
        if best is None or not best.results or best.results[0].hypothesis.score < self.min_score:
            return
        t = stamp_s(msg.header.stamp)
        pose = pose_at(self.hist, t)
        if pose is None and time.time() - t < 0.5:  # lịch sử lệch đồng hồ: khung mới thì pose mới nhất vẫn đúng
            pose = self.pose
        if pose is None:  # khung cũ mà không có pose lúc chụp: tia từ chỗ khác = lửa sai chỗ, bỏ
            return
        p, q = pose.position, pose.orientation
        c = best.bbox.center.position
        gz = ground_filter(self.ground)
        e = ray_ground(pixel_ray(c.x, c.y, self.K, self.D), np.array([p.x, p.y, p.z]),
                       quat_to_rot(q.x, q.y, q.z, q.w), self.cam_off, gz)
        if e is None:
            return
        self.hits.append(e)
        del self.hits[:-self.window]
        if len(self.hits) == self.confirm:
            self.get_logger().info(f'PHAT HIEN lua tai ({e[0]:.1f}, {e[1]:.1f}), mat dat z {gz:.1f}')
        if len(self.hits) >= self.confirm:
            m = np.median(self.hits, axis=0)
            out = PointStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'map'
            out.point.x, out.point.y, out.point.z = map(float, m)
            self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(FireLocator())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def _selfcheck():
    cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
    K, D, w, h = load_calib(os.path.join(cfg, 'drone_cam_3840x2160.yaml'))
    cam_off = np.array([0.26, 0.0, -0.18])

    def project(target, drone, R):
        """Điểm ENU -> điểm ảnh 4K qua méo thật, đúng chiều run_gui sẽ gửi."""
        opt = R_OPT_TO_BODY.T @ (R.T @ (target - drone - R @ cam_off))
        assert opt[2] > 0
        return cv2.projectPoints(opt[None], np.zeros(3), np.zeros(3), K, D)[0].ravel()

    for yaw, roll in [(0.0, 0.0), (0.7, 0.08), (-2.0, -0.05)]:
        cr, sr, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(yaw / 2), np.sin(yaw / 2)
        R = quat_to_rot(sr * cy, sr * sy, cr * sy, cr * cy)
        drone = np.array([10.0, -4.0, 40.0])
        for off in (np.zeros(3), np.array([12.0, 6.0, 0.0])):   # giữa ảnh và gần mép (méo lớn)
            for gz in (0.0, 4.0):                                 # đất phẳng / mái nhà 4 m
                tgt = np.array([10.0, -4.0, gz]) + off
                u, v = project(tgt, drone, R)
                assert 0 <= u < w and 0 <= v < h, (u, v)
                e = ray_ground(pixel_ray(u, v, K, D), drone, R, cam_off, gz)
                assert np.linalg.norm(e - tgt) < 0.05, (yaw, off, gz, e, tgt)
    # mặt đất đoán sai (lửa trên mái 30 m, coi là z = 0): ngay dưới thì gần đúng, xa trục thì lệch
    R = np.eye(3)
    drone = np.array([0.0, 0.0, 60.0])
    for off, ok in ((np.array([0.3, 0.0, 0.0]), True), (np.array([15.0, 0.0, 0.0]), False)):
        tgt = np.array([0.0, 0.0, 30.0]) + off
        e = ray_ground(pixel_ray(*project(tgt, drone, R), K, D), drone, R, cam_off, 0.0)
        err = np.linalg.norm(e[:2] - tgt[:2])
        assert (err < 0.5) if ok else (err > 5.0), (off, err)  # mép ảnh lệch ~15 m: phải bay tới trên
    assert ray_ground(np.array([0.0, -1.0, 0.0]), drone, R, cam_off, 0.0) is None, 'tia ngang trời'
    hist = [(100.0 + i * 0.1, i) for i in range(50)]                         # 10 Hz, 100.0 .. 104.9
    assert pose_at(hist, 102.03) == 20 and pose_at(hist, 102.07) == 21, 'lấy pose gần giờ chụp nhất'
    assert pose_at(hist, 99.6) == 0 and pose_at(hist, 99.4) is None, 'ngoài lịch sử quá 0.5 s: không đoán'
    assert pose_at(hist, 105.3) == 49 and pose_at([], 1.0) is None
    print('fire_locator selfcheck OK')


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
