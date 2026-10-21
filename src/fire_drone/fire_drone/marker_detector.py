"""Mã ArUco = đóm lửa. Thấy mã trong fire_ids -> vị trí ENU -> /fire/target.

Camera nadir /rgb/image (ros_gz_bridge) + pose MAVROS. Vị trí 3D lấy bằng
solvePnP với kích thước mã đã biết, nên KHÔNG cần giả định mặt đất phẳng.
Mã không thuộc fire_ids bị bỏ qua - sau này dùng id khác làm bãi đáp, v.v.

K: sim lấy /rgb/camera_info (Gazebo, không méo). Camera thật không có topic đó -> tham số
`calib` = file camera_info của webcam (config/drone_cam_3840x2160.yaml, calib ChArUco
21/10/2026 bằng scripts/calib_charuco.py): K co giãn theo chiều rộng khung tới (run_gui co
cả khung 4K, không cắt), méo giữ nguyên và đưa vào solvePnP.

Zoom số (/fire/zoom, fire_mission ra lệnh) làm Ở ĐÂY như run_gui.py làm trên Jetson:
cắt giữa 1/zoom của khung gốc, co về out_size, rồi mới dò; K đổi theo. Khung đã cắt
phát lại ở /rgb/zoomed cho video GUI - người vận hành thấy đúng cái drone đang tìm.

    python3 marker_detector.py --selfcheck
"""
import os
import sys

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

# Hệ quang (x phải, y xuống, z ra trước) -> hệ thân, cho camera gắn pitch +90°
# (nadir) như rgb_cam_link của firehawk_x4: thân = (-y_q, -x_q, -z_q).
# Chép từ firedrop_sim fire_detector (đã bay đúng). Sai dấu ở đây = mục tiêu bị
# định vị đối xứng qua drone: đúng khoảng cách, ngược hướng.
R_OPT_TO_BODY = np.array([[0.0, -1.0, 0.0],
                          [-1.0, 0.0, 0.0],
                          [0.0, 0.0, -1.0]])


def quat_to_rot(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def marker_obj_points(size):
    """4 góc mã trong hệ của mã, đúng thứ tự ArUco/IPPE_SQUARE trả về."""
    h = size / 2.0
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)


def marker_enu(corners, K, size, drone_pos, R_body, cam_off, D=None):
    """4 góc pixel -> tâm mã trong ENU. None nếu PnP không giải được."""
    ok, _, tvec = cv2.solvePnP(marker_obj_points(size), corners.reshape(4, 2).astype(np.float64),
                               K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return drone_pos + R_body @ (cam_off + R_OPT_TO_BODY @ tvec.ravel())


def zoom_k(K, x0, y0, sx, sy):
    """K của ảnh cắt từ (x0, y0) rồi co giãn sx, sy (cv2.resize: tâm điểm ảnh lệch 0.5)."""
    K2 = K.copy()
    K2[0, 0], K2[1, 1] = K[0, 0] * sx, K[1, 1] * sy
    K2[0, 2] = (K[0, 2] - x0 + 0.5) * sx - 0.5
    K2[1, 2] = (K[1, 2] - y0 + 0.5) * sy - 0.5
    return K2


def load_calib(path):
    """File camera_info ROS -> (K, D, rộng, cao) lúc calib."""
    c = yaml.safe_load(open(path))
    K = np.array(c['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(c['distortion_coefficients']['data'], dtype=np.float64)
    return K, D, c['image_width'], c['image_height']


def calib_for(calib, w, h):
    """K cho khung w x h lấy từ calib ở độ phân giải khác: co giãn CẢ khung (không cắt).
    Khác tỉ lệ khung thì là đã cắt - K co giãn sai, phải calib lại."""
    K, D, cw, ch = calib
    if abs(w / h - cw / ch) > 1e-3:
        raise ValueError(f'khung {w}x{h} khac ti le calib {cw}x{ch}')
    return zoom_k(K, 0, 0, w / cw, h / ch), D


def zoom_crop(w, h, zoom, out_w, out_h):
    """Vùng cắt giữa 1/zoom của khung w x h -> (x0, y0, cw, ch, sx, sy) để co về out_w x out_h."""
    cw, ch = round(w / zoom), round(h / zoom)
    return (w - cw) // 2, (h - ch) // 2, cw, ch, out_w / cw, out_h / ch


def make_detector():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):  # OpenCV >= 4.7
        return cv2.aruco.ArucoDetector(d).detectMarkers
    p = cv2.aruco.DetectorParameters_create()  # 4.5 (apt của Humble, có thể là Pi)
    return lambda img: cv2.aruco.detectMarkers(img, d, parameters=p)


class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.fire_ids = set(p('fire_ids', [0]))
        self.size = p('marker_size', 1.5)        # cạnh vùng mã ĐEN (m), khớp model fire_marker
        # rgb_cam_link so với base_link trong firehawk_x4/model.sdf
        self.cam_off = np.array(p('camera_offset', [0.26, 0.0, -0.18]))
        self.confirm = p('confirm_frames', 5)
        self.window = p('window', 20)
        # khung ra = --width/--height của run_gui trên Jetson (luồng H.264 về GUI)
        self.out_w, self.out_h = p('out_size', [960, 540])
        path = p('calib', '')  # rỗng = sim, K từ /rgb/camera_info
        self.calib = load_calib(path) if path else None
        self.zoom = 1.0
        self.detect = make_detector()
        self.K, self.D = None, None
        self.pose = None
        self.hits = {}  # id -> các ước lượng ENU gần nhất
        self.pub = self.create_publisher(PointStamped, '/fire/target', 10)
        self.zoomed_pub = self.create_publisher(Image, '/rgb/zoomed', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32, '/fire/zoom', self.on_zoom, latched)
        self.create_subscription(Image, '/rgb/image', self.on_image, qos_profile_sensor_data)
        if self.calib is None:
            self.create_subscription(CameraInfo, '/rgb/camera_info', self.on_info,
                                     qos_profile_sensor_data)
        else:
            self.get_logger().info(f'K tu file calib {path}')
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos_profile_sensor_data)
        self.get_logger().info(f'dom lua = ArUco 4x4_50 id {sorted(self.fire_ids)}, canh {self.size} m')

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def on_pose(self, msg):
        self.pose = msg.pose

    def on_zoom(self, msg):
        z = max(1.0, float(msg.data))
        if z != self.zoom:
            self.get_logger().info(f'zoom x{z:g}')
        self.zoom = z

    def on_image(self, msg):
        if self.calib is not None and self.K is None:
            self.K, self.D = calib_for(self.calib, msg.width, msg.height)
        if self.K is None or self.pose is None:
            return
        # ponytail: tự giải mã rgb8 thay cv_bridge - một reshape, bớt một phụ thuộc.
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        x0, y0, cw, ch, sx, sy = zoom_crop(msg.width, msg.height, self.zoom, self.out_w, self.out_h)
        rgb = cv2.resize(img[y0:y0 + ch, x0:x0 + cw], (self.out_w, self.out_h),
                         interpolation=cv2.INTER_AREA if sx < 1 else cv2.INTER_LINEAR)
        K = zoom_k(self.K, x0, y0, sx, sy)
        out = Image(header=msg.header, height=self.out_h, width=self.out_w, encoding='rgb8',
                    step=self.out_w * 3, data=rgb.tobytes())
        self.zoomed_pub.publish(out)
        corners, ids, _ = self.detect(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        if ids is None:
            return
        # ponytail: pose mới nhất, không đồng bộ theo stamp. Lúc bay nhanh lệch
        # ~v*trễ; lúc treo trên mục tiêu thì ~0 và median bên dưới nuốt phần cũ.
        p, q = self.pose.position, self.pose.orientation
        pos = np.array([p.x, p.y, p.z])
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        for c, i in zip(corners, ids.ravel()):
            if int(i) not in self.fire_ids:
                continue
            e = marker_enu(c, K, self.size, pos, R, self.cam_off, self.D)
            if e is None:
                continue
            h = self.hits.setdefault(int(i), [])
            h.append(e)
            del h[:-self.window]
            if len(h) == self.confirm:
                self.get_logger().info(f'PHAT HIEN dom lua id {i} tai ({e[0]:.2f}, {e[1]:.2f}, {e[2]:.2f})')
            if len(h) >= self.confirm:
                m = np.median(h, axis=0)
                out = PointStamped()
                out.header.stamp = msg.header.stamp
                out.header.frame_id = 'map'
                out.point.x, out.point.y, out.point.z = map(float, m)
                self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(MarkerDetector())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch tắt: không phải lỗi


def _selfcheck():
    K = np.array([[554.0, 0, 320.0], [0, 554.0, 240.0], [0, 0, 1]])
    size, cam_off = 1.5, np.array([0.26, 0.0, -0.18])
    target = np.array([18.0, 12.0, 0.0])
    yaw = 0.7
    for drone, roll in [(np.array([16.0, 13.0, 20.0]), 0.0), (np.array([19.0, 11.0, 8.0]), 0.08)]:
        # quaternion yaw + roll nhẹ (nghiêng như lúc bay)
        cr, sr, cy, sy = np.cos(roll / 2), np.sin(roll / 2), np.cos(yaw / 2), np.sin(yaw / 2)
        R = quat_to_rot(sr * cy, sr * sy, cr * sy, cr * cy)
        # chiếu ngược 4 góc (mã nằm ngang, trục mã = trục ENU) vào ảnh
        cam = drone + R @ cam_off
        px = []
        for o in marker_obj_points(size):
            opt = R_OPT_TO_BODY.T @ (R.T @ (target + o - cam))
            assert opt[2] > 0, 'góc mã phải nằm trước ống kính'
            px.append((K @ opt / opt[2])[:2])
        e = marker_enu(np.array(px), K, size, drone, R, cam_off)
        assert np.linalg.norm(e - target) < 0.02, (e, target)

        # Zoom: khung gốc 1920x1080 cắt giữa rồi co về 960x540. Góc mã ở toạ độ ảnh ra
        # + K đã đổi phải cho lại ĐÚNG vị trí - lệch tâm/tỉ lệ là mục tiêu trôi theo zoom.
        Kn = np.array([[1371.0, 0, 959.5], [0, 1371.0, 539.5], [0, 0, 1]])
        for zoom in (1.0, 2.0, 3.0, 5.0):
            x0, y0, cw, ch, sx, sy = zoom_crop(1920, 1080, zoom, 960, 540)
            pn = []
            for o in marker_obj_points(size):
                opt = R_OPT_TO_BODY.T @ (R.T @ (target + o - cam))
                u, v = (Kn @ opt / opt[2])[:2]
                pn.append(((u - x0 + 0.5) * sx - 0.5, (v - y0 + 0.5) * sy - 0.5))
            e = marker_enu(np.array(pn), zoom_k(Kn, x0, y0, sx, sy), size, drone, R, cam_off)
            assert np.linalg.norm(e - target) < 0.02, (zoom, e, target)
        assert zoom_crop(1920, 1080, 2.0, 960, 540) == (480, 270, 960, 540, 1.0, 1.0), \
            'x2 từ 1080p = 960x540 điểm ảnh THẬT, không co giãn'

    # Calib thật của webcam: file 4K co về 1280x720 phải trùng file 720p mà calib_charuco.py
    # ghi; góc mã chiếu QUA méo (projectPoints) + D trong PnP phải ra lại đúng vị trí.
    cfg = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
    cal = load_calib(os.path.join(cfg, 'drone_cam_3840x2160.yaml'))
    K720, D = calib_for(cal, 1280, 720)
    K720_file = load_calib(os.path.join(cfg, 'drone_cam_1280x720.yaml'))[0]
    assert np.allclose(K720, K720_file, atol=1e-3), (K720, K720_file)
    try:
        calib_for(cal, 1280, 960)
        raise AssertionError('khung 4:3 phải bị từ chối')
    except ValueError:
        pass
    drone, R = np.array([16.0, 13.0, 20.0]), quat_to_rot(0, 0, np.sin(0.35), np.cos(0.35))
    cam = drone + R @ cam_off
    for off in (np.zeros(3), np.array([7.0, -3.0, 0.0])):  # giữa ảnh và gần mép (méo lớn nhất)
        obj = np.array([R_OPT_TO_BODY.T @ (R.T @ (target + off + o - cam))
                        for o in marker_obj_points(size)])
        px = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), K720, D)[0].reshape(4, 2)
        assert ((px >= 0) & (px < [1280, 720])).all(), 'mã phải nằm trong ảnh'
        # 20 m: có D lệch 0,6 / 2,2 cm (sai số lặp khử méo của OpenCV), bỏ D lệch 4 / 21 cm
        e = marker_enu(px, K720, size, drone, R, cam_off, D)
        assert np.linalg.norm(e - target - off) < 0.05, (off, e)
        if off.any():
            e0 = marker_enu(px, K720, size, drone, R, cam_off)
            assert np.linalg.norm(e0 - target - off) > 0.1, 'bỏ méo thì ở mép ảnh phải lệch rõ'
    print('marker_detector selfcheck OK')


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
