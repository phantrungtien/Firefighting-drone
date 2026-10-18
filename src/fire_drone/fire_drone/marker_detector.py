"""Mã ArUco = đóm lửa. Thấy mã trong fire_ids -> vị trí ENU -> /fire/target.

Camera nadir /rgb/image (ros_gz_bridge) + pose MAVROS. Vị trí 3D lấy bằng
solvePnP với kích thước mã đã biết, nên KHÔNG cần giả định mặt đất phẳng.
Mã không thuộc fire_ids bị bỏ qua - sau này dùng id khác làm bãi đáp, v.v.

Zoom số (/fire/zoom, fire_mission ra lệnh) làm Ở ĐÂY như run_gui.py làm trên Jetson:
cắt giữa 1/zoom của khung gốc, co về out_size, rồi mới dò; K đổi theo. Khung đã cắt
phát lại ở /rgb/zoomed cho video GUI - người vận hành thấy đúng cái drone đang tìm.

    python3 marker_detector.py --selfcheck
"""
import sys

import cv2
import numpy as np
import rclpy
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


def marker_enu(corners, K, size, drone_pos, R_body, cam_off):
    """4 góc pixel -> tâm mã trong ENU. None nếu PnP không giải được."""
    ok, _, tvec = cv2.solvePnP(marker_obj_points(size), corners.reshape(4, 2).astype(np.float64),
                               K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
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
        self.zoom = 1.0
        self.detect = make_detector()
        self.K = None
        self.pose = None
        self.hits = {}  # id -> các ước lượng ENU gần nhất
        self.pub = self.create_publisher(PointStamped, '/fire/target', 10)
        self.zoomed_pub = self.create_publisher(Image, '/rgb/zoomed', qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32, '/fire/zoom', self.on_zoom, latched)
        self.create_subscription(Image, '/rgb/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/rgb/camera_info', self.on_info, qos_profile_sensor_data)
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
            e = marker_enu(c, K, self.size, pos, R, self.cam_off)
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
    print('marker_detector selfcheck OK')


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
