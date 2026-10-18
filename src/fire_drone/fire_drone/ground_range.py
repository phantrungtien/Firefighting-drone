"""Khoảng cách tới MẶT ĐẤT bằng camera 2D, không lidar: độ "nở" của ảnh khi hạ.

Hạ thẳng đứng một đoạn dh (EKF/khí áp kế đo chênh lệch tương đối rất chuẩn), các điểm trên
một mặt phẳng nở ra trên ảnh đúng tỉ lệ s = D_trước / D_sau, nên khoảng cách lúc sau
D = dh / (s - 1). Càng gần càng chuẩn - đúng lúc fire_mission cần dừng ở drop_alt.

Chỉ lấy điểm trong VÒNG quanh đám cháy: bỏ đĩa giữa (lửa/khói tự chuyển động, không phải
mặt cứng) và bỏ phần xa (nhà cao bên cạnh không nằm dưới đường hạ). Trong vòng đó có thể có
nhiều mặt - mái nhà đang cháy và cỏ quanh nhà: RANSAC tách lần lượt từng mặt (similarity,
nên xoay mũi/trôi ngang không làm lệch), rồi lấy mặt GẦN NHẤT - thứ drone sẽ đâm vào. Lấy
mặt nhiều điểm nhất thì ra cỏ, drone dừng 20 m trên cỏ = 16 m trên mái đang cháy.

Chỉ chạy khi fire_mission bật (/fire/ground_enable, lúc hạ để thả): bay ngang/RTL qua nhà
cao là ảnh không phải một mặt, ra số rác. Không phụ thuộc zoom: luôn dùng toàn khung gốc.
Giả định camera xấp xỉ nadir, mặt có vân (nước/cát/mái trơn thì không ra số).

    /rgb/image + /mavros/local_position/pose -> /fire/ground_z (std_msgs/Float32: z ENU của
    mặt gần nhất dưới drone, cùng hệ với pose)
    python3 ground_range.py --selfcheck
"""
import math
import sys

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

MIN_PTS = 30        # điểm tối thiểu để tin một mặt


def detect_ring(gray, inner_frac, outer_frac, n=800):
    """Điểm góc trong vòng inner..outer (x bề ngang ảnh) quanh tâm ảnh = quanh đám cháy."""
    h, w = gray.shape
    mask = np.zeros((h, w), np.uint8)
    cv2.circle(mask, (w // 2, h // 2), int(outer_frac * w), 255, -1)
    cv2.circle(mask, (w // 2, h // 2), int(inner_frac * w), 0, -1)
    pts = cv2.goodFeaturesToTrack(gray, n, 0.01, 6, mask=mask)
    return np.empty((0, 1, 2), np.float32) if pts is None else pts


def track(prev, cur, pts):
    """KLT prev -> cur. (điểm mới, mặt nạ bám được)."""
    if len(pts) == 0:
        return pts, np.zeros(0, bool)
    nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev, cur, pts, None, winSize=(21, 21), maxLevel=3)
    return nxt, st.ravel() == 1


def surfaces(p0, p1, max_n=3):
    """Các mặt trong cặp điểm p0 -> p1: [(tỉ lệ nở s, số điểm)], nhiều điểm trước. RANSAC
    similarity lần lượt, bỏ điểm đã khớp rồi tìm mặt tiếp theo."""
    out = []
    while len(p0) >= MIN_PTS and len(out) < max_n:
        # 2.5 px: điểm KLT trên vân mờ lệch 1-3 px qua 25 khung (1 px chỉ giữ 38/124 điểm mái),
        # còn hai mặt cách nhau 4 m ở 20-24 m thì lệch nhau ~25 px ở rìa vòng - vẫn tách rõ.
        M, inl = cv2.estimateAffinePartial2D(p0, p1, method=cv2.RANSAC,
                                             ransacReprojThreshold=2.5)
        if M is None:
            break
        inl = inl.ravel().astype(bool)
        if inl.sum() < MIN_PTS:
            break
        out.append((math.hypot(M[0, 0], M[1, 0]), int(inl.sum())))
        p0, p1 = p0[~inl], p1[~inl]
    return out


def looming_dist(s, dh):
    """Khoảng cách tới mặt SAU khi hạ dh m (âm = leo) mà ảnh nở s lần. None nếu vô nghĩa."""
    if s is None or abs(s - 1.0) < 1e-3:
        return None
    d = dh / (s - 1.0)
    return d if d > 0 else None


def nearest(surfs, dh):
    """[(khoảng cách, số điểm)] của các mặt, gần trước. Phần tử đầu = mặt sẽ đâm vào."""
    return sorted((d, n) for d, n in ((looming_dist(s, dh), n) for s, n in surfs) if d)


class GroundRange(Node):
    def __init__(self):
        super().__init__('ground_range')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.work_w = p('work_width', 960)          # co khung về đây trước khi bám điểm
        self.inner = p('inner_frac', 0.08)          # đĩa giữa bỏ qua (đám cháy) / bề ngang
        self.outer = p('outer_frac', 0.36)          # ngoài vòng này không tính (~27 độ lệch trục)
        self.min_step = p('min_step', 3.0)          # dh tối thiểu giữa hai khung đem so (m)
        self.step_frac = p('step_frac', 0.1)        # ... và tối thiểu 10% khoảng cách hiện có
        self.cam_dz = p('camera_dz', -0.18)         # rgb_cam_link dưới base_link (model.sdf)
        # Xa hơn thì không gửi: hạ 11 m ở 120 m ảnh chỉ nở ~9%, drone vừa phanh còn nghiêng ->
        # đo sai 12-24 m (18/10/2026). Quyết định dừng nằm ở 20-30 m, không cần số ở trên này.
        self.max_dist = p('max_dist', 80.0)
        self.enabled = False
        self.pose = None
        self.prev = None                            # khung xám trước
        self.p0 = self.p1 = None                    # điểm ở khung gốc / khung hiện tại
        self.z0 = 0.0                               # z camera lúc khung gốc
        self.d_guess = None
        self.pub = self.create_publisher(Float32, '/fire/ground_z', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/fire/ground_enable', self.on_enable, latched)
        self.create_subscription(Image, '/rgb/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self.on_pose, qos_profile_sensor_data)

    def on_pose(self, msg):
        self.pose = msg.pose

    def on_enable(self, msg):
        if msg.data != self.enabled:
            self.get_logger().info('BAT do mat dat' if msg.data else 'TAT do mat dat')
        self.enabled, self.p0, self.d_guess = msg.data, None, None

    def restart(self, gray, z):
        self.prev, self.z0 = gray, z
        self.p0 = detect_ring(gray, self.inner, self.outer)
        self.p1 = self.p0.copy()

    def on_image(self, msg):
        if not self.enabled or self.pose is None:
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        h = round(msg.height * self.work_w / msg.width)
        gray = cv2.cvtColor(cv2.resize(img, (self.work_w, h), interpolation=cv2.INTER_AREA),
                            cv2.COLOR_RGB2GRAY)
        # ponytail: pose mới nhất, không khớp stamp; hạ 1.5 m/s x trễ ~50 ms = 7 cm/đoạn 3+ m.
        # Không bù nghiêng: drone treo thẳng trên đám cháy, nghiêng vài độ sai < 1%.
        z = self.pose.position.z + self.cam_dz
        if self.p0 is None or len(self.p0) < MIN_PTS:
            return self.restart(gray, z)
        nxt, ok = track(self.prev, gray, self.p1)
        self.p0, self.p1, self.prev = self.p0[ok], nxt[ok], gray
        dh = self.z0 - z
        if abs(dh) < max(self.min_step, self.step_frac * (self.d_guess or abs(z))):
            return
        found = nearest(surfaces(self.p0, self.p1), dh)
        if found:
            d = found[0][0]
            self.d_guess = d
            far = d > self.max_dist
            if not far:
                self.pub.publish(Float32(data=float(z - d)))
            self.get_logger().info(
                f'{"(qua xa, KHONG gui) " if far else ""}mat gan nhat z {z - d:.2f} '
                f'(cach {d:.1f} m); cac mat qua {dh:.1f} m: '
                + ', '.join(f'{dd:.1f} m/{n} diem' for dd, n in found))
        self.restart(gray, z)


def main():
    rclpy.init()
    try:
        rclpy.spin(GroundRange())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def _selfcheck():
    # Camera nadir 960x540 70° hạ 10 m qua 25 khung. Đất giả 0.1 m/px; giữa ảnh là MÁI NHÀ
    # 12x8 m cao hơn đất `roof` m (vân riêng), trên đó đĩa NHẤP NHÁY mỗi khung (= lửa/khói).
    rng = np.random.default_rng(1)

    def texture(n):
        t = cv2.GaussianBlur((rng.random((n, n)) * 255).astype(np.uint8), (0, 0), 2)
        for _ in range(n * 2):
            cv2.circle(t, tuple(rng.integers(0, n, 2).tolist()), int(rng.integers(2, 15)),
                       int(rng.integers(0, 255)), -1)
        return t

    g, rf = texture(1500), texture(400)
    f, (w, h) = 480 / math.tan(math.radians(35)), (960, 540)

    def frame(d, roof, shift):
        def place(src, k, cx, cy):     # ảnh src (0.1 m/px) tâm (cx, cy) px, k px ảnh / px src
            M = np.float32([[k, 0, w / 2 - k * (cx + shift[0])], [0, k, h / 2 - k * (cy + shift[1])]])
            return cv2.warpAffine(src, M, (w, h), flags=cv2.INTER_LINEAR), M
        im, _ = place(g, 0.1 * f / d, 750, 750)
        if roof:
            r, M = place(rf, 0.1 * f / (d - roof), 200, 200)
            box = cv2.transform(np.float32([[[140, 160], [260, 160], [260, 240], [140, 240]]]), M)
            mask = np.zeros((h, w), np.uint8)
            cv2.fillConvexPoly(mask, box.astype(np.int32)[0], 255)
            im[mask > 0] = r[mask > 0]
        cv2.circle(im, (w // 2, h // 2), 40, int(rng.integers(0, 255)), -1)
        return im

    def fly(d0, d1, roof):
        ds = np.linspace(d0, d1, 26)
        prev = frame(ds[0], roof, (0, 0))
        p0 = detect_ring(prev, 0.08, 0.36)
        p1 = p0.copy()
        for i, d in enumerate(ds[1:], 1):
            cur = frame(d, roof, (0.3 * i, -0.2 * i))   # trôi ngang chút trong lúc hạ
            nxt, ok = track(prev, cur, p1)
            p0, p1, prev = p0[ok], nxt[ok], cur
        return nearest(surfaces(p0, p1), d0 - d1)

    flat = fly(60.0, 50.0, 0)
    assert flat and abs(flat[0][0] - 50.0) < 1.0, flat
    # Đất ở 34 -> 24 m, mái cao 4 m ở 30 -> 20 m: phải ra MÁI (20 m), dù cỏ nhiều điểm hơn.
    house = fly(34.0, 24.0, 4.0)
    assert abs(house[0][0] - 20.0) < 1.0, f'phải đo tới mái (20 m), ra {house}'
    assert any(abs(d - 24.0) < 1.0 for d, _ in house), f'cỏ quanh nhà (24 m) phải là mặt thứ hai: {house}'
    assert looming_dist(1.0, 5.0) is None and looming_dist(0.9, 5.0) is None, 'không nở / ngược dấu'
    assert abs(looming_dist(0.9, -5.0) - 50.0) < 1e-9, 'leo 45 -> 50 m: ảnh co 0.9, ra khoảng cách SAU'
    print('ground_range selfcheck OK: dat phang ' + ', '.join(f'{d:.2f} m/{n}' for d, n in flat)
          + ' | nha: ' + ', '.join(f'{d:.2f} m/{n}' for d, n in house))


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
