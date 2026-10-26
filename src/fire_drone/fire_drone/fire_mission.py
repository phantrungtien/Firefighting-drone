"""Nhiệm vụ chữa cháy theo điểm người vận hành chấm trên bản đồ GUI_NATIVE.

    GUI: chạm-giữ bản đồ -> "Đóm lửa ở đây" -> trượt xác nhận
      -> WebSocket -> ros2_bridge -> /gcs/command {"action":"fire_target","args":{lat,lon,alt}}

Node: chờ lệnh -> (arm + cất cánh nếu đang dưới đất) -> bay thẳng tới điểm chấm
-> bay vòng nở dần quanh điểm đó tới khi detector thấy đóm lửa -> treo trên đóm lửa
-> thả bóng -> bay một vòng KIỂM TRA quanh đám cháy -> RTL. Mỗi bước báo STATUSTEXT
lên GCS (tab Thông báo).

Cú chấm là người vận hành GIAO một việc có giới hạn. Gạt khỏi GUIDED (cần RC, nút
mode của GUI) hay nút đỏ lúc nào thì node buông lúc đó: không thả, không đổi mode.
Huỷ task (bridge SIGTERM) / lỗi giữa chừng: TREO tại chỗ + báo, không đổi mode (user 27/10/2026).

    ros2 launch fire_drone fire_mission.launch.py
    python3 fire_mission.py --selfcheck
"""
import json
import math
import signal
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from mavros_msgs.msg import StatusText
from mavros_msgs.srv import CommandLong
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

from fire_drone.guided import GuidedNode

# Thả/đóng bằng gripper, đúng đường của kênh 5 trên tay RC (RC5_OPTION 19, FC thật đọc
# 26/10/2026: servo ở SERVO16, GRIP_GRAB 1900 - thử thật bóng rơi gọn, GRIP_RELEASE 1100).
# Tên ArduPilot NGƯỢC với cơ cấu này: GRAB = 1900 = MỞ thả bóng (gạt RC5 lên),
# RELEASE = 1100 = ĐÓNG (RC5 về). PWM nằm ở param FC, không ở đây.
MAV_CMD_DO_GRIPPER = 211
GRIP_OPEN, GRIP_CLOSE = 1, 0   # GRIPPER_ACTION_GRAB, GRIPPER_ACTION_RELEASE
EARTH_R = 6378137.0


def pilot_has_it(state):
    """Đang bay mà không ở GUIDED = phi công (cần RC / GCS) đã giành lái."""
    return state.armed and state.mode != 'GUIDED'


def parse_fire_cmd(raw):
    """JSON từ /gcs/command -> (lat, lon, alt, drop, radius), hoặc None nếu không hợp lệ.
    alt = độ cao bay, drop = hạ cách lửa bao nhiêu m để thả, radius = phạm vi tìm quanh điểm
    chấm - người vận hành chọn trên GUI; thiếu thì None (dùng số của node).

    Đến từ mạng (WebSocket của bridge) nên kiểm hết: sai kiểu, NaN, ngoài khoảng
    đều bỏ - một tọa độ rác là drone bay đi một nơi không ai chọn.
    """
    try:
        msg = json.loads(raw)
        if msg.get('action') != 'fire_target':
            return None
        args = msg['args']
        lat, lon = float(args['lat']), float(args['lon'])
        alt = None if args.get('alt') is None else float(args['alt'])
        drop = None if args.get('drop') is None else float(args['drop'])
        radius = None if args.get('radius') is None else float(args['radius'])
    except (ValueError, TypeError, KeyError, AttributeError):
        return None
    if not (math.isfinite(lat) and math.isfinite(lon)) or abs(lat) > 90 or abs(lon) > 180:
        return None
    if any(a is not None and not math.isfinite(a) for a in (alt, drop, radius)):
        return None
    return lat, lon, alt, drop, radius


def pick_radius(radius, default, max_r):
    """Phạm vi tìm người vận hành chọn -> m, hoặc chuỗi lý do từ chối. 0 = chỉ treo ở điểm
    chấm rồi zoom. Thiếu thì search_radius của node."""
    r = default if radius is None else radius
    if not 0 <= r <= max_r:
        return f'Pham vi tim {r:g} m ngoai 0-{max_r:g} m: tu choi'
    return r


def pick_alts(alt, drop, lo, hi, search_alt, drop_alt):
    """(độ cao bay, hạ cách lửa) người vận hành chọn -> (alt, drop), hoặc chuỗi lý do từ chối.
    Thiếu thì lấy mặc định của node; drop mặc định không cao hơn alt (không leo lên để thả).
    drop trong [lo, alt]: thấp hơn min_alt là sát lửa / mặt đất, cao hơn alt là leo lên thả.
    Từ chối chứ không kẹp: người vận hành phải bay đúng số đã chọn, hoặc biết là không."""
    alt = search_alt if alt is None else alt
    drop = min(drop_alt, alt) if drop is None else drop
    if not lo <= alt <= hi:
        return f'Do cao {alt:g} m ngoai {lo:g}-{hi:g} m: tu choi'
    if not lo <= drop <= alt:
        return f'Ha cach lua {drop:g} m ngoai {lo:g}-{alt:g} m: tu choi'
    return alt, drop


def ll_to_enu(lat, lon, lat0, lon0):
    """(đông, bắc) mét của (lat, lon) so với (lat0, lon0).
    ponytail: phẳng cục bộ - sai < 1 cm trong bán kính 1 km, đủ cho một ổ cháy."""
    return (math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0)),
            math.radians(lat - lat0) * EARTH_R)


def hold_z(err, pz, fire_z, drop_alt, descend_r):
    """Độ cao setpoint khi tiến tới đóm lửa. Lệch ngang > descend_r: KHÔNG hạ (giữ độ cao
    đang có, thấp hơn mức thả thì lên) - hạ chéo là cắt qua cây/nhà giữa đường. Tới ngay
    trên rồi mới hạ thẳng đứng xuống drop_alt trên đóm lửa."""
    if err > descend_r:
        return max(pz, fire_z + drop_alt)
    return fire_z + drop_alt


def descent_zoom(z0, h0, h):
    """Zoom khi hạ: giữ đóm lửa cỡ như lúc thấy nó (z0 ở độ cao h0) - càng xuống càng
    zoom ra, về x1 thì thôi. Không zoom thêm quá z0. Làm tròn 0.1 cho đỡ gửi liên tục."""
    return round(max(1.0, min(z0, z0 * h / h0)), 1)


def ground_filter(zs):
    """z mặt gần nhất từ chuỗi số đo ground_range. GẦN hơn thì tin NGAY (số mới nhất cao hơn
    trung vị = vừa thấy mái/tán cây: dừng hạ liền, nhầm thì chỉ là thả cao hơn chút); XA hơn
    thì phải trung vị 3 số. Trung vị trơn để mái chỉ kịp hiện 1 số trước khi tới nơi -> thả
    16 m trên mái thay vì 20 (18/10/2026). Chưa có số nào: ngang chỗ cất cánh (z = 0).
    KHÔNG BAO GIỜ dưới 0 (user 27/10/2026: không hạ dưới điểm đặt): số đo chỉ được đẩy điểm
    thả LÊN (thấy mái), không kéo XUỐNG thấp hơn drop_alt trên chỗ cất cánh - đo sai thấp là
    drone hạ sát đất. Đám cháy dưới hố thấp hơn chỗ cất cánh thì thả cao hơn, chịu."""
    if not zs:
        return 0.0
    return max(0.0, statistics.median(zs[-3:]), zs[-1])


def orbit_points(cx, cy, radius, step=6.0):
    """Các điểm trên vòng tròn tâm (cx, cy), cách nhau ~step m, khép kín vòng."""
    n = max(6, math.ceil(2 * math.pi * radius / step))
    return [(cx + radius * math.cos(2 * math.pi * k / n),
             cy + radius * math.sin(2 * math.pi * k / n)) for k in range(n + 1)]


class FireMission(GuidedNode):
    def __init__(self):
        super().__init__('fire_mission')
        p = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.search_alt = p('search_alt', 20.0)    # mặc định khi GUI không gửi alt
        self.min_alt = p('min_alt', 5.0)            # alt GUI gửi ngoài khoảng này = từ chối
        self.max_alt = p('max_alt', 120.0)
        # Ở 20 m camera 70° 16:9 (= camera Jetson) thấy ~28x16 m: vòng 8 m chồng mép rộng.
        self.ring_step = p('ring_step', 8.0)
        self.search_radius = p('search_radius', 24.0)     # mặc định khi GUI không gửi radius
        self.max_search_radius = p('max_search_radius', 50.0)  # radius GUI gửi quá = từ chối
        self.dwell = p('dwell', 5.0)                # treo ở điểm chấm cho detector đủ khung
        # Zoom số trên Jetson (cắt giữa khung webcam 4K): x1 không thấy thì x2, x3... tới đây.
        # Đo 18/10/2026: mã 3 m cần >= ~25 px mới đọc chắc; 110 m x1 chỉ ~18 px -> 0 khung.
        self.max_zoom = p('max_zoom', 5.0)
        self.zoom = 1.0
        self.max_range = p('max_range', 300.0)      # điểm chấm xa hơn = bấm nhầm, từ chối
        # Thả (và vòng kiểm tra) cách đóm lửa bao nhiêu m. Đo 18/10/2026: ở 100 m mã ArUco
        # còn ~11 px, PnP lệch 1.8 m ngang, 9 m dọc, vòng kiểm tra báo nhầm hết lửa; ở
        # 40 m lệch 0.4 m. Nên tìm ở cao bao nhiêu cũng được, thả thì hạ về đây.
        self.drop_alt = p('drop_alt', 20.0)
        self.drop_tol = p('drop_tol', 0.3)          # lệch ngang tối đa lúc thả (m)
        self.descend_r = p('descend_radius', 2.0)   # lệch ngang hơn mức này thì chưa hạ
        self.check_radius = p('check_radius', 6.0)
        self.target = None
        self.driving = False                        # node đã arm/lái drone (không chỉ đang chờ chấm)
        self.seen = 0                               # số lần detector báo, để vòng kiểm tra đếm
        self.hint = None                            # (lat, lon) người vận hành chấm
        self.fix = None
        self.vel = TwistStamped()
        self.yaw = 0.0
        self.create_subscription(PointStamped, '/fire/target', self._on_target, 10)
        self.ground = []    # z mặt đất quanh đám cháy, ground_range đo bằng camera 2D
        self.create_subscription(Float32, '/fire/ground_z',
                                 lambda m: self.ground.append(m.data), 10)
        self.create_subscription(String, '/gcs/command', self._on_cmd, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._set('fix'), qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                 self._set('vel'), qos_profile_sensor_data)
        self.sp_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.txt_pub = self.create_publisher(StatusText, '/mavros/statustext/send', 10)
        self.cmd_cli = self.create_client(CommandLong, '/mavros/cmd/command')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.zoom_pub = self.create_publisher(Float32, '/fire/zoom', latched)
        self.zoom_pub.publish(Float32(data=1.0))    # task trước chết giữa chừng ở x3 thì về x1
        # ground_range chỉ chạy lúc hạ để thả: bay ngang/RTL qua nhà cao ảnh không phải một mặt
        self.ground_on = self.create_publisher(Bool, '/fire/ground_enable', latched)
        self.ground_on.publish(Bool(data=False))

    def _on_target(self, msg):
        self.target = msg
        self.seen += 1

    def _on_cmd(self, msg):
        cmd = parse_fire_cmd(msg.data)
        if cmd is None:
            return  # lệnh khác của GUI, không phải việc của node này
        if self.hint is not None:
            return self.notify('Da co nhiem vu, bo qua diem cham moi', StatusText.WARNING)
        lat, lon, alt, drop, radius = cmd
        # Kiểm HẾT rồi mới đổi gì: lệnh bị từ chối không được để lại ring_step/drop_alt nửa vời.
        ok = pick_alts(alt, drop, self.min_alt, self.max_alt, self.search_alt, self.drop_alt)
        r = pick_radius(radius, self.search_radius, self.max_search_radius)
        for bad in (ok, r):
            if isinstance(bad, str):
                return self.notify(bad, StatusText.WARNING)
        alt, drop = ok
        self.search_radius = r
        # Vết camera tỉ lệ thuận độ cao: giữ độ chồng mép giữa các vòng tìm.
        self.ring_step *= alt / self.search_alt
        # Bay đi/về/tìm ở `alt`, hạ xuống `drop` m trên mặt quanh đóm lửa rồi thả.
        self.search_alt, self.drop_alt = alt, drop
        self.hint = (lat, lon)

    def notify(self, text, severity=StatusText.INFO):
        """Log + STATUSTEXT lên GCS. MAVLink cắt ở 50 byte nên giữ chữ không dấu, ngắn."""
        # HAI DÒNG RIÊNG, không gộp: rclpy nhớ mức log theo DÒNG gọi, cùng một dòng
        # lúc info lúc warn là ValueError -> node chết giữa task (đã dính 16/10).
        if severity <= StatusText.WARNING:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        self.txt_pub.publish(StatusText(severity=severity, text=('FIRE: ' + text)[:50]))

    def set_zoom(self, z):
        if z != self.zoom:
            self.zoom = z
            self.zoom_pub.publish(Float32(data=float(z)))

    def send_sp(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z
        sp.pose.orientation.z = math.sin(self.yaw / 2)
        sp.pose.orientation.w = math.cos(self.yaw / 2)
        self.sp_pub.publish(sp)

    def give_up(self, why):
        """Bỏ nhiệm vụ. Phi công đang lái thì KHÔNG đổi mode - để nguyên cho họ."""
        self.set_zoom(1.0)  # trả góc rộng cho người vận hành, không đụng gì tới bay
        if pilot_has_it(self.state):
            self.get_logger().warning(f'luc dang: {why}')
            self.notify(f'PHI CONG LAI ({self.state.mode}): dung task', StatusText.WARNING)
        else:
            self.notify(f'{why} -> RTL', StatusText.WARNING)
            self.climb_rtl()
        return False

    def hold_here(self):
        """Task dừng ngang (Huỷ task, lỗi): setpoint cuối là điểm đang bay TỚI, GUIDED sẽ bay
        tiếp tới đó rồi mới treo. Gửi chỗ đang đứng = treo NGAY. Không đổi mode: lấy lại lái
        là việc của người bay (RC / nút mode qua SiK)."""
        if not (self.driving and self.state.armed and self.state.mode == 'GUIDED'):
            return  # chưa lái / phi công đã giành / đã RTL: không phải việc của node
        p = self.pose.pose.position
        for _ in range(3):  # ponytail: 3 lần không chờ ack, topic đã nối sẵn với MAVROS
            self.send_sp(p.x, p.y, p.z)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.notify('TREO tai cho: task dung ngang', StatusText.CRITICAL)  # <= 50 byte

    def climb_rtl(self):
        """Leo THẲNG ĐỨNG lại search_alt rồi mới RTL. Cao hơn RTL_ALT thì RTL giữ nguyên độ
        cao hiện tại mà bay thẳng về: ở độ cao thả là cắt ngang cây/nhà lúc đi đã vượt qua."""
        p = self.pose.pose.position
        if self.state.armed and self.fly_to(p.x, p.y, self.search_alt,
                                            timeout=30.0 + self.search_alt) == 'pilot':
            return
        self.set_mode('RTL')

    def dist_to(self, x, y, z):
        p = self.pose.pose.position
        return math.dist((p.x, p.y, p.z), (x, y, z))

    def fly_to(self, x, y, z, tol=2.0, timeout=90.0, stop=lambda: False):
        """Stream setpoint tới khi tới nơi / stop() / phi công giành lái / hết giờ.
        Trả 'ok'|'stop'|'pilot'|'timeout'."""
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            self.send_sp(x, y, z)
            rclpy.spin_once(self, timeout_sec=0.1)
            if pilot_has_it(self.state):
                return 'pilot'
            if stop():
                return 'stop'
            if self.dist_to(x, y, z) < tol:
                return 'ok'
        return 'timeout'

    def fly_path(self, pts, z, stop=lambda: False):
        """Bay lần lượt qua pts. 'stop' | 'pilot' | 'done'."""
        for x, y in pts:
            r = self.fly_to(x, y, z, stop=stop)
            if r in ('stop', 'pilot'):
                return r
        return 'done'

    def search_around(self, cx, cy):
        """Tìm ở x1; không thấy thì về treo ở điểm chấm, zoom x2 tìm lại, x3... tới max_zoom.
        True = thấy đóm lửa."""
        z = 1.0
        while True:
            res = self.search_once(cx, cy, z)
            if res is not None:
                return res
            z += 1.0
            if z > self.max_zoom + 1e-6:
                return False
            self.notify(f'Khong thay, zoom x{z:.0f} tim lai')
            self.set_zoom(z)

    def search_once(self, cx, cy, zoom):
        """Treo ở điểm chấm `dwell` s, rồi các vòng nở dần tới search_radius, vòng thưa/dày
        theo vết camera (hẹp đi `zoom` lần). True = thấy, False = phi công giành lái, None = không."""
        found = lambda: self.target is not None  # noqa: E731
        res = self.fly_to(cx, cy, self.search_alt, stop=found)
        if res == 'ok':
            # Tới nơi đi ngay thì detector không kịp đủ confirm_frames: bay 110 m (vòng 44 m
            # > search_radius, chỉ còn điểm giữa) là bỏ cuộc lúc vừa tới (18/10/2026).
            res = self.fly_to(cx, cy, self.search_alt, tol=-1.0, timeout=self.dwell, stop=found)
        if res == 'timeout':
            path, step = [], self.ring_step / zoom
            r = step
            while r <= self.search_radius + 1e-6:
                path += orbit_points(cx, cy, r)
                r += step
            res = self.fly_path(path, self.search_alt, stop=found)
        return {'stop': True, 'pilot': False}.get(res)

    def fire_z(self):
        """z mặt GẦN NHẤT quanh đám cháy (mái nhà đang cháy, tán cây... chứ không phải cỏ xa
        hơn) để tính độ cao thả: trung vị 3 số đo ground_range gần nhất (camera 2D, thay lidar;
        KHÔNG dùng z PnP - lửa thật không có cỡ chuẩn như mã ArUco).
        Chưa có số nào thì coi ngang chỗ cất cánh (z = 0): đất dốc/mái nhà là sai."""
        return ground_filter(self.ground)

    def hold_over_fire(self, settle=2.0, timeout=90.0):
        """Tới ngay trên đóm lửa, hạ thẳng đứng xuống drop_alt trên mặt đất chỗ đó (mục tiêu
        và số đo mặt đất vẫn được tinh chỉnh khi hạ) tới khi lệch < drop_tol, đúng độ cao,
        gần như đứng yên `settle` giây."""
        self.ground = []
        try:
            return self._hold(settle, timeout)
        finally:
            self.ground_on.publish(Bool(data=False))

    def _hold(self, settle, timeout):
        end, since, measuring = time.time() + timeout, None, False
        z0, h0 = self.zoom, self.pose.pose.position.z - self.fire_z()
        while rclpy.ok() and time.time() < end:
            t, p, fz = self.target.point, self.pose.pose.position, self.fire_z()
            self.set_zoom(descent_zoom(z0, h0, p.z - fz))
            herr = math.hypot(p.x - t.x, p.y - t.y)
            if not measuring and herr <= self.descend_r:
                # Bật đo mặt đất khi bắt đầu HẠ thẳng đứng, không phải lúc còn bay ngang tới:
                # bay ngang thì ảnh trôi chứ không nở, số đo đầu sai tới 12 m (18/10/2026).
                measuring = True
                self.ground_on.publish(Bool(data=True))
            z = hold_z(herr, p.z, fz, self.drop_alt, self.descend_r)
            self.send_sp(t.x, t.y, z)
            rclpy.spin_once(self, timeout_sec=0.1)
            if pilot_has_it(self.state):
                return None
            p, v = self.pose.pose.position, self.vel.twist.linear
            err = math.hypot(p.x - t.x, p.y - t.y)
            if (err < self.drop_tol and abs(p.z - fz - self.drop_alt) < 0.5
                    and math.hypot(v.x, v.y, v.z) < 0.2):
                since = since or time.time()
                if time.time() - since >= settle:
                    return err
            else:
                since = None
        return None

    def gripper(self, action):
        req = CommandLong.Request(command=MAV_CMD_DO_GRIPPER, param1=1.0, param2=float(action))
        r = self._call(self.cmd_cli, req)
        return bool(r and r.success)

    def hint_local(self):
        """Điểm chấm (lat, lon) -> (x, y) local ENU, neo vào cặp GPS/pose hiện tại."""
        e, n = ll_to_enu(*self.hint, self.fix.latitude, self.fix.longitude)
        p = self.pose.pose.position
        return p.x + e, p.y + n

    def run(self):
        log = self.get_logger()
        if not self.wait_until(lambda: self.state.connected, 60.0):
            return log.error('MAVROS chua connected')
        self.request_streams()
        if not self.wait_until(lambda: self.pose.header.stamp.sec > 0 and self.fix, 120.0):
            return log.error('khong co pose/GPS')
        self.notify('San sang. Cham-giu ban do GUI -> "Chua chay tai day"')  # đúng chữ trên nút GUI (touch.wp_fire)
        # Chờ NGƯỜI, không phải chờ máy - không có hạn là đúng.
        self.wait_until(lambda: self.hint is not None, float('inf'))
        if not rclpy.ok():
            return False
        hx, hy = self.hint_local()
        p = self.pose.pose.position
        if math.hypot(hx - p.x, hy - p.y) > self.max_range:
            self.notify(f'Diem cham xa {math.hypot(hx - p.x, hy - p.y):.0f} m > '
                        f'{self.max_range:.0f} m: tu choi', StatusText.WARNING)
            return False
        self.notify(f'Nhan diem chay ({hx:.0f}, {hy:.0f}) m, bay {self.search_alt:.0f} m')
        self.target, self.seen = None, 0  # mục tiêu cũ (nếu có) không được dẫn đường

        self.driving = True
        if not self.state.armed and not self.arm():
            return self.give_up('Arm that bai')
        if self.state.mode != 'GUIDED':
            self.set_mode('GUIDED')       # người vận hành vừa giao việc = đã chọn
        # WPNAV_SPEED_UP 2 m/s: 100 m mất ~50 s, hạn mặc định 60 s của takeoff() là sát nút.
        if self.alt < 1.0 and not self.takeoff(self.search_alt, timeout=30.0 + self.search_alt):
            return self.give_up('Cat canh that bai')
        q = self.pose.pose.orientation   # giữ nguyên hướng mũi
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        self.notify('Bay toi diem cham, tim quanh do')
        if not self.search_around(hx, hy):
            # bị giành lái cũng trả False - phải tách ra, không thì báo nhầm "không thấy"
            return self.give_up('dang tim' if pilot_has_it(self.state)
                                else f'Khong thay lua trong {self.search_radius:.0f} m')
        t = self.target.point
        self.notify(f'Thay lua ({t.x:.1f}, {t.y:.1f}) x{self.zoom:g}, ha {self.drop_alt:.0f} m tha')
        # WPNAV_SPEED_DN 1.5 m/s: hạ 100 -> 20 m mất ~55 s, cộng thời gian canh lại tâm.
        err = self.hold_over_fire(timeout=60.0 + self.search_alt)
        if err is None:
            return self.give_up('Khong treo on dinh, KHONG tha')
        t, fz = self.target.point, self.fire_z()
        if not self.gripper(GRIP_OPEN):
            self.notify('DO_GRIPPER bi tu choi', StatusText.ERROR)
        self.notify(f'DA THA BONG, lech {err:.2f} m, cach dat {self.pose.pose.position.z - fz:.1f} m')
        if not self.ground:
            self.notify('Khong do duoc mat dat (thieu van?): tinh theo muc nha', StatusText.WARNING)
        log.info(f'mat dat: camera 2D {fz:.2f} ({len(self.ground)} so do), PnP ma ArUco {t.z:.2f}')
        self.wait_until(lambda: False, 2.0)
        if not self.gripper(GRIP_CLOSE):   # như gạt RC5 về 994
            self.notify('DO_GRIPPER dong bi tu choi', StatusText.WARNING)

        # Vòng kiểm tra: còn thấy đóm lửa là còn cháy (một bóng thì chỉ báo được).
        fx, fy = t.x, t.y
        self.seen = 0
        self.notify('Bay vong kiem tra quanh dam chay')
        # Ở độ cao thả: cao hơn thì mã quá nhỏ, vòng kiểm tra báo nhầm hết lửa (đo ở 100 m).
        if self.fly_path(orbit_points(fx, fy, self.check_radius), fz + self.drop_alt) == 'pilot':
            return self.give_up('Dang kiem tra')
        if self.seen:
            self.notify(f'KIEM TRA: VAN THAY dom lua ({self.seen} khung)', StatusText.WARNING)
        else:
            self.notify('KIEM TRA: khong con thay dom lua')

        self.notify(f'Len {self.search_alt:.0f} m roi ve nha (RTL)')
        self.set_zoom(1.0)
        self.climb_rtl()
        landed = self.wait_until(lambda: not self.state.armed, 240.0)
        log.info('RTL xong, da disarm' if landed else 'RTL qua han')
        return landed


def main():
    rclpy.init()
    # Huỷ task = bridge SIGTERM cả nhóm. rclpy tự bắt SIGTERM và tắt context: mọi vòng
    # `while rclpy.ok()` thoát giữa chừng, run() chạy tiếp nhánh lạ, không gửi được gì nữa.
    # Đổi thành KeyboardInterrupt (đặt SAU rclpy.init để đè bộ bắt của nó) -> kịp treo.
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = FireMission()
    ok = False
    try:
        ok = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.hold_here()
        except Exception as e:  # Ctrl-C: rclpy đã đóng context, FC giữ setpoint cuối
            node.get_logger().error(f'khong treo duoc: {e}')
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


def _selfcheck():
    from types import SimpleNamespace as S
    # phi công gạt sang LOITER/RTL... giữa chừng -> nhiệm vụ phải buông
    assert pilot_has_it(S(armed=True, mode='LOITER')) and pilot_has_it(S(armed=True, mode='RTL'))
    assert not pilot_has_it(S(armed=True, mode='GUIDED'))
    assert not pilot_has_it(S(armed=False, mode='STABILIZE')), 'chưa arm thì chưa ai lái'

    # lệnh từ mạng: chỉ đúng action + số hữu hạn trong khoảng mới qua
    ok = '{"action":"fire_target","args":{"lat":-35.36,"lon":149.16}}'
    assert parse_fire_cmd(ok) == (-35.36, 149.16, None, None, None), 'GUI cũ không gửi alt vẫn chạy'
    ok = '{"action":"fire_target","args":{"lat":-35.36,"lon":149.16,"alt":15}}'
    assert parse_fire_cmd(ok) == (-35.36, 149.16, 15.0, None, None), 'GUI chưa có ô hạ cách lửa'
    ok = '{"action":"fire_target","args":{"lat":-35.36,"lon":149.16,"alt":5,"drop":3,"radius":6}}'
    assert parse_fire_cmd(ok) == (-35.36, 149.16, 5.0, 3.0, 6.0)
    for bad in ['rac', '{"action":"goto","args":{"lat":1,"lon":2}}', '{"action":"fire_target"}',
                '{"action":"fire_target","args":{"lat":"NaN","lon":0}}',
                '{"action":"fire_target","args":{"lat":91,"lon":0}}',
                '{"action":"fire_target","args":{"lat":1,"lon":"x"}}', '[1,2]',
                '{"action":"fire_target","args":{"lat":1,"lon":2,"alt":"NaN"}}',
                '{"action":"fire_target","args":{"lat":1,"lon":2,"alt":"cao"}}',
                '{"action":"fire_target","args":{"lat":1,"lon":2,"alt":5,"drop":"inf"}}',
                '{"action":"fire_target","args":{"lat":1,"lon":2,"radius":"NaN"}}']:
        assert parse_fire_cmd(bad) is None, bad

    # phạm vi tìm: 0 = chỉ treo ở điểm chấm; quá max_search_radius / âm = từ chối
    assert pick_radius(None, 2.0, 10.0) == 2.0, 'GUI cũ: search_radius của node'
    assert pick_radius(0.0, 2.0, 10.0) == 0.0 and pick_radius(10.0, 2.0, 10.0) == 10.0
    assert isinstance(pick_radius(12.0, 2.0, 10.0), str) and isinstance(pick_radius(-1.0, 2.0, 10.0), str)

    # độ cao bay / hạ cách lửa: drone thật 2-5 m (fire_mission_real.yaml), mặc định 5 / 3
    real = (2.0, 5.0, 5.0, 3.0)
    assert pick_alts(None, None, *real) == (5.0, 3.0), 'GUI cũ: mặc định của node'
    assert pick_alts(4.0, 2.0, *real) == (4.0, 2.0)
    assert pick_alts(2.0, None, *real) == (2.0, 2.0), 'bay 2 m mà mặc định thả 3 m = leo lên thả'
    assert pick_alts(5.0, 5.0, *real) == (5.0, 5.0), 'thả ngay ở độ cao bay được'
    for alt, drop in [(20.0, None), (1.0, None), (5.0, 1.5), (4.0, 4.5), (None, 6.0)]:
        assert isinstance(pick_alts(alt, drop, *real), str), (alt, drop)
    assert pick_alts(None, None, 5.0, 120.0, 20.0, 20.0) == (20.0, 20.0), 'sim giữ nguyên'

    # 0.001 độ vĩ = 111.3 m bắc; 0.001 độ kinh ở vĩ độ 60 = một nửa
    e, n = ll_to_enu(10.001, 106.0, 10.0, 106.0)
    assert abs(e) < 1e-6 and abs(n - 111.32) < 0.01
    e, n = ll_to_enu(60.0, 25.001, 60.0, 25.0)
    assert abs(e - 55.66) < 0.01 and abs(n) < 1e-6

    # hạ: còn lệch ngang thì không xuống (lên nếu thấp hơn mức thả), tới nơi mới xuống
    assert hold_z(30.0, 100.0, 0.0, 20.0, 2.0) == 100.0, 'hạ chéo xuyên nhà'
    assert hold_z(30.0, 10.0, 0.0, 20.0, 2.0) == 20.0
    assert hold_z(1.0, 100.0, 0.0, 20.0, 2.0) == 20.0
    assert hold_z(1.0, 100.0, 3.0, 20.0, 2.0) == 23.0, 'drop_alt tính trên z đóm lửa'

    # zoom khi hạ: thấy ở x2 / 110 m -> 55 m còn x1, 20 m về x1; không bao giờ quá z0
    assert descent_zoom(2.0, 110.0, 110.0) == 2.0
    assert descent_zoom(2.0, 110.0, 82.5) == 1.5
    assert descent_zoom(2.0, 110.0, 20.0) == 1.0
    assert descent_zoom(3.0, 100.0, 130.0) == 3.0, 'bị gió đẩy lên cao hơn thì cũng không zoom thêm'

    # lọc mặt đất: thấy mái là tin ngay, số xa lạc loài thì trung vị bỏ qua
    assert ground_filter([]) == 0.0
    assert ground_filter([0.1, -0.5, 4.8]) == 4.8, 'vừa thấy mái mà vẫn hạ theo cỏ'
    assert ground_filter([4.8, 4.6, -20.0]) == 4.6, 'một số xa lạc loài kéo drone xuống'
    assert ground_filter([4.8, 0.2, 0.1]) == 0.2, 'mái biến mất 2 lần liền thì theo số đông'
    assert ground_filter([-1.5, -2.0, -1.8]) == 0.0, 'đo mặt thấp hơn chỗ cất cánh: hạ dưới điểm đặt'
    assert ground_filter([-0.5, -20.0, -3.0]) == 0.0

    pts = orbit_points(18, 12, 8)
    assert all(abs(math.hypot(x - 18, y - 12) - 8) < 1e-9 for x, y in pts)
    assert pts[0] == pts[-1] or math.dist(pts[0], pts[-1]) < 1e-9, 'vòng phải khép kín'
    assert max(math.dist(a, b) for a, b in zip(pts, pts[1:])) <= 6.0 + 1e-9
    print('fire_mission selfcheck OK')


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
