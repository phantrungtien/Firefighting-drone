"""Nhiệm vụ chữa cháy theo điểm người vận hành chấm trên bản đồ GUI_NATIVE.

    GUI: chạm-giữ bản đồ -> "Đóm lửa ở đây" -> trượt xác nhận
      -> WebSocket -> ros2_bridge -> /gcs/command {"action":"fire_target","args":{lat,lon,alt}}

Node: chờ lệnh -> (arm + cất cánh nếu đang dưới đất) -> bay thẳng tới điểm chấm
-> bay vòng nở dần quanh điểm đó tới khi detector thấy đóm lửa -> TREO TẠI CHỖ, chờ người
vận hành trượt xác nhận trên GUI (user 29/10/2026; /fire/ask -> bridge -> topic `mission`.ask,
trả lời {"action":"fire_confirm","args":{"id","ok"}}) -> treo trên đóm lửa -> thả bóng
-> bay thẳng về chỗ ARM ở độ cao tìm (GUIDED) rồi LAND - không vòng kiểm tra, không RTL (user 16/11/2026: cần
cycle time thấp). "Không phải lửa" / bỏ giữa chừng = RTL như cũ.
Mỗi bước báo STATUSTEXT lên GCS (tab Thông báo).

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
from geometry_msgs.msg import PointStamped, PolygonStamped, PoseArray, PoseStamped, TwistStamped
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


TEXT_MAX = 49   # STATUSTEXT 50 byte, MAVROS giữ 1 byte kết chuỗi (đo 16/11/2026: 58 ký tự gửi đi, tới GCS còn 49)


def fit_text(text, limit=TEXT_MAX):
    """Cắt cho vừa `limit` BYTE UTF-8 mà không xẻ đôi ký tự. Chữ có dấu tốn 2-3 byte/ký tự: cắt theo số ký tự
    thì MAVROS chặt giữa một chữ, GCS hiện ký tự hỏng."""
    return text.encode()[:limit].decode(errors='ignore')


def pilot_has_it(state):
    """Đang bay mà không ở GUIDED = phi công (cần RC / GCS) đã giành lái."""
    return state.armed and state.mode != 'GUIDED'


def fc_block(connected, armed, prearm_ok, has_pos):
    """FC chưa sẵn sàng thì KHÔNG nhận điểm cháy, không ARM, không đổi mode - chỉ báo lý do (= luật
    Commands.auto_block của GUI_NATIVE f70cea3). Đo 07/11/2026 15:18: FC 'PreArm: GPS 1: Bad fix' +
    'Radio failsafe on' mà task vẫn nhận điểm, node đợi prearm trong arm() -> lỗi vừa hết là tự ARM cất
    cánh. Đã ARM (điểm mới giữa task) chỉ cần vị trí. Trả lý do ngắn (STATUSTEXT, xem fit_text) hoặc None."""
    if not connected:
        return 'FC chưa kết nối'
    if not has_pos:
        return 'chưa có định vị GPS'
    if not armed and prearm_ok is not True:
        return 'PreArm chưa đạt' if prearm_ok is False else 'chưa rõ trạng thái PreArm'
    return None


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


def parse_confirm(raw, ask_id):
    """JSON /gcs/command -> True (thả) / False (không phải lửa) / None (lệnh khác, rác, hoặc trả
    lời cho LẦN HỎI KHÁC). id phải khớp: một cú trượt đến trễ qua 4G không được thả cho đóm
    lửa sau mà người vận hành chưa nhìn."""
    try:
        msg = json.loads(raw)
        if msg.get('action') != 'fire_confirm':
            return None
        args = msg['args']
        ok, i = args['ok'], args['id']
    except (ValueError, TypeError, KeyError, AttributeError):
        return None
    if not isinstance(ok, bool) or isinstance(i, bool) or i != ask_id:
        return None
    return ok


def pick_radius(radius, default, max_r):
    """Phạm vi tìm người vận hành chọn -> m, hoặc chuỗi lý do từ chối. 0 = chỉ treo ở điểm
    chấm rồi zoom. Thiếu thì search_radius của node."""
    r = default if radius is None else radius
    if not 0 <= r <= max_r:
        return f'Từ chối: bán kính {r:g} ngoài 0-{max_r:g} m'
    return r


def pick_alts(alt, drop, lo, hi, search_alt, drop_alt):
    """(độ cao bay, hạ cách lửa) người vận hành chọn -> (alt, drop), hoặc chuỗi lý do từ chối.
    Thiếu thì lấy mặc định của node; drop mặc định không cao hơn alt (không leo lên để thả).
    drop trong [lo, alt]: thấp hơn min_alt là sát lửa / mặt đất, cao hơn alt là leo lên thả.
    Từ chối chứ không kẹp: người vận hành phải bay đúng số đã chọn, hoặc biết là không."""
    alt = search_alt if alt is None else alt
    drop = min(drop_alt, alt) if drop is None else drop
    if not lo <= alt <= hi:
        return f'Từ chối: độ cao {alt:g} ngoài {lo:g}-{hi:g} m'
    if not lo <= drop <= alt:
        return f'Từ chối: mức thả {drop:g} ngoài {lo:g}-{alt:g} m'
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


def quad_area(q):
    """Diện tích tứ giác (4 góc theo vòng), m²."""
    return abs(sum(q[i][0] * q[(i + 1) % 4][1] - q[(i + 1) % 4][0] * q[i][1] for i in range(4))) / 2


def area_grid(quad, n=7, inset=0.15):
    """Lưới n x n điểm BÊN TRONG tứ giác đám cháy (4 góc theo vòng), co vào `inset` mỗi mép: các ứng viên điểm thả.
    Co mép: hộp lửa rộng hơn ngọn lửa một chút, thả sát mép là thả ra ngoài."""
    a, b, c, d = quad
    k = [inset + (1 - 2 * inset) * i / (n - 1) for i in range(n)]
    return [tuple((1 - u) * (1 - v) * a[i] + u * (1 - v) * b[i] + u * v * c[i] + (1 - u) * v * d[i] for i in (0, 1))
            for u in k for v in k]


def person_tracks(sightings, speed, noise=0.5):
    """[(giờ chụp, x, y)] -> mỗi người MỘT lần thấy mới nhất. Lần thấy sau nằm trong tầm đi bộ (speed x dt + noise)
    của một người đã biết = chính người đó đã đi tới đây -> thay. Không ai đi tới được = người khác. Trong cùng một
    khung, mỗi người chỉ nhận một hộp. Người ra khỏi khung / Gemini sót giữ lần thấy cuối, vùng có thể có họ nở dần.
    ponytail: gán tham lam theo thứ tự; hai người đi sát nhau có thể bị gộp - khi đó vẫn còn người kia ở gần."""
    tracks = []
    for ts, x, y in sorted(sightings):
        free = [k for k, (tt, tx, ty) in enumerate(tracks)
                if tt < ts and math.dist((tx, ty), (x, y)) <= speed * (ts - tt) + noise]
        if free:
            k = min(free, key=lambda k: math.dist(tracks[k][1:], (x, y)))
            tracks[k] = (ts, x, y)
        else:
            tracks.append((ts, x, y))
    return tracks


def pick_drop(center, cands, people, clear, cur=None, margin=0.3, speed=0.0):
    """Điểm thả NÉ NGƯỜI: trong đám cháy, cách mọi người >= clear m, gần tâm lửa nhất. None = chưa có chỗ nào đủ
    xa (giữ vị trí, không thả). Không có ai: tâm lửa như cũ.
    people [(x, y, tuổi)]: tuổi = từ lúc CHỤP khung thấy người tới lúc bóng chạm đất (s). Người có thể đã đi
    `speed` x tuổi m theo hướng bất kỳ: khoảng cách tính trừ đi quãng đó. Sim 16/11/2026: người ra khỏi khung
    (5 m chỉ thấy ±1.65 m chiều hẹp), quét báo 0 người, bản cũ quên sau 6 s -> người đi 0.3 m/s quay lại, bóng
    rơi cách 0.81 m. Quét 0 người chỉ chứng minh TRONG KHUNG không có ai, không chứng minh người đã đi xa.
    Giữ điểm đang dùng `cur` chừng nào nó còn cách người >= clear; chọn điểm MỚI thì đòi clear + margin - người
    đi sát ngưỡng không làm drone đổi điểm qua lại mỗi vòng lặp."""
    if not people:
        return center
    gap = lambda q: min(math.dist(q, h[:2]) - speed * h[2] for h in people)  # noqa: E731
    if cur is not None and gap(cur) >= clear:
        return cur
    ok = [q for q in [center] + list(cands) if gap(q) >= clear + margin]
    return min(ok, key=lambda q: math.dist(q, center)) if ok else None


def drop_height(drop_alt, search_alt, saw_person):
    """Độ cao thả trên mặt đất. Nhiệm vụ đã thấy người: KHÔNG hạ, thả từ độ cao tìm (user 16/11/2026). Camera
    nhìn thẳng xuống chỉ thấy người trong ~0.39 x (độ cao - 0.9 m) theo chiều hẹp khung: thả 3 m chỉ thấy người
    cách ~0.6 m (sim: người đứng cách 1.1 m lọt khung, Gemini báo 0 người, bóng rơi cách người 1.11 m); 5 m thấy
    ~1.5 m = person_clear. Đổi lại bóng rơi xa hơn nên lệch hơn một chút."""
    return max(drop_alt, search_alt) if saw_person else drop_alt


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
        # Thả cách đóm lửa bao nhiêu m. Đo 18/10/2026: ở 100 m mã ArUco
        # còn ~11 px, PnP lệch 1.8 m ngang, 9 m dọc; ở
        # 40 m lệch 0.4 m. Nên tìm ở cao bao nhiêu cũng được, thả thì hạ về đây.
        self.drop_alt = p('drop_alt', 20.0)
        self.drop_tol = p('drop_tol', 0.3)          # lệch ngang tối đa lúc thả (m)
        self.descend_r = p('descend_radius', 2.0)   # lệch ngang hơn mức này thì chưa hạ
        # THẢ NÉ NGƯỜI (user 16/11/2026): điểm bóng chạm đất phải cách mọi người đã thấy (trừ quãng họ có thể đi
        # person_speed m/s kể từ lúc chụp, xem pick_drop) ít nhất person_clear m. Người đã thấy nhớ person_memory s
        # (cả lúc họ ra khỏi khung) - 120 s > person_wait: trong lúc chờ thả không bao giờ quên ai.
        # ít nhất person_clear m - dời điểm thả trong đám cháy (/fire/area), không có chỗ thì giữ vị trí chờ tới
        # person_wait giây rồi về mà KHÔNG thả. Người từ /fire/persons (fire_locator <- run_gui YOLO + Gemini).
        self.person_clear = p('person_clear', 1.5)
        self.person_memory = p('person_memory', 120.0)
        # Đi bộ thường 1.0-1.4 m/s; sim actor 0.31. 0.5 = đi chậm/quanh quẩn. Với trễ Gemini ~2-4 s + rơi ~1 s
        # người đi nhanh hơn thì không né bằng vị trí được: drone chờ rồi về KHÔNG thả (person_wait).
        self.person_speed = p('person_speed', 0.5)
        # Chỉ người thấy trong person_zone m quanh tâm lửa mới tính. Sim 16/11/2026 KHÔNG có người: Gemini quét lúc
        # vừa cất cánh ở chỗ nhà báo 5 "người" (càng / bóng drone sát đất) -> bản cũ coi cả nhiệm vụ "có người",
        # vùng có-thể-có-người nở 0.5 m/s không giới hạn, 20 s sau phủ tới đám cháy cách 11 m -> hủy thả.
        self.person_zone = p('person_zone', 6.0)
        self.person_wait = p('person_wait', 60.0)
        # Chỉ thả khi lần QUÉT NGƯỜI cả khung gần nhất (frame 'map_scan' = Gemini; YOLO không thấy người nhìn từ đỉnh
        # đầu nên khung YOLO rỗng không chứng minh được gì) chụp chưa quá person_fresh giây. Đo sim 16/11/2026:
        # Gemini 2-10 s/lần, có lần quá hạn - tin cũ hơn thế thì người đã đi tới chỗ khác. Chưa từng có lần quét
        # nào trong task (không khoá Gemini / mất mạng từ đầu): không chặn, thả như trước khi có tính năng này.
        self.person_fresh = p('person_fresh', 4.0)
        self.people, self.area, self.hold_why, self.scan_t = [], None, None, None
        self.saw_person = False     # đã thấy người QUANH ĐÁM CHÁY (drop_height), xem person_zone
        self.target = None
        self.driving = False                        # node đã arm/lái drone (không chỉ đang chờ chấm)
        self.home = None                            # (x, y) chỗ ARM nếu node tự cất cánh - go_home bay về đây
        self.hint = None                            # (lat, lon) người vận hành chấm
        self.fix, self.fix_t = None, 0.0
        self.vel = TwistStamped()
        self.yaw = 0.0
        self.create_subscription(PointStamped, '/fire/target', self._on_target, 10)
        self.ground = []    # z mặt đất quanh đám cháy, ground_range đo bằng camera 2D
        self.create_subscription(Float32, '/fire/ground_z',
                                 lambda m: self.ground.append(m.data), 10)
        self.create_subscription(String, '/gcs/command', self._on_cmd, 10)
        self.create_subscription(PoseArray, '/fire/persons', self._on_people, 10)
        self.create_subscription(PolygonStamped, '/fire/area', self._on_area, 10)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 self._on_fix, qos_profile_sensor_data)
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
        # Đang chờ người vận hành xác nhận thả: JSON {"id","x","y"}, "" = không hỏi gì.
        self.ask_pub = self.create_publisher(String, '/fire/ask', latched)
        self.ask_pub.publish(String(data=''))
        self.ask_id, self.answer = 0, None

    def _on_target(self, msg):
        self.target = msg

    def _on_people(self, msg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # lúc CHỤP khung, không phải lúc tin tới
        self.people += [(ts, q.position.x, q.position.y) for q in msg.poses]
        if msg.header.frame_id == 'map_scan':
            self.scan_t = max(self.scan_t or 0.0, ts)

    def _on_area(self, msg):
        quad = [(q.x, q.y) for q in msg.polygon.points]
        # Giữ tứ giác RỘNG NHẤT đã thấy: hộp YOLO chỉ ôm một ngọn lửa, xuống thấp thì đám cháy tràn khung bị cắt
        if len(quad) == 4 and (self.area is None or quad_area(quad) > quad_area(self.area)):
            self.area = quad

    def people_now(self, center, ahead=0.0):
        """[(x, y, tuổi)] người đã thấy trong person_memory giây, trong person_zone m quanh `center` (ENU);
        tuổi tính tới `ahead` s sau bây giờ. Có ai thì bật saw_person (dính tới hết nhiệm vụ)."""
        now = time.time()
        self.people = [q for q in self.people if now - q[0] < self.person_memory]
        near = [q for q in self.people if math.dist(q[1:], center) <= self.person_zone]
        self.saw_person = self.saw_person or bool(near)
        return [(x, y, now - ts + ahead) for ts, x, y in person_tracks(near, self.person_speed)]

    def _on_cmd(self, msg):
        ok = parse_confirm(msg.data, self.ask_id)
        if ok is not None and self.answer is None:
            self.answer = ok
            return
        cmd = parse_fire_cmd(msg.data)
        if cmd is None:
            return  # lệnh khác của GUI, không phải việc của node này
        if self.hint is not None:
            return self.notify('Đang có nhiệm vụ: không nhận mới', StatusText.WARNING)
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

    def fc_why(self):
        """Lý do FC chưa sẵn sàng (fc_block) hoặc None. Vị trí = pose < 1 s + GPS có fix < 2 s."""
        now = time.monotonic()
        has_pos = now - self.pose_t < 1.0 and self.fix is not None and now - self.fix_t < 2.0
        return fc_block(self.state.connected, self.state.armed, self.prearm_ok, has_pos)

    def _on_fix(self, msg):
        # CHI giu goi CO fix: sim 07/11/2026 1/9 goi status -1 xen ke (va goi (0, 0) cua MAVROS khac cung domain)
        # -> lay goi cuoi la luc co luc khong vi tri, hint_local ra diem cham xa 17 000 km.
        if msg.status.status >= 0 and (msg.latitude or msg.longitude):
            self.fix, self.fix_t = msg, time.monotonic()

    def notify(self, text, severity=StatusText.INFO):
        """Log + STATUSTEXT lên GCS, chữ CÓ DẤU (user 16/11/2026). Cả câu kèm 'FIRE: ' phải vừa TEXT_MAX byte
        UTF-8 - viết ngắn; dài hơn thì fit_text cắt đuôi. GCS phải đọc byte gốc dạng UTF-8 (GUI_NATIVE
        core/adapters/sik.py text_utf8): pymavlink tự giải mã ASCII, chữ có dấu thành ký tự hỏng."""
        # HAI DÒNG RIÊNG, không gộp: rclpy nhớ mức log theo DÒNG gọi, cùng một dòng
        # lúc info lúc warn là ValueError -> node chết giữa task (đã dính 16/10).
        if severity <= StatusText.WARNING:
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        self.txt_pub.publish(StatusText(severity=severity, text=fit_text('FIRE: ' + text)))

    def set_zoom(self, z):
        if z != self.zoom:
            self.zoom = z
            self.zoom_pub.publish(Float32(data=float(z)))

    def send_sp(self, x, y, z):
        """z = độ cao so với HOME (chỗ ARM) như mọi độ cao trong node này; ra FC mới cộng z0 về hệ gốc EKF."""
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x, y, z + self.z0
        sp.pose.orientation.z = math.sin(self.yaw / 2)
        sp.pose.orientation.w = math.cos(self.yaw / 2)
        self.sp_pub.publish(sp)

    def give_up(self, why):
        """Bỏ nhiệm vụ. Phi công đang lái thì KHÔNG đổi mode - để nguyên cho họ."""
        self.set_zoom(1.0)  # trả góc rộng cho người vận hành, không đụng gì tới bay
        if pilot_has_it(self.state):
            self.get_logger().warning(f'lúc đang: {why}')
            self.notify(f'PHI CÔNG CAN THIỆP ({self.state.mode}): dừng', StatusText.WARNING)
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
            self.send_sp(p.x, p.y, self.alt)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.notify('GIỮ VỊ TRÍ: nhiệm vụ gián đoạn', StatusText.CRITICAL)

    def climb_rtl(self):
        """Leo THẲNG ĐỨNG lại search_alt rồi mới RTL. Cao hơn RTL_ALT thì RTL giữ nguyên độ
        cao hiện tại mà bay thẳng về: ở độ cao thả là cắt ngang cây/nhà lúc đi đã vượt qua."""
        p = self.pose.pose.position
        if self.state.armed and self.fly_to(p.x, p.y, self.search_alt,
                                            timeout=30.0 + self.search_alt) == 'pilot':
            return
        if not self.ensure_mode('RTL', timeout=240.0) and self.state.armed and self.state.mode == 'GUIDED':
            self.notify('KHÔNG VÀO ĐƯỢC RTL: cần phi công', StatusText.CRITICAL)

    def go_home(self, t_rel, close_after=1.5):
        """Sau khi thả (user 16/11/2026, cycle time thấp): leo THẲNG ĐỨNG lên search_alt, bay thẳng về chỗ ARM ở
        search_alt (GUIDED - cùng đường, cùng độ cao lúc đi), LAND tại đó. Thay RTL: RTL leo lên RTL_ALT (sim 15 m)
        rồi hạ chậm - đo sim 16/11 mất ~50 s của chu trình 78 s. Cơ cấu thả đóng `close_after` s sau lúc nhả,
        TRONG LÚC bay (bản cũ đứng chờ 2 s). Không có chỗ ARM (giao điểm lúc đang bay) / bay về quá hạn: RTL.
        True = đã hạ cánh, disarm."""
        pending = [True]

        def close():   # gọi trong vòng bay (stop=): đóng cơ cấu đúng một lần, không dừng bay
            if pending[0] and time.time() - t_rel >= close_after:
                pending[0] = False
                if not self.gripper(GRIP_CLOSE):   # như gạt RC5 về 994
                    self.notify('Lệnh đóng cơ cấu bị từ chối', StatusText.WARNING)
            return False

        p = self.pose.pose.position
        if self.fly_to(p.x, p.y, self.search_alt, tol=0.5, timeout=30.0 + self.search_alt, stop=close) == 'pilot':
            return False
        if self.home is None:
            res = 'none'
        else:
            self.notify(f'Về chỗ ARM ở {self.search_alt:.0f} m rồi hạ cánh')
            d = math.dist(self.home, (self.pose.pose.position.x, self.pose.pose.position.y))
            # hạn = 2 s/m: drone thật WP_SPD ~1 m/s
            res = self.fly_to(*self.home, self.search_alt, tol=0.5, timeout=30.0 + 2.0 * d, stop=close)
        if pending[0]:   # về tới nơi trước close_after (chỗ ARM ngay dưới đám cháy): chờ nốt rồi đóng
            self.wait_until(lambda: close() or not pending[0], close_after + 1.0)
        if res == 'pilot':
            return False
        if res != 'ok':
            self.notify('Không về được chỗ ARM: RTL' if self.home else 'Quay về (RTL)', StatusText.WARNING)
            self.climb_rtl()
            return self.wait_until(lambda: not self.state.armed, 240.0)
        return self.land(timeout=120.0)

    def dist_to(self, x, y, z):
        p = self.pose.pose.position
        return math.dist((p.x, p.y, self.alt), (x, y, z))

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
            self.notify(f'Chưa phát hiện: tìm lại ở zoom x{z:.0f}')
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
        Chưa có số nào thì coi ngang chỗ cất cánh (z = 0): đất dốc/mái nhà là sai.
        ground_range gửi z hệ gốc EKF -> trừ z0 về độ cao so với chỗ ARM (sàn 0 của ground_filter = chỗ ARM)."""
        return ground_filter([g - self.z0 for g in self.ground])

    def hold_over_fire(self, settle=2.0, timeout=90.0):
        """Tới ngay trên đóm lửa, hạ thẳng đứng xuống drop_alt trên mặt đất chỗ đó (mục tiêu
        và số đo mặt đất vẫn được tinh chỉnh khi hạ) tới khi lệch < drop_tol, đúng độ cao,
        gần như đứng yên `settle` giây. Có người quanh đó: dời điểm thả / chờ, xem pick_drop."""
        self.ground, self.hold_why = [], None
        try:
            return self._hold(settle, timeout)
        finally:
            self.ground_on.publish(Bool(data=False))

    def _hold(self, settle, timeout):
        end, since, measuring = time.time() + timeout, None, False
        zoom0, h0 = self.zoom, self.alt - self.fire_z()
        cands = area_grid(self.area) if self.area else []   # đám cháy như lúc còn thấy trọn (độ cao tìm)
        aim, told, wait0, park, high = None, None, None, None, self.saw_person
        while rclpy.ok() and time.time() < end:
            t, p, fz = self.target.point, self.pose.pose.position, self.fire_z()
            dh = drop_height(self.drop_alt, self.search_alt, self.saw_person)
            if self.saw_person and not high and dh > self.drop_alt:   # thấy người giữa lúc hạ: lên lại
                high = True
                self.notify(f'CÓ NGƯỜI: thả từ độ cao {dh:.0f} m', StatusText.WARNING)
            # tuổi tính tới lúc bóng CHẠM ĐẤT nếu nhả NGAY: rơi tự do dh m + trễ cơ cấu. Không cộng `settle`: phép
            # kiểm chạy lại mỗi vòng tới đúng lúc nhả (sim 16/11: cộng thừa 2 s x 0.5 m/s = vùng cấm rộng thêm 1 m,
            # không bao giờ thả được)
            people = self.people_now((t.x, t.y), ahead=math.sqrt(2 * dh / 9.81) + 0.3)
            aim = pick_drop((t.x, t.y), cands, people, self.person_clear, aim, speed=self.person_speed)
            stale = self.scan_t is not None and time.time() - self.scan_t > self.person_fresh
            say = ('wait' if aim is None else 'stale' if stale else
                   'move' if people and math.dist(aim, (t.x, t.y)) > 0.3 else None)
            if say and say != told:
                self.notify({'wait': 'CÓ NGƯỜI: chờ điểm thả an toàn', 'stale': 'Chờ dữ liệu dò người'}.get(say) or
                            f'CÓ NGƯỜI: dời điểm thả {math.dist(aim, (t.x, t.y)):.1f} m', StatusText.WARNING)
            told = say
            if aim is None or stale:   # chưa được thả: đếm giờ chờ, quá person_wait thì về mà KHÔNG thả
                wait0 = wait0 or time.time()
                if time.time() - wait0 > self.person_wait:
                    self.hold_why = 'Có người: hủy thả' if aim is None else 'Mất dò người: hủy thả'
                    return None
            elif wait0:   # thời gian chờ không tính vào hạn canh tâm
                end, wait0 = end + time.time() - wait0, None
            if aim is None:
                # Không còn chỗ nào trong đám cháy đủ xa người: đứng yên tại chỗ, giữ độ cao
                park, since = park or (p.x, p.y, self.alt), None
                self.send_sp(*park)
                rclpy.spin_once(self, timeout_sec=0.1)
                if pilot_has_it(self.state):
                    return None
                continue
            park = None
            t = PointStamped().point
            t.x, t.y = aim
            self.set_zoom(descent_zoom(zoom0, h0, self.alt - fz))
            herr = math.hypot(p.x - t.x, p.y - t.y)
            if not measuring and herr <= self.descend_r:
                # Bật đo mặt đất khi bắt đầu HẠ thẳng đứng, không phải lúc còn bay ngang tới:
                # bay ngang thì ảnh trôi chứ không nở, số đo đầu sai tới 12 m (18/10/2026).
                measuring = True
                self.ground_on.publish(Bool(data=True))
            z = hold_z(herr, self.alt, fz, dh, self.descend_r)
            self.send_sp(t.x, t.y, z)
            rclpy.spin_once(self, timeout_sec=0.1)
            if pilot_has_it(self.state):
                return None
            p, v = self.pose.pose.position, self.vel.twist.linear
            err = math.hypot(p.x - t.x, p.y - t.y)
            if (err < self.drop_tol and abs(self.alt - fz - dh) < 0.5
                    and math.hypot(v.x, v.y, v.z) < 0.2 and not stale):
                since = since or time.time()
                if time.time() - since >= settle:
                    return err
            else:
                since = None
        return None

    def ask_operator(self):
        """Thấy lửa: TREO tại chỗ đang đứng, chờ người vận hành nhìn video (có hộp lửa) rồi trượt
        xác nhận trên GUI. True = thả, False = không phải lửa, None = phi công giành lái.
        Chờ NGƯỜI nên không có hạn (như chờ điểm chấm); pin yếu thì failsafe pin của FC lo."""
        p, t = self.pose.pose.position, self.target.point
        x, y, z = p.x, p.y, self.alt
        self.ask_id += 1
        self.answer = None
        self.ask_pub.publish(String(data=json.dumps(
            {'id': self.ask_id, 'x': round(t.x, 1), 'y': round(t.y, 1)})))
        self.notify('PHÁT HIỆN CHÁY: chờ xác nhận thả', StatusText.WARNING)
        try:
            res = self.fly_to(x, y, z, tol=-1.0, timeout=float('inf'),
                              stop=lambda: self.answer is not None)
        finally:
            self.ask_pub.publish(String(data=''))   # GUI cất thanh trượt
        return None if res == 'pilot' else self.answer

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
        self.notify('Sẵn sàng nhận mục tiêu')
        # Chờ NGƯỜI, không phải chờ máy - không có hạn là đúng. Kiểm FC Ở ĐÂY chứ không trong _on_cmd:
        # GUI giao điểm ~1 s sau khi bật task, node chưa nhận /mavros/state nào -> 'FC chua noi' nhầm (e2e 07/11).
        while True:
            self.wait_until(lambda: self.hint is not None, float('inf'))
            if not rclpy.ok():
                return False
            why = self.fc_why()
            if not why:
                break
            self.notify(f'Từ chối: {why}', StatusText.WARNING)  # không ARM, không đổi mode: bỏ điểm, chờ điểm mới
            self.hint = None
        hx, hy = self.hint_local()
        p = self.pose.pose.position
        if math.hypot(hx - p.x, hy - p.y) > self.max_range:
            self.notify(f'Từ chối: cự ly {math.hypot(hx - p.x, hy - p.y):.0f} m vượt '
                        f'{self.max_range:.0f} m', StatusText.WARNING)
            return False
        self.notify(f'Mục tiêu ({hx:.0f}, {hy:.0f}) m, độ cao {self.search_alt:.0f} m')
        self.target, self.area = None, None  # mục tiêu cũ (nếu có) không được dẫn đường

        self.driving = True
        # 15 s chứ không 90 s mặc định: FC hỏng prearm giữa chừng thì THÔI, không ngồi đợi lỗi hết rồi tự ARM
        if not self.state.armed and not self.arm(timeout=15.0):
            return self.give_up('ARM không thành công')
        # Home vừa đặt lại lúc ARM: chốt mặt đất TRƯỚC mọi phép so độ cao (self.alt, takeoff, setpoint).
        if not self.latch_ground():
            return self.give_up('Thiếu độ cao tương đối')
        if self.state.mode != 'GUIDED':
            self.set_mode('GUIDED')       # người vận hành vừa giao việc = đã chọn
        if self.alt < 1.0:   # tự cất cánh từ đây = chỗ ARM: go_home về đúng chỗ này. Đang bay sẵn: RTL như cũ
            self.home = (self.pose.pose.position.x, self.pose.pose.position.y)
        # WPNAV_SPEED_UP 2 m/s: 100 m mất ~50 s, hạn mặc định 60 s của takeoff() là sát nút.
        if self.alt < 1.0 and not self.takeoff(self.search_alt, timeout=30.0 + self.search_alt):
            return self.give_up('Cất cánh không thành công')
        q = self.pose.pose.orientation   # giữ nguyên hướng mũi
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        self.notify('Tiếp cận mục tiêu và tìm kiếm')
        if not self.search_around(hx, hy):
            # bị giành lái cũng trả False - phải tách ra, không thì báo nhầm "không thấy"
            return self.give_up('đang tìm kiếm' if pilot_has_it(self.state)
                                else f'Không phát hiện cháy trong {self.search_radius:.0f} m')
        t = self.target.point
        self.notify(f'Phát hiện cháy ({t.x:.1f}, {t.y:.1f}) m, x{self.zoom:g}')
        ok = self.ask_operator()
        if not ok:
            return self.give_up('chờ xác nhận' if ok is None else 'Người vận hành bác bỏ')
        self.people_now((self.target.point.x, self.target.point.y))   # cập nhật saw_person quanh đám cháy
        dh = drop_height(self.drop_alt, self.search_alt, self.saw_person)
        self.notify(f'Xác nhận thả: hạ tới {dh:.0f} m' if dh == self.drop_alt else
                    f'Xác nhận, có người: thả từ {dh:.0f} m')
        # WPNAV_SPEED_DN 1.5 m/s: hạ 100 -> 20 m mất ~55 s, cộng thời gian canh lại tâm.
        err = self.hold_over_fire(timeout=60.0 + self.search_alt)
        if err is None:
            return self.give_up(self.hold_why or 'Chưa ổn định: hủy thả')
        t, fz = self.target.point, self.fire_z()
        if not self.gripper(GRIP_OPEN):
            self.notify('Lệnh mở cơ cấu bị từ chối', StatusText.ERROR)   # MAV_CMD_DO_GRIPPER
        t_rel = time.time()
        self.notify(f'ĐÃ THẢ: sai lệch {err:.2f} m, cao {self.alt - fz:.1f} m')   # cao = trên mặt quanh lửa
        seen = self.people_now((t.x, t.y))
        if seen:
            x, y, age = min(seen, key=lambda h: h[2])
            log.info(f'luc tha: lan thay nguoi gan nhat {age:.1f} s truoc, cach drone '
                     f'{math.hypot(x - self.pose.pose.position.x, y - self.pose.pose.position.y):.1f} m')
        else:
            log.info('luc tha: chua thay nguoi nao trong nhiem vu')
        if not self.ground:
            self.notify('Thiếu số đo mặt đất: mốc ARM', StatusText.WARNING)   # ground_range không ra số (thiếu vân?)
        log.info(f'mat dat: camera 2D {fz:.2f} ({len(self.ground)} so do), PnP ma ArUco {t.z - self.z0:.2f}')
        self.set_zoom(1.0)
        landed = self.go_home(t_rel)
        log.info('ha canh xong, da disarm' if landed else 'chua ha canh / phi cong lai')
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

    # xác nhận thả: chỉ đúng action + ok bool + id CỦA LẦN HỎI NÀY
    c = lambda ok, i: json.dumps({'action': 'fire_confirm', 'args': {'ok': ok, 'id': i}})  # noqa: E731
    assert parse_confirm(c(True, 3), 3) is True and parse_confirm(c(False, 3), 3) is False
    assert parse_confirm(c(True, 2), 3) is None, 'trượt cho lần hỏi trước tới trễ -> không thả'
    assert parse_confirm(ok, 3) is None, 'lệnh fire_target không phải câu trả lời'
    for bad in [c('yes', 3), c(1, 3), c(True, True), c(True, '3'), 'rac', '[]',
                '{"action":"fire_confirm"}', '{"action":"fire_confirm","args":{"ok":true}}']:
        assert parse_confirm(bad, 3) is None, bad

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

    # thả né người: đám lửa giả 3.6 x 2.55 m quanh (0, 0), người đi vòng bán kính 1 m tâm (0.5, 0)
    quad = [(-1.8, -1.275), (1.8, -1.275), (1.8, 1.275), (-1.8, 1.275)]
    assert abs(quad_area(quad) - 3.6 * 2.55) < 1e-9
    grid = area_grid(quad)
    assert len(grid) == 49 and all(abs(x) <= 1.8 * 0.7 + 1e-9 and abs(y) <= 1.275 * 0.7 + 1e-9 for x, y in grid)
    assert pick_drop((0.0, 0.0), grid, [], 1.5) == (0.0, 0.0), 'không có ai: thả giữa đám cháy như cũ'
    assert pick_drop((0.0, 0.0), grid, [(3.5, 0.0, 0)], 1.5) == (0.0, 0.0), 'người ở xa: vẫn thả giữa'
    d = pick_drop((0.0, 0.0), grid, [(0.5, 0.0, 0)], 1.5)
    assert d in grid and math.dist(d, (0.5, 0.0)) >= 1.8 and d[0] < 0, ('người gần tâm: dời sang phía bên kia', d)
    assert pick_drop((0.0, 0.0), grid, [(0.5, 0.0, 0), (d[0] + 1.6, d[1], 0)], 1.5, cur=d) == d, 'còn >= clear: giữ'
    assert pick_drop((0.0, 0.0), grid, [(0.5, 0.0, 0), (d[0] + 1.0, d[1], 0)], 1.5, cur=d) != d, 'người lại gần: đổi'
    crowd = [(x, y, 0) for x in (-1.5, 0.0, 1.5) for y in (-1.0, 1.0)]
    assert pick_drop((0.0, 0.0), grid, crowd, 1.5) is None, 'kín người: không thả'
    assert pick_drop((0.0, 0.0), [], [(0.5, 0.0, 0)], 1.5) is None, 'không biết đám cháy rộng bao nhiêu: chờ, không dời bừa'
    # người thấy 11 s trước cách 1.2 m rồi ra khỏi khung (sim 16/11: bản cũ quên, bóng rơi cách người 0.81 m):
    # đi 0.5 m/s thì giờ có thể ở bất kỳ đâu trong 5.5 m -> không có điểm nào trong đám cháy an toàn
    assert pick_drop((0.0, 0.0), grid, [(1.2, 0.0, 11.0)], 1.5, speed=0.5) is None, 'người ra khỏi khung: không quên'
    assert pick_drop((0.0, 0.0), grid, [(1.2, 0.0, 11.0), (2.5, 0.0, 1.0)], 1.5, speed=0.5) is None, 'lần thấy cũ vẫn tính'
    # gộp lần thấy: người đi vòng rồi ra khỏi khung -> còn MỘT người, lần thấy cuối; thấy lại ở xa -> thay
    walk = [(t, 0.3 * t, 0.0) for t in range(5)]                          # 0.3 m/s theo x, thấy mỗi giây
    assert person_tracks(walk, 0.5) == [(4, 1.2, 0.0)]
    assert person_tracks(walk + [(4, -2.0, 0.0)], 0.5) == [(4, 1.2, 0.0), (4, -2.0, 0.0)], 'cùng khung: hai người'
    assert person_tracks(walk + [(10, 3.4, 0.0)], 0.5) == [(10, 3.4, 0.0)], 'thấy lại sau 6 s, đi tới được: cùng người'
    # chỉ người quanh đám cháy: báo nhầm lúc cất cánh ở nhà (sim 16/11, cách lửa 11 m) không chặn thả, không bật cờ
    now = time.time()
    fake = S(people=[(now - 20, 0.6, 0.1), (now - 20, 0.2, 0.5)], person_memory=120.0, person_zone=6.0,
             person_speed=0.5, saw_person=False)
    assert FireMission.people_now(fake, (-4.0, 10.0)) == [] and not fake.saw_person, 'người ở nhà, lửa cách 11 m'
    fake.people.append((now - 1, -3.0, 10.5))
    near = FireMission.people_now(fake, (-4.0, 10.0), ahead=1.0)
    assert len(near) == 1 and near[0][:2] == (-3.0, 10.5) and abs(near[0][2] - 2.0) < 0.1 and fake.saw_person
    far = pick_drop((0.0, 0.0), grid, [(2.5, 0.0, 1.0)], 1.5, speed=0.5)
    assert far is not None and math.dist(far, (2.5, 0.0)) - 0.5 >= 1.5, 'thấy mới, đủ xa (tính cả quãng đi): thả được'
    assert drop_height(3.0, 5.0, False) == 3.0 and drop_height(3.0, 5.0, True) == 5.0, 'có người: thả từ độ cao tìm'
    assert drop_height(5.0, 5.0, True) == 5.0
    for w in ('CÓ NGƯỜI: chờ điểm thả an toàn', 'CÓ NGƯỜI: dời điểm thả 99.9 m', 'Có người: hủy thả -> RTL',
              'CÓ NGƯỜI: thả từ độ cao 120 m', 'Xác nhận, có người: thả từ 120 m',
              'Chờ dữ liệu dò người', 'Mất dò người: hủy thả -> RTL', 'Về chỗ ARM ở 120 m rồi hạ cánh',
              'Không về được chỗ ARM: RTL'):
        assert fit_text('FIRE: ' + w) == 'FIRE: ' + w, w

    pts = orbit_points(18, 12, 8)
    assert all(abs(math.hypot(x - 18, y - 12) - 8) < 1e-9 for x, y in pts)
    assert pts[0] == pts[-1] or math.dist(pts[0], pts[-1]) < 1e-9, 'vòng phải khép kín'
    assert max(math.dist(a, b) for a, b in zip(pts, pts[1:])) <= 6.0 + 1e-9
    # FC chưa sẵn sàng -> từ chối điểm cháy (07/11/2026 15:18: GPS Bad fix + Radio failsafe)
    assert fc_block(False, False, True, True) == 'FC chưa kết nối'
    assert fc_block(True, False, True, False) == 'chưa có định vị GPS'
    assert fc_block(True, False, False, True) == 'PreArm chưa đạt'
    assert fc_block(True, False, None, True) == 'chưa rõ trạng thái PreArm'
    assert fc_block(True, False, True, True) is None
    assert fc_block(True, True, False, True) is None, 'da ARM (diem moi giua task): chi can vi tri'
    assert fc_block(True, True, None, False) == 'chưa có định vị GPS'
    # chữ có dấu: đo theo BYTE. Các câu dài nhất với số xấu nhất phải tới GCS nguyên vẹn
    for w in ('Từ chối: chưa rõ trạng thái PreArm', 'Đang có nhiệm vụ: không nhận mới', 'Tiếp cận mục tiêu và tìm kiếm',
              'ĐÃ THẢ: sai lệch 0.30 m, cao 120.0 m', 'PHI CÔNG CAN THIỆP (STABILIZE): dừng', 'GIỮ VỊ TRÍ: nhiệm vụ gián đoạn',
              'ĐÁNH GIÁ: còn cháy (999 khung hình)', 'ĐÁNH GIÁ: không còn phát hiện cháy', 'PHÁT HIỆN CHÁY: chờ xác nhận thả',
              'Mục tiêu (-300, -300) m, độ cao 120 m', 'Phát hiện cháy (-300.0, -300.0) m, x5',
              pick_alts(120.5, None, 2.0, 120.0, 20.0, 20.0), pick_alts(120.0, 121.0, 5.0, 120.0, 20.0, 20.0),
              pick_radius(62.5, 2.0, 50.0), 'Không phát hiện cháy trong 50 m -> RTL', 'Cất cánh không thành công -> RTL'):
        assert fit_text('FIRE: ' + w) == 'FIRE: ' + w, (w, len(('FIRE: ' + w).encode()))
    cut = fit_text('FIRE: ' + 'ửa' * 30)
    assert len(cut.encode()) <= TEXT_MAX and cut.encode().decode() == cut and cut.endswith(('ử', 'a')), 'cắt giữa ký tự'
    print('fire_mission selfcheck OK')


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
