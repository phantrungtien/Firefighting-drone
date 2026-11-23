"""Thử 2.3 phi công giành lái trên SITL (không Gazebo). Giả GUI (/gcs/command), giả lửa (/fire/target),
giả đo mặt đất (/fire/ground_z); giành lái = đổi LOITER + RC ga giữa, như gạt công tắc RC thật.
python3 takeover_test.py none|search|descend|home"""
import math, sys, threading, time, json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float32, Bool
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import SetMode
from pymavlink import mavutil
from fire_drone.fire_mission import ll_to_enu

SC = sys.argv[1]
NORTH = 6.0   # điểm chấm cách chỗ ARM 6 m về bắc


class Mav(threading.Thread):
    """Đọc thẳng SITL cổng 5762: mode thật + SERVO16 (gripper), không qua MAVROS."""
    def __init__(self):
        super().__init__(daemon=True)
        self.m = mavutil.mavlink_connection('tcp:127.0.0.1:5762', source_system=251)
        self.mode, self.s16, self.opened = '?', 0, None

    def run(self):
        self.m.wait_heartbeat()
        self.m.mav.request_data_stream_send(1, 1, mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1)
        while True:
            x = self.m.recv_match(type=['HEARTBEAT', 'SERVO_OUTPUT_RAW'], blocking=True, timeout=1)
            if x is None:
                continue
            if x.get_type() == 'HEARTBEAT' and x.get_srcComponent() == 1:
                self.mode = mavutil.mode_string_v10(x)
            elif x.get_type() == 'SERVO_OUTPUT_RAW' and x.port == 0:   # MAVLink2: kênh 9-16 nằm trong port 0
                self.s16 = x.servo16_raw
                if self.s16 > 1500 and self.opened is None:
                    self.opened = time.time()


class Fake(Node):
    def __init__(self):
        super().__init__('takeover_test')
        self.state, self.pose, self.fix = State(), None, None
        self.ground_on, self.z_arm, self.t_ground = False, None, None
        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 lambda m: setattr(self, 'pose', m), qos_profile_sensor_data)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 lambda m: setattr(self, 'fix', m), qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/fire/ground_enable', self._ge, latched)
        self.cmd = self.create_publisher(String, '/gcs/command', 10)
        self.tgt = self.create_publisher(PointStamped, '/fire/target', 10)
        self.gz = self.create_publisher(Float32, '/fire/ground_z', 10)
        self.rc = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.setmode = self.create_client(SetMode, '/mavros/set_mode')

    def _ge(self, m):
        self.ground_on = m.data
        if m.data and self.t_ground is None:
            self.t_ground = time.time()

    def spin(self, s):
        end = time.time() + s
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def log(t0, *a):
    print(f'[{time.time() - t0:6.1f}s]', *a, flush=True)


def main():
    rclpy.init()
    n, mav = Fake(), Mav()
    mav.start()
    t0 = time.time()
    while not (n.fix and n.pose and n.state.connected and n.fix.status.status >= 0):
        n.spin(0.5)
    n.spin(25)   # EKF/GPS SITL ổn định, fire_mission "Sẵn sàng"
    home_ll = (n.fix.latitude, n.fix.longitude)
    hint = (home_ll[0] + NORTH / 111320.0, home_ll[1])
    n.cmd.publish(String(data=json.dumps({'action': 'fire_target', 'args': {
        'lat': hint[0], 'lon': hint[1], 'alt': 5.0, 'drop': 3.0, 'radius': 2.0}})))
    log(t0, 'giao diem chay', SC)
    hx = hy = None
    took, t_took, last = None, None, ''
    while time.time() - t0 < 240:
        n.spin(0.2)
        p = n.pose.pose.position
        if n.state.armed and n.z_arm is None:
            n.z_arm = p.z
        if hx is None and n.fix:
            e, nn = ll_to_enu(*hint, n.fix.latitude, n.fix.longitude)
            hx, hy = p.x + e, p.y + nn
        alt = p.z - (n.z_arm or 0.0)
        near = math.hypot(p.x - hx, p.y - hy) < 3.0
        if SC != 'search' and near and alt > 2.0:   # "camera thấy lửa" khi tới gần
            m = PointStamped()
            m.header.frame_id = 'map'
            m.header.stamp = n.get_clock().now().to_msg()
            m.point.x, m.point.y, m.point.z = hx, hy, n.z_arm or 0.0
            n.tgt.publish(m)
        if n.ground_on and n.z_arm is not None:
            n.gz.publish(Float32(data=float(n.z_arm)))
        s = f'mode {mav.mode:8s} armed {n.state.armed} alt {alt:4.1f} s16 {mav.s16}'
        if s != last and int(time.time() * 5) % 10 == 0:
            log(t0, s)
            last = s
        # khi nao gianh lai
        trig = {'search': n.state.mode == 'GUIDED' and alt > 4.5 and near,
                'descend': n.t_ground is not None and time.time() - n.t_ground > 1.5,
                'home': mav.opened is not None and time.time() - mav.opened > 1.0}.get(SC, False)
        if trig and took is None:
            took, t_took = (mav.opened is not None), time.time()
            n.setmode.call_async(SetMode.Request(custom_mode='LOITER'))
            log(t0, f'>>> PHI CONG GIANH LAI (LOITER) - pha {SC}, da tha truoc do: {took}')
        if t_took:
            rc = OverrideRCIn()
            rc.channels = [65535] * 18
            rc.channels[2] = 1500   # ga giữa: LOITER giữ độ cao
            n.rc.publish(rc)
            if time.time() - t_took > 20:
                break
        if SC == 'none' and not n.state.armed and mav.opened:
            break
    # ket qua
    log(t0, f'KET QUA {SC}: mode cuoi {mav.mode}, armed {n.state.armed}, '
            f'tha bong {"CO" if mav.opened else "KHONG"}'
            + (f' ({mav.opened - t_took:+.1f}s so voi luc gianh)' if mav.opened and t_took else ''))
    ok = {'none': mav.opened is not None and not n.state.armed,
          'search': mav.opened is None and mav.mode == 'LOITER',
          'descend': mav.opened is None and mav.mode == 'LOITER',
          'home': mav.mode == 'LOITER'}[SC]
    log(t0, 'DAT' if ok else 'KHONG DAT')
    # tra ve dat cho lan sau
    n.setmode.call_async(SetMode.Request(custom_mode='LAND'))
    n.spin(1)


main()
