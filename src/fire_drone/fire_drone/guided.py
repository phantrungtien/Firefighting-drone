"""GuidedNode: GUIDED -> arm -> takeoff -> land qua MAVROS. Mọi vòng chờ đều có hạn.

Rút gọn từ simtofly_mavros_sitl/guided_base.py (đã bay thật trong SITL).
Nhiệm vụ cứu hỏa sau này kế thừa GuidedNode; chạy file này = bài thử cất cánh:

    ros2 run fire_drone takeoff_test --ros-args -p alt:=5.0
    python3 guided.py --selfcheck      # kiểm cổng arm, không cần ROS chạy
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, SysStatus
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# MAV_SYS_STATUS_PREARM_CHECK - mavros_msgs không có hằng số tên này.
PREARM_CHECK_BIT = 1 << 28


def arm_gate(prearm_ok, waited, blind_after):
    """Được gửi lệnh arm chưa?

    True  -> prearm đã qua, gửi.  False -> chưa qua, KHÔNG gửi (mỗi lệnh arm bị
    từ chối là một ack nữa; dồn dập thì MAVROS dính lỗi std::future_error và chết).
    None  -> chưa có sys_status: chờ blind_after giây rồi thử mù.
    """
    if prearm_ok is None:
        return waited >= blind_after
    return prearm_ok


def mode_retry_ok(state, start_mode):
    """ensure_mode còn được gửi lại không: còn bay VÀ mode vẫn là mode lúc bắt đầu (chưa ai đổi)."""
    return state.armed and state.mode == start_mode


class GuidedNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.state = State()
        self.pose = PoseStamped()
        self.pose_t = 0.0           # monotonic lúc nhận pose cuối (pose cũ = EKF mất vị trí / MAVROS chết)
        self.prearm_ok = None
        qos = qos_profile_sensor_data  # MAVROS publish BEST_EFFORT
        self.create_subscription(State, '/mavros/state', self._set('state'), qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, qos)
        self.create_subscription(SysStatus, '/mavros/sys_status', self._sys_cb, qos)
        self.arm_cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_cli = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.rate_cli = self.create_client(StreamRate, '/mavros/set_stream_rate')

    def _set(self, attr):
        return lambda msg: setattr(self, attr, msg)

    def _on_pose(self, msg):
        self.pose, self.pose_t = msg, time.monotonic()

    def _sys_cb(self, msg):
        # ArduPilot bật bit này đúng lúc mọi prearm check qua.
        if msg.sensors_present & PREARM_CHECK_BIT:
            self.prearm_ok = bool(msg.sensors_health & PREARM_CHECK_BIT)

    @property
    def alt(self):
        return self.pose.pose.position.z

    def _call(self, cli, req):
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'không thấy service {cli.srv_name}')
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.result()

    def wait_until(self, cond, timeout):
        end = time.time() + timeout
        while rclpy.ok() and time.time() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def request_streams(self, hz=10):
        """Xin FC gửi mọi luồng MAVLink. Pixhawk thật 23/10/2026 để MAVn_* = 0: có HEARTBEAT,
        /mavros/state connected, nhưng KHÔNG pose/IMU/GPS - task chờ pose rồi bỏ cuộc mà không
        rõ vì sao. REQUEST_DATA_STREAM chỉ sống tới lần FC khởi động lại nên gọi mỗi lần nối.
        SITL đã có luồng sẵn, gọi thêm vô hại."""
        self._call(self.rate_cli, StreamRate.Request(stream_id=0, message_rate=hz, on_off=True))

    def set_mode(self, mode):
        r = self._call(self.mode_cli, SetMode.Request(custom_mode=mode))
        return bool(r and r.mode_sent)

    def ensure_mode(self, mode, timeout=240.0, every=2.0):
        """set_mode tới khi /mavros/state XÁC NHẬN `mode`. Gọi một lần có thể rơi: sim 07/11/2026
        'không thấy service /mavros/set_mode' lúc vào RTL -> drone treo GUIDED 4 phút rồi task bỏ.
        Thôi thử khi người khác đã đổi mode (phi công giành lái) hoặc đã disarm: không giằng lái."""
        start, end, last = self.state.mode, time.time() + timeout, 0.0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.mode == mode:
                return True
            if not mode_retry_ok(self.state, start):
                return False
            if time.time() - last >= every:
                if last:
                    self.get_logger().warning(f'{mode} chua vao (dang {self.state.mode}), thu lai')
                last = time.time()
                self.set_mode(mode)
        return False

    def arm(self, mode='GUIDED', timeout=90.0, blind_after=15.0):
        """Chờ prearm qua + đúng mode (xác nhận qua /mavros/state), rồi arm."""
        start = time.time()
        last_try = 0.0
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state.armed:
                return True
            if time.time() - last_try < 2.0:
                continue
            if not arm_gate(self.prearm_ok, time.time() - start, blind_after):
                continue
            last_try = time.time()
            # mode_sent chỉ là FCU nhận YÊU CẦU; GUIDED bị từ chối khi EKF chưa
            # có vị trí. Arm lúc đó = arm trong STABILIZE, setpoint vô tác dụng.
            if self.state.mode != mode:
                self.set_mode(mode)
                continue
            r = self._call(self.arm_cli, CommandBool.Request(value=True))
            if r and r.success:
                return True
        return False

    def takeoff(self, alt, tol=0.5, timeout=60.0):
        r = self._call(self.takeoff_cli, CommandTOL.Request(altitude=float(alt)))
        if not (r and r.success):
            return False
        return self.wait_until(lambda: abs(self.alt - alt) < tol, timeout)

    def land(self, timeout=90.0):
        self.ensure_mode('LAND', timeout)
        return self.wait_until(lambda: not self.state.armed, timeout)


def main():
    rclpy.init()
    node = GuidedNode('takeoff_test')
    alt = node.declare_parameter('alt', 5.0).value
    log = node.get_logger()
    ok = False
    try:
        connected = node.wait_until(lambda: node.state.connected, 60.0)
        if connected:
            node.request_streams()
        if not connected:
            log.error('MAVROS chưa connected sau 60 s')
        # Pose chỉ có sau khi EKF đặt origin (cần GPS): SITL lạnh ~20-40 s,
        # ngoài trời có thể lâu hơn.
        elif not node.wait_until(lambda: node.pose.header.stamp.sec > 0, 120.0):
            log.error('không có /mavros/local_position/pose - thiếu luồng MAVLink hoặc chưa có GPS?')
        elif not node.arm():
            log.error('arm thất bại')
        elif not node.takeoff(alt):
            log.error(f'không lên được {alt} m (đang {node.alt:.2f} m)')
        else:
            log.info(f'đã lên {node.alt:.2f} m - treo 5 s rồi LAND')
            node.wait_until(lambda: False, 5.0)
            ok = node.land()
            log.info('đã hạ cánh và disarm' if ok else 'LAND quá hạn')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


def _selfcheck():
    assert arm_gate(True, 0, 15) and arm_gate(True, 99, 15)
    assert not arm_gate(False, 600, 15), 'chờ lâu không phải lý do bỏ qua prearm'
    assert not arm_gate(None, 14.9, 15) and arm_gate(None, 15, 15)
    assert 1467088303 & PREARM_CHECK_BIT  # sensors_health thật của ArduPilot đã sẵn sàng
    # ensure_mode: lần gọi đầu rơi (service vắng) -> thử lại tới khi state báo RTL
    from types import SimpleNamespace as NS
    calls = []

    def fake_set(mode):
        calls.append(mode)
        if len(calls) >= 3:
            fake.state.mode = mode
    fake = NS(state=NS(mode='GUIDED', armed=True), set_mode=fake_set,
              get_logger=lambda: NS(warning=lambda *_: None))
    ok, spin = rclpy.ok, rclpy.spin_once
    rclpy.ok, rclpy.spin_once = (lambda: True), (lambda *a, **k: None)
    try:
        assert GuidedNode.ensure_mode(fake, 'RTL', timeout=2.0, every=0.0) and calls == ['RTL'] * 3
        fake.state.mode, calls[:] = 'GUIDED', []
        fake.set_mode = lambda m: setattr(fake.state, 'mode', 'POSHOLD')  # phi công gạt sang POSHOLD
        assert not GuidedNode.ensure_mode(fake, 'RTL', timeout=2.0, every=0.0)
        assert fake.state.mode == 'POSHOLD', 'không giằng lại lái'
        fake.state.mode, fake.state.armed = 'GUIDED', False
        assert not GuidedNode.ensure_mode(fake, 'RTL', timeout=2.0, every=0.0)  # đã disarm: thôi
    finally:
        rclpy.ok, rclpy.spin_once = ok, spin
    print('guided selfcheck OK')


if __name__ == '__main__':
    _selfcheck() if '--selfcheck' in sys.argv else main()
