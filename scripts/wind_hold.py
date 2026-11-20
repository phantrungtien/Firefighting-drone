#!/usr/bin/env python3
"""Giữ vị trí (POSHOLD) ở 20 m trong gió cấp 0-5 Beaufort - drone THẬT (models/firehawk_x8 + tham số FC thật
src/fire_drone/config/firehawk_x8.parm), mang bóng 0.5 kg. In bảng so sánh, ghi logs/wind_hold_<giờ>/.

    source setup.bash && python3 scripts/wind_hold.py            # cấp 0 1 2 3 4 5, Gazebo có hình
    python3 scripts/wind_hold.py --levels 0 3 --measure 30       # thử nhanh
    ./scripts/run_sim.sh stop                                     # tắt sim sau khi xem xong

Chỉ Gazebo + SITL (không MAVROS/GUI), lệnh bằng pymavlink cổng 5760 - như ~/firedrop_sim/scripts/fly_hover.py.
POSHOLD như drone thật bay 29/10 (log129), cần giữ giữa bằng RC_CHANNELS_OVERRIDE (SITL không có tay RC).

Gió (m/s ở độ cao bay, hướng cố định) = trung bình cấp Beaufort + giật Dryden tầm thấp MIL-F-8785C ở 20 m:
sigma_u = sigma_v = 0.18 V, L_u = 116 m, L_v = 58 m, Gauss-Markov bậc 1 tau = L/V. Cùng seed cho mọi cấp.
Lực gió lên thân: plugin WindDrag (xem models/firehawk_x8/model.sdf). Mọi khoảng thời gian tính theo GIỜ SIM.
"""
import argparse
import json
import math
import os
import pathlib
import random
import re
import statistics
import subprocess
import sys
import threading
import time

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
from gz.msgs10.clock_pb2 import Clock  # noqa: E402
from gz.msgs10.pose_pb2 import Pose  # noqa: E402
from gz.msgs10.vector3d_pb2 import Vector3d  # noqa: E402
from gz.transport13 import Node  # noqa: E402
from pymavlink import mavutil  # noqa: E402

WS = pathlib.Path(__file__).resolve().parent.parent
BEAUFORT = {0: (0.0, 0.0, 0.2), 1: (0.9, 0.3, 1.5), 2: (2.5, 1.6, 3.3), 3: (4.4, 3.4, 5.4),
            4: (6.7, 5.5, 7.9), 5: (9.4, 8.0, 10.7), 6: (12.3, 10.8, 13.8), 7: (15.5, 13.9, 17.1),
            8: (19.0, 17.2, 20.7)}   # cấp: (giữa dải dùng để thử, thấp, cao) m/s
PREARM = 0x10000000


def make_world():
    w = (WS / 'worlds/firedrop_forest.sdf').read_text()
    for a, b in (('model://firehawk_x4', 'model://firehawk_x8'), ('model://elide_fireball', 'model://fireball_05kg')):
        assert a in w, a
        w = w.replace(a, b)
    # gió nền của world = 0: LiftDrag của cánh TRỪ gió world, WindDrag đã lo phần gió (không tính hai lần)
    w, n = re.subn(r'<wind>\s*<linear_velocity>[^<]*</linear_velocity>', '<wind>\n      <linear_velocity>0 0 0</linear_velocity>', w)
    assert n == 1
    p = WS / 'logs/wind_world.sdf'
    p.write_text(w)
    return p


def port_open(port):
    out = subprocess.run(['ss', '-ltun'], capture_output=True, text=True).stdout
    return re.search(rf':{port}\b', out) is not None


def wait_port(port, timeout):
    t = time.time()
    while time.time() - t < timeout:
        if port_open(port):
            return True
        time.sleep(1)
    return False


class Gust:
    """Gió ngang (E, N) = V * hướng + giật Dryden (Gauss-Markov bậc 1) dọc/ngang hướng gió."""

    def __init__(self, seed, frm_deg):
        self.rng = random.Random(seed)
        self.u = self.v = 0.0
        to = math.radians(frm_deg + 180.0)       # thổi TỚI hướng này (độ từ Bắc, theo chiều kim đồng hồ)
        self.d = (math.sin(to), math.cos(to))     # (E, N)
        self.V = 0.0

    def step(self, dt, V, turb):
        if V > 0.05:
            for k, L in (('u', 116.0), ('v', 58.0)):
                a = math.exp(-dt * V / L)
                x = getattr(self, k) * a + turb * V * math.sqrt(1 - a * a) * self.rng.gauss(0, 1)
                setattr(self, k, x)
        else:
            self.u = self.v = 0.0
        along = V + self.u
        de, dn = self.d
        return along * de - self.v * dn, along * dn + self.v * de


class Truth:
    """Gazebo: giờ sim, vị trí thật drone; phát gió /firehawk/wind."""

    def __init__(self):
        self.n = Node()
        self.sim = None
        self.pose = None
        self.lock = threading.Lock()
        self.rows = []      # (wall, sim, x, y, z, we, wn)
        self.wind = (0.0, 0.0)
        self.n.subscribe(Clock, '/clock', self._clock)
        self.n.subscribe(Pose, '/model/firehawk/pose', self._pose)
        self.pub = self.n.advertise('/firehawk/wind', Vector3d)

    def _clock(self, m):
        self.sim = m.sim.sec + m.sim.nsec * 1e-9

    def _pose(self, m):
        with self.lock:
            self.pose = (m.position.x, m.position.y, m.position.z)
            if self.sim is not None:
                self.rows.append((time.time(), self.sim, *self.pose, *self.wind))

    def set_wind(self, e, n):
        self.wind = (e, n)
        self.pub.publish(Vector3d(x=e, y=n, z=0.0))


def sim_sleep(tr, s, every=None):
    t0 = tr.sim
    while tr.sim - t0 < s:
        if every:
            every()
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--levels', type=int, nargs='+', default=[0, 1, 2, 3, 4, 5])
    ap.add_argument('--alt', type=float, default=20.0)
    ap.add_argument('--measure', type=float, default=60.0, help='giây sim đo mỗi cấp')
    ap.add_argument('--settle', type=float, default=15.0, help='giây sim chờ sau khi đổi gió')
    ap.add_argument('--wind-from', type=float, default=90.0, help='gió TỪ hướng (độ, 90 = từ Đông)')
    ap.add_argument('--turb', type=float, default=0.18, help='sigma giật / gió trung bình')
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--mode', default='POSHOLD', help='ALT_HOLD = đối chứng: không giữ vị trí, drone phải trôi theo gió')
    a = ap.parse_args()

    for p in (9002, 5760):
        if port_open(p):
            sys.exit(f'cong {p} dang ban: tat sim cu truoc (./scripts/run_sim.sh stop)')
    out = WS / 'logs' / time.strftime('wind_hold_%Y%m%d_%H%M%S')
    out.mkdir(parents=True)
    world = make_world()
    gz = ['gz', 'sim', '-r', '-v2', str(world)] + (['-s'] if a.headless else [])
    subprocess.Popen(gz, cwd=WS / 'worlds', stdout=open(out / 'gz.log', 'w'), stderr=subprocess.STDOUT,
                     start_new_session=True)
    print('[cho] Gazebo mo cong 9002...', flush=True)
    if not wait_port(9002, 180):
        sys.exit('Gazebo khong mo cong 9002')
    subprocess.Popen([str(pathlib.Path.home() / 'ardupilot/Tools/autotest/sim_vehicle.py'), '-v', 'ArduCopter',
                      '-f', 'gazebo-iris', '--model', 'JSON', '--no-mavproxy', '-w',
                      f'--add-param-file={WS}/src/fire_drone/config/firehawk_x8.parm'],
                     cwd=WS / 'logs', stdout=open(out / 'sitl.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
    print('[cho] SITL mo cong 5760...', flush=True)
    if not wait_port(5760, 300):
        sys.exit('SITL khong mo cong 5760')
    time.sleep(3)

    tr = Truth()
    # 255 = SYSID_MYGCS mặc định: ArduPilot BỎ QUA RC_CHANNELS_OVERRIDE từ system id khác (thử 20/11: POSHOLD tự hạ)
    m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
    m.wait_heartbeat(timeout=60)
    mav = []              # (wall, type, dict)
    mlock = threading.Lock()
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            x = m.recv_match(blocking=True, timeout=0.5)
            if x is None:
                continue
            t = x.get_type()
            if t in ('ATTITUDE', 'SERVO_OUTPUT_RAW', 'VFR_HUD', 'LOCAL_POSITION_NED', 'STATUSTEXT', 'HEARTBEAT',
                     'SYS_STATUS', 'GLOBAL_POSITION_INT', 'PARAM_VALUE'):
                d = x.to_dict()
                with mlock:
                    mav.append((time.time(), t, d))
                if t == 'STATUSTEXT':
                    print('   FC:', d['text'], flush=True)
    threading.Thread(target=reader, daemon=True).start()

    def last(t):
        with mlock:
            for w, ty, d in reversed(mav):
                if ty == t:
                    return d

    def rc_center():
        m.mav.rc_channels_override_send(m.target_system, m.target_component, 1500, 1500, 1500, 1500,
                                        65535, 65535, 65535, 65535)

    # EKF3 như máy thật (SIM_JSON đặt mặc định AHRS_EKF_TYPE 10 = vị trí thật của sim, không lỗi định vị)
    for _ in range(5):
        m.mav.param_set_send(m.target_system, 1, b'AHRS_EKF_TYPE', 3.0, mavutil.mavlink.MAV_PARAM_TYPE_INT8)
        time.sleep(1)
    while tr.sim is None:
        time.sleep(0.2)
    print('[cho] PreArm + EKF...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 240:
        s = last('SYS_STATUS')
        if s and s['onboard_control_sensors_health'] & PREARM and s['onboard_control_sensors_present'] & PREARM:
            break
        time.sleep(1)
    else:
        sys.exit('PreArm khong dat sau 240 s')
    sim_sleep(tr, 5)

    def mode(name):   # m.flightmode / motors_armed() do luồng reader cập nhật - KHÔNG recv ở luồng này
        for _ in range(20):
            m.set_mode(name)
            time.sleep(0.5)
            if m.flightmode == name:
                return True
        sys.exit(f'khong vao duoc {name}')

    mode('GUIDED')
    for _ in range(20):
        m.arducopter_arm()
        time.sleep(1)
        if m.motors_armed():
            break
    else:
        sys.exit('khong ARM duoc')
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                            0, 0, 0, 0, 0, 0, 0, a.alt)
    print(f'[bay] cat canh len {a.alt} m', flush=True)
    t0 = tr.sim
    while (last('GLOBAL_POSITION_INT') or {}).get('relative_alt', 0) / 1000.0 < a.alt - 0.3:
        if tr.sim - t0 > 120:
            sys.exit('khong len toi do cao')
        time.sleep(0.2)
    sim_sleep(tr, 5)
    so, hud = last('SERVO_OUTPUT_RAW'), last('VFR_HUD')
    print(f"[bay] treo GUIDED: ga {hud['throttle']} %, PWM 8 motor {[so[f'servo{i}_raw'] for i in range(1, 9)]}", flush=True)
    rc_center()
    mode(a.mode)
    print(f'[bay] {a.mode}, giu can giua', flush=True)
    sim_sleep(tr, 10, rc_center)
    z = last('GLOBAL_POSITION_INT')['relative_alt'] / 1000.0
    if abs(z - a.alt) > 1.5:
        mode('LAND')
        sys.exit(f'POSHOLD khong giu do cao ({z:.1f} m): RC override khong vao?')

    segs = []
    V = 0.0
    for lv in a.levels:
        Vt = BEAUFORT[lv][0]
        gust = Gust(20261120, a.wind_from)      # cùng chuỗi ngẫu nhiên cho mọi cấp
        t_ramp = tr.sim
        print(f'[gio] cap {lv}: {Vt} m/s (dai {BEAUFORT[lv][1]}-{BEAUFORT[lv][2]})', flush=True)
        phase_end = {'ramp': 5.0, 'settle': 5.0 + a.settle, 'measure': 5.0 + a.settle + a.measure}
        w0 = None
        last_sim = tr.sim
        V0 = V
        while True:
            now = tr.sim
            el = now - t_ramp
            if el >= phase_end['measure']:
                break
            V = V0 + (Vt - V0) * min(1.0, el / 5.0)
            e, n = gust.step(max(now - last_sim, 1e-3), V, a.turb)
            last_sim = now
            tr.set_wind(e, n)
            if w0 is None and el >= phase_end['settle']:
                w0 = (time.time(), now)
            rc_center()
            time.sleep(0.05)
        segs.append({'level': lv, 'V': Vt, 'wall': (w0[0], time.time()), 'sim': (w0[1], tr.sim)})
        p = tr.pose
        print(f'      xong cap {lv}: vi tri ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})', flush=True)
    tr.set_wind(0.0, 0.0)
    print('[bay] LAND', flush=True)
    mode('LAND')
    t0 = time.time()
    while time.time() - t0 < 180 and m.motors_armed():
        time.sleep(1)
    stop.set()
    time.sleep(1)

    with tr.lock:
        rows = list(tr.rows)
    json.dump({'segs': segs, 'args': vars(a)}, open(out / 'segs.json', 'w'))
    with open(out / 'truth.csv', 'w') as f:
        f.write('wall,sim,x,y,z,wind_e,wind_n\n')
        f.writelines(','.join(f'{v:.4f}' for v in r) + '\n' for r in rows)
    with open(out / 'mav.jsonl', 'w') as f:
        f.writelines(json.dumps({'w': w, 't': t, 'd': {k: v for k, v in d.items() if not isinstance(v, (bytes, bytearray))}},
                                default=str) + '\n' for w, t, d in mav)
    table = analyse(rows, mav, segs)
    (out / 'bang.md').write_text(table)
    print('\n' + table)
    print(f'du lieu: {out}\ntat sim: ./scripts/run_sim.sh stop')


def pct(xs, q):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(q * len(xs)))]


def analyse(rows, mav, segs):
    """Bảng: lệch ngang so với điểm giữ (= vị trí trung bình lúc lặng gió, hoặc đầu cấp đầu tiên), độ cao,
    nghiêng, motor."""
    def win(s):
        a, b = s['sim']
        return [r for r in rows if a <= r[1] <= b]
    ref = segs[0]
    r0 = win(ref)
    hx, hy, hz = (statistics.mean(r[k] for r in r0) for k in (2, 3, 4))
    lines = ['| Cấp gió | Gió TB / giật max (m/s) | Lệch TB (m) | Lệch RMS (m) | Lệch P95 (m) | Lệch max (m) '
             '| Độ cao ±max (m) | Nghiêng TB / max (°) | PWM motor TB / max | Ga (%) |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for s in segs:
        r = win(s)
        if not r:
            continue
        d = [math.hypot(x - hx, y - hy) for _, _, x, y, *_ in r]
        mx, my = statistics.mean(x for _, _, x, *_ in r), statistics.mean(y for _, _, _, y, *_ in r)
        dz = [abs(z - hz) for *_, z, _, _ in r]
        ws = [math.hypot(e, n) for *_, e, n in r]
        a, b = s['wall']
        att = [d_ for w, t, d_ in mav if t == 'ATTITUDE' and a <= w <= b]
        tilt = [math.degrees(math.acos(max(-1, min(1, math.cos(q['roll']) * math.cos(q['pitch']))))) for q in att]
        so = [d_ for w, t, d_ in mav if t == 'SERVO_OUTPUT_RAW' and a <= w <= b]
        pw = [[q[f'servo{i}_raw'] for i in range(1, 9)] for q in so]
        hud = [d_['throttle'] for w, t, d_ in mav if t == 'VFR_HUD' and a <= w <= b]
        lines.append(
            f"| {s['level']} | {statistics.mean(ws):.1f} / {max(ws):.1f} | {math.hypot(mx - hx, my - hy):.2f} "
            f"| {math.sqrt(statistics.mean(x * x for x in d)):.2f} | {pct(d, 0.95):.2f} | {max(d):.2f} "
            f"| {max(dz):.2f} | {statistics.mean(tilt) if tilt else float('nan'):.1f} / {max(tilt) if tilt else float('nan'):.1f} "
            f"| {statistics.mean(sum(p) / 8 for p in pw) if pw else 0:.0f} / {max(max(p) for p in pw) if pw else 0} "
            f"| {statistics.mean(hud) if hud else 0:.0f} |")
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    main()
