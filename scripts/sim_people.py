"""BỘ DÒ NGƯỜI LÝ TƯỞNG cho SIM: vị trí THẬT của models/walking_actor -> /fire/persons (frame 'map_scan').

Chỉ để thử LOGIC thả né người của fire_mission (và vòng ngắm đỏ / cảnh báo bay tay của run_gui) tách khỏi phần
nhận dạng: đo 16/11/2026, camera nhìn thẳng xuống thì YOLO COCO không ra hộp nào cho người, Gemini 3.5-flash chỉ
thấy ~1/2 số khung và mất 2-10 s/lần. KHÔNG chạy trên drone thật - ở đó /fire/persons đến từ fire_locator.

    NGUOI=1 ANH_A0=1 PERSON_SCAN=0 ./scripts/run_sim.sh        (PERSON_SCAN=0: tắt Gemini dò người, chỉ còn bộ này)
    ROS_DOMAIN_ID=42 python3 scripts/sim_people.py [--hz 5] [--latency 0.3]

Quỹ đạo đọc từ chính model.sdf của actor (waypoint theo giờ SIM, lặp), giờ sim lấy từ gz /clock.
"""
import argparse
import bisect
import os
import re

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')  # gz.msgs10 build bằng protobuf cũ

import rclpy  # noqa: E402
from geometry_msgs.msg import Pose, PoseArray  # noqa: E402
from gz.msgs10.clock_pb2 import Clock  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402

SDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'walking_actor', 'model.sdf')


def load_path(sdf=SDF):
    """[(t, x, y)] các waypoint của actor."""
    pts = re.findall(r'<time>([\d.]+)</time><pose>([-\d.]+) ([-\d.]+)', open(sdf).read())
    return [(float(t), float(x), float(y)) for t, x, y in pts]


def actor_xy(path, sim_t):
    """Vị trí actor ở giờ sim `sim_t`: nội suy thẳng giữa hai waypoint, lặp theo chu kỳ."""
    t = sim_t % path[-1][0]
    i = max(1, bisect.bisect_right([p[0] for p in path], t))
    (t0, x0, y0), (t1, x1, y1) = path[i - 1], path[min(i, len(path) - 1)]
    k = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return x0 + k * (x1 - x0), y0 + k * (y1 - y0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hz', type=float, default=5.0)
    ap.add_argument('--latency', type=float, default=0.3, help='tin tới trễ bao nhiêu giây so với lúc "chụp"')
    a = ap.parse_args()
    path, sim = load_path(), [None]
    gz = GzNode()
    gz.subscribe(Clock, '/clock', lambda m: sim.__setitem__(0, m.sim.sec + m.sim.nsec * 1e-9))
    rclpy.init()
    node = rclpy.create_node('sim_people')
    pub = node.create_publisher(PoseArray, '/fire/persons', 10)

    def tick():
        if sim[0] is None:
            return
        x, y = actor_xy(path, sim[0] - a.latency)
        msg, q = PoseArray(), Pose()
        t = node.get_clock().now().nanoseconds * 1e-9 - a.latency   # stamp = lúc "chụp", giờ tường như run_gui
        msg.header.stamp.sec, msg.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
        msg.header.frame_id = 'map_scan'
        q.position.x, q.position.y = x, y
        msg.poses.append(q)
        pub.publish(msg)

    node.create_timer(1.0 / a.hz, tick)
    node.get_logger().info(f'nguoi THAT cua sim -> /fire/persons {a.hz:g} Hz, tre {a.latency:g} s')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    path = [(0.0, 0.0, 1.0), (5.0, 1.0, 0.0), (10.0, 0.0, -1.0), (15.0, -1.0, 0.0), (20.0, 0.0, 1.0)]
    assert actor_xy(path, 2.5) == (0.5, 0.5) and actor_xy(path, 22.5) == (0.5, 0.5), 'nội suy + lặp chu kỳ'
    assert len(load_path()) > 10 and load_path()[0][0] == 0.0
    main()
