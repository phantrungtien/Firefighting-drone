"""E2E task Chua chay: GUI THAT (offscreen) -> RemoteAdapter -> ros2_bridge --spawn ->
fire_mission -> SITL Gazebo. Bam bang chuot QTest tren man bay cam ung, khong goi tat.

    python3 e2e_gui_fire.py [alt] [E N]   alt = do cao chon tren thanh truot (mac dinh 20);
                                          E N = diem cham (m dong/bac tu nha), mac dinh = ma
                                          ArUco (18, 12). Cham lech de thu nguoi bam khong trung.
"""
import math
import os
import sys
import time

GUI = "/home/phantrungtien/GUI_NATIVE"
os.chdir(GUI)
sys.path.insert(0, GUI)

from PySide6.QtCore import QEventLoop, QPointF, Qt, QTimer
from PySide6.QtQuick import QQuickItem
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

app = QApplication([])
import laptop.voice  # noqa: E402
laptop.voice.enabled = lambda: False

import yaml  # noqa: E402
from core import bus  # noqa: E402
from core.field import REGISTRY, haversine_m  # noqa: E402
from laptop.app import MainWindow  # noqa: E402
from laptop.widgets.map_widget import deg2num  # noqa: E402

T0 = time.time()
ALT = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
TAP = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (18.0, 12.0)


def log(*a):
    print(f"[{time.time() - T0:6.1f}s]", *a, flush=True)


def wait(ms):
    loop = QEventLoop()
    QTimer.singleShot(ms, loop.quit)
    loop.exec()


def until(pred, s, step=200):
    t = time.time()
    while time.time() - t < s:
        if pred():
            return True
        wait(step)
    return False


v = REGISTRY.value
texts = []
bus.on("text", lambda e: (texts.append(e["data"].get("text")),
                          log("TEXT", e["src"], e["data"].get("text"))))

profiles = yaml.safe_load(open("config/connections.yaml"))
prof = next(p for p in profiles if p["name"] == "SITL")
win = MainWindow(profiles)
win.resize(1280, 760)
win.show()
win.connect_to(prof)
view = win.touch_view
root = view.rootObject()
slider = root.findChild(QQuickItem, "slider")


def drag(frac=1.0):
    h = slider.height()
    a = slider.mapToScene(QPointF(h / 2, h / 2)).toPoint()
    b = slider.mapToScene(QPointF(h / 2 + frac * (slider.width() - h), h / 2)).toPoint()
    QTest.mousePress(view, Qt.LeftButton, Qt.NoModifier, a)
    for i in range(1, 11):
        QTest.mouseMove(view, a + (b - a) * i / 10)
        wait(10)
    QTest.mouseRelease(view, Qt.LeftButton, Qt.NoModifier, b)
    wait(300)


def tap(item):
    QTest.mouseClick(view, Qt.LeftButton, Qt.NoModifier,
                     item.mapToScene(QPointF(item.width() / 2, item.height() / 2)).toPoint())
    wait(150)


st = lambda: win.touch._state  # noqa: E731

# 1. Cho telemetry SiK + bridge + EKF/GPS san sang
ok = until(lambda: v("position.lat") is not None and v("mission.running") is not None, 120)
log("telemetry SiK + bridge:", ok, "mission.running =", repr(v("mission.running")),
    "mode", v("heartbeat.mode"))
ok = until(lambda: (st().get("ready") or {}).get("ok"), 180, 500)
log("san sang ARM (PREARM_CHECK):", ok, st().get("ready"))
home = (v("position.lat"), v("position.lon"))
log("vi tri drone", home)

# 2. Nut ⚑ -> truot -> bridge bat fire_mission
task_btn = root.findChild(QQuickItem, "taskBtn")
tap(task_btn)
log("bam ⚑ -> pending =", root.property("pending"))
drag(1.0)
t_ask = time.time()
ok = until(lambda: v("mission.running") == "fire_mission", 30)
log(f"bridge bao running=fire_mission: {ok} sau {time.time() - t_ask:.1f} s; "
    f"taskFire={st()['taskFire']}")

# 3. Diem cham: mac dinh ma ArUco o Gazebo (18, 12) — gia su ENU (E, N)
lat0, lon0 = home
tgt = (lat0 + TAP[1] / 111320.0, lon0 + TAP[0] / (111320.0 * math.cos(math.radians(lat0))))
log("diem cham", TAP, "m tu nha")
m = win.touch.map
m.follow = False
m.center = tgt
m.update()
wait(200)
cx, cy = deg2num(*m.center, m.zoom)
px = m._to_px(*tgt, cx, cy)
root.setProperty("wpX", px.x())
root.setProperty("wpY", px.y())
root.setProperty("wpOpen", True)
wait(200)
chip = root.findChild(QQuickItem, "wpFire")
log("chip 'Chua chay tai day' hien:", chip.isVisible())
tap(chip)
aimed = root.property("gotoAt")
log("pending =", root.property("pending"), "toa do chot", aimed,
    f"lech dich {haversine_m(*aimed, *tgt):.2f} m")
root.setProperty("fireAlt", ALT)
wait(200)
log("thanh truot:", slider.property("text"))
drag(1.0)
log("da truot xac nhan diem chay")

# 4. Theo doi toi khi node xong (RTL + disarm) hoac het 420 s
t_fire = time.time()
last = 0
armed_seen = False
while time.time() - t_fire < 420:
    wait(500)
    armed = v("heartbeat.armed")
    armed_seen |= armed is True
    if time.time() - last > 10:
        last = time.time()
        lat, lon = v("position.lat"), v("position.lon")
        d = haversine_m(lat, lon, *tgt) if lat else None
        log(f"mode {v('heartbeat.mode')} armed {armed} alt {v('position.alt_rel')} "
            f"cach dich {d and round(d, 1)} m, task {v('mission.running')!r}")
    if armed_seen and armed is False and v("mission.running") == "":
        break
log(f"ket thuc sau {time.time() - t_fire:.0f} s: armed {v('heartbeat.armed')}, "
    f"task {v('mission.running')!r}, taskFire {st()['taskFire']}")
root.setProperty("wpOpen", True)
wait(300)
log("chip diem chay sau khi task xong:", chip.isVisible())
fire = [t for t in texts if t and t.startswith("FIRE")]
log(f"{len(fire)} dong FIRE: ->", fire)
win.disconnect()
