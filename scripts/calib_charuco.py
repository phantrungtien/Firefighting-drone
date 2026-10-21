"""Calib noi tai camera drone (webcam XC-TECH 4K) bang bang ChArUco.

    sudo systemctl stop mjpeg-stream          # service dang giu /dev/video0
    python3 calib_charuco.py chup --sx 7 --sy 5 --dict DICT_4X4_50
        # cam bang di chuyen truoc camera: gan/xa, nghieng, ra ca 4 goc anh.
        # Tu luu khi bang DUNG YEN va o cho moi; du 30 anh thi tu dung.
    python3 calib_charuco.py calib --sx 7 --sy 5 --dict DICT_4X4_50 --sq 40 --mk 30
    sudo systemctl start mjpeg-stream

Calib o 3840x2160 (dung che do service chup) roi suy ra 1280x720: run_gui.py co
nvvidconv ca khung 4K xuong 1280x720 KHONG cat (left=0 right=3840), cung ti le
16:9 -> K nhan 1/3, he so meo giu nguyen. Dung --zoom (cat giua) thi file 720p
KHONG con dung — phai calib lai hoac doi cx, cy.

--sq/--mk (mm) chi doi thang do tinh tien, K va meo khong doi; van nen do that bang
thuoc cho dung. Khong biet dict thi `chup --dict auto` thu het bo pho bien.
"""
import argparse
import glob
import os
import threading
import time

import cv2
import numpy as np

W, H = 3840, 2160
OUT_W, OUT_H = 1280, 720
DIR = os.path.expanduser(os.environ.get("CALIB_DIR", "~/pi5/calib"))
DICTS = ["DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_5X5_50", "DICT_5X5_100",
         "DICT_5X5_250", "DICT_6X6_50", "DICT_6X6_250", "DICT_7X7_50", "DICT_ARUCO_ORIGINAL"]
STILL_PX = 4.0     # goc lech < 4 px giua hai khung lien tiep = bang dung yen (khong nhoe)
NEW_POS = 0.12     # tam bang dich > 12% chieu rong, hoac dien tich doi > 30% = cho moi
MIN_CORNERS = 8
TILT_MIN, TILT_NEW = 25.0, 15.0  # --nghieng: do nghieng toi thieu / lech huong toi thieu


def board(a):
    b = cv2.aruco.CharucoBoard((a.sx, a.sy), a.sq, a.mk,
                               cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a.dict)))
    b.setLegacyPattern(a.legacy)  # bang in tu OpenCV < 4.6, so o chan: thu --legacy neu it goc
    return b


def detect(det, img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cc, ids, _, _ = det.detectBoard(g)
    return (cc, ids) if ids is not None and len(ids) >= MIN_CORNERS else (None, None)


def auto_dict(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def count(d):
        ids = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, d))).detectMarkers(g)[1]
        return 0 if ids is None else len(ids)
    n = {d: count(d) for d in DICTS}
    best = max(n, key=n.get)
    return best, n[best]


def chup(a):
    # V4L2 + giai MJPEG bang CPU (~vai fps o 4K, du cho calib). cv2 pip khong co GStreamer
    # nen khong dung duoc nvjpegdec nhu service.
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened() or cap.get(cv2.CAP_PROP_FRAME_WIDTH) != W:
        raise SystemExit("khong mo duoc camera — da `sudo systemctl stop mjpeg-stream` chua?")
    os.makedirs(DIR, exist_ok=True)
    if a.dict == "auto":
        for _ in range(60):
            ok, img = cap.read()
            if ok:
                d, n = auto_dict(img)
                print(f"dict: {d} ({n} marker)", flush=True)
                if n >= 4:
                    a.dict = d
                    break
        else:
            raise SystemExit("khong thay marker nao — dua bang vao giua khung hinh")
    det = cv2.aruco.CharucoDetector(board(a))
    # --nghieng: chup bo sung anh NGHIENG cho fx/fy. Vi tri trong anh khong con la tieu chi
    # (bang nghieng giua anh trung cho voi anh cu) — dung K da calib tinh phap tuyen bang
    # (solvePnP), luu khi nghieng >= TILT_MIN va lech huong >= TILT_NEW voi moi anh nghieng
    # da luu trong lan chay nay.
    if a.nghieng:
        K0 = np.array(yaml_list(f"{DIR}/cam_{W}x{H}.yaml", "camera_matrix")).reshape(3, 3)
        D0 = np.array(yaml_list(f"{DIR}/cam_{W}x{H}.yaml", "distortion_coefficients"))
        normals = []
    saved, prev = [], None  # saved: (tam x, tam y, dien tich, khung bao) cua cac anh da luu
    for f in sorted(glob.glob(f"{DIR}/*.jpg")):  # chay lai: nho vi tri anh cu, khong chup trung
        cc, _ = detect(det, cv2.imread(f))
        if cc is not None:
            saved.append(pos(cc))
    n = len(glob.glob(f"{DIR}/*.jpg"))
    if a.xem:
        serve(a.xem)
        print(f"xem truc tiep: http://<ip jetson>:{a.xem}/", flush=True)
    print(f"luu vao {DIR}, dang co {n} anh, can {a.n}", flush=True)
    msg = ""
    while n < a.n:
        ok, img = cap.read()
        if not ok:
            continue
        cc, ids = detect(det, img)
        tilt = None
        if a.nghieng and cc is not None:
            o, i = det.getBoard().matchImagePoints(cc, ids)
            ok, rv, _ = cv2.solvePnP(o, i, K0, D0)
            if ok:
                nv = cv2.Rodrigues(rv)[0][:, 2]
                tilt = (nv, float(np.degrees(np.arccos(min(1.0, abs(nv[2]))))))
        if a.xem:
            preview(img, cc, saved, n, a.n, (f"nghieng {tilt[1]:.0f} deg | " if tilt else "") + msg)
        if cc is None:
            prev = None
            continue
        pts = dict(zip(ids.ravel(), cc.reshape(-1, 2)))
        still = prev is not None and len(set(pts) & set(prev)) >= MIN_CORNERS and max(
            np.linalg.norm(pts[i] - prev[i]) for i in set(pts) & set(prev)) < STILL_PX
        prev = pts
        if not still:
            continue
        cx, cy, area, box = pos(cc)
        if a.nghieng:
            # bang nho / it goc: solvePnP de lat huong (do 21/10: anh 9 goc ra 89 deg) -> bo
            if tilt is None or len(ids) < 20 or area < 0.02 or tilt[1] < TILT_MIN or any(
                    np.degrees(np.arccos(min(1.0, abs(tilt[0] @ m)))) < TILT_NEW for m in normals):
                continue
            normals.append(tilt[0])
        elif any(np.hypot(cx - x, cy - y) < NEW_POS and abs(area - s) / s < 0.3 for x, y, s, _ in saved):
            continue
        saved.append((cx, cy, area, box))
        cv2.imwrite(f"{DIR}/{time.strftime('%H%M%S')}_{n:02d}.jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        n += 1
        t = f", nghieng {tilt[1]:.0f} deg" if a.nghieng else ""
        msg = f"LUU {n}: {len(ids)} goc, {area:.0%} khung{t}"
        print(f"  luu {n}/{a.n}: {len(ids)} goc, tam ({cx:.2f},{cy:.2f}), {area:.0%} khung{t}", flush=True)
    print(f"xong. dict {a.dict} -> chay: calib --dict {a.dict} ...", flush=True)


def yaml_list(path, key):
    """data: [...] cua khoa `key` trong file camera_info do write() ghi."""
    lines = open(path).read().split(key + ":", 1)[1]
    return [float(v) for v in lines.split("data: [", 1)[1].split("]", 1)[0].split(",")]


def pos(cc):
    p = cc.reshape(-1, 2) / [W, H]
    cx, cy = p.mean(0)
    area = cv2.contourArea(cv2.convexHull(p.astype(np.float32)))
    return cx, cy, area, (*p.min(0), *p.max(0))


# Xem truc tiep (--xem PORT): MJPEG 1280x720 qua HTTP, mo bang trinh duyet bat ky.
# Xanh la = cho da co anh, cham do = goc dang nhan. ponytail: mot khung moi nhat dung chung,
# moi nguoi xem mot thread; ~vai fps vi giai 4K bang CPU.
_jpg, _cv = None, threading.Condition()


def preview(img, cc, saved, n, total, msg):
    global _jpg
    v = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_AREA)
    ov = v.copy()
    for *_, (x0, y0, x1, y1) in saved:
        cv2.rectangle(ov, (int(x0 * 1280), int(y0 * 720)), (int(x1 * 1280), int(y1 * 720)), (0, 200, 0), -1)
    v = cv2.addWeighted(ov, 0.3, v, 0.7, 0)
    k = 0
    if cc is not None:
        k = len(cc)
        for x, y in cc.reshape(-1, 2) / 3:
            cv2.circle(v, (int(x), int(y)), 4, (0, 0, 255), -1)
    cv2.putText(v, f"{n}/{total} anh | {k} goc | {msg}", (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0, 0, 0), 5)
    cv2.putText(v, f"{n}/{total} anh | {k} goc | {msg}", (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (255, 255, 255), 2)
    with _cv:
        _jpg = cv2.imencode(".jpg", v, [cv2.IMWRITE_JPEG_QUALITY, 70])[1].tobytes()
        _cv.notify_all()


def serve(port):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    class H(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
            self.end_headers()
            try:
                while True:
                    with _cv:
                        _cv.wait(2)
                        j = _jpg
                    if j:
                        self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(j)).encode() + b"\r\n\r\n" + j + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()


def calib(a):
    det = cv2.aruco.CharucoDetector(board(a))
    b = det.getBoard()
    obj, imgp, names = [], [], []
    for f in sorted(glob.glob(f"{DIR}/*.jpg")):
        img = cv2.imread(f)
        if img.shape[1::-1] != (W, H):
            print(f"  bo {f}: {img.shape[1]}x{img.shape[0]}, khong phai {W}x{H}")
            continue
        cc, ids = detect(det, img)
        if cc is None:
            print(f"  bo {f}: < {MIN_CORNERS} goc")
            continue
        o, i = b.matchImagePoints(cc, ids)
        obj.append(o)
        imgp.append(i)
        names.append(os.path.basename(f))
    if len(obj) < 10:
        raise SystemExit(f"chi {len(obj)} anh dung duoc, can >= 10")
    rms, K, dist, rv, tv = cv2.calibrateCamera(obj, imgp, (W, H), None, None)
    # Sai so tung anh: anh nao lech xa trung binh (nhoe, nhan sai goc) thi xoa roi chay lai
    errs = []
    for o, i, r, t in zip(obj, imgp, rv, tv):
        pr, _ = cv2.projectPoints(o, r, t, K, dist)
        errs.append(float(np.sqrt(np.mean(np.sum((pr.reshape(-1, 2) - i.reshape(-1, 2)) ** 2, axis=1)))))
    for nm, e in sorted(zip(names, errs), key=lambda x: -x[1])[:5]:
        print(f"  lech nhat: {nm} {e:.2f} px")
    print(f"RMS {rms:.3f} px tren {len(obj)} anh ({W}x{H})")
    print(f"fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}")
    print(f"meo {np.round(dist.ravel(), 4).tolist()}")
    print(f"goc nhin ngang {np.degrees(2 * np.arctan(W / 2 / K[0, 0])):.1f} deg")
    write(f"{DIR}/cam_{W}x{H}.yaml", K, dist, W, H)
    s = OUT_W / W
    K2 = K.copy()
    K2[0, 0] *= s
    K2[1, 1] *= s
    K2[:2, 2] = (K[:2, 2] + 0.5) * s - 0.5  # tam diem anh, khong phai goc diem anh
    write(f"{DIR}/cam_{OUT_W}x{OUT_H}.yaml", K2, dist, OUT_W, OUT_H)


def write(path, K, dist, w, h):
    """Dinh dang camera_info cua ROS (camera_calibration_parsers doc duoc)."""
    def mat(name, m, r, c):
        return (f"{name}:\n  rows: {r}\n  cols: {c}\n  data: [" +
                ", ".join(f"{v:.6f}" for v in np.asarray(m).ravel()) + "]\n")
    P = np.hstack([K, np.zeros((3, 1))])
    with open(path, "w") as f:
        f.write(f"image_width: {w}\nimage_height: {h}\ncamera_name: drone_cam\n")
        f.write(mat("camera_matrix", K, 3, 3))
        f.write("distortion_model: plumb_bob\n")
        f.write(mat("distortion_coefficients", dist.ravel()[:5], 1, 5))
        f.write(mat("rectification_matrix", np.eye(3), 3, 3))
        f.write(mat("projection_matrix", P, 3, 4))
    print(f"ghi {path}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("buoc", choices=["chup", "calib"])
    p.add_argument("--sx", type=int, required=True, help="so O theo chieu ngang (khong phai so goc)")
    p.add_argument("--sy", type=int, required=True, help="so O theo chieu doc")
    p.add_argument("--sq", type=float, default=40.0, help="canh o vuong, mm")
    p.add_argument("--mk", type=float, default=30.0, help="canh marker, mm")
    p.add_argument("--dict", default="auto")
    p.add_argument("--legacy", action="store_true")
    p.add_argument("-n", type=int, default=30, help="so anh can chup")
    p.add_argument("--nghieng", action="store_true", help="chup bo sung anh nghieng (can calib truoc)")
    p.add_argument("--xem", type=int, default=8080, help="cong xem truc tiep (0 = tat)")
    a = p.parse_args()
    if a.buoc == "calib" and a.dict == "auto":
        p.error("calib can --dict (buoc chup in ra dict tim duoc)")
    chup(a) if a.buoc == "chup" else calib(a)
