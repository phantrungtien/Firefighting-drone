"""Xem anh nhiet MLX90640 truc tiep tren man hinh Jetson (khong truyen mang).

Chay:  python3 thermal/mlx_view.py
Phim:  q = thoat | c = doi bang mau | f = lat trai/phai | s = luu anh
"""
import os, sys, time

os.environ.setdefault("QT_LOGGING_RULES", "default.warning=false")  # tat canh bao locale cua Qt

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mlx_i2c import open_mlx, read_frame  # doc sensor qua /dev/i2c-7, dia chi 0x33

SCALE = 20                      # moi pixel nhiet -> o 20x20 tren man hinh
W, H = 32 * SCALE, 24 * SCALE   # 640 x 480
BAR_W = 60                      # thanh thang mau ben phai
COLORMAPS = [("JET", cv2.COLORMAP_JET), ("INFERNO", cv2.COLORMAP_INFERNO),
             ("HOT", cv2.COLORMAP_HOT), ("TURBO", cv2.COLORMAP_TURBO)]


def fix_dead(a):
    """Pixel hong (-273 C) -> lay trung binh cac pixel xung quanh."""
    bad = a < -100
    if bad.any():
        p = np.pad(np.where(bad, np.nan, a), 1, mode="edge")
        for r, c in np.argwhere(bad):
            a[r, c] = np.nanmean(p[r:r + 3, c:c + 3])
    return a


def draw(a, cmap):
    lo, hi = float(a.min()), float(a.max())
    span = max(hi - lo, 2.0)
    # 1) nhiet do -> 0..255 -> phong to muot -> to mau
    gray = np.clip((a - lo) / span * 255, 0, 255).astype(np.uint8)
    gray = cv2.resize(gray, (W, H), interpolation=cv2.INTER_CUBIC)
    img = cv2.applyColorMap(gray, cmap)

    # 2) diem nong nhat (+ trang) va tam anh (o vuong)
    r, c = np.unravel_index(a.argmax(), a.shape)
    hx, hy = int((c + .5) * SCALE), int((r + .5) * SCALE)
    cv2.drawMarker(img, (hx, hy), (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
    tx, ty = min(hx + 8, W - 60), max(hy - 8, 45)  # giu chu trong khung khi diem nong o sat mep
    cv2.putText(img, f"{hi:.1f}", (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    center = float(a[11:13, 15:17].mean())
    cv2.rectangle(img, (W // 2 - 10, H // 2 - 10), (W // 2 + 10, H // 2 + 10), (255, 255, 255), 1)
    cv2.putText(img, f"{center:.1f} C", (W // 2 + 14, H // 2 + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # 3) thanh thang mau: tren = nong, duoi = lanh
    ramp = np.linspace(255, 0, H).astype(np.uint8).reshape(H, 1)
    bar = cv2.applyColorMap(np.repeat(ramp, 20, axis=1), cmap)
    bar = cv2.copyMakeBorder(bar, 0, 0, 5, BAR_W - 25, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    cv2.putText(bar, f"{lo + span:.0f}", (26, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    cv2.putText(bar, f"{lo:.0f}", (26, H - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return np.hstack([img, bar])


def main():
    print("Dang mo MLX90640 tren /dev/i2c-7 ...")
    for i in range(5):  # thu lai neu doc EEPROM loi (day long)
        try:
            mlx = open_mlx(7, rate_hz=8)
            break
        except Exception as e:
            print("loi mo sensor:", e, "-> thu lai")
            time.sleep(0.5)
    else:
        sys.exit("Khong mo duoc sensor, kiem tra day chan 1/3/5/6")

    frame = [0.0] * 768
    cmap_i, flip, n, t0, fps = 0, False, 0, time.time(), 0.0
    win = "MLX90640 thermal"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, (W + BAR_W) * 3 // 2, H * 3 // 2)

    while True:
        try:
            read_frame(mlx, frame)    # doc 2 nua anh qua I2C + doi ra do C
        except (RuntimeError, OSError) as e:  # chi bat loi sensor/I2C, loi code thi dung luon
            print("loi doc khung:", e)
            time.sleep(0.5)
            continue
        n += 1
        if n <= 2:                   # 2 khung dau sau khi bat la rac
            continue
        a = fix_dead(np.array(frame, dtype=np.float32).reshape(24, 32))
        if flip:
            a = np.fliplr(a)

        now = time.time()
        fps = 0.8 * fps + 0.2 / max(now - t0, 1e-3)
        t0 = now
        img = draw(a, COLORMAPS[cmap_i][1])
        cv2.putText(img, f"min {a.min():.1f}  max {a.max():.1f} C  {fps:.1f} fps  [{COLORMAPS[cmap_i][0]}]",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.imshow(win, img)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q") or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
        if k == ord("c"):
            cmap_i = (cmap_i + 1) % len(COLORMAPS)
        if k == ord("f"):
            flip = not flip
        if k == ord("s"):
            out = os.path.join(os.path.dirname(os.path.abspath(__file__)), f"thermal_{time.strftime('%H%M%S')}.png")
            cv2.imwrite(out, img)
            print("da luu", out)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
