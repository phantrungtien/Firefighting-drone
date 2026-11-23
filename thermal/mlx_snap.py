"""Grab one MLX90640 frame on /dev/i2c-7, fix dead pixels, save heatmap PNG."""
import os, sys
import numpy as np, cv2
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mlx_i2c import open_mlx, read_frame


def fix_dead(a):
    bad = a < -200
    if bad.any():
        p = np.pad(np.where(bad, np.nan, a), 1, mode="edge")
        for r, c in np.argwhere(bad):
            a[r, c] = np.nanmean(p[r:r + 3, c:c + 3])
    return a


mlx = open_mlx(7, rate_hz=4)
f = [0.0] * 768
for _ in range(3):  # first frames after power-up are garbage
    read_frame(mlx, f)
a = fix_dead(np.array(f).reshape(24, 32))
print(f"min {a.min():.1f}  mean {a.mean():.1f}  max {a.max():.1f} C")
img = cv2.normalize(a, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
img = cv2.applyColorMap(cv2.resize(img, (640, 480), interpolation=cv2.INTER_CUBIC), cv2.COLORMAP_INFERNO)
r, c = np.unravel_index(a.argmax(), a.shape)
cv2.drawMarker(img, (int(c * 20 + 10), int(r * 20 + 10)), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
cv2.putText(img, f"max {a.max():.1f}C  min {a.min():.1f}C", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "snap.png")
cv2.imwrite(out, img)
print("saved", out)
