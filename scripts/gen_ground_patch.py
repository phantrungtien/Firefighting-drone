"""Sinh vân mặt đất kiểu ảnh chụp rừng từ trên cao cho models/ground_patch, và vân mái
ngói cho models/house_c4 (nhà cấp 4 có đóm lửa trên nóc).

Địa hình gốc chỉ có ảnh cỏ 256x256 kéo trên 200 m: gần như trơn, ground_range (đo khoảng
cách bằng độ nở của ảnh) không bắt được điểm. Tấm này có chi tiết ở mọi cỡ - tán cây
1-5 m thấy từ 120 m, bụi/đá 0.1-0.5 m thấy ở 20 m.

    python3 scripts/gen_ground_patch.py     # ghi ground.jpg + roof.jpg trong models/*/materials
"""
import os

import cv2
import numpy as np

W_M, H_M = 50.0, 34.0          # khớp <size> trong models/ground_patch/model.sdf
PX_PER_M = 80                  # 1.25 cm/px: ở 20 m camera thấy ~1.5 cm/px
MODELS = os.path.join(os.path.dirname(__file__), '..', 'models')
OUT = os.path.join(MODELS, 'ground_patch', 'materials', 'textures', 'ground.jpg')
ROOF_OUT = os.path.join(MODELS, 'house_c4', 'materials', 'textures', 'roof.jpg')
ROOF_M = (12.0, 8.0)           # khớp mái trong models/house_c4/model.sdf


def blur_noise(rng, h, w, cell_px):
    """Nhiễu mịn cỡ hạt ~cell_px, trong khoảng [-1, 1]."""
    small = rng.standard_normal((max(2, h // cell_px), max(2, w // cell_px))).astype(np.float32)
    return np.clip(cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC) / 2.5, -1, 1)


def roof(rng):
    """Mái bằng lát gạch lá nem 12x8 m như nhà cấp 4 ở VN: gạch đất nung 0.3 m, mạch vữa
    xám, màu đều (lệch vài %), ngả cũ loang rộng và vệt nước/rêu nhạt sát mép. Không có mảng
    đậm tương phản mạnh - mái thật không lỏm chổm như vậy."""
    w, h = int(ROOF_M[0] * PX_PER_M), int(ROOF_M[1] * PX_PER_M)
    img = np.empty((h, w, 3), np.float32)
    img[:] = (78, 112, 168)                                   # BGR đất nung
    tile = int(0.3 * PX_PER_M)
    for y in range(0, h, tile):
        for x in range(0, w, tile):
            v = rng.uniform(0.94, 1.06)
            img[y:y + tile, x:x + tile] *= np.float32([v, v * rng.uniform(0.98, 1.02), v])
    img += blur_noise(rng, h, w, 220)[..., None] * np.float32([-6, -10, -14]) * 2  # ngả cũ
    yy, xx = np.mgrid[0:h, 0:w]
    edge = np.minimum.reduce([xx, yy, w - 1 - xx, h - 1 - yy]) / PX_PER_M      # m tới mép
    damp = np.clip(1.0 - edge / 1.2, 0, 1) * (0.5 + 0.5 * blur_noise(rng, h, w, 60))
    img -= damp[..., None] * np.float32([20, 22, 40])         # vệt nước, rêu sát tường chắn
    for x in range(0, w, tile):                               # mạch vữa
        cv2.line(img, (x, 0), (x, h), (120, 128, 132), 2)
    for y in range(0, h, tile):
        cv2.line(img, (0, y), (w, y), (120, 128, 132), 2)
    img += blur_noise(rng, h, w, 2)[..., None] * 6            # hạt gạch
    img = cv2.GaussianBlur(img, (0, 0), 0.7)
    os.makedirs(os.path.dirname(ROOF_OUT), exist_ok=True)
    cv2.imwrite(ROOF_OUT, np.clip(img, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(f'{ROOF_OUT}: {w}x{h}')


def main():
    rng = np.random.default_rng(18102026)
    w, h = int(W_M * PX_PER_M), int(H_M * PX_PER_M)
    base = np.array([60, 110, 85], np.float32)  # BGR cỏ khô ngả xanh
    img = np.empty((h, w, 3), np.float32)
    img[:] = base
    for cell, amp in ((400, 35), (80, 20), (12, 12)):        # đồi cỏ, mảng, hạt
        img += blur_noise(rng, h, w, cell)[..., None] * amp
    for _ in range(6):                                         # lối mòn đất
        pts = np.cumsum(rng.normal(0, 60, (40, 2)), 0) + rng.uniform(0, [w, h])
        cv2.polylines(img, [pts.astype(np.int32)], False, (70, 105, 135), int(rng.uniform(40, 90)))
    for _ in range(int(W_M * H_M * 0.12)):                     # tán cây 1-5 m
        c = rng.uniform(0, [w, h]).astype(int)
        r = int(rng.uniform(0.5, 2.5) * PX_PER_M)
        g = rng.uniform(0.55, 1.0)
        cv2.circle(img, tuple(c), r, (30 * g, 75 * g, 35 * g), -1, cv2.LINE_AA)
        cv2.circle(img, tuple(c - r // 4), r // 2, (45 * g, 105 * g, 50 * g), -1, cv2.LINE_AA)
    for _ in range(int(W_M * H_M * 3)):                        # bụi, đá 0.1-0.5 m
        c = rng.uniform(0, [w, h]).astype(int)
        col = rng.choice([(40, 80, 50), (120, 130, 135), (50, 70, 95)]) * rng.uniform(0.7, 1.2)
        cv2.circle(img, tuple(c), int(rng.uniform(0.05, 0.25) * PX_PER_M), col.tolist(), -1)
    img += blur_noise(rng, h, w, 3)[..., None] * 10
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    cv2.imwrite(OUT, np.clip(img, 0, 255).astype(np.uint8), [cv2.IMWRITE_JPEG_QUALITY, 88])
    print(f'{OUT}: {w}x{h}, {os.path.getsize(OUT) / 1e6:.1f} MB')
    roof(rng)


if __name__ == '__main__':
    main()
