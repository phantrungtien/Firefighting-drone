# Camera nhiệt MLX90640 (32x24) trên Jetson

Nối header 40 chân Jetson Orin Nano: chân 1 3V3, 6 GND, 3 SDA, 5 SCL → `/dev/i2c-7` (400 kHz), địa chỉ `0x33`.
Không phải thiết bị `/dev/video`.

Cài: `pip install --user smbus2 && pip install --user --no-deps adafruit-circuitpython-mlx90640 adafruit-circuitpython-busdevice`
(không cần Blinka: `mlx_i2c.py` giả `busio` bằng smbus2).

| Tệp | Việc |
|---|---|
| `mlx_i2c.py` | Shim busio → smbus2, `open_mlx()` / `read_frame()` |
| `mlx_view.py` | Xem trực tiếp (q thoát, c bảng màu, f lật, s lưu ảnh) |
| `mlx_snap.py` | Chụp 1 khung ra PNG |
| `mlx_read.py`, `mlx_test.py` | Đọc thử / lưu `last_frame.npy` |

Lưu ý: 2 khung đầu sau khi cấp nguồn là rác; điểm chết (hàng 1, cột 30) đọc -273 °C, được thay bằng trung bình lân cận;
4 Hz ≈ 0.5 s mỗi khung đủ.
