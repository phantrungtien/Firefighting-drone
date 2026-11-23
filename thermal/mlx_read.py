"""Doc MLX90640 truc tiep qua I2C (/dev/i2c-7, dia chi 0x33) va in ra terminal."""
import os, sys, time
from smbus2 import SMBus, i2c_msg

BUS, ADDR = 7, 0x33


def read_words(bus, reg, n):
    """Gui dia chi thanh ghi 16-bit, doc ve n word 16-bit (big-endian)."""
    w = i2c_msg.write(ADDR, [reg >> 8, reg & 0xFF])
    r = i2c_msg.read(ADDR, 2 * n)
    bus.i2c_rdwr(w, r)
    b = bytes(r)
    return [(b[i] << 8) | b[i + 1] for i in range(0, len(b), 2)]


with SMBus(BUS) as bus:
    print("1) Serial ID (EEPROM 0x2407..0x2409):", [hex(x) for x in read_words(bus, 0x2407, 3)])
    print("2) Thanh ghi dieu khien 0x800D:", hex(read_words(bus, 0x800D, 1)[0]))

    # cho sensor bao co du lieu moi (bit 3 cua thanh ghi trang thai 0x8000)
    while not (read_words(bus, 0x8000, 1)[0] & 0x08):
        time.sleep(0.01)
    status = read_words(bus, 0x8000, 1)[0]
    raw = read_words(bus, 0x0400, 832)  # RAM: 768 pixel + du lieu nhiet do chip
    print(f"3) Status 0x8000 = {hex(status)} -> subpage {status & 1}")
    print("   16 word tho dau tien cua RAM 0x0400:", " ".join(f"{x:04x}" for x in raw[:16]))
    print("   (so tho chua phai do C, can he so hieu chuan trong EEPROM de doi)\n")

# 4) Doi sang do C bang thu vien (dung he so EEPROM), in bang 24x32
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mlx_i2c import open_mlx, read_frame

mlx = open_mlx(BUS, rate_hz=4)
f = [0.0] * 768
for _ in range(3):  # 2 khung dau sau khi bat la rac
    read_frame(mlx, f)
print("4) Nhiet do 24 hang x 32 cot (do C):")
for r in range(24):
    print(" ".join(f"{f[r * 32 + c]:3.0f}" if f[r * 32 + c] > -200 else "  x" for c in range(32)))
print(f"\n   min {min(v for v in f if v > -200):.1f}  max {max(f):.1f} C   (x = pixel hong)")
