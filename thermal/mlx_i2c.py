"""busio.I2C-compatible shim over smbus2 so adafruit_mlx90640 runs on Jetson without Blinka."""
import sys, types
from smbus2 import SMBus, i2c_msg

# adafruit_bus_device evaluates type hints from busio/circuitpython_typing at import; stub them.
for _name, _attrs in (("busio", ("I2C", "SPI")),
                      ("circuitpython_typing", ("ReadableBuffer", "WriteableBuffer"))):
    if _name not in sys.modules:
        _m = types.ModuleType(_name)
        for _a in _attrs:
            setattr(_m, _a, object)
        sys.modules[_name] = _m


class SMBusI2C:
    def __init__(self, bus=7):
        self.bus = SMBus(bus)

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def writeto(self, addr, buf, *, start=0, end=None):
        data = list(bytes(buf[start:end]))
        if not data:  # probe
            self.bus.i2c_rdwr(i2c_msg.read(addr, 1))
            return
        self.bus.i2c_rdwr(i2c_msg.write(addr, data))

    def readfrom_into(self, addr, buf, *, start=0, end=None):
        end = len(buf) if end is None else end
        m = i2c_msg.read(addr, end - start)
        self.bus.i2c_rdwr(m)
        buf[start:end] = bytes(m)

    def writeto_then_readfrom(self, addr, out_buf, in_buf, *, out_start=0, out_end=None,
                              in_start=0, in_end=None):
        in_end = len(in_buf) if in_end is None else in_end
        w = i2c_msg.write(addr, list(bytes(out_buf[out_start:out_end])))
        r = i2c_msg.read(addr, in_end - in_start)
        self.bus.i2c_rdwr(w, r)
        in_buf[in_start:in_end] = bytes(r)


def open_mlx(bus=7, rate_hz=8):
    import adafruit_mlx90640 as m
    mlx = m.MLX90640(SMBusI2C(bus))
    mlx.refresh_rate = {1: m.RefreshRate.REFRESH_1_HZ, 2: m.RefreshRate.REFRESH_2_HZ,
                        4: m.RefreshRate.REFRESH_4_HZ, 8: m.RefreshRate.REFRESH_8_HZ,
                        16: m.RefreshRate.REFRESH_16_HZ}[rate_hz]
    mlx.want_rate = mlx.refresh_rate  # nho toc do mong muon de phat hien sensor reset
    mlx.resets = 0
    return mlx


def _check_reset(mlx):
    """Sensor mat dien chap chon -> tu reset ve mac dinh 2 Hz (thanh ghi 0x800D = 0x1901).
    Phat hien bang cach doc lai toc do, sai thi dat lai."""
    want = getattr(mlx, "want_rate", None)
    if want is not None and mlx.refresh_rate != want:
        mlx.resets += 1
        print(f"!! MLX90640 bi reset (mat nguon?) lan {mlx.resets} -> dat lai toc do", flush=True)
        mlx.refresh_rate = want


def read_frame(mlx, frame, tries=5):
    """getFrame co thu lai. Khung doc dung luc sensor dang ghi / vua doi toc do do
    co the cho so vo ly -> thu vien bao ValueError (math domain error). Bo khung do, doc lai."""
    for _ in range(tries):
        try:
            _check_reset(mlx)
            mlx.getFrame(frame)
        except (ValueError, OSError):
            continue
        good = [v for v in frame if v > -100]
        if len(good) > 760 and max(good) < 400:
            return frame
    raise RuntimeError("MLX90640: %d khung lien tiep bi loi, kiem tra day I2C" % tries)
