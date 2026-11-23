import sys, time
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__file__))
from mlx_i2c import open_mlx, read_frame

mlx = open_mlx(7, rate_hz=4)
print("serial:", [hex(x) for x in mlx.serial_number])
f = [0.0] * 768
for i in range(6):
    t = time.time()
    read_frame(mlx, f)
    a = np.array(f).reshape(24, 32)
    print(f"frame {i}: {time.time()-t:.2f}s  min {a.min():.1f}  mean {a.mean():.1f}  max {a.max():.1f} C")
np.save(__import__('os').path.join(__import__('os').path.dirname(__file__), 'last_frame.npy'), a)
