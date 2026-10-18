# Checklist task tự hành Chữa cháy

Cập nhật 18/10/2026. `[x]` = đã kiểm (ghi ở đâu, ra sao), `[ ]` = còn phải kiểm.
Thứ tự đề xuất: 2.1 → 3.1 → 2.3 / 2.4 (làm ngay trong sim, không cần phần cứng), rồi 4.1 / 4.2
khi Jetson bật lại.

## 0. Đã kiểm

- [x] **Thực tế:** servo thả bóng trên FC thật (`servo_channel` 9, `pwm_release` 1900).
- [x] **Thực tế:** bay drone thật mang bóng, thả bằng tay cầm RC.
- [x] **Thực tế:** failsafe pin / mất GCS có trên FC thật.
- [x] **Sim:** GUI chọn độ cao trên thanh trượt → `fire_target` mang `alt` → node bay đúng độ cao
  (40 m: thả lệch 0.02 m so với mục tiêu).
- [x] **Sim:** toà nhà 100 m chắn đường bay, bay 110–120 m qua mái cả lúc đi lẫn RTL (leo lại
  `alt` rồi mới RTL).
- [x] **Sim:** 120 m, mã 1.5 m: x1 không thấy → zoom x2 thấy sau 1.4 s, hạ thì zoom ra dần về x1.
- [x] **Sim:** `ground_range` (camera 2D, không lidar) đất phẳng sai ±0.4 m ở 20–50 m, ±1.2 m ở 60–110 m.
- [x] **Sim:** đóm lửa trên nóc nhà cấp 4 (mái 4 m): tách được mái / cỏ từ ~27 m, dừng ~21 m trên
  mái, bóng chạm mái lệch 7–14 cm.
- [x] **Sim:** chấm vượt qua nhà 29 m (ngoài bán kính tìm 24 m) → tự thấy đám cháy, quay lại thả.

## 1. Đã sửa, chưa kiểm đủ

- [ ] `ground_range` bỏ số đo khi xa hơn `max_dist` 80 m — mới qua selfcheck, chưa bay sim.
- [ ] Selfcheck toàn bộ GUI_NATIVE (18/10 mới chạy `check_touch`, `check_i18n`, `check_video`,
  `check_tabs`).

## 2. Sim — kịch bản có thể gây va chạm

- [ ] **2.1 Không đo được mặt nào** (mái trơn, khói che): đang coi lửa ngang chỗ cất cánh → nhà
  30 m là hạ xuống dưới nóc. Thêm mức sàn (chưa có số đo thì không hạ tiếp / thả tại chỗ), thử
  với nhà cao mái trơn.
- [ ] **2.2 Mái nhỏ trên nhà cao** lọt trong đĩa giữa ảnh bị bỏ (`inner_frac`) → chỉ đo được đất
  phía dưới → hạ ngang mái. Thử tháp 20–30 m mái 4x4 m. Chắc chắn nhất: lidar hướng xuống.
- [ ] **2.3 Phi công giành lái** (đổi mode / nút đỏ) ở từng pha: tìm, zoom, hạ, vòng kiểm tra —
  node phải buông, không thả, không đổi mode.
- [ ] **2.4 Mất liên lạc giữa task:** mất LTE/WebSocket, mất SiK, bấm Huỷ task lúc đang hạ.
- [ ] **2.5 Node `fire_mission` chết giữa task** → drone treo GUIDED, chưa có gì tự xử lý.
- [ ] **2.6 Từ chối / thất bại:** chấm xa > 300 m, `alt` ngoài 5–120 m, không thấy ở mọi mức
  zoom (phải trả zoom x1 rồi RTL) — mới có selfcheck, chưa bay.
- [ ] **2.7 Đám cháy trên dốc / đồi:** `ground_range` giả định vùng quanh lửa phẳng; `alt` tính so
  với chỗ cất cánh (`TERRAIN_ENABLE` của FC chưa dùng).

## 3. Sim đang dễ hơn thực tế

- [ ] **3.1 Gió lên quả bóng:** world có gió 4 m/s nhưng model bóng chưa bật `enable_wind` →
  lệch 7–14 cm là quá đẹp. Bật gió, đo lệch thật, bù điểm thả theo gió EKF
  (`ballistics.py` của firedrop_sim).
- [ ] **3.2 Bước "lửa đã tắt":** mã ArUco không biến mất khi trúng bóng → vòng kiểm tra luôn báo
  "VAN THAY". Cho mã tự ẩn khi bóng chạm.
- [ ] **3.3** Không có khói, nhiệt, luồng khí bốc lên, ảnh nhòe; Gazebo không có cản lăn (bóng
  lăn mãi sau khi chạm — đo lệch ở điểm chạm, không ở chỗ dừng).
- [ ] **3.4** Thay mô hình bằng file SolidWorks drone + bóng (đang chờ).
- [ ] **3.5** Nhiều đóm lửa / nhiều bóng; detector hiện gộp mọi lần thấy cùng một id.

## 4. Trước khi bay TỰ HÀNH thật

- [ ] **4.1 YOLO lửa trên Jetson → `/fire/target`:** `fire_mission` hiện chỉ nhận mục tiêu từ mã
  ArUco trong sim. Thiếu lớn nhất.
- [ ] **4.2 Camera:** webcam gắn hướng nào (code giả định nhìn thẳng xuống)? Calib K, HFOV thật
  (đang ước 70°), đo `camera_offset`.
- [ ] **4.3 `run_gui.py` trên Jetson:** quay 4K, đổi zoom theo `/fire/zoom` khi đang bay (không dựng
  lại pipeline - mất ~8 s), gửi khung toàn cảnh cho `ground_range`; đo tải giải mã 4K MJPG.
- [ ] **4.4 MAVROS trên companion:** `fcu_url` (vd `/dev/ttyAMA0:921600`), đặt `SRx_*` cho cổng
  SERIAL nối companion (thiếu thì không có pose).
- [ ] **4.5 Tham số FC:** `RTL_ALT` (bản dump 17/09 = 15 m: RTL do failsafe bay về ở độ cao này,
  có thể đâm vật cản quanh vùng bay); fence nếu bật (100 m / 150 m xung đột task 120 m / 300 m).
- [ ] **4.6 Bay tự hành theo bậc:** HITL → bay thấp với tấm ArUco in đúng 1.5 m → lửa thật.
  GPS không RTK lệch 1–3 m; mục tiêu đo tương đối bằng camera nên ít ảnh hưởng.
