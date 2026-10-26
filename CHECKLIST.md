# Checklist task tự hành Chữa cháy

Cập nhật 24/10/2026. `[x]` = đã kiểm (ghi ở đâu, ra sao), `[ ]` = còn phải kiểm.
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

- [ ] **Thả bằng `DO_GRIPPER`** (26/10/2026, thay `DO_SET_SERVO 9 1900`: FC thật giờ để servo ở
  SERVO16 = gripper, SERVO9_FUNCTION 0 -> lệnh cũ không tới servo). SITL đạt: GRAB -> SERVO9 1900,
  RELEASE -> 1100. **Thực tế 23:11 đạt:** từ Jetson (pymavlink, ttyACM0) GRAB -> SERVO16 1900 sau
  0.1 s, bóng rơi gọn; 2 s sau RELEASE -> 1100. **27/10 qua MAVROS** (`/mavros/cmd/command`, đúng
  đường fire_mission): 2/2 lần ack 9 ms, SERVO16 1900 sau ~40 ms, đóng 1100 sau ~90 ms. Sim Gazebo bay
  trọn đạt (tờ A0, bóng cách tâm 0.04-0.13 m). Chưa: trọn task trên drone thật.
- [ ] `ground_range` bỏ số đo khi xa hơn `max_dist` 80 m — mới qua selfcheck, chưa bay sim.
- [x] Selfcheck toàn bộ GUI_NATIVE: `selfcheck: PASS` 27/10/2026 (`b53f84b`).

## 2. Sim — kịch bản có thể gây va chạm

- [ ] **2.1 Không đo được mặt nào** — 27/10/2026 đã thêm sàn: `ground_filter` không bao giờ < 0 = không
  bao giờ hạ dưới "hạ cách lửa" tính từ chỗ cất cánh (đo sai thấp không kéo drone xuống). CÒN: vật cao
  mà không đo được (sim 27/10: mã trên mái 4 m, 0 số đo -> thả 6 m trên đất = ~2 m trên mái). Cũ:
  (mái trơn, khói che): đang coi lửa ngang chỗ cất cánh → nhà
  30 m là hạ xuống dưới nóc. Thêm mức sàn (chưa có số đo thì không hạ tiếp / thả tại chỗ), thử
  với nhà cao mái trơn.
- [ ] **2.2 Mái nhỏ trên nhà cao** lọt trong đĩa giữa ảnh bị bỏ (`inner_frac`) → chỉ đo được đất
  phía dưới → hạ ngang mái. Thử tháp 20–30 m mái 4x4 m. Chắc chắn nhất: lidar hướng xuống.
- [ ] **2.3 Phi công giành lái** (đổi mode / nút đỏ) ở từng pha: tìm, zoom, hạ, vòng kiểm tra —
  node phải buông, không thả, không đổi mode.
- [ ] **2.4 Mất liên lạc giữa task:** mất LTE/WebSocket, mất SiK, bấm Huỷ task lúc đang hạ.
- [x] **2.5 Huỷ task / node `fire_mission` chết giữa task** (user chốt 27/10/2026: TREO + báo GUI,
  không tự đổi mode). Huỷ (SIGTERM): node gửi setpoint tại chỗ + STATUSTEXT `FIRE: TREO tai cho`;
  bridge báo đỏ (severity 2) "drone TREO o GUIDED, khong ai lai" ngay rồi nhắc mỗi 5 s, tối đa
  30 s, thôi khi đổi mode / disarm / bật task mới. Node chết (kill -9) và bridge khởi động lại
  lúc drone GUIDED cũng báo. Sim 27/10: huỷ lúc 4.5 m/s -> trôi 6.9 m rồi về treo đúng chỗ huỷ,
  báo đỏ sau 0.8 s; kill -9 -> báo sau ~1 s. Sửa luôn bridge chờ 6 s mỗi lần huỷ (zombie `ros2
  run`), Jetson còn 0.9 s. ⚠ kill -9 / MAVROS chết: không gửi được setpoint treo -> drone bay
  NỐT tới điểm đang bay tới rồi mới treo. Chưa có: tự RTL/LOITER (chỉ báo).
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

Drone thật bay **tối đa 5 m** (user 24/10/2026): bỏ các mốc thử ở xa / trên cao cho drone thật
(số zoom theo khoảng cách ở 4.1 chỉ để tham khảo); sim giữ nguyên 20-120 m.

- [ ] **4.1 YOLO lửa trên Jetson → `/fire/target`:** code xong 23/10/2026, lửa thật mới thử hột quẹt
  (24/10). `run_gui.py` (pi5) gửi hộp lửa `/fire/detections` (tâm theo điểm ảnh khung 4K, đã đổi ngược
  zoom, ~4.5 Hz) → `fire_locator` chiếu tia qua tâm hộp cắt mặt z = `ground_z` (chưa có thì 0) →
  `/fire/target`. Thử hộp giả giữa ảnh ở 30 m ra đúng ngay dưới camera. Chạy:
  `ros2 launch fire_drone fire_mission.launch.py real:=true`. Lửa trên mái cao ở mép ảnh lệch tới
  ~15 m cho tới khi bay tới trên (tự hội tụ). 23/10 ảnh lửa trên điện thoại trước webcam: 39/39 khung
  có hộp Fire 0.82-0.84, video GUI thấy hộp đúng lửa. Zoom: cùng ảnh ở giữa, x1 tâm (1772, 1220) điểm 0.67,
  x2 (1766, 1218) điểm 0.78 - lệch 6 px 4K ~ 5 cm ở 20 m: đổi ngược vùng cắt đúng.
  Lửa thật 24/10/2026: hột quẹt trước camera bắt khá tốt (user, đánh giá bằng mắt, chưa có số đo),
  có Gemini xác minh (`fire_verify`, pi5 `bd0f144`). Còn: lửa lớn hơn, nhìn
  xuống từ <= 5 m.
  ⚠ BÁO NHẦM: 23/10 trong phòng lab, THANH AN TOÀN của robot (vàng/cam) bị báo Fire 18/19 khung ở x2 (điểm 0.41,
  hộp 35x138 px YOLO) - model bắt theo màu (user). Đủ confirm 5 khung của fire_locator -> bay thật là
  thả bóng vào vật đó. Đề xuất: người vận hành xác nhận khung có hộp trước khi thả (cần ros2_bridge +
  nút GUI); gom ảnh báo nhầm từ camera drone để train lại; cân nhắc kiểm nhấp nháy / nâng `--fire-conf`.
  Gom ảnh: `run_gui --fire-save ~/fire_dataset` (bật trong service 23/10) lưu khung có hộp lửa
  (1280x720 như YOLO thấy, chưa vẽ) + nhãn YOLO các hộp model đoán, <= 1 khung/s. Train lại: xoá hộp
  sai -> ảnh âm, sửa hộp đúng -> ảnh dương. Tên file có zoom và điểm (`..._x2.0_d0.41.jpg`).
  Ngưỡng cỡ (ảnh lửa điện thoại lệch tâm ~(+205, -22) px 4K, quét x1..x6, toạ độ 4K lệch <= 4 px giữa
  các mức): YOLO thấy 44x28 px -> 10/19 khung d0.37; 80x54 -> 17/19 d0.41; 98x63 -> 19/19 d0.59; 116x74
  -> 19/19 d0.78. Bắt chắc khi cạnh ngắn >= ~55-60 px: zoom x5 thấy lửa 1 m tới ~38 m, 2 m ~75 m,
  3 m ~113 m. x6 (= 640 px thật, bằng ảnh vào YOLO) thêm ~20%. Lửa thật có thể dễ hơn - đo lại.
- [x] **4.2 Camera:** gắn nhìn thẳng xuống như sim (user 23/10/2026). **27/10/2026 đã kiểm hướng:** tay đưa
  vào từ phía mũi -> hiện từ mép TRÊN ảnh, từ bên phải -> mép PHẢI (chụp video trên tablet) = đúng
  `R_OPT_TO_BODY`, không xoay/lật. `camera_offset` đo bằng thước: trước 20 cm, giữa, thấp 2 cm ->
  `[0.20, 0.0, -0.02]` trong launch `real:=true` (trước đó là số sim 0.26, 0, -0.18). Lệch yaw vài độ
  mắt không thấy - ở 5 m lửa gần giữa ảnh thì không đáng kể (bay tới trên rồi mới thả).
- [x] **4.2b Cơ cấu thả bóng** (user 23/10/2026): ngay dưới tâm FC, chỉ lệch theo chiều đứng -> không cần
  bù ngang; lệch đứng vài cm không đáng kể khi thả từ 20 m.
- [x] **4.3 `run_gui.py` trên Jetson** (23/10/2026): pipeline gst chạy trong tiến trình (`_GstPipe`),
  `/fire/zoom` đổi vùng cắt nvvidconv lúc đang chạy - khung thứ 2 (~70 ms) đã đúng, 31 fps ở x1..x5;
  nhánh xám toàn cảnh 960x540 sau `valve`, chỉ mở khi `/fire/ground_enable`: chính 30 + xám 29 fps.
  `ground_range` chạy ngay trong `run_gui` (không qua DDS), nhận ~30 khung xám/s (log
  `[fire] do mat dat: N khung xam/s`). Pipeline chạy trong tiến trình CON `gst_zoom.py`: nvjpegdec
  thỉnh thoảng SEGV (`gst_adapter_push` assertion) - để trong `run_gui` là chết cả service (2 lần /
  30 s), ở con thì `_watch` dựng lại sau 5 s, giữ zoom (thử `kill -SEGV`). Service đã cài 23/10.
  Chưa thử `ground_range` khi hạ thật (bàn thí nghiệm camera đứng yên).
- [ ] **4.4 MAVROS trên companion** (23/10/2026, USB `/dev/ttyACM0`, Pixhawk 6C): connected, IMU/GPS 10 Hz.
  Sửa 2 lỗi: (1) MAVROS 2.16 (apt Jetson) đặt topic theo tên node -> namespace 'mavros' ra
  `/mavros/mavros/state`; `mavros.launch.py` giờ lấy namespace theo `node.launch` của bản đang cài.
  (2) FC để `MAVn_*` = 0 -> có HEARTBEAT nhưng không pose/IMU/GPS; `GuidedNode.request_streams()` xin
  lại mỗi lần nối (thử tắt luồng -> gọi -> IMU về 10 Hz). Còn: `local_position/pose` chưa ra vì trong
  nhà không có GPS fix - thử ngoài trời; (USB "rớt" 23/10 là rút tay, không phải lỗi).
  Firmware mới đổi tên: `SRx_*` -> `MAVn_*`, `RTL_ALT` -> `RTL_ALT_M`, `WPNAV_SPEED*` -> `WP_SPD*`.
  Node trên Jetson phải cùng `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` +
  `CYCLONEDDS_URI=file://$HOME/jetson/cyclonedds_tailscale.xml` với mjpeg-stream.service.
- [x] **4.4b `ros2_bridge.py` trên Jetson** (27/10/2026): service `drone-ros` (MAVROS + fire_locator +
  bridge 8765, `--mission-params ~/jetson/fire_mission_real.yaml`). Bridge gửi luôn giới hạn độ cao
  của YAML lên GUI: bảng "Chữa cháy tại đây" có "Bay tìm" 2-5 m và "Hạ cách lửa" 2..độ cao bay (mặc
  định 5 / 3), `fire_target` mang `drop`; node từ chối drop ngoài [min_alt, alt]. Sim: drop 6 m ->
  "DA THA BONG ... cach dat 6.0 m". Chưa: giao điểm cháy thật trên drone.
- [ ] **4.5 Tham số FC** — đọc lại 27/10/2026 (ArduCopter 4.7.0): ĐÃ SỬA `RTL_ALT_M` 2, failsafe pin
  (21.6 V RTL / 21.0 V LAND), mất RC = LAND, 1 la bàn, tắt lidar. Fence: user 27/10 KHÔNG bật
  (`FENCE_ENABLE` 0; node tự giới hạn alt 2-5 m, max_range 30 m, phạm vi tìm <= 10 m). `FS_GCS_ENABLE` bật cũng vô
  tác dụng với GUI_NATIVE: FC chỉ tính HEARTBEAT sysid 255, GUI là 254 và không gửi HEARTBEAT.
  Ghi chép cũ (đọc 23/10/2026): `RTL_ALT_M` 15 - QUÁ trần 5 m, phải hạ (RTL do failsafe bay về ở độ cao này, có thể
  đâm vật cản); `FENCE_ENABLE` 0 (bật thì `FENCE_ALT_MAX` 100 chặn task 120 m). ⚠ `BATT_FS_LOW_ACT` 0,
  `BATT_FS_CRT_ACT` 0, `BATT_CRT_VOLT` 0, `FS_GCS_ENABLE` 0 = failsafe pin và mất GCS đang TẮT, ngược
  mục 0. PreArm: `Rangefinder 1: Not Detected` (có cấu hình lidar), `Compasses inconsistent`.
- [x] **4.5b Vòng tìm drone thật 2 m** (user 23/10/2026): `real:=true` đặt `search_radius` 2 m (sim 24 m).
  Ở 20 m bước vòng 8/zoom m: x1-x3 chỉ treo ở điểm chấm rồi zoom, x4/x5 bay một vòng 2 / 1.6 m.
  Điểm chấm phải trúng đám cháy trong vài mét. Đổi: `search_radius:=...` khi launch.
  `max_zoom` 6 (sim 5): x6 = 640 px thật của khung 4K; ảnh lửa điện thoại x5 d0.6, x6 d0.78.
- [ ] **4.6 Bay tự hành theo bậc:** HITL → bay thấp với tấm ArUco in đúng 1.5 m → lửa thật.
  GPS không RTK lệch 1–3 m; mục tiêu đo tương đối bằng camera nên ít ảnh hưởng.
