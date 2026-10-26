# fire_drone_ws

ROS 2 Humble + MAVROS cho drone ArduPilot cứu hỏa tự hành: người vận hành chấm
điểm cháy trên bản đồ GUI_NATIVE, drone tự bay tới, tự tìm đóm lửa bằng camera,
thả bóng chữa cháy, bay vòng kiểm tra rồi về. Mô phỏng: ArduCopter SITL + Gazebo
Harmonic. Đóm lửa trong sim là **mã ArUco** (camera/model nhận lửa không bắt được
lửa Gazebo).

## Chạy nhanh

```bash
cd ~/fire_drone_ws && source /opt/ros/humble/setup.bash && colcon build   # lần đầu / sau khi sửa code
~/fire_drone_ws/scripts/run_sim.sh          # Gazebo GUI + SITL + MAVROS + detector + ros2_bridge + video
cd ~/GUI_NATIVE && python3 -m laptop.app    # chọn profile "SITL"
~/fire_drone_ws/scripts/run_sim.sh stop     # tắt hết
```
Trong GUI: **⚑ Chữa cháy** (màn bay cảm ứng) hoặc tab Điều khiển → **Bắt đầu task**
→ giữ 2 s trên bản đồ chỗ nghi cháy → **Chữa cháy tại đây** → trượt xác nhận.
Bóng chỉ có một quả: muốn thả lại thì chạy lại `run_sim.sh`.

Tuỳ chọn: `HEADLESS=1` (không cửa sổ Gazebo), `VIDEO=0` (không MJPEG), `MISSION=1`
(bật luôn node nhiệm vụ, không cần GUI). Log từng tiến trình ở `logs/`.

## Cách hoạt động

```
 GUI_NATIVE (laptop)                       máy chạy sim / companion
 ───────────────────                       ───────────────────────────────────────────────
 nút task / chấm bản đồ ──WebSocket 8765──> ros2_bridge.py --spawn
                                              ├─ "mission" -> ros2 run fire_drone fire_mission
                                              └─ "fire_target" -> /gcs/command (JSON)
                                                                       │
 tab Thông báo <──MAVLink tcp:5762─────── ArduPilot SITL <─MAVROS─ fire_mission ◄── /fire/target
 (STATUSTEXT "FIRE: ...")                  ▲ JSON 9002                                ▲
                                           │                                   marker_detector
 khung camera <──MJPEG http 8080── mjpeg_server ◄── /rgb/zoomed ◄─ marker_detector ◄─ /rgb/image ◄─ Gazebo
                                                                                (camera nadir 70°)
```

**Ai làm gì**

| Thành phần | Việc |
|---|---|
| Gazebo (`worlds/firedrop_forest.sdf`) | Vật lý drone FireHawk X4 + bóng, camera nadir 1920x1080@15 Hz 70° có nhiễu (camera Jetson là 4K; sim chỉ thật tới zoom x2), tấm ArUco ở (18, 12) |
| ArduCopter SITL | Bộ điều khiển bay thật (code firmware), nối Gazebo qua JSON 9002; MAVLink 5760 (MAVROS), 5762 (GUI) |
| `mavros.launch.py` | MAVROS cho ArduPilot, đã cấm các plugin làm nó abort khi sim chậm, bật TF map->base_link |
| `marker_detector` | (chỉ sim, thay YOLO lửa) Ảnh -> zoom số -> ArUco DICT_4X4_50; mã trong `fire_ids` = đóm lửa; solvePnP (cạnh 1.5 m) -> vị trí trong `map`; xác nhận 5 khung, publish median 20 lần gần nhất lên `/fire/target` |
| `ground_range` | Khoảng cách bằng **camera 2D, không lidar**: khi hạ dh m, điểm trong vòng quanh đám cháy (bỏ đĩa giữa = lửa/khói, bỏ phần xa) nở s lần -> D = dh / (s − 1). KLT + RANSAC tách từng mặt (mái nhà, cỏ...), lấy mặt **gần nhất** -> `/fire/ground_z` (không gửi khi xa hơn `max_dist` 80 m: ảnh nở quá ít, đo sai 12–24 m). Chỉ chạy khi `fire_mission` bật `/fire/ground_enable` - lúc đã ở ngay trên đám cháy và bắt đầu hạ thẳng đứng. Cần mặt có vân (`models/ground_patch`, mái `models/house_c4`; sinh bằng `scripts/gen_ground_patch.py`) |
| `fire_mission` | Task Chữa cháy (dưới). GUI bật/huỷ qua bridge |
| `ros2_bridge.py` (GUI_NATIVE) | WebSocket <-> ROS2; bật/giết node task theo danh sách trắng |
| `mjpeg_server.py` (GUI_NATIVE) | Đẩy `/rgb/zoomed` (khung đã zoom, 960x540) ra `http://<máy>:8080/stream` (~4 fps, 25 KB/s, xem được qua LTE/Tailscale) |

**Task Chữa cháy (`fire_mission`)**

1. Chờ `/gcs/command {"action":"fire_target","args":{lat,lon,alt}}` (`alt` = độ cao chọn
   trên thanh trượt GUI, thiếu thì dùng `search_alt` 20 m). Lệnh từ mạng nên kiểm hết: sai
   kiểu / NaN / ngoài khoảng bị bỏ; điểm xa hơn `max_range` 300 m hay `alt` ngoài
   `min_alt`–`max_alt` (5–120 m) bị từ chối, báo lên tab Thông báo.
2. Đổi lat/lon -> toạ độ local (neo vào cặp GPS/pose hiện tại).
3. Đang dưới đất thì arm + cất cánh `alt` m; đang bay thì chuyển GUIDED. `alt` là độ cao
   **đi / tìm / về** — người vận hành chọn nó cao hơn cây, nhà trên đường bay. Khoảng
   cách vòng tìm co giãn theo (`ring_step * alt / 20`) để vẫn chồng mép.
4. Bay thẳng tới điểm chấm, treo `dwell` 5 s, rồi bay **vòng nở dần** quanh đó tới khi
   detector thấy đóm lửa (ở 20 m camera thấy ~28x16 m, vòng 8 m chồng mép). Không thấy
   thì về treo ở điểm chấm, **zoom x2** tìm lại, rồi x3... tới `max_zoom` 5 (zoom số:
   Jetson cắt giữa khung webcam 4K, luồng về GUI vẫn 960x540). Vòng tìm dày lên theo zoom.
   Lệnh zoom: `/fire/zoom` (std_msgs/Float32, transient local). Đo 18/10/2026 (mã 3 m, camera 960x540 vẽ thẳng):
   cần cạnh mã >= ~25 px mới đọc chắc (80 m x1: 80 %, 100 m: 18 %, 110 m: 0 %).
5. Bay ngang tới **ngay trên** đóm lửa (lệch < 2 m mới hạ — hạ chéo là cắt qua nhà giữa
   đường), hạ **thẳng đứng** xuống `drop_alt` 20 m trên đóm lửa (bay thấp hơn 20 m thì thả
   ở `alt`, không leo lên), vừa hạ vừa zoom ra dần về x1. "Trên đóm lửa" tính theo mặt
   đất `ground_range` đo bằng camera 2D trong lúc hạ (càng gần càng chuẩn; lọc bất đối xứng:
   số cho thấy mặt GẦN hơn - vừa thấy mái nhà/tán cây - tin ngay, mặt xa hơn phải trung vị 3
   số), KHÔNG theo z PnP của mã - lửa thật không có cỡ chuẩn. Thử 18/10/2026 với đóm lửa trên
   nóc nhà cấp 4 (mái 4 m): mái hiện ra ở ~27 m, drone dừng ~21 m trên mái, bóng lệch 7–14 cm. Chưa có số đo nào thì coi
   mặt đất ngang chỗ cất cánh và báo lên GCS.
   Lệch < 0.3 m, đúng độ cao, đứng yên 2 s -> `DO_GRIPPER` GRAB (servo GRIP_GRAB 1900, như gạt RC5 lên) thả bóng,
   2 s sau RELEASE (1100, như RC5 về) đóng lại.
   Vì sao 20 m: đo 18/10/2026, ở 100 m mã còn ~11 px, PnP lệch 1.8 m ngang / 9 m dọc.
6. **Vòng kiểm tra** bán kính 6 m ở độ cao thả: đếm số khung còn thấy đóm lửa -> báo
   "VAN THAY" / "khong con thay" (ở 100 m vòng này báo nhầm hết lửa).
7. Leo thẳng đứng lại `alt` rồi mới RTL (RTL bay về ở độ cao hiện tại - ở 20 m là đâm
   vào cái nhà lúc đi đã vượt qua). Bỏ task giữa chừng cũng vậy. Chờ disarm, node thoát.

Mỗi bước gửi STATUSTEXT `FIRE: ...` lên GCS (≤ 50 byte, không dấu).

**An toàn / quyền lái** — ngoại lệ có ý của nguyên tắc "laptop toàn quyền" của
GUI_NATIVE (`core/authority.py` bên đó): node chỉ lái khi người vận hành bật task VÀ
chấm điểm. Đổi mode khác GUIDED (RC, nút mode GUI) hay nút đỏ RTL/LAND/DISARM (đi SiK)
-> node buông ngay, **không thả, không tự đổi mode**. Huỷ task -> bridge giết node,
drone treo tại chỗ (GUIDED giữ điểm cuối).

## Đã đo trong sim (15–16/10/2026)

| | Kết quả |
|---|---|
| Task đầy đủ, điểm chấm lệch mã ~6 m | thấy mã sau 4 s tìm, thả lệch 0.01 m, RTL, tổng 126 s |
| Gạt LOITER giữa đường | node báo "PHI CONG LAI (LOITER): dung task", 12 s sau vẫn LOITER |
| Detector | lệch ~0.02 m khi treo trên mã |
| Bóng chạm đất | lệch 0.05 m (đo pose bóng 50 Hz) |

Trong sim vòng kiểm tra luôn "VAN THAY" vì tấm ArUco không tắt như lửa thật.

## Lưu ý mô phỏng

- **Gió** `<wind>` của world KHÔNG tác động lên drone hay bóng (không link nào có
  `<enable_wind>`) - độ chính xác thả ở trên là lạc quan.
- Chấm độ chính xác bằng điểm **chạm đất**, không bằng chỗ bóng nằm yên: Gazebo không
  có ma sát lăn nên bóng lăn mãi (56 s sau lăn tới 2 m).
- Camera có nhiễu Gaussian 0.007: camera sạch cho cảnh tĩnh ra JPEG giống từng byte
  -> GUI_NATIVE báo "camera đứng hình" (ảnh không đổi > 2 s).
- Gazebo GUI có khung "Camera nadir" (khối `<gui>` cuối world). Có `<gui>` trong world
  thì Gazebo bỏ qua `~/.gz/sim/8/gui.config`, nên khối đó chép đủ plugin mặc định.
- `run_sim.sh` cd vào `worlds/` trước khi chạy gz: gz tìm `model://` cả ở thư mục hiện
  hành, chạy từ `~` là vớ nhầm `~/terrain`.

## Cấu trúc

```
src/fire_drone/
  launch/mavros.launch.py         MAVROS cho ArduPilot (SITL hoặc FC thật)
  launch/fire_mission.launch.py   cầu nối camera + marker_detector (+ fire_mission nếu mission:=true)
  config/mavros_pluginlists.yaml  cấm plugin làm MAVROS abort khi sim chậm
  config/mavros_tf.yaml           bật TF map -> base_link
  config/drone_cam_*.yaml         calib webcam drone thật (camera_info ROS, 4K + 720p) - marker_detector -p calib:=
  config/firehawk_x4.parm         tham số SITL drone Gazebo (SR*, gripper SERVO9 thả bóng, PID)
  config/sitl.parm                SR0_* cho SITL không Gazebo
  fire_drone/guided.py            GuidedNode (connect/arm/takeoff/land) + takeoff_test
  fire_drone/marker_detector.py   ArUco -> /fire/target
  fire_drone/fire_mission.py      task Chữa cháy
scripts/run_sim.sh                bật/tắt cả bộ
scripts/calib_charuco.py          chụp + calib webcam bằng bảng ChArUco (chạy trên Jetson)
models/                           TẠM chép từ ~/firedrop_sim - thay bằng mô hình SolidWorks sau
  firehawk_x4                     quad ZD650 4.69 kg (hộp/trụ, chưa mesh), camera nadir có nhiễu
  elide_fireball, fireball_05kg   bóng chữa cháy - vật rời tì lên chốt bay1
  fire_marker                     tấm ArUco 2x2 m (vùng đen 1.5 m) = đóm lửa
  realsense_d435i, terrain, burning_house
worlds/firedrop_forest.sdf        bãi cỏ 200x200 m, mã ArUco ở (18, 12), khung camera trong GUI
```

Kiểm tra nhanh không cần sim:
```bash
python3 src/fire_drone/fire_drone/guided.py --selfcheck
python3 src/fire_drone/fire_drone/marker_detector.py --selfcheck
python3 src/fire_drone/fire_drone/fire_mission.py --selfcheck   # sau khi source install/setup.bash
```
Sửa file Python thì phải `colcon build` lại (bản cài là bản copy, không phải symlink).

## Chạy tay từng phần

```bash
# SITL không Gazebo + bài thử cất cánh 5 m
cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -w \
  --add-param-file=$HOME/fire_drone_ws/src/fire_drone/config/sitl.parm
source ~/fire_drone_ws/setup.bash && ros2 launch fire_drone mavros.launch.py
source ~/fire_drone_ws/setup.bash && ros2 run fire_drone takeoff_test --ros-args -p alt:=5.0

# Giao điểm cháy không qua GUI (khi MISSION=1)
ros2 topic pub --once -w 1 /gcs/command std_msgs/String \
  "{data: '{\"action\":\"fire_target\",\"args\":{\"lat\":<lat>,\"lon\":<lon>,\"alt\":20}}'}"
```
`-w` của sim_vehicle xoá EEPROM: `--add-param-file` chỉ là giá trị mặc định, EEPROM cũ sẽ đè nếu không xoá.

## Việc còn lại

Xem [CHECKLIST.md](CHECKLIST.md): đã kiểm gì (sim + thực tế), còn phải kiểm gì trước khi bay
tự hành thật.
