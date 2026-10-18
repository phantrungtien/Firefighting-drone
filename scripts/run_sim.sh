#!/usr/bin/env bash
# Bật cả bộ: Gazebo (GUI + khung camera) -> SITL -> MAVROS -> detector
# + ros2_bridge (cửa của GUI_NATIVE) + video MJPEG.
#
# Task Chữa cháy bật TỪ GUI_NATIVE (nút task / tab Điều khiển) - bridge khởi động
# node fire_mission, rồi chạm-giữ bản đồ -> "Chữa cháy tại đây".
#
#   ./scripts/run_sim.sh            chạy, để nguyên cho xem
#   HEADLESS=1 ./scripts/run_sim.sh không GUI Gazebo
#   MISSION=1 ./scripts/run_sim.sh  bật luôn node fire_mission (không cần GUI;
#                                   giao điểm cháy bằng /gcs/command, xem README)
#   ./scripts/run_sim.sh stop       tắt hết
#
# Mỗi tiến trình một file log trong logs/. Thứ tự dựa trên trạng thái sẵn sàng
# (cổng 9002, cổng 5760), không sleep cứng: lần đầu waf build lại arducopter
# mất cả phút, sleep cứng thì MAVROS nối vào cổng chưa mở và chết câm.
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="$WS/logs"

stop() {
  # mẫu [x]... để pkill không khớp chính dòng lệnh của script này
  pkill -f "[m]avros_node"; pkill -f "[a]rducopter -S"; pkill -f "[x]term.*ArduCopter"
  pkill -f "[s]im_vehicle.py"  # wrapper không tự chết theo arducopter -> mỗi lần chạy sót một con
  pkill -f "[p]arameter_bridge"; pkill -f "[m]arker_detector"; pkill -f "[g]round_range"; pkill -f "[f]ire_mission"
  pkill -f "[r]os2_bridge.py"
  pkill -f "[g]z sim"
  # gz sim gui có lúc lờ SIGTERM (18/10/2026 còn sống sau 5 s) -> chờ 3 s rồi giết hẳn
  for _ in 1 2 3; do pgrep -f "[g]z sim" > /dev/null || break; sleep 1; done
  pkill -KILL -f "[g]z sim"
  # -KILL: rclpy nuốt SIGTERM để tắt phần ROS, luồng HTTP vẫn sống giữ cổng 8080
  # -> lần chạy sau server mới chết vì "Address already in use".
  pkill -KILL -f "[m]jpeg_server"
  return 0
}
[ "${1:-}" = "stop" ] && { stop; echo "da tat"; exit 0; }

stop; sleep 2
mkdir -p "$LOG"
source "$WS/setup.bash"

GUI=""; [ "${HEADLESS:-0}" = "1" ] && GUI="-s"
# cd vào worlds/: gz tìm model:// cả ở THƯ MỤC HIỆN HÀNH. Gọi script từ ~ là nó vớ
# nhầm ~/terrain (thư mục khác, không có model.config) -> world không lên (16/10/2026).
# Không dùng logs/: SITL tạo logs/terrain/ ở đó.
(cd "$WS/worlds" && setsid -f gz sim $GUI -v3 -r "$WS/worlds/firedrop_forest.sdf" > "$LOG/gz.log" 2>&1)
echo "[cho] Gazebo mo cong FDM 9002..."
for i in $(seq 1 120); do ss -lun | grep -qE ":9002\b" && break; sleep 1; done

# -w xoá EEPROM: --add-param-file chỉ là MẶC ĐỊNH, EEPROM cũ sẽ đè nếu không xoá.
(cd "$LOG" && setsid -f ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris \
   --model JSON --no-mavproxy -w --add-param-file="$WS/src/fire_drone/config/firehawk_x4.parm" \
   > "$LOG/sitl.log" 2>&1)
echo "[cho] SITL mo cong 5760 (lan dau co the build lai ~60 s)..."
for i in $(seq 1 300); do ss -ltn | grep -qE ":5760\b" && break; sleep 1; done

setsid -f ros2 launch fire_drone mavros.launch.py > "$LOG/mavros.log" 2>&1
M="false"; [ "${MISSION:-0}" = "1" ] && M="true"
setsid -f ros2 launch fire_drone fire_mission.launch.py mission:=$M > "$LOG/mission.log" 2>&1

# Camera sim -> MJPEG cổng 8080 (mjpeg_server của ~/GUI_NATIVE): xem bằng trình
# duyệt http://<máy này>:8080/stream, qua Tailscale cũng được. Nhẹ hơn hẳn đẩy cả
# màn hình Gazebo qua LTE. --every 3 --quality 60 ~ 5 fps, cỡ 1 Mbit/s. Khung ĐÃ ZOOM
# (/rgb/zoomed của marker_detector, 960x540) như Jetson gửi về, không phải khung gốc.
# Tắt: VIDEO=0. Không có ~/GUI_NATIVE thì im lặng bỏ qua.
MJPEG="$HOME/GUI_NATIVE/tools/mjpeg_server.py"
if [ "${VIDEO:-1}" != "0" ] && [ -f "$MJPEG" ]; then
  setsid -f python3 "$MJPEG" --ros-topic /rgb/zoomed --every 3 --quality 60 > "$LOG/video.log" 2>&1
  echo "Video camera:   http://$(tailscale ip -4 2>/dev/null || echo 127.0.0.1):8080/stream"
fi
# Cửa của GUI_NATIVE: WebSocket 8765 <-> ROS2. --spawn để nút task bật được
# fire_mission (danh sách trắng trong bridge). Chạy SAU khi đã source setup.bash
# của workspace này, không thì `ros2 run fire_drone ...` của bridge không thấy gói.
BRIDGE="$HOME/GUI_NATIVE/tools/ros2_bridge.py"
if [ -f "$BRIDGE" ]; then
  setsid -f python3 "$BRIDGE" --spawn > "$LOG/bridge.log" 2>&1
  echo "GUI_NATIVE:     profile 'SITL' -> nut task Chua chay"
fi
echo "Dang chay. Xem:  tail -f $LOG/mission.log"
echo "Tat:            $WS/scripts/run_sim.sh stop"
