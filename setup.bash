# source ~/fire_drone_ws/setup.bash
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

# Gazebo: model:// tìm trong models/, ArduPilotPlugin lấy từ ~/gz_ws đã build.
# Nối thêm, không ghi đè - ghi đè là mất plugin của workspace khác.
case ":$GZ_SIM_RESOURCE_PATH:" in *":$WS/models:"*) ;; *)
  export GZ_SIM_RESOURCE_PATH="$WS/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}" ;; esac
AP_GZ="$HOME/gz_ws/src/ardupilot_gazebo/build"
if [ -f "$AP_GZ/libArduPilotPlugin.so" ]; then
  case ":$GZ_SIM_SYSTEM_PLUGIN_PATH:" in *":$AP_GZ:"*) ;; *)
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$AP_GZ${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}" ;; esac
else
  echo "CANH BAO: khong thay $AP_GZ/libArduPilotPlugin.so -> drone Gazebo khong noi duoc SITL" >&2
fi
