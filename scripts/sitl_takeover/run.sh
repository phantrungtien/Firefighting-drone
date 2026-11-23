#!/bin/bash
# ./run.sh none|search|descend|home  - SITL octa-quad + MAVROS + fire_mission (tham so that) + gia GUI/lua
D=$(cd $(dirname $0) && pwd); SC=$1
source /opt/ros/humble/setup.bash; source ~/Firefighting-drone/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; unset CYCLONEDDS_URI
mkdir -p /tmp/sitl_takeover && cd /tmp/sitl_takeover
P=~/ardupilot/Tools/autotest/default_params
setsid ~/ardupilot/build/sitl/bin/arducopter -S -w --model octa-quad --speedup 1 -I0 \
  --home 10.8411,106.8098,10,0 --defaults $P/copter.parm,$P/copter-octaquad.parm,$D/real.parm > sitl.log 2>&1 & P1=$!
sleep 3
setsid ros2 launch fire_drone mavros.launch.py fcu_url:=tcp://127.0.0.1:5760 > mavros.log 2>&1 & P2=$!
setsid ros2 run fire_drone fire_mission --ros-args --params-file ~/jetson/fire_mission_real.yaml > mission_$SC.log 2>&1 & P3=$!
python3 -u $D/takeover_test.py $SC 2>&1 | tee test_$SC.log
sleep 3
grep -E "FIRE|PHI|FAILSAFE|Tu choi|Từ chối|THẢ|lúc đang|RTL|GUIDED|WARN|ERROR" mission_$SC.log | tail -15
for p in $P3 $P2 $P1; do kill -INT -- -$p 2>/dev/null; done; sleep 3; for p in $P3 $P2 $P1; do kill -9 -- -$p 2>/dev/null; done
sleep 2
