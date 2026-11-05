#!/bin/bash
# Start tools/teleop_web.py on the Pi with the fleet link: ROS 2 sourced,
# fleet_link next to it under /ferb. Auto only uses the turns with
# fleet.enabled: true in tools/teleop_web.yaml.
# Usage: scripts/start_teleop_fleet.sh   (ROS_DOMAIN_ID defaults to 0)
REPO=$(cd "$(dirname "$0")/.." && pwd)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
source /opt/ros/lyrical/setup.bash
source "$REPO/install/setup.bash"

pkill -f 'python3.*[t]eleop_web.py'
pkill -f '[f]leet_link'
sleep 1

cfg=$(ros2 pkg prefix mobile_base)/share/mobile_base/config/collector.yaml
nohup ros2 run mobile_base fleet_link --ros-args -r __ns:=/ferb \
    --params-file "$cfg" -p robot_name:=ferb -p want_timeout_s:=3.0 \
    > /tmp/fleet_link.log 2>&1 < /dev/null &
disown

cd "$REPO"
nohup python3 tools/teleop_web.py > /tmp/teleop_web.log 2>&1 < /dev/null &
disown
