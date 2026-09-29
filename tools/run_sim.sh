#!/usr/bin/env bash
# Launch the Clover Gazebo simulation in this VM.
#
# Handles two traps that a bare `roslaunch` hits here:
#
# 1. PX4 SITL runs an interactive shell (pxh>). If stdin reaches EOF it prints
#    "Exiting NOW." and quits. roslaunch then reports a segfault in sitl_0 -
#    that segfault is teardown wreckage, NOT the cause. Running under nohup,
#    setsid, or with `< /dev/null` all trigger it. Piping from a process that
#    never closes keeps stdin open.
#
# 2. Gazebo needs an X display to create a GL context, even with gui:=false.
#    Without one, camera sensors fail ("Unable to create CameraSensor.
#    Rendering is disabled"), which starves optical flow, so the EKF never
#    gets a stable vision position and OFFBOARD cannot hold - the drone arms
#    but will not climb.
#
# Usage:  ./run_sim.sh [roslaunch args]
#   ./run_sim.sh                 # headless server, camera works
#   ./run_sim.sh gui:=true       # with the Gazebo GUI window
# NOTE: no `set -u` - ROS setup.bash references unbound variables
set -o pipefail

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

export DISPLAY="${DISPLAY:-:0}"
if [ -z "${XAUTHORITY:-}" ]; then
  for c in "/run/user/$(id -u)/gdm/Xauthority" "$HOME/.Xauthority"; do
    if [ -e "$c" ]; then export XAUTHORITY="$c"; break; fi
  done
fi

if ! DISPLAY="$DISPLAY" xdpyinfo >/dev/null 2>&1; then
  echo "WARNING: cannot reach X display '$DISPLAY'." >&2
  echo "         Camera sensors will not render and OFFBOARD will not hold." >&2
  echo "         Make sure the VM's desktop session is logged in." >&2
fi

ARGS="${*:-gui:=false}"
echo "launching: roslaunch clover_simulation simulator.launch $ARGS"
sleep infinity | roslaunch clover_simulation simulator.launch $ARGS
