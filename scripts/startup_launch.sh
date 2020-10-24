#!/bin/bash

function log() {
  logger -s -p user.$1 ${@:2}
}

log info "picam: Using workspace setup file /opt/ros/kinetic/setup.bash"
log info "picam: Using workspace setup file /home/ubuntu/catkin_ws/devel/setup.bash"
source /opt/ros/kinetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
JOB_FOLDER=/etc/ros/kinetic/picam.d

log_path="/tmp"
launch_path="/home/ubuntu/catkin_ws/src/ros_picam/launch"

export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://core-armada-1.local:11311
export ROS_HOME=${ROS_HOME:=$(echo ~ubuntu)/.ros}
export ROS_LOG_DIR=$log_path

log info "picam: ROS_HOSTNAME=$ROS_HOSTNAME"
log info "picam: ROS_MASTER_URI=$ROS_MASTER_URI, ROS_HOME=$ROS_HOME"
log info "picam: LAUNCH_PATH=$launch_path, ROS_LOG_DIR=$log_path"

LAUNCH_FILENAME=$launch_path/client.launch
# Launch
roslaunch $LAUNCH_FILENAME --wait &
PID=$!

log info "picam: Started roslaunch as background process, PID $PID, ROS_LOG_DIR=$ROS_LOG_DIR"
echo "$PID" > $log_path/picam.pid

wait "$PID"
