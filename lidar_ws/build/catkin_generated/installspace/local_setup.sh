#!/usr/bin/env sh
# generated from catkin/cmake/template/local_setup.sh.in

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
<<<<<<< HEAD
: ${_CATKIN_SETUP_DIR:=/home/zxj/桌面/learngit/lidar_ws/install}
=======
: ${_CATKIN_SETUP_DIR:=/home/zxj/learngit/lidar_ws/install}
>>>>>>> 09226450a16102fd150477c4c838dbded307c8c4
CATKIN_SETUP_UTIL_ARGS="--extend --local"
. "$_CATKIN_SETUP_DIR/setup.sh"
unset CATKIN_SETUP_UTIL_ARGS
