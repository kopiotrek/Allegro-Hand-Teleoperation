# Allegro Hand Teleoperation

## Setup instructions

## Running teleoperation

add to bashrc:

source /opt/ros/noetic/setup.bash
export PYTHONPATH=$HOME/RPL/DIME-IK-TeleOp:$PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
alias catkin_make='catkin_make -DPYTHON_EXECUTABLE=$HOME/RPL/DIME-IK-TeleOp/ik_teleop/teleop_env/bin/python'
