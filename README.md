# Allegro Hand Teleoperation

## Setup instructions Quest
I. Using sidequest on Linux Ubuntu 20.04:I. Turn on developer mode in your Meta Quest device
II. Download SideQuest Desktop App from https://sidequestvr.com/setup-howtotar -xf SideQuest-0.10.42.tar.xz
chmod +x SideQuest-0.10.42/sidequest
./SideQuest-0.10.42/sidequest --no-sandbox
in the upper-right corner press "Install APK from folder on your computer" and choose AllegroHandTeleoperation.apk
In Meta Quest navigate to apps, then "Unknown Sources" - there you will find the app installed and ready to use


## Setup instructions Allegro Hand Controller
Clone https://github.com/NYU-robot-learning/Allegro-Hand-Controller-DIME.git


## Running teleoperation

add to bashrc:

source /opt/ros/noetic/setup.bash
export PYTHONPATH=$HOME/RPL/DIME-IK-TeleOp:$PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
alias catkin_make='catkin_make -DPYTHON_EXECUTABLE=$HOME/RPL/DIME-IK-TeleOp/ik_teleop/teleop_env/bin/python'
