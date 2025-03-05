# Allegro Hand Teleoperation
Using Ubuntu 20.04 and ROS Noetic
in case of GPG key error during ROS installation:
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

## Setup instructions Quest
Using sidequest on Linux Ubuntu 20.04:
I. Turn on developer mode in your Meta Quest device
II. Download SideQuest Desktop App from https://sidequestvr.com/setup-howto
tar -xf SideQuest-0.10.42.tar.xz
chmod +x SideQuest-0.10.42/sidequest
./SideQuest-0.10.42/sidequest --no-sandbox
in the upper-right corner press "Install APK from folder on your computer" and choose AllegroHandTeleoperation.apk
In Meta Quest navigate to apps, then "Unknown Sources" - there you will find the app installed and ready to use
Don't forget to connect to correct network!


## Setup instructions Allegro Hand Controller
git clone https://github.com/NYU-robot-learning/Allegro-Hand-Controller-DIME.git
sudo apt-get install cmake gcc g++ libpopt-dev
sudo apt-get install ros-noetic-libpcan


mkdir ~/ros_ws/drivers
cd ~/ros_ws/drivers
wget https://www.peak-system.com/quick/PCAN-Linux-Driver
tar -xvzf PCAN-Linux-Driver
cd peak-linux-driver-8.20.0/
make clean
make NET=NO_NETDEV_SUPPORT
sudo make install 
sudo modprobe pcan

cd ~/ros_ws/drivers
wget https://www.peak-system.com/quick/BasicLinux
tar -xvzf BasicLinux
cd PCAN-Basic_Linux-4.10.0.4/libpcanbasic/
make
sudo make install

# Note - sometimes the driver installation needs to be redone after pc restart

# Test the installation
cat /proc/pcan
ls -l /dev/pcan*

# Test the connection to robot
cd ~/ros_ws/Allegro-Hand-Controller-DIME
catkin_make
source devel/setup.bash
roslaunch allegro_hand allegro_hand.launch
Press "H" to home the robot

## Running teleoperation

add to bashrc:

source /opt/ros/noetic/setup.bash
#export PYTHONPATH=$HOME/RPL/DIME-IK-TeleOp:$PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages
#alias catkin_make='catkin_make -DPYTHON_EXECUTABLE=$HOME/ros_ws/Allegro-Hand-Teleoperation/ik_teleop/teleop_env/bin/python'
