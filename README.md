# Allegro Hand Teleoperation

This guide will help you set up and run teleoperation for the Allegro Hand robot.

---

## Prerequisites

### Get a machine with Ubuntu 20.04 and install ROS Noetic
If you encounter a **GPG key error** during ROS installation, run:
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
```

---

## 1. Install Allegro-Hand-Teleoperation

### Clone this repository

### Add Environment Variables to `.bashrc`
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export PYTHONPATH=$HOME/ros_ws/Allegro-Hand-Teleoperation:\$PYTHONPATH" >> ~/.bashrc
echo "export PYTHONPATH=\$PYTHONPATH:/opt/ros/noetic/lib/python3/dist-packages" >> ~/.bashrc
source ~/.bashrc
```

### Set Up Virtual Environment
```bash
sudo apt install python3.8-venv python3-pip
cd ~/ros_ws/Allegro-Hand-Teleoperation/ik_teleop/
python3 -m venv venv_teleop
source venv_teleop/bin/activate
pip install -r requirements_teleop.txt
```

## 2. Setup Instructions for Allegro Hand Controller

### Install Dependencies & Clone Repository
```bash
git clone https://github.com/NYU-robot-learning/Allegro-Hand-Controller-DIME.git
sudo apt-get install cmake gcc g++ libpopt-dev ros-noetic-libpcan
```

### Build the Controller
```bash
cd ~/ros_ws/Allegro-Hand-Controller-DIME
catkin_make
```

---

## 3. Install and Configure PEAK-CAN Drivers

### Install PEAK System CAN Drivers
```bash
mkdir ~/ros_ws/drivers && cd ~/ros_ws/drivers
wget https://www.peak-system.com/quick/PCAN-Linux-Driver
```
```bash
tar -xvzf PCAN-Linux-Driver
cd peak-linux-driver-8.20.0/
make clean
make NET=NO_NETDEV_SUPPORT
sudo make install
sudo modprobe pcan
```

### Install PCAN-Basic API
```bash
cd ~/ros_ws/drivers
wget https://www.peak-system.com/quick/BasicLinux
```
```bash
tar -xvzf BasicLinux
cd PCAN-Basic_Linux-4.10.0.4/libpcanbasic/
make
sudo make install
```
> **Note:** Sometimes the driver installation needs to be redone after a PC restart.

### Test the Driver Installation
```bash
cat /proc/pcan
ls -l /dev/pcan*
```

### Test Connection to the Allegro Hand
```bash
cd ~/ros_ws/Allegro-Hand-Controller-DIME
catkin_make
source devel/setup.bash
roslaunch allegro_hand allegro_hand.launch
```
Press **H** to home the robot.

---

## 4. Install ROS-TCP-Endpoint

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ..
catkin_make
```


---

## 5. Setup Instructions for Meta Quest

To use Meta Quest for teleoperation, follow these steps:

### Install SideQuest on Ubuntu 20.04
1. Enable **Developer Mode** on your Meta Quest device.
2. Download **SideQuest Desktop App** from [SideQuestVR](https://sidequestvr.com/setup-howto):
   ```bash
   tar -xf SideQuest-0.10.42.tar.xz
   chmod +x SideQuest-0.10.42/sidequest
   ./SideQuest-0.10.42/sidequest --no-sandbox
   ```
3. In SideQuest, click **"Install APK from folder on your computer"**, then select `AllegroHandTeleoperation.apk`.
4. On your Meta Quest, go to **Apps > Unknown Sources** to find and launch the installed application.
5. **Ensure your device is connected to the correct network!**

## 6.

### Start the Teleoperation
1. **Ensure the Allegro Hand is connected and the driver is working.**
2. Activate the virtual environment:
   ```bash
   source ~/ros_ws/Allegro-Hand-Teleoperation/ik_teleop/venv_teleop/bin/activate
   ```
3. Run the teleoperation script:
   ```bash
   python ~/ros_ws/Allegro-Hand-Teleoperation/start_teleop.py --ip=192.168.7.XXX
   ```
4. **Press "START ALL"** in the interface.
5. Start the Allegro-Hand-Teleoperation app in Meta Quest, check the IP and otherwise just click continue everywhere, then press skip
6. Teleoperate the hand!

---

## Troubleshooting
- If the **PCAN driver doesn't work after a reboot**, try reinstalling it using the steps in Section 3.
- Ensure all dependencies are installed correctly.
- Make sure the **Allegro Hand is connected and powered on** before running teleoperation.
- Double-check that your **Meta Quest and PC are on the same network**.

---

With this guide, you should be able to successfully set up and run Allegro Hand Teleoperation. 🚀

