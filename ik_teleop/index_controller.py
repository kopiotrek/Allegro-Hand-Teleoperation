import rospy
import os
from geometry_msgs.msg import PoseArray
import numpy as np
from datetime import datetime
from ik_teleop.ik_core.allegro_retargeters import AllegroKDL
from ik_teleop.ik_core.allegro_operator import AllegroHandOperator
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time


# List of all ROS Topics
XR_KEYPOINTS_TOPIC = '/quest/keypoints_transformed' 
PAUSE_TELEOP_TOPIC = '/quest/pause' 
JOINT_STATE_TOPIC = '/allegroHand/joint_states' 

class TeleOp(object):
    def __init__(self):
        self.finger_type = 'index'
        self.node_name = self.finger_type + '_controller'
        rospy.loginfo(f"{self.finger_type} controller starting...")

        try:
            rospy.init_node(f'hardware_teleop_{self.finger_type}')
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.finger_type+str(e)}')
            pass
        self.desired_joint_angles = np.array([0.0, 0.28113237, 0.16851817, 0.0, 0.0, 0.17603329, 
            0.21581194, 0.0, 0.2928223, 0.16747166, 1.45242466, 1.45812127, 0.69531447, 1.1, 1.1, 1.1])
        self.desired_joint_angles_delta = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.allegro_hand_config = get_yaml_data(get_path_in_package("configs/allegro_sim.yaml"))

        self.allegro_hand_operator = AllegroHandOperator(self.allegro_hand_config)
        self.current_joint_pose = None
        self.cmd_joint_state = None
        self.pause = True

        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber(XR_KEYPOINTS_TOPIC, PoseArray, self._callback_knuckle_coordinates, queue_size=1)
        rospy.Subscriber(PAUSE_TELEOP_TOPIC, Bool, self._sub_pause_teleop, queue_size=1)
        
        # rospy.Subscriber(JOINT_COMM_DELTA_TOPIC, JointState, self._sub_callback_joint_cmd)

        # self.joint_comm_publisher = rospy.Publisher(f'/allegroHand/{self.finger_type}/joint_cmd', JointState, queue_size=1)
        self.joint_comm_publisher_delta = rospy.Publisher(f'/allegroHand/{self.finger_type}/joint_cmd_delta', JointState, queue_size=1)
        rospy.loginfo(f"{self.node_name}: Initialized!")
    

    def _sub_callback_joint_state(self, data):
        self.current_joint_pose = data

    def _sub_pause_teleop(self, data):
        if data.data:
            rospy.loginfo(f"{self.node_name}: ||")
        else:
            rospy.loginfo(f"{self.node_name}: ▷")
        self.pause = data.data

    # def _sub_callback_joint_cmd(self, data):
    #     cmd_joint_state = data.position
    #     current_angles = self.current_joint_pose.position

    #     desired_angles = np.array(cmd_joint_state) + np.array(current_angles)

    #     desired_js = copy(self.current_joint_pose)
    #     desired_js.position = list(desired_angles)
    #     desired_js.effort = list([])
    #     desired_js.velocity = list([])
    #     self.joint_comm_publisher.publish(desired_js)


    def hand_pose(self, action=np.zeros(16)):
        if self.current_joint_pose == None:
            rospy.loginfo(f'{self.node_name}: No joint data received!')
            return
    
        current_angles = np.array(self.current_joint_pose.position)  # Convert JointState to numpy array
        if self.finger_type == 'index':
            action[:4] = action[:4]  # Use the provided action values
            action[5:] = current_angles[5:]  # Retain the current joint angles

        elif self.finger_type == 'middle':
            action[:4] = current_angles[:4]  # Retain the current joint angles
            action[5:8] = action[5:8]  # Use the provided action values
            action[9:] = current_angles[9:]  # Retain the current joint angles

        elif self.finger_type == 'ring':
            action[:8] = current_angles[:8]  # Retain the current joint angles
            action[9:12] = action[9:12]  # Use the provided action values
            action[13:] = current_angles[13:]  # Retain the current joint angles

        elif self.finger_type == 'thumb':
            action[:12] = current_angles[:12]  # Retain the current joint angles
            action[13:] = action[13:]  # Use the provided action values

        desired_angles = np.array(action)
        desired_angles_delta = desired_angles - current_angles  # Correct subtraction

        self.desired_joint_angles_delta = copy(self.current_joint_pose)
        self.desired_joint_angles_delta.position = list(desired_angles_delta)
        self.desired_joint_angles_delta.effort = list([])
        self.desired_joint_angles_delta.velocity = list([])
        self.joint_comm_publisher_delta.publish(self.desired_joint_angles_delta)
    
    def _callback_knuckle_coordinates(self, msg):
        if not self.pause:
            self.desired_joint_angles = self.allegro_hand_operator._apply_retargeted_angles(self.finger_type)
            self.hand_pose(self.desired_joint_angles)
        else:
            if self.current_joint_pose != None:
                current_angles_array = np.array(self.current_joint_pose.position)  # Ensure correct type
                self.hand_pose(current_angles_array)

    def control_finger(self):
        while not rospy.is_shutdown():
            continue

if __name__ == '__main__':
    tp = TeleOp()
    tp.control_finger()