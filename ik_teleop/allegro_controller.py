import concurrent.futures
import signal
import sys
import rospy
import os
import numpy as np
from copy import copy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
import multiprocessing
import time

JOINT_STATE_TOPIC = "/allegroHand/joint_states"

class AllegroController:
    def __init__(self):
        self.node_name = 'allegro_controller'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass
        self.finger_types = ['index', 'middle', 'ring', 'thumb']
        self.current_joint_state = JointState()
        self.processes = []  # Track subprocesses

        # To track publishing frequency
        self.last_publish_time = None  # Timestamp of the last message
        self.publish_timestamps = []  # List of (timestamp, frequency)

        self.joint_comm_publisher = rospy.Publisher('/allegroHand/joint_cmd', JointState, queue_size=1)
        self.joint_comm_delta_publisher = rospy.Publisher('/kth_franka_plant/in/allegro_cmd', JointState, queue_size=1)
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self._sub_callback_joint_state)
        rospy.Subscriber('/quest/keypoints_transformed', PoseArray, self._callback_knuckle_coordinates, queue_size=1)
        rospy.Subscriber('/allegroHand/index/joint_cmd_delta', JointState, self._sub_callback_index_delta_cmd)
        rospy.Subscriber('/allegroHand/middle/joint_cmd_delta', JointState, self._sub_callback_middle_delta_cmd)
        rospy.Subscriber('/allegroHand/ring/joint_cmd_delta', JointState, self._sub_callback_ring_delta_cmd)
        rospy.Subscriber('/allegroHand/thumb/joint_cmd_delta', JointState, self._sub_callback_thumb_delta_cmd)

        rospy.Subscriber('/kth_franka_plant/in/allegro_cmd', JointState, self._sub_callback_delta_cmd)
        rospy.loginfo(f'{self.node_name}: Initialized!')
	
    	
    def _callback_knuckle_coordinates(self, data):
        # Calculate average and lowest frequency from the last 10 seconds

        # Rest of the code for publishing desired joint states
        # try:
        oculus_keypoints = self.finger_coords[finger]
        cmd_delta_joint_state = list(self.index_delta_cmd[0:4]) + list(self.middle_delta_cmd[4:8]) + list(self.ring_delta_cmd[8:12]) + list(self.thumb_delta_cmd[12:])
        current_angles = self.current_joint_state.position

        desired_delta_js = copy(self.current_joint_state)
        desired_delta_js.position = list(np.array(cmd_delta_joint_state))
        desired_delta_js.effort = []
        desired_delta_js.velocity = []
        self.joint_comm_delta_publisher.publish(desired_delta_js)

        desired_angles = np.array(cmd_delta_joint_state) + np.array(current_angles)
        desired_angles[0] = 0
        desired_angles[4] = 0
        desired_angles[8] = 0
        desired_js = copy(self.current_joint_state)
        desired_js.position = list(desired_angles)
        desired_js.effort = []
        desired_js.velocity = []
        if self.index_mutex and self.middle_mutex and self.ring_mutex and self.thumb_mutex is True:
            current_time = rospy.get_time()  # Get current time

            # Calculate and log frequency
            if self.last_publish_time is not None:
                delta_time = current_time - self.last_publish_time
                frequency = 1.0 / delta_time
                self.publish_timestamps.append((current_time, frequency))  # Store timestamp and frequency
                #rospy.loginfo(f"{self.node_name}: Publishing frequency: {frequency:.2f} Hz")

            self.last_publish_time = current_time  # Update the last publish time
            self._calculate_recent_frequency_stats(current_time)

            self.joint_comm_publisher.publish(desired_js)
            self.index_mutex = False
            self.middle_mutex = False
            self.ring_mutex = False
            self.thumb_mutex = False
        # except:
        #     rospy.loginfo(f'{self.node_name}: ERROR: IK solutions missing! Waiting...')
        #     time.sleep(.5)
        #     pass




    def _sub_callback_delta_cmd(self, data):
        cmd_delta_joint_state = data.position
        current_angles = self.current_joint_state.position

        desired_angles = np.array(cmd_delta_joint_state) + np.array(current_angles)
        desired_angles[0] = 0
        desired_angles[4] = 0
        desired_angles[8] = 0
        
        desired_js = copy(self.current_joint_state)
        desired_js.position = list(desired_angles)
        desired_js.effort = []
        desired_js.velocity = []

        current_time = rospy.get_time()  # Get current time
        # Calculate and log frequency
        if self.last_publish_time is not None:
            delta_time = current_time - self.last_publish_time
            frequency = 1.0 / delta_time
            self.publish_timestamps.append((current_time, frequency))  # Store timestamp and frequency
            #rospy.loginfo(f"{self.node_name}: Publishing frequency: {frequency:.2f} Hz")
        self.last_publish_time = current_time  # Update the last publish time
        self._calculate_recent_frequency_stats(current_time)
        self.joint_comm_publisher.publish(desired_js)

    def _calculate_recent_frequency_stats(self, current_time):
        # Keep only the data from the last 10 seconds
        self.publish_timestamps = [
            (timestamp, frequency) for timestamp, frequency in self.publish_timestamps
            if current_time - timestamp <= 10
        ]

        if self.publish_timestamps:
            # Extract frequencies from the last 10 seconds
            frequencies = [freq for _, freq in self.publish_timestamps]
            average_frequency = np.mean(frequencies)
            lowest_frequency = np.min(frequencies)

            # Log or publish the stats
            #rospy.loginfo(f"{self.node_name}: Average frequency (last 10s): {average_frequency:.2f} Hz")
            #rospy.loginfo(f"{self.node_name}: Lowest frequency (last 10s): {lowest_frequency:.2f} Hz")

    # Because kth_franka_plant sends delta commands we need current angles to create new joint_cmd message
    def _sub_callback_joint_state(self, data):
        self.current_joint_state = data


    def _sub_callback_index_delta_cmd(self, data):
        self.index_delta_cmd = data.position
        self.index_mutex = True

    def _sub_callback_middle_delta_cmd(self, data):
        self.middle_delta_cmd = data.position
        self.middle_mutex = True

    def _sub_callback_ring_delta_cmd(self, data):
        self.ring_delta_cmd = data.position
        self.ring_mutex = True

    def _sub_callback_thumb_delta_cmd(self, data):
        self.thumb_delta_cmd = data.position
        self.thumb_mutex = True

    
    def run(self):
        try:
            rospy.spin()
        except Exception as e:
            rospy.loginfo(f"{self.node_name}: Exception occurred: {e}")
        finally:
            rospy.loginfo(f"{self.node_name}: Terminated.")



if __name__ == '__main__':
    node_name = 'allegro_controller'
    #rospy.init_node(node_name)
    allegro_controller = AllegroController()
    allegro_controller.run()
    # Set up signal handler for Ctrl+C


    # try:
    #     rospy.spin()
    # except Exception as e:
    #     rospy.loginfo(f"{node_name}: Exception occurred: {e}")
    # finally:
    #     rospy.loginfo(f"{node_name}: Terminated.")
