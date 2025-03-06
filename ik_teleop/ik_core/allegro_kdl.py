from ikpy import chain
import rospy
import numpy as np
from copy import deepcopy as copy
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.constants import *
from ik_teleop.teleop_utils.vectorops import *
from ik_teleop.ik_core.allegro_ik import ThumbIK, FingerIK 
import time
import warnings
warnings.filterwarnings('ignore', category=UserWarning, module='ikpy')


from visualization_msgs.msg import Marker

THUMB_IK_MARKER_TOPIC = '/ik_marker/thumb'
INDEX_IK_MARKER_TOPIC = '/ik_marker/index'
MIDDLE_IK_MARKER_TOPIC = '/ik_marker/middle'
RING_IK_MARKER_TOPIC = '/ik_marker/ring'

class AllegroKDL(object):
    def __init__(self):
        self.node_name = "allegro_kdl"
        # try:
        #     rospy.init_node(self.node_name)
        # except rospy.ROSException as e:
        #     rospy.loginfo(f'Node initialization failed: {self.node_name}')
        #     pass
        # Getting the URDF path
        urdf_path = get_path_in_package("robot/assets/allegro_hand_right.urdf")
        # urdf_path = get_path_in_package("robot/assets/allegro_hand_right.urdf")

        # Loading Allegro Hand configs
        self.hand_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_info.yaml"))
        # self.hand_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_info.yaml"))
        self.finger_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))
        # self.finger_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))

        # Parsing chains from the urdf file
        self.chains = {}
        for finger in self.hand_configs['fingers'].keys():
            self.chains[finger] = chain.Chain.from_urdf_file(
                urdf_path, 
                base_elements = [
                    self.finger_configs['links_info']['base']['link'], 
                    self.finger_configs['links_info'][finger]['link']
                ], 
                name = finger
            )
        self.thumb_ik = ThumbIK()
        self.finger_ik = FingerIK()
        self.thumb_ik_marker_publisher = rospy.Publisher(THUMB_IK_MARKER_TOPIC, Marker, queue_size=1)
        self.index_ik_marker_publisher = rospy.Publisher(INDEX_IK_MARKER_TOPIC, Marker, queue_size=1)
        self.ring_ik_marker_publisher = rospy.Publisher(RING_IK_MARKER_TOPIC, Marker, queue_size=1)
        self.middle_ik_marker_publisher = rospy.Publisher(MIDDLE_IK_MARKER_TOPIC, Marker, queue_size=1)

    
    def finger_forward_kinematics(self, finger_type, input_angles):
        # Checking if the number of angles is equal to 4
        if len(input_angles) != self.hand_configs['joints_per_finger']:
            rospy.loginfo(f'{self.node_name}: Incorrect number of angles')
            return 

        # Checking if the input finger type is a valid one
        if finger_type not in self.hand_configs['fingers'].keys():
            rospy.loginfo(f'{self.node_name}: Finger type does not exist')
            return
        
        # Clipping the input angles based on the finger type
        finger_info = self.finger_configs['links_info'][finger_type]
        for iterator in range(len(input_angles)):
            if input_angles[iterator] > finger_info['joint_max'][iterator]:
                input_angles[iterator] = finger_info['joint_max'][iterator]
            elif input_angles[iterator] < finger_info['joint_min'][iterator]:
                input_angles[iterator] = finger_info['joint_min'][iterator]

        # Padding values at the beginning and the end to get for a (1x6) array
        input_angles = list(input_angles)
        input_angles.insert(0, 0)
        input_angles.append(0)

        # Performing Forward Kinematics 
        output_frame = self.chains[finger_type].forward_kinematics(input_angles)
        return output_frame[:3, 3], output_frame[:3, :3]

    
    def create_marker_msg(self, position, scale, r, g, b, a):
        marker = Marker()
        marker.header.frame_id = "palm_link"  # Change to your frame of reference if needed
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a  # Alpha (transparency)
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        return marker


    def finger_inverse_kinematics(self, finger_type, input_position, curr_finger_angles):
        # Checking if the input figner type is a valid one
        if finger_type not in self.hand_configs['fingers'].keys():
            rospy.loginfo(f'{self.node_name}: Finger type does not exist')
            return

        if finger_type == 'thumb':

            input_position += [0,0,-0.023]

            self.thumb_ik_marker_publisher.publish(self.create_marker_msg(input_position, 0.02, 1, 0, 0, 1))

            rotation_angles = (np.pi + np.deg2rad(5), 0, 0)
            input_position += [0.0182, -0.016958, 0.073288]
            input_position = rotate_point(input_position, rotation_angles)

            output_angles = self.thumb_ik.compute_ik(input_position)
            # rospy.loginfo(f"Computed Joint Angles (IK) {finger_type}:", output_angles)
            output_angles = np.append(output_angles, 0)
            return output_angles[0:4]

        elif finger_type == 'index':

            input_position += [-0.034,0,0] #because it is the distance from IK goal to robot hand surface

            self.index_ik_marker_publisher.publish(self.create_marker_msg(input_position, 0.02, 1, 0, 0, 1))
            
            rotation_angles = (-np.deg2rad(5), 0, 0)
            input_position = rotate_point(input_position, rotation_angles)
            input_position += [0, -0.045098, -0.014293]

            output_angles = self.finger_ik.compute_ik(finger_type, input_position, curr_finger_angles)
            return output_angles[0:4]

        elif finger_type == 'middle':

            input_position += [-0.034,0,0] #because it is the distance from IK goal to robot hand surface

            self.middle_ik_marker_publisher.publish(self.create_marker_msg(input_position, 0.02, 1, 0, 0, 1))


            input_position += [0, 0, -0.0166]



            output_angles = self.finger_ik.compute_ik(finger_type, input_position, curr_finger_angles)
            return output_angles[0:4]

        elif finger_type == 'ring':

            input_position += [-0.034,0,0] #because it is the distance from IK goal to robot hand surface

            self.ring_ik_marker_publisher.publish(self.create_marker_msg(input_position, 0.02, 1, 0, 0, 1))

            input_position += [0, 0.045098, -0.014293]
            rotation_angles = (-np.deg2rad(5), 0, 0)
            input_position = rotate_point(input_position, rotation_angles)

            output_angles = self.finger_ik.compute_ik(finger_type, input_position, curr_finger_angles)
            return output_angles[0:4]

        

    def get_fingertip_coords(self, joint_positions):
        # index_coords = self.finger_forward_kinematics('index', joint_positions[:4])[0]
        # middle_coords = self.finger_forward_kinematics('middle', joint_positions[4:8])[0]
        # ring_coords = self.finger_forward_kinematics('ring', joint_positions[8:12])[0]
        # thumb_coords = self.finger_forward_kinematics('thumb', joint_positions[12:16])[0]
        
        index_coords = self.finger_forward_kinematics('index', joint_positions[OCULUS_JOINTS['index']])[0]
        middle_coords = self.finger_forward_kinematics('middle', joint_positions[OCULUS_JOINTS['middle']])[0]
        ring_coords = self.finger_forward_kinematics('ring', joint_positions[OCULUS_JOINTS['ring']])[0]
        thumb_coords = self.finger_forward_kinematics('thumb', joint_positions[OCULUS_JOINTS['thumb']])[0]


        finger_tip_coords = np.hstack([index_coords, middle_coords, ring_coords, thumb_coords])
        return np.array(finger_tip_coords)

   

# if __name__ == '__main__':
#     ik_control = AllegroKDL()

#     # Set desired position and orientation
#     # output_frame = ik_control.finger_forward_kinematics('thumb',[ 0.49158065,  0.62981548, -2.99420419,  3.29378238])  # Example position
#     thumb_joint_angles = ik_control.finger_inverse_kinematics('thumb', [ 0.02698166,  0.16099207, -0.07196472])
#     rospy.loginfo(f"thumb_joint_angles {thumb_joint_angles}")
#     # ik_control.rotation = np.array([1, 0, 0, 0])     # Identity quaternion

#     # Perform the IK update
#     # ik_control.Update()