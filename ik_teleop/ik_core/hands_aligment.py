#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from xml.etree import ElementTree as ET
import numpy as np
from ik_teleop.teleop_utils.constants import *
from copy import deepcopy as copy
from ik_teleop.teleop_utils.vectorops import *
import time 
from ik_teleop.teleop_utils.files import *
from visualization_msgs.msg import Marker, MarkerArray


ROBOT_JOINTS_RAW = {
    'index': [0, 10, 12, 11],
    'middle': [13, 15, 17, 16],
    'ring': [18, 2, 4, 3],
    'thumb': [6, 7, 9, 8],
    'knuckles': [0, 13, 8],
}
ROBOT_JOINTS_NK = { 
    'index': [0, 1, 2, 3],
    'middle': [4, 5, 6, 7],
    'ring': [8, 9, 10, 11],
    'thumb': [12, 13, 14, 15],
}

ROBOT_KEYPOINTS_COUNT = 20

class AllegroRetargetingOptimizer:
    def __init__(self):
        self.node_name = 'allegro_retargeting_optimizer'
        try:
            rospy.init_node(self.node_name)
        except rospy.ROSException as e:
            rospy.loginfo(f'Node initialization failed: {self.node_name}')
            pass


        rospy.Subscriber('/quest/joint_poses', PoseArray, callback=self._get_XR_joints_poses, queue_size=1)
        self.pub_mod = rospy.Publisher('/quest/keypoints_transformed', PoseArray, queue_size=10)
        self.pub_marker_mod = rospy.Publisher('/quest/marker_keypoints_transformed', MarkerArray, queue_size=10)
        self.pub = rospy.Publisher('/quest/keypoints', PoseArray, queue_size=10)
        self.pub_marker = rospy.Publisher('/quest/marker_keypoints', MarkerArray, queue_size=10)

        self.finger_coords = []
        self.finger_orientations = []
        self.robot_coords = []
        self.robot_coords_array = []
        self.keypoint_translation_array = []
        # self.knuckle_points = (OCULUS_JOINTS['knuckles'][3], OCULUS_JOINTS['knuckles'][0])

        #####################################
        self.urdf_file = get_path_in_package("robot/assets/allegro_hand_right.urdf")
        rospy.Subscriber('/allegroHand/tf', TFMessage, self.tf_callback)
        self.tree = ET.parse(self.urdf_file)
        self.allegro_keypoints_pose_array = []

        rospy.loginfo(f"{self.node_name}: Started AllegroRetargetingOptimizer Node")





    def _get_XR_joints_poses(self, msg):
        finger_coords_array = []
        finger_orientation_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS:
            rospy.loginfo(f"{self.node_name}: ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        self.finger_coords_array_np = np.array(finger_coords_array)
        for i in range(OCULUS_NUM_KEYPOINTS):
            finger_orientation_array.append(self.orientation_to_array(msg.poses[i].orientation)) 
        self.finger_orientation_array_np = np.array(finger_orientation_array)

        self.transform_keypoints()
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            index = self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle = self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring = self.finger_coords_array_np[OCULUS_JOINTS['ring']],
            thumb = self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
        )
        
        self.pub_marker.publish(self.create_marker_array_msg(1,0,0))
        self.pub.publish(self.create_pose_array_msg())

        self.get_keypoint_difference()

        self.align_hand_to_robot()

        self.pub_marker_mod.publish(self.create_marker_array_msg(0,1,0))
        self.pub_mod.publish(self.create_pose_array_msg())

    def create_pose_array_msg(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "palm_link"
        for finger in self.finger_coords:
            for joint in range(0, len(self.finger_coords[finger])):
                pose = Pose()
                pose.position.x = self.finger_coords[finger][joint][0]
                pose.position.y = self.finger_coords[finger][joint][1]
                pose.position.z = self.finger_coords[finger][joint][2]
                pose.orientation.x = self.finger_orientations[finger][joint][0]
                pose.orientation.y = self.finger_orientations[finger][joint][1]
                pose.orientation.z = self.finger_orientations[finger][joint][2]
                pose.orientation.w = self.finger_orientations[finger][joint][3]
                pose_array_msg.poses.append(pose)
        return pose_array_msg
        
    def create_marker_msg(self, position, r=1.0, g=0.0, b=0.0, a=.5, scale=0.02):
        marker = Marker()
        marker.header.frame_id = "palm_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        return marker

    def create_marker_array_msg(self, r, g, b):
        marker_array_msg = MarkerArray()
        marker_id = 0
        for finger in self.finger_coords:
            for joint in self.finger_coords[finger]:
                marker = self.create_marker_msg(joint, r, g, b)
                marker.id = marker_id
                marker_array_msg.markers.append(marker)
                marker_id += 1
        return marker_array_msg

    def _translate_coords(self, coords):
        return copy(coords) - coords[12]
        
    def _get_coord_frame(self, wrist_coord, index_knuckle_coord, little_knuckle_coord):
        z_axis = normalize_vector(wrist_coord)
        x_axis = normalize_vector(index_knuckle_coord - little_knuckle_coord)
        y_axis = normalize_vector(np.cross(z_axis, x_axis))
        
        return [y_axis, x_axis, -z_axis] # Change from left-handed unity system to right-handed

    def transform_keypoints(self):
        self.finger_coords_array_np = self._translate_coords(self.finger_coords_array_np)
        original_coord_frame = self._get_coord_frame(
            self.finger_coords_array_np[OCULUS_JOINTS['wrist'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['index'][0]],
            self.finger_coords_array_np[OCULUS_JOINTS['ring'][0]]
        )

        if np.linalg.det(original_coord_frame) == 0:
            rospy.logerr("Original coord frame is singular and cannot be inverted")
            return

        try:
            # Compute the rotation matrix
            rotation_matrix = np.linalg.solve(original_coord_frame, np.eye(3)).T

            # Transform positions
            finger_coords = (rotation_matrix @ self.finger_coords_array_np.T).T

            # Transform orientations
            transformed_orientations = []
            for quaternion in self.finger_orientation_array_np:
                rotation_matrix_quat = quaternion_matrix(quaternion)[:3, :3]
                transformed_matrix = rotation_matrix @ rotation_matrix_quat
                full_transform_matrix = np.eye(4)
                full_transform_matrix[:3, :3] = transformed_matrix
                transformed_quat = quaternion_from_matrix(full_transform_matrix)
                transformed_orientations.append(transformed_quat)

            self.finger_coords_array_np = finger_coords
            self.finger_orientation_array_np = np.array(transformed_orientations)
        except np.linalg.LinAlgError as e:
            rospy.logerr(f"Error computing rotation matrix: {e}")

        # Update finger coordinate dictionary
        self.finger_coords = dict(
            wrist=self.finger_coords_array_np[OCULUS_JOINTS['wrist']],
            thumb=self.finger_coords_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_coords_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_coords_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_coords_array_np[OCULUS_JOINTS['ring']],
        )
        # Update finger orientation dictionary
        self.finger_orientations = dict(
            wrist=self.finger_orientation_array_np[OCULUS_JOINTS['wrist']],
            thumb=self.finger_orientation_array_np[OCULUS_JOINTS['thumb']],
            index=self.finger_orientation_array_np[OCULUS_JOINTS['index']],
            middle=self.finger_orientation_array_np[OCULUS_JOINTS['middle']],
            ring=self.finger_orientation_array_np[OCULUS_JOINTS['ring']],
        )

        

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def orientation_to_array(self, orientation):
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])




    def get_keypoint_difference(self):

        # Create an empty list to store the Euclidean distances between corresponding keypoints
        self.keypoint_difference_array = []
        # Iterate over each finger and calculate the Euclidean distance between corresponding keypoints
        for finger in ROBOT_JOINTS_NK:
            # Get the robot's keypoints for this finger
            robot_keypoints = self.robot_coords[finger]
            # Get the oculus' keypoints for this finger
            oculus_keypoints = self.finger_coords[finger]


            # Ensure both robot_keypoints and oculus_keypoints are arrays (for multiple keypoints in a finger)
            for idx, (r_point, o_point) in enumerate(zip(robot_keypoints, oculus_keypoints)):
                formatted_r_point = np.array([f"{coord:.5f}" for coord in r_point])
                formatted_o_point = np.array([f"{coord:.5f}" for coord in o_point])
            
                # Calculate Euclidean distance between corresponding robot and Oculus keypoints
                translation = r_point - o_point
                self.keypoint_translation_array.append(translation)


    def align_hand_to_robot(self):
        # Robot finger lengths (in meters)
        robot_finger_lengths = {
            'index': 0.1527,
            'middle': 0.1527,
            'ring': 0.1527,
            'thumb': 0.1308, #0.0363 is real length of servo no.1
        }

        # Calculate scaling factors for each finger based on their lengths
        finger_scales = {}
        for finger in ['index', 'middle', 'ring', 'thumb']:
            # Calculate the total length of the current finger
            total_length = 0.0
            finger_joints = self.finger_coords[finger]
            for i in range(len(finger_joints) - 1):
                link_length = np.linalg.norm(finger_joints[i] - finger_joints[i + 1])
                total_length += link_length

            # Calculate the scaling factor (robot length / XR length)
            scaling_factor = robot_finger_lengths[finger] / total_length
            finger_scales[finger] = scaling_factor
            

        # Apply the scaling factors and translation to align fingers with the robot's measurements
        translation_array_offset = {'index': 0, 'middle': 4, 'ring': 8, 'thumb': 12}
        for finger in self.finger_coords:
            if finger in translation_array_offset:
                for joint_idx in range(0, len(self.finger_coords[finger])):
                    if joint_idx != 0:
                        scale = finger_scales[finger]
                        tmp_coord = self.finger_coords[finger][joint_idx] - self.finger_coords[finger][0]
                        tmp_coord *= scale
                        self.finger_coords[finger][joint_idx] = tmp_coord + self.finger_coords[finger][0]
                        self.finger_coords[finger][joint_idx] = tmp_coord + self.finger_coords[finger][0]
                        self.finger_coords[finger][joint_idx] += self.keypoint_translation_array[translation_array_offset[finger]]
                    else:
                        self.finger_coords[finger][joint_idx] += self.keypoint_translation_array[translation_array_offset[finger]]

        self.keypoint_translation_array = []

    def parse_origin(self, element):
        xyz = element.attrib.get('xyz', '0 0 0').split()
        rpy = element.attrib.get('rpy', '0 0 0').split()

        translation = np.eye(4)
        translation[:3, 3] = list(map(float, xyz))

        roll, pitch, yaw = map(float, rpy)
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])
        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        rotation = Rz @ Ry @ Rx

        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation[:3, 3]

        return transform

    def get_transform_from_urdf(self, start_joint, end_joint):
        root = self.tree.getroot()

        joint_map = {}
        for joint in root.findall('joint'):
            joint_name = joint.attrib['name']
            origin = joint.find('origin')
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']

            joint_map[joint_name] = {
                'origin': self.parse_origin(origin),
                'parent': parent,
                'child': child
            }

        chain = []
        current_joint = start_joint
        while True:
            if current_joint not in joint_map:
                raise ValueError(f"Joint {current_joint} not found in URDF")
            chain.append(current_joint)
            child_link = joint_map[current_joint]['child']

            if any(joint.attrib['name'] == end_joint and joint.find('parent').attrib['link'] == child_link for joint in root.findall('joint')):
                chain.append(end_joint)
                break

            next_joint = None
            for joint in root.findall('joint'):
                if joint.find('parent').attrib['link'] == child_link:
                    next_joint = joint.attrib['name']
                    break

            if not next_joint:
                raise ValueError(f"Cannot find a joint connecting link {child_link} to the chain")

            current_joint = next_joint

        transform = np.eye(4)
        for joint in chain:
            transform = transform @ joint_map[joint]['origin']

        return transform

    def compute_transform_chain(self, child_frame, parent_frame, transforms_map):
        total_translation = np.array([0.0, 0.0, 0.0])
        total_rotation_matrix = np.eye(4)

        current_frame = child_frame
        while current_frame != parent_frame:
            if current_frame not in transforms_map:
                raise ValueError(f"No transform found for frame {current_frame}")

            transform = transforms_map[current_frame]
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ])
            rotation = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]

            current_transform_matrix = quaternion_matrix(rotation)
            current_transform_matrix[:3, 3] = translation
            total_rotation_matrix = np.dot(current_transform_matrix, total_rotation_matrix)

            current_frame = transform.header.frame_id

        total_translation = total_rotation_matrix[:3, 3]
        return total_translation, total_rotation_matrix

    def quaternion_multiply(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ]

    def calculate_fingertip_pose(self, tip_joint, translation, rotation_quaternion):
        parent_joint = tip_joint.replace("_tip","")
        urdf_transform = self.get_transform_from_urdf(parent_joint, tip_joint)
        urdf_translation = urdf_transform[:3, 3]
        urdf_translation[2]*=0.6 # to represent true fingertip

        urdf_rotation_quaternion = quaternion_from_matrix(urdf_transform)

        # First, transform the URDF translation to the same frame as the joint's translation
        transformed_urdf_translation = self.apply_transform(translation, rotation_quaternion, urdf_translation)

        # Combine rotation (multiply the quaternions: joint's rotation * URDF's rotation)
        combined_rotation = rotation_quaternion

        # Combine the final translation (add the transformed URDF translation to the joint translation)
        combined_translation = [
            translation[0] + transformed_urdf_translation[0],
            translation[1] + transformed_urdf_translation[1],
            translation[2] + transformed_urdf_translation[2]
        ]

        # Prepare the tip pose
        tip_pose = Pose()
        tip_pose.position.x = combined_translation[0]
        tip_pose.position.y = combined_translation[1]
        tip_pose.position.z = combined_translation[2]
        tip_pose.orientation.x = combined_rotation[0]
        tip_pose.orientation.y = combined_rotation[1]
        tip_pose.orientation.z = combined_rotation[2]
        tip_pose.orientation.w = combined_rotation[3]
        return(tip_pose)

    def tf_callback(self, data):
        transforms_map = {t.child_frame_id: t for t in data.transforms}

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = "palm_link"

        for transform in data.transforms:
            child_frame = transform.child_frame_id
            pose = Pose()

            if child_frame == "link_X":
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0
            else:
                try:
                    translation, rotation_matrix = self.compute_transform_chain(child_frame, "palm_link", transforms_map)
                    rotation_quaternion = quaternion_from_matrix(rotation_matrix)

                    pose.position.x = translation[0]
                    pose.position.y = translation[1]
                    pose.position.z = translation[2]    
                    pose.orientation.x = rotation_quaternion[0]
                    pose.orientation.y = rotation_quaternion[1]
                    pose.orientation.z = rotation_quaternion[2]
                    pose.orientation.w = rotation_quaternion[3]
                    # rospy.loginfo(f"Joint: {child_frame}, Pose: Position({translation})")

                    # Add fingertip poses if necessary
                    if "link_3" in child_frame:
                        pose_array_msg.poses.append(self.calculate_fingertip_pose("joint_3.0_tip", translation, rotation_quaternion))
                    elif "link_7" in child_frame:
                        pose_array_msg.poses.append(self.calculate_fingertip_pose("joint_7.0_tip", translation, rotation_quaternion))
                    elif "link_11" in child_frame:
                        pose_array_msg.poses.append(self.calculate_fingertip_pose("joint_11.0_tip", translation, rotation_quaternion))
                    elif "link_15" in child_frame:
                        pose_array_msg.poses.append(self.calculate_fingertip_pose("joint_15.0_tip", translation, rotation_quaternion))

                except ValueError as e:
                    rospy.logwarn(e)

            pose_array_msg.poses.append(pose)
            
        for i in range(ROBOT_KEYPOINTS_COUNT):
            self.robot_coords_array.append(self.position_to_array(pose_array_msg.poses[i].position))    
        self.robot_coords_array_np = np.array(self.robot_coords_array)
        self.robot_coords = dict(
            index = self.robot_coords_array_np[ROBOT_JOINTS_RAW['index']],
            middle = self.robot_coords_array_np[ROBOT_JOINTS_RAW['middle']],
            ring = self.robot_coords_array_np[ROBOT_JOINTS_RAW['ring']],
            thumb = self.robot_coords_array_np[ROBOT_JOINTS_RAW['thumb']],
        )

    def apply_transform(self, base_translation, base_rotation_quaternion, urdf_translation):
        """Apply the URDF translation and rotate it by the base's rotation."""
        # Convert the translation to the base frame
        urdf_translation_vector = np.array(urdf_translation)
        rotation_matrix = self.quaternion_to_matrix(base_rotation_quaternion)

        # Apply the rotation
        transformed_urdf_translation = np.dot(rotation_matrix, urdf_translation_vector)

        return transformed_urdf_translation.tolist()

    def quaternion_to_matrix(self, quaternion):
        """Convert a quaternion to a rotation matrix."""
        x, y, z, w = quaternion
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
        ])

    def run(self):
        rospy.spin()



if __name__ == "__main__":
    listener = AllegroRetargetingOptimizer()
    listener.run()