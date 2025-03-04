#!/usr/bin/env python3

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from xml.etree import ElementTree as ET
from ik_teleop.teleop_utils.files import *

import numpy as np

class AllegroKeypointsPublisher:
    def __init__(self):
        rospy.init_node('allegro_keypoints_publisher')
        self.urdf_file = get_path_in_package("robot/assets/allegro_hand_right.urdf")
        rospy.Subscriber('/allegroHand/tf', TFMessage, self.tf_callback)
        self.pub = rospy.Publisher('/allegroHand/keypoints', PoseArray, queue_size=10)
        rospy.loginfo("Started Allegro Hand TF Listener Node")
        self.tree = ET.parse(self.urdf_file)
    
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
        # rospy.loginfo(f"Tip Joint: {tip_joint}, Pose: Position({combined_translation}), Orientation({combined_rotation})")
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

        self.pub.publish(pose_array_msg)


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
    listener = AllegroKeypointsPublisher()
    listener.run()
