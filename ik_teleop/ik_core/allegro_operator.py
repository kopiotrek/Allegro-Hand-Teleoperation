from copy import deepcopy as copy
import rospy
from geometry_msgs.msg import PoseArray, Pose

from shapely.geometry import Point, Polygon 
from shapely.ops import nearest_points
from ik_teleop.ik_core.allegro_retargeters import *
from ik_teleop.teleop_utils.files import *
from ik_teleop.teleop_utils.vectorops import *
from ik_teleop.teleop_utils.constants import *

class AllegroHandOperator:
    def __init__(self, finger_configs):
        self.node_name = "allegro_hand_operator"
        if not rospy.core.is_initialized():
            try:
                rospy.init_node('allegro_hand_operator')
            except rospy.ROSInterruptException:
                pass
        self._transformed_hand_keypoint_subscriber = rospy.Subscriber('/quest/keypoints_transformed', PoseArray, callback=self._get_joints_poses, queue_size=1)
        JOINT_COUNT = 26
        # Initializing the  finger configs
        self.finger_configs = finger_configs
        self.finger_coords = []
        self.finger_orientations = []

        #Initializing the solvers for allegro hand
        self.fingertip_solver = AllegroKDLControl()
        #self.finger_joint_solver = AllegroJointControl()

        # Initialzing the moving average queues
        self.moving_average_queues = {
            'thumb': [],
            'index': [],
            'middle': [],
            'ring': []
        }

        self.last_desired_joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.263, 0.0, 0.0, 0.0])

        # Getting the bounds for the allegro hand
        allegro_bounds_path = get_path_in_package('configs/allegro.yaml')
        self.allegro_bounds = get_yaml_data(allegro_bounds_path)

    def position_to_array(self, position):
        return np.array([position.x, position.y, position.z])

    def orientation_to_array(self, orientation):
        return np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    # Get the joints poses
    def _get_joints_poses(self, msg):
        finger_coords_array = []
        finger_orientation_array = []
        if len(msg.poses) < OCULUS_NUM_KEYPOINTS_TRANSFORMED:
            rospy.loginfo(f"{self.node_name}: ERROR: not enough joints received")
            return
        for i in range(OCULUS_NUM_KEYPOINTS_TRANSFORMED):
            # finger_poses_array.append(msg.poses[i])
            finger_coords_array.append(self.position_to_array(msg.poses[i].position))     
        finger_coords_array_np = np.array(finger_coords_array)
        for i in range(OCULUS_NUM_KEYPOINTS_TRANSFORMED):
            # finger_poses_array.append(msg.poses[i])
            finger_orientation_array.append(self.orientation_to_array(msg.poses[i].orientation)) 
        finger_orientation_array_np = np.array(finger_orientation_array)
        self.finger_coords = dict(
            wrist = finger_coords_array_np[OCULUS_JOINTS_TRANSFOMED['wrist']],
            index = finger_coords_array_np[OCULUS_JOINTS_TRANSFOMED['index']],
            middle = finger_coords_array_np[OCULUS_JOINTS_TRANSFOMED['middle']],
            ring = finger_coords_array_np[OCULUS_JOINTS_TRANSFOMED['ring']],
            thumb = finger_coords_array_np[OCULUS_JOINTS_TRANSFOMED['thumb']],
        )

        self.finger_orientations = dict(
            wrist = finger_orientation_array_np[OCULUS_JOINTS_TRANSFOMED['wrist']],
            index = finger_orientation_array_np[OCULUS_JOINTS_TRANSFOMED['index']],
            middle = finger_orientation_array_np[OCULUS_JOINTS_TRANSFOMED['middle']],
            ring = finger_orientation_array_np[OCULUS_JOINTS_TRANSFOMED['ring']],
            thumb = finger_coords_array_np[OCULUS_JOINTS_TRANSFOMED['thumb']],
        )
        return

    # Apply the retargeted angles to the robot
    def _apply_retargeted_angles(self, finger_type):
        # while not rospy.is_shutdown():
            desired_joint_angles = self.last_desired_joint_angles

            # IK
            desired_joint_angles = self.fingertip_solver.finger_3D_motion(
                    finger_type = finger_type,
                    finger_joint_coords = self.finger_coords[finger_type],
                    moving_avg_arr = self.moving_average_queues[finger_type],
                    curr_angles = desired_joint_angles
                )

            self.last_desired_joint_angles = desired_joint_angles

            
            return desired_joint_angles

            
