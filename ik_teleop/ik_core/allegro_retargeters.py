import numpy as np
from abc import ABC
from copy import deepcopy as copy
from .allegro_kdl import AllegroKDL
from  ik_teleop.teleop_utils.files import *
from  ik_teleop.teleop_utils.vectorops import *

class AllegroKinematicControl(ABC):
    def __init__(self, bounded_angles = True):

        # Loading the Allegro Hand configs
        self.hand_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_info.yaml"))
        self.finger_configs = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_link_info.yaml"))
        self.bound_info = get_yaml_data(get_path_in_package("robot/allegro/configs/allegro_bounds.yaml"))

        self.time_steps = self.bound_info['time_steps']

        self.bounded_angles = bounded_angles
        self.bounds = {}
        for finger in self.hand_configs['fingers'].keys():
            self.bounds[finger] = np.array(self.bound_info['jointwise_angle_bounds'][
                self.finger_configs['links_info'][finger]['offset'] : self.finger_configs['links_info'][finger]['offset'] + 4
            ])

    def _get_curr_finger_angles(self, curr_angles, finger_type):
        return np.array(curr_angles[
            self.finger_configs['links_info'][finger_type]['offset'] : self.finger_configs['links_info'][finger_type]['offset'] + 4
        ])


class AllegroKDLControl(AllegroKinematicControl):
    def __init__(self,  bounded_angles = True):
        super().__init__(bounded_angles)
        self.solver = AllegroKDL()
        self.ajc = AllegroJointControl()

    def calculate_desired_angles(
        self, 
        finger_type, 
        finger_joint_coords, 
        moving_avg_arr, 
        curr_angles
    ):
        tip_coord = finger_joint_coords[3]

        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)  
        calc_finger_angles = self.solver.finger_inverse_kinematics(finger_type, tip_coord, curr_finger_angles)

        desired_angles = np.array(copy(curr_angles))

        # Applying angular bounds
        if self.bounded_angles is True:
            del_finger_angles = calc_finger_angles - curr_finger_angles
            clipped_del_finger_angles = np.clip(del_finger_angles, - self.bounds[finger_type], self.bounds[finger_type])
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] += clipped_del_finger_angles[idx]
        else:
            for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = calc_finger_angles[idx]

        return desired_angles 

    def finger_3D_motion(
        self, 
        finger_type,
        finger_joint_coords, 
        moving_avg_arr, 
        curr_angles
    ):
        # Compute the desired joint angles based on the transformed coordinates
        return self.calculate_desired_angles(
            finger_type,
            finger_joint_coords, 
            moving_avg_arr, 
            curr_angles
        )
