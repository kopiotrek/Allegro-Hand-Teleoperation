import numpy as np
from abc import ABC
from copy import deepcopy as copy
from .allegro_kdl import AllegroKDL
from  ik_teleop.teleop_utils.files import *
from  ik_teleop.teleop_utils.vectorops import *

class AllegroKinematicControl(ABC):
    def __init__(self, bounded_angles = True):
        # np.set_printoptions(suppress = True)

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


class AllegroJointControl(AllegroKinematicControl):
    def __init__(self, bounded_angles = True):
        super().__init__(bounded_angles)
        # np.set_printoptions(suppress = True)

        self.linear_scaling_factors = self.bound_info['linear_scaling_factors']
        self.rotatory_thumb_scaling_factors = self.bound_info['rotatory_thumb_scaling_factors']

    def _get_filtered_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        
        for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx]
                

        for idx in range(1, 16):
            if desired_angles[idx] > 2.5:
                desired_angles[idx] = 0

        return desired_angles 

    def _get_filtered_thumb_angles(self, finger_type, calc_finger_angles, curr_angles, moving_avg_arr):
        
        curr_finger_angles = self._get_curr_finger_angles(curr_angles, finger_type)
        avg_finger_angles = moving_average(calc_finger_angles, moving_avg_arr, self.time_steps)       
        desired_angles = np.array(copy(curr_angles))

        
        for idx in range(self.hand_configs['joints_per_finger']):
                desired_angles[self.finger_configs['links_info'][finger_type]['offset'] + idx] = avg_finger_angles[idx-1]
                

        return desired_angles 


    def calculate_finger_angles(self, finger_type, finger_joint_coords, metacarpals_coords, curr_angles, moving_avg_arr):
        calc_finger_angles = []
        # rotatory_joint_angle = self.calculate_rotatory_joint_angle(finger_type, finger_joint_coords, metacarpals_coords)
        rotatory_joint_angle = 0.0
        calc_finger_angles.append(rotatory_joint_angle * self.linear_scaling_factors[0])
        if finger_type == 'index':
            idx = 1
        elif finger_type == 'middle':
            idx = 2
        elif finger_type == 'ring':
            idx = 3
        angle = calculate_angle(
            metacarpals_coords[idx],
            finger_joint_coords[0],
            finger_joint_coords[1]
        )

        angle *= self.linear_scaling_factors[idx]
        if angle > 1.71:
            angle = 1.71
        elif angle < -0.296:
            angle = -0.296
        calc_finger_angles.append(angle)
        for idx in range(1, self.hand_configs['joints_per_finger']-1):
            # print(f"finger_joint_coords{idx} {finger_joint_coords[idx]}")

            angle = calculate_angle(
                finger_joint_coords[idx - 1],
                finger_joint_coords[idx],
                finger_joint_coords[idx + 1]
            )
            angle *= self.linear_scaling_factors[idx]
            if angle > 1.71:
                angle = 1.71
            elif angle < -0.274:
                angle = -0.274
            calc_finger_angles.append(angle)

        filtered_angles = self._get_filtered_angles(finger_type, calc_finger_angles, curr_angles, moving_avg_arr)
        return filtered_angles


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
        # print(f"finger_joint_coords {finger_joint_coords}")
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

    def finger_1D_motion(
        self, 
        finger_type, 
        hand_y_val, 
        robot_x_val, 
        robot_y_val, 
        y_hand_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 1D control along the Z direction - used in index and middle fingers at a fixed depth and fixed y
        '''
        x_robot_coord = robot_x_val
        y_robot_coord = robot_y_val
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_2D_motion(
        self, 
        finger_type, 
        hand_x_val,
        hand_y_val, 
        robot_x_val, 
        x_hand_bound, 
        y_hand_bound, 
        y_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in Y and Z directions - used in ring finger at a fixed depth
        '''
        x_robot_coord = robot_x_val
        y_robot_coord = linear_transform(hand_x_val, x_hand_bound, y_robot_bound)
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles

    def finger_2D_depth_motion(
        self, 
        finger_type, 
        hand_y_val, 
        robot_y_val, 
        hand_z_val, 
        y_hand_bound, 
        z_hand_bound, 
        x_robot_bound, 
        z_robot_bound, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in X and Z directions - used in index and middle fingers at a varied depth
        '''
        x_robot_coord = linear_transform(hand_z_val, z_hand_bound, x_robot_bound)
        y_robot_coord = robot_y_val
        z_robot_coord = linear_transform(hand_y_val, y_hand_bound, z_robot_bound)
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]

        desired_angles = self.calculate_desired_angles(finger_type, transformed_coords, moving_avg_arr, curr_angles)
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


    def thumb_motion_2D(
        self, 
        hand_coordinates, 
        xy_hand_bounds, 
        yz_robot_bounds, 
        robot_x_val, 
        moving_avg_arr, 
        curr_angles
    ):
        '''
        For 2D control in Y and Z directions - human bounds are mapped to robot bounds
        '''
        y_robot_coord, z_robot_coord = perspective_transform(
            (hand_coordinates[0], hand_coordinates[1]), 
            xy_hand_bounds, 
            yz_robot_bounds
        )

        x_robot_coord = robot_x_val        
        transformed_coords = [x_robot_coord, y_robot_coord, z_robot_coord]
        
        desired_angles = self.calculate_desired_angles('thumb', transformed_coords, moving_avg_arr, curr_angles)
        return desired_angles
