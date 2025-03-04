import numpy as np
from scipy.optimize import minimize


class ThumbIK:
    def __init__(self):
        self.articulation_chain = [{"position": 0.225} for _ in range(4)]
        self.position = np.zeros(3)
        self.rotation = np.eye(3)
        self.x_des = np.eye(4)
        self.dh_params = []
        self.TEE = np.eye(4)
        # Add joint limits
        self.q_min = np.array([0.225, -0.368, -0.281, -0.262])  # Minimum joint angles
        self.q_max = np.array([1.555, 1.152, 1.719, 1.799])    # Maximum joint angles

    def get_current_state(self):
        return np.array([joint["position"] for joint in self.articulation_chain])

    def set_dh_params(self, joint_angles):
        self.dh_params = [
            [0.0, 0.0,  np.pi/2,           joint_angles[0]],          # Joint 1
            [0.0, 0.0554,  -np.pi/2,       joint_angles[1]-np.pi/2], # Joint 2
            [0.0514, 0.0,  0.0,            joint_angles[2]-np.pi/2],          # Joint 3
            [0.0593, 0.0,  0.0,            joint_angles[3]]           # Joint 4
        ]

    def get_transformation_matrix(self, i, dh):
        a, d, alpha, theta = dh[i]
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta)*np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_TEE(self):
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            T = T @ self.get_transformation_matrix(i, self.dh_params)
        self.TEE = T



    def compute_ik(self, desired_position):
        """
        Compute inverse kinematics using optimization with joint limits.
        """
        def objective(q):
            # Update DH parameters and compute forward kinematics
            self.set_dh_params(q)
            self.compute_TEE()
            # Compute position error
            pos_error = desired_position - self.TEE[:3, 3]
            # Return squared error norm
            return np.sum(pos_error**2)

        # Initial joint angles
        q0 = self.get_current_state().astype(float)

        # Bounds for joint limits as a sequence of (min, max) pairs
        bounds = [(self.q_min[i], self.q_max[i]) for i in range(len(q0))]

        # Solve IK using optimization
        result = minimize(
            objective,
            q0,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-10, 'maxiter': 100}
        )

        if result.success:
            print(f"Converged in {result.nit} iterations.")
            q = result.x
        else:
            print("IK did not converge.")
            q = q0  # Return initial guess or handle as needed

        return q

class FingerIK:
    def __init__(self):
        self.articulation_chains = [[{"position": 0.0} for _ in range(4)] for _ in range(3)]
        self.dh_params = []
        self.TEE = np.eye(4)
        # Add joint limits
        self.q_min = np.array([-0.57, -0.296, -0.274, -0.327])  # Minimum joint angles
        self.q_max = np.array([0.57, 1.71, 1.809, 1.718])    # Maximum joint angles

    def set_dh_params(self, joint_angles):

        # self.dh_params = [
        #     # Trans X, Trans Z, Rot X, Rot Z
        #     [0.0,       0.0166,  -np.pi/2,          joint_angles[0]],          # Joint 1
        #     [0.054,     0.0,     0.0,               joint_angles[1]-np.pi/2],         # Joint 2
        #     [0.0384,    0.0,     0.0,               joint_angles[2]],          # Joint 3
        #     [0.0437,    0.0,     0.0,               joint_angles[3]]           # Joint 4
        # ]
        self.dh_params = [
            # Trans X, Trans Z, Rot X, Rot Z
            [0.0,       0.0166,  -np.pi/2,          0],          # Joint 1
            [0.054,     0.0,     0.0,               joint_angles[1]-np.pi/2],         # Joint 2
            [0.0384,    0.0,     0.0,               joint_angles[2]],          # Joint 3
            [0.0437,    0.0,     0.0,               joint_angles[3]]           # Joint 4
        ]
    def get_transformation_matrix(self, i, dh):
        a, d, alpha, theta = dh[i]
        # a = trans_x
        # d = trans_z
        # alpha = rot_x
        # theta = rot_z
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def compute_TEE(self, finger_index):
        T = np.eye(4)
        for i in range(len(self.dh_params)):
            T = T @ self.get_transformation_matrix(i, self.dh_params)
        self.TEE = T

    def compute_ik(self, finger_type, desired_position, q0):
        """
        Compute inverse kinematics using optimizaboundstion with joint limits.
        """

        if finger_type == 'index':
            finger_index = 0
        elif finger_type == 'middle':
            finger_index = 1
        elif finger_type == 'ring':
            finger_index = 2
        else:
            print(f"Wrong finger type {finger_type}")
        def objective(q):
            # Update DH parameters and compute forward kinematics
            self.set_dh_params(q)
            self.compute_TEE(finger_index)
            # Compute position error
            pos_error = desired_position - self.TEE[:3, 3]
            # Compute the variance of the error components
            variance_penalty = np.var(pos_error)
            # Return squared error norm with penalty
            # print(f"pos_error**2 {np.sum(pos_error**2)} variance_penalty {variance_penalty}")
            return np.sum(pos_error**2) + variance_penalty

        # Bounds for joint limits as a sequence of (min, max) pairs
        bounds = [(self.q_min[i], self.q_max[i]) for i in range(len(q0))]

        # Solve IK using optimization
        result = minimize(
            objective,
            q0,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-10, 'maxiter': 100}
        )

        if result.success:
            print(f"Converged in {result.nit} iterations.")
            q = result.x
            self.set_dh_params(q0)
            self.compute_TEE(finger_index)
            # pos_error = desired_position - self.TEE[:3, 3]
            # print(f"actual_position: {self.TEE[:3, 3]}")
            # print(f"desired_position: {desired_position}")
            # print(f"pos_error: {pos_error}")
            # print(f"q: {q}")
        else:
            print("IK did not converge.")
            q = q0  # Return initial guess or handle as needed

        return q