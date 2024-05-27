# Author: Nithin Vasishta (adapted from motion_imitation)
# Last Edited: 26th August


import numpy as np


def ConvertToSkewSymmetric(x: np.ndarray):
    return np.array([[   0, -x[2],  x[1]],
                    [ x[2],     0, -x[0]],
                    [-x[1],  x[0],    0]], dtype=float)


class SingleRigidBodyModel():
    def __init__(self, mass, inertia, num_legs = 2, dt = 0.025, planning_horizon = 10):
        
        self.num_legs = num_legs
        self.inv_inertia = np.linalg.pinv(inertia)
        self.inv_mass = 1.0 / mass

        self.A_mat = None
        self.B_mat = None

        self.dt = dt
        self.planning_horizon = planning_horizon


    def prepareModel(self, com_roll_pitch_yaw, foot_contact_states, foot_positions, discretize = True):

        self.contact_condition = (foot_contact_states > 0).astype(int)
        self.calc_A_mat(com_roll_pitch_yaw)
        self.calc_B_mat(foot_positions)

        if discretize:
            self.discretize_model()

        return self.A_mat, self.B_mat


    def discretize_model(self):
        '''
        Generates A_hat and B-hat matrices s.t
        x_{k+1} = A_hat x_{k} + B_hat u_{k}
        According to Euler's integration scheme
        A_hat = I + A.T
        B_hat = B.T
        where T is the sampling interval (1/sampling_frequency)
        :return: A_hat and B-hat
        '''
        self.A_mat = np.eye(self.A_mat.shape[0]) + self.A_mat * self.dt
        self.B_mat = self.B_mat * self.dt


    def calc_A_mat(self, com_roll_pitch_yaw):
        A = np.zeros((13, 13))
        cos_yaw = np.cos(com_roll_pitch_yaw[2])
        sin_yaw = np.sin(com_roll_pitch_yaw[2])
        cos_pitch = np.cos(com_roll_pitch_yaw[1])
        tan_pitch = np.tan(com_roll_pitch_yaw[1])

        angular_velocity_to_rpy_rate = np.array([
            [cos_yaw / cos_pitch, sin_yaw / cos_pitch, 0],
            [-sin_yaw, cos_yaw, 0],
            [cos_yaw * tan_pitch, sin_yaw * tan_pitch, 1]])

        A[0:3, 6:6 + 3] = angular_velocity_to_rpy_rate
        A[3, 9] = 1
        A[4, 10] = 1
        A[5, 11] = 1
        A[11, 12] = 1

        self.A_mat = A


    def calc_B_mat(self, foot_positions):
        B = np.zeros((13, 3*self.num_legs))
        for i in range(self.num_legs):
            B[6:6 + 3, i * 3:i * 3 + 3] = self.contact_condition[i] * \
                                self.inv_inertia @ ConvertToSkewSymmetric(foot_positions[i])
            B[9, i * 3] = self.contact_condition[i] * self.inv_mass
            B[10, i * 3 + 1] = self.contact_condition[i] * self.inv_mass
            B[11, i * 3 + 2] = self.contact_condition[i] * self.inv_mass

        self.B_mat = B


    def getReferenceTraj(self, x_init, ref_desired_stats, terrain_slope):

        roll_desired_init = ref_desired_stats["relative_torso_roll_desired"]
        pitch_desired_init = ref_desired_stats["relative_torso_pitch_desired"]
        yaw_desired_init = ref_desired_stats["relative_torso_yaw_desired"]
        x_desired_init = ref_desired_stats["relative_torso_x_desired"]
        y_desired_init = ref_desired_stats["relative_torso_y_desired"]
        z_desired_init = ref_desired_stats["relative_torso_z_desired"]
        vx_desired = ref_desired_stats["vx_desired"]
        vy_desired = ref_desired_stats["vy_desired"]
        yaw_rate_desired = ref_desired_stats["yaw_rate_desired"]

        # x_init has 3 com_rpy, 3 com_pos, 3_com_angular_vel, com_vel, 1_gravity
        cos0 = np.cos(np.radians(terrain_slope))
        sin0 = np.sin(np.radians(terrain_slope))

        x_ref = np.zeros((13*self.planning_horizon))

        x_i = x_init.copy()
        x_i[0:3] = np.array([roll_desired_init, pitch_desired_init, yaw_desired_init])
        x_i[3:6] = np.array([x_desired_init, y_desired_init, z_desired_init])
        x_i[6:9] = np.array([0, 0, yaw_rate_desired])
        x_i[9:12] = np.array([vx_desired, vy_desired, 0])
        x_i[12] = -9.81 # gravity

        for i in range(self.planning_horizon):

            x_i[0:3] = np.array([x_i[0], x_i[1], x_i[2] + yaw_rate_desired*self.dt])  # yaw need not be wrapped around 2pi, 
            x_i[3:6] = np.array([x_i[3] + vx_desired*cos0*self.dt, 
                                 x_i[4] + vy_desired*self.dt, 
                                 x_i[5] - vx_desired*sin0*self.dt]) # For our setting, vx_desired is negative for forward motion

            x_ref[i*13:(i+1)*13] = x_i.copy()

        return x_ref


    def getForceConstraints(self, friction_coeffs, max_z_force):
        right_mu, left_mu = friction_coeffs
        c = np.zeros(12*self.planning_horizon)
        C = np.zeros((12*self.planning_horizon, 6*self.planning_horizon))

        for i in range(self.planning_horizon):
            c[i*12] = max_z_force
            c[i*12 + 1] = 0
            c[i*12 + 6] = max_z_force
            c[i*12 + 6 + 1] = 0

            # Right Leg
            #   z force
            C[i*12, i*6:i*6 + 3] = [0, 0, 1]
            C[i*12 + 1, i*6:i*6 + 3] = [0, 0, -1]
            #   friction cone in x direction
            C[i*12 + 2, i*6:i*6 + 3] = [1, 0, -1*right_mu]
            C[i*12 + 3, i*6:i*6 + 3] = [-1, 0, -1*right_mu]
            #   friction cone in y direction
            C[i*12 + 4, i*6:i*6 + 3] = [0, 1, -1*right_mu]
            C[i*12 + 5, i*6:i*6 + 3] = [0, -1, -1*right_mu]


            # Left Leg
            #   z force
            C[i*12 + 6, i*6 + 3:i*6 + 6] = [0, 0, 1]
            C[i*12 + 7, i*6 + 3:i*6 + 6] = [0, 0, -1]
            #   friction cone in x direction
            C[i*12 + 8, i*6 + 3:i*6 + 6] = [1, 0, -1*left_mu]
            C[i*12 + 9, i*6 + 3:i*6 + 6] = [-1, 0, -1*left_mu]
            #   friction cone in y direction
            C[i*12 + 10, i*6 + 3:i*6 + 6] = [0, 1, -1*left_mu]
            C[i*12 + 11, i*6 + 3:i*6 + 6] = [0, -1, -1*left_mu]

        return c, C