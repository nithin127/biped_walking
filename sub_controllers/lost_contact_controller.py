# Author: Nithin Vasishta
# Last Edited: 26th August


import numpy as np
import mujoco as mj

PI = np.pi

class LostContactController():

    # Draws a direct line between what should be the position of the stance leg
    # and where its currently at. Corrects the position gradually
    # Should also account for feedforward torque

    def __init__(self, model, data, args, task_info):

        self.model = model
        self.data = data
        self.args = args
        self.task_info = task_info

        self.swing_time = (1./args.gait_frequency)/2
        self.hip_height_desired = args.hip_height_desired

        self.pcorr = args.pos_correction_coefficient_lost_contact_controller
        # self.acorr = args.ang_correction_coefficient_lost_contact_controller
        self.Kplc = args.position_gain_lost_contact_controller
        self.Kvlc = args.velocity_gain_lost_contact_controller

        self.only_z = None


    def reset_method(self):
        self.only_z = self.args.lost_contact_only_z


    def get_hip_position(self, phase):

        ### Needs to be updated with forward kinematics
        if phase < PI:
            isite_hip = self.task_info.robot["site_left_hip_id"]
        else:
            isite_hip = self.task_info.robot["site_right_hip_id"]

        return self.data.site_xpos[isite_hip].copy()


    def get_action(self, phase, only_z = False):

        if phase < PI:

            info = "Left stance"
            ibody = self.task_info.robot["body_left_shank_id"]
            isite_hip = self.task_info.robot["site_left_hip_id"]
            isite_foot = self.task_info.robot["site_left_foot_mid_id"]
            start_index_qv = self.task_info.robot["qvel_left_leg_start_id"]
            initial_x_offset = self.task_info.robot["end_effector_offset_left_x"]
            initial_y_offset = self.task_info.robot["end_effector_offset_left_y"]

        else:

            info = "Right stance"            
            ibody = self.task_info.robot["body_right_shank_id"]
            isite_hip = self.task_info.robot["site_right_hip_id"]
            isite_foot = self.task_info.robot["site_right_foot_mid_id"]
            start_index_qv = self.task_info.robot["qvel_right_leg_start_id"]
            initial_x_offset = self.task_info.robot["end_effector_offset_right_x"]
            initial_y_offset = self.task_info.robot["end_effector_offset_right_y"]


        # The stance leg moves backward relative to the COM
        # We decide the position of the stance leg based on the normalised phase

        if self.only_z:

            foot_pos = self.data.site_xpos[isite_foot]
            terrain_height = self.task_info.terrain['ground_height'](foot_pos[0])
            delta_pos_ee = np.array([0, 0, terrain_height-foot_pos[2]])

        else:

            normalised_phase = (phase % PI) / PI

            torso_xpos = self.data.qpos[0]
            terrain_slope = self.task_info.terrain['ground_slope'](torso_xpos)

            step_length_x = self.swing_time*self.args.vx_desired
            step_length_y = self.swing_time*self.args.vy_desired

            x_vel_offset = self.pcorr*(self.data.qvel[0] - self.args.vx_desired)
            y_vel_offset = self.pcorr*(self.data.qvel[1] - self.args.vy_desired)

            x = -1*(0.5 - normalised_phase)*step_length_x + initial_x_offset + x_vel_offset
            y = -1*(0.5 - normalised_phase)*step_length_y + initial_y_offset + y_vel_offset
            z = -1*self.hip_height_desired + x*np.tan(np.radians(terrain_slope))

            desired_foot_position = np.array([x, y, z])

            hip_pos = self.get_hip_position(phase)
            current_foot_position_world_frame = self.data.site_xpos[isite_foot]
            current_foot_position = current_foot_position_world_frame - hip_pos

            delta_pos_ee = desired_foot_position - current_foot_position


        jacp = np.zeros((3, self.model.nv))
        mj.mj_jac(self.model, self.data, jacp, None, self.data.site_xpos[isite_foot], ibody)
        J = jacp[:, start_index_qv:start_index_qv+3]

        force_lc = self.Kplc*(delta_pos_ee)
        
        # foot_velocity = J @ data.qvel[start_index_qv:start_index_qv+3]
        # delta_vel_ee = 0 - foot_velocity
        # force_lc += self.Kvlc*(delta_vel_ee)

        torque_lc = J.T@force_lc
        info = {"ctrl": "Lost Contact", "force": force_lc, "x_ref": None, "x_res": None}


        return torque_lc, info
