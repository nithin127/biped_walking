# Author: Nithin Vasishta
# Last Edited: 26th August


import numpy as np
import mujoco as mj

from utils.swing_utils import gen_swing_foot_trajectory

PI = np.pi

class SwingLegController():

    def __init__ (self, model, data, args, task_info):

        self.model = model
        self.data = data
        self.args = args
        self.task_info = task_info

        self.hip_position = None
        self.init_foot_position = np.ones(3)*np.inf
        self.target_foot_position = np.ones(3)*np.inf
        self.estimated_target_world_frame = np.ones(3)*np.inf

        dt = model.opt.timestep
        omega = 2*PI*args.gait_frequency

        self.swing_time = (1./args.gait_frequency)/2
        self.num_timesteps_swing_phase = np.pi/(omega*dt)

        self.start_debug = False
        self.reset_done = False


    def get_hip_position(self, phase):

        ### Needs to be updated with forward kinematics
        if phase < PI:
            isite_hip = self.task_info.robot["site_right_hip_id"]
        else:
            isite_hip = self.task_info.robot["site_left_hip_id"]

        return self.data.site_xpos[isite_hip].copy()


    def reset_swing_trajectory(self, phase):

        if phase < PI:

            isite_foot = self.task_info.robot["site_right_foot_mid_id"]
            initial_x_offset = self.task_info.robot["end_effector_offset_right_x"]
            initial_y_offset = self.task_info.robot["end_effector_offset_right_y"]
            hip_offset_from_torso = np.array(self.task_info.robot["right_hip_torso_offset"])

        else:

            isite_foot = self.task_info.robot["site_left_foot_mid_id"]
            initial_x_offset = self.task_info.robot["end_effector_offset_left_x"]
            initial_y_offset = self.task_info.robot["end_effector_offset_left_y"]
            hip_offset_from_torso = np.array(self.task_info.robot["left_hip_torso_offset"])

        ## Raibert's heuristic ##

        # For details, please refer to chapter 2 in "Legged robots that balance" by
        # Marc Raibert. The key idea is to stablize the swing foot's location based on
        # the CoM moving speed.

        torso_pos = self.data.qpos[0:3]
        torso_velocity = self.data.qvel[0:3]
        torso_yaw_rate = self.data.qvel[5]

        twisting_vector = np.array((-hip_offset_from_torso[1], hip_offset_from_torso[0], 0))
        hip_horizontal_velocity = torso_velocity + torso_yaw_rate * twisting_vector
        target_hip_horizontal_velocity = np.array([self.args.vx_desired, self.args.vy_desired, 0]) \
                                         + self.args.yaw_rate_desired * twisting_vector

        # These angles (sin0, cos0, tan0) come into play only when terrain slope is not 0

        terrain_slope = self.task_info.terrain['ground_slope'](torso_pos[0])

        cos0 = np.cos(np.radians(terrain_slope))
        sin0 = np.sin(np.radians(terrain_slope))
        tan0 = sin0/cos0

        target_x_foot_position = cos0 * np.array([self.args.vx_desired, 0, 0]) * self.swing_time/2
        target_y_foot_position =    1 * np.array([0, self.args.vy_desired, 0]) * self.swing_time/2
        target_z_foot_position = sin0 * np.array([0, 0, self.args.vx_desired]) * self.swing_time/2 

        self.target_foot_position = target_x_foot_position + target_y_foot_position + target_z_foot_position \
                                  + np.array([initial_x_offset, initial_y_offset, -1*self.args.hip_height_desired])

        Kprx = self.args.x_position_gain_raibert_controller
        Kpry = self.args.y_position_gain_raibert_controller

        target_correction_x = -1 * Kprx*(target_hip_horizontal_velocity[0] - hip_horizontal_velocity[0])
        target_correction_y = -1 * Kpry*(target_hip_horizontal_velocity[1] - hip_horizontal_velocity[1])
        target_correction_z = -1 * target_correction_x * tan0 

        self.hip_position = self.get_hip_position(phase)
        self.init_foot_position = self.data.site_xpos[isite_foot] - self.hip_position
        self.target_foot_position += np.array([target_correction_x, target_correction_y, target_correction_z])

        final_hip_position = self.hip_position + self.swing_time*hip_horizontal_velocity
        self.estimated_target_world_frame = self.target_foot_position + final_hip_position

        # print(f"init: {self.init_foot_position}\ntarget: {self.target_foot_position}")
        # print(f"from vel: {target_x_foot_position[0]}\tfrom offset: {initial_x_offset}")
        # print(f"target_hip: {target_hip_horizontal_velocity[0]}\tactual_hip: {hip_horizontal_velocity[0]}\tresult: {target_correction_x}")
        # print(f"hip: {self.hip_position[2]}\ttorso: {self.data.xpos[1][2]}")
        # input("\n\n")

    def get_action(self, phase):

        if not self.reset_done:
            # This is the first time the swing leg controller is calling reset_swing_trajectory()
            # after being created. We do this here so that high_level_controller can modify the args
            # before the reset_swing_trajectory() is called
            self.reset_swing_trajectory(phase)
            self.reset_done = True

        if phase < PI:

            ibody = self.task_info.robot["body_right_shank_id"]
            isite_foot = self.task_info.robot["site_right_foot_mid_id"]
            start_index_qv = self.task_info.robot["qvel_right_leg_start_id"]

        else:

            ibody = self.task_info.robot["body_left_shank_id"]
            isite_foot = self.task_info.robot["site_left_foot_mid_id"]
            start_index_qv = self.task_info.robot["qvel_left_leg_start_id"]


        ## Compute desired and current position/velocity

        normalised_phase = np.fmod(phase, PI)/PI
        desired_foot_position = gen_swing_foot_trajectory(normalised_phase, self.init_foot_position,
                                                          self.target_foot_position, self.args.foot_clearance)

        jacp = np.zeros((3, self.model.nv))
        mj.mj_jac(self.model, self.data, jacp, None, self.data.site_xpos[isite_foot], ibody)
        J = jacp[:, start_index_qv:start_index_qv+3]

        vel_end_effector = J @ self.data.qvel[start_index_qv:start_index_qv+3]
        current_foot_position = self.data.site_xpos[isite_foot] - self.get_hip_position(phase)                

        delta_x = desired_foot_position - current_foot_position
        delta_q = np.linalg.pinv(J) @ delta_x        
        delta_vx = 0 - vel_end_effector 
        delta_vq = 0 - self.data.qvel[start_index_qv:start_index_qv+3]

        ## Compute torque

        Kp = self.args.position_gain_swing
        Kv = self.args.velocity_gain_swing
        
        torque_pos = J.T@(Kp*(delta_x))
        torque_vel = J.T@(Kv*(delta_vx))


        ## Feed forward torque
        # See: https://studywolf.wordpress.com/2013/09/07/robot-control-3-accounting-for-mass-and-gravity/
        # And see: https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf

        torque_ff = np.zeros(3)

        # if not self.args.no_feedforward_torque:
        #     # We have already considered desired accelearion above
        #     # And we ignore the Coriolis effect. Only the gravity term remains
        #     # g(q) = sum_i (J_i.transpose @ F_gravity_i)

        #     for i in range(3):
        #         # To do: Store leg mass in robot_info and query from there
        #         # we shouldn't query from model.body_mass
        #         F_gi = self.model.body_mass[ibody-i]*np.array([0, 0, -9.81])
        #         jacp_i = np.zeros((3, self.model.nv))
        #         mj.mj_jac(self.model, self.data, jacp_i, None, self.data.xipos[ibody-i], ibody-i)
        #         Ji = jacp_i[:, start_index_qv:start_index_qv+3].copy()
        #         g_qi = Ji.T @ F_gi
        #         torque_ff += g_qi
        #         # print(torque_ff)
        # else:
        #     pass

        # input("")

        torque_overall = torque_pos 
        # torque_overall += torque_vel
        # torque_overall +=  torque_ff

        ## Debug

        # print("foot_clearance: ", self.foot_clearance)
        # print("----init-----", np.round(self.init_foot_position, 4))
        # print("----target---", np.round(self.target_foot_position, 4))
        # print("----current--", np.round(current_foot_position, 4))
        # print("----desired--", np.round(desired_foot_position, 4))
        # input("")

        if self.start_debug:
            rec = input("ipdb (y/n)?")            
            if rec == "y":
                import ipdb; ipdb.set_trace()
            else:
                pass

        else:
            pass


        info_sw = {"torque_pos": torque_pos,
                   "torque_vel": torque_vel,
                   "torque_ff": torque_ff,
                   "torque_overall": torque_overall}


        return torque_overall, info_sw
