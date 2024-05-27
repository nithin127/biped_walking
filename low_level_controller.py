# Author: Nithin Vasishta
# Created: 24th July, 2023

import os, time
import numpy as np

import sub_controllers.staircase_controller as staircase_ctrl
from sub_controllers.swing_leg_controller import SwingLegController
from sub_controllers.stance_leg_controller import StanceLegController
from sub_controllers.lost_contact_controller import LostContactController

from utils.visualise_stats import dataVisualiser
from infrastructure.kinematics import Kinematics


PI = np.pi
init_phase = PI


class LowLevelController():

    def __init__(self, model, data, args, task_info):

        self.model = model
        self.data = data
        self.args = args
        self.task_info = task_info

        self._n_steps = 0
        self._n_mode_changes = 0

        self.dt = model.opt.timestep

        # The initial controller phases (phase_swing, phase_mpc, phase_lost_contact) are 
        # set to 2*pi so that the controllers are queried at the beginning of the script execution
        # (the controllers are executed when their respective phases crosses 2*pi)

        self.phase = init_phase
        self.omega = 2*PI*args.gait_frequency

        self.phase_mpc = 2*PI
        self.omega_mpc = 2*PI*args.mpc_frequency

        self.phase_swing = 2*PI
        self.omega_swing= 2*PI*args.swing_frequency

        self.phase_lost_contact = 2*PI
        self.omega_lost_contact = 2*PI*args.lost_contact_frequency
        self.is_lost_contact = False

        self.in_stair_manuever = 0
        self.info_stair = None
        
        self.info_st = None        
        self.info_sw = None        
        self.torque_st = None
        self.torque_sw = None

        self.K = Kinematics(model, data, task_info)
        self.sw = SwingLegController(model, data, args, task_info)
        self.st = StanceLegController(model, data, args, task_info)
        self.lc = LostContactController(model, data, args, task_info)
        self.reset(first_time=True)

        # Printing and debugging
        self.data_visualiser = dataVisualiser()


    def reset(self, first_time=False):

        if first_time:
            # The first time this is being done in set_action() in swing leg controller
            pass
        else:
            self.sw.reset_swing_trajectory(self.phase) 

        self.st.reset_qp_solver()    
        self.lc.reset_method()


    def reset_args(self, new_args):
        self.args = new_args


    def set_action(self):
        """
        This function sets the torque values for each joints according to the phase
        and contact condition of the legs. This torque is executed during mj_step
        """

        # The MPC controller for stance is executed when at least one of the conditions are satisfied
        # 1. The MPC phase crosses 2*pi
        # 2. When the overall gait phase crosses pi or 2*pi 
        #    (this when legs change their stance/swing phase)
        # 3. When the contact is just re-established (after losing contact)
        #
        # When the contact is lost for stance leg, lost contact controller takes over

        # Similarly. the raibert swing controller is executed when at least one of the conditions are satisfied
        # 1. The swing phase crosses 2*pi
        # 2. When the overall gait phase crosses pi or 2*pi 
        #    (this when legs change their stance/swing phase)

        self._n_steps += 1
        execute_mpc, self.phase_mpc = np.divmod(self.phase_mpc + self.omega_mpc*self.dt, 2*PI)
        execute_lc, self.phase_lost_contact = np.divmod(self.phase_lost_contact + self.omega_lost_contact*self.dt, 2*PI)
        execute_raibert_swing, self.phase_swing = np.divmod(self.phase_swing + self.omega_swing*self.dt, 2*PI)

        new_phase = np.fmod(self.phase + self.omega*self.dt, 2*PI)
        mode_change = self.change_controller(self.phase, new_phase)
        self.phase = new_phase


        # When phase crosses pi or 2pi, the mode changes
        # Swing leg changes to stance and vice versa

        if mode_change:

            execute_mpc = 1
            execute_raibert_swing = 1
            execute_lc = 1

            self._n_mode_changes += 1
            self.reset()

            if self.args.stair_height != 0:
                self.engage_staircase_controller(self.phase)
            else:
                pass


            ## Printing and plotting 

            if self.args.plot =="mode_change" and self._n_mode_changes % self.args.skip_num_reset == 0:
                self.data_visualiser.plot_debug_mode_change(self)
            else:
                pass

        else:
            pass


        ## Query Raibert Swing Controller

        if execute_raibert_swing:
            self.torque_sw, self.info_sw = self.sw.get_action(self.phase)
        else:
            pass


        ## Query MPC Stance / Lost Contact Controller

        # If contact is not established for stance leg, the lost contact controller takes its place
        # Otherwise, MPC stance is executed. When contact is re-established, we mandatorily execute MPC
        # When contact is lost, we mandatorily execute lost contact controller

        foot_contact_states = self.K.estimate_contact('terrain', self.task_info.terrain, self.in_stair_manuever)

        if self.phase < PI:
            proper_contact_condition = True if foot_contact_states[1] == True else False
        else:
            proper_contact_condition = True if foot_contact_states[0] == True else False


        if not proper_contact_condition:

            if not self.is_lost_contact:
                # Contact just lost
                execute_lc = 1
                # input("lc")
                self.phase_lost_contact = 0
                self.is_lost_contact = True
            else:
                # Contact not yet established
                pass

            if execute_lc:
                self.torque_st, self.info_st = self.lc.get_action(self.phase)
            else:
                pass

        else:

            if self.is_lost_contact:
                # Contact just re-established
                execute_mpc = 1
                self.phase_mpc = 0
                self.is_lost_contact = False
            else:
                # Contact has been maintained
                pass

            if execute_mpc:
                self.torque_st, self.info_st = self.st.get_action(self.phase)
            else:
                pass

        ## Set the control values for swing and stance leg

        if self.phase < PI:
            self.data.ctrl[0:3] = self.torque_sw
            self.data.ctrl[3:6] = self.torque_st

        else:
            self.data.ctrl[0:3] = self.torque_st
            self.data.ctrl[3:6] = self.torque_sw

        # input(self.torque_sw)
        # input(self.torque_st)


        ## Printing and plotting

        self.data_visualiser.update_step(self)

        if self.args.plot =="step" and self._n_steps > self.args.skip_timesteps:
            self.data_visualiser.plot_debug_step(self)
        else:
            pass

        return True


    def change_controller(self, phase, new_phase):
        return True if (phase-PI)*(new_phase-PI) < 0 else False


    def engage_staircase_controller(self, phase):

        # If swing leg target is colliding with stairs (or is very close)
        # Initiate stair manuever:
        #   Phase 1: Reduce swing target distance of the first leg
        #   Phase 2: The next swing leg targets a position on the stairs
        #   Phase 3: The leg on the stairs pushes up the robot

        if self.in_stair_manuever == 0:

            phase, self.info_stair = staircase_ctrl.decide_phase(self.sw)

            if phase == 1: 

                self.in_stair_manuever = 1
                staircase_ctrl.phase1_modify(self.sw, self.info_stair)
            
            elif phase == 2:

                self.in_stair_manuever = 2
                staircase_ctrl.phase2_modify(self.sw, self.info_stair, method=1)

            else:
                pass
       
        elif self.in_stair_manuever == 1:

            self.in_stair_manuever = 2
            staircase_ctrl.phase2_modify(self.sw, self.info_stair, method=2)

        elif self.in_stair_manuever == 2:

            self.in_stair_manuever = 0
            staircase_ctrl.phase3_modify(self.sw, self.info_stair)

        else:
            raise("Invalid stair manuever ID")

        self.st.reset_qp_solver(self.in_stair_manuever)
        self.lc.reset_method(self.in_stair_manuever)

