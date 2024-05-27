# Author: Nithin Vasishta
# Created: 24th July 2023
# Last Edited: 3rd September 2023

import copy
import numpy as np
import mujoco as mj
from collections import deque

import utils_high_level_ctrl as hc
from low_level_controller import LowLevelController

class HighLevelController():

    '''
    This controller will be used for:

    1. Carrying out custom commands: Walk to position (x, y), Stand in place, Turn etc. 
    2. Modification of args during sinusoidal walks
    3. Joystick commands
    '''

    def __init__(self, model, data, args, task_info):

        self.model = model
        self.data = data
        self.args = args
        self.task_info = task_info

        self.low_ctrl = LowLevelController(self.model, self.data, self.args, self.task_info)
        self.original_args = copy.deepcopy(args)

        self.slope_state = -1
        self.vel_x = deque([], maxlen=100)


    def set_action(self):

        n_steps = self.low_ctrl._n_steps
        torso_xpos = self.data.qpos[0]
        slope = self.task_info.terrain['ground_slope'](torso_xpos)
        delta_slope = self.task_info.terrain['delta_ground_slope'](torso_xpos)
        self.vel_x.append(self.data.qvel[0])

        # print(f"steps: {n_steps}\tvel: {np.mean(self.vel_x)}")

        ## Modify arguments to give to MPC
        # Forces given by MPC are a trade-off between different statistics that we want to satisfy (vx_desired, yaw_rate_desired, pitch_desired etc.)
        # Therefore, these forces would mostly never satisfy the required statistics completely. There we change the arguments given in original_args
        # and pass the modified args (self.args) to the low_level_controller, so that the overall outcome matches the desired statistics

        if self.original_args.terrain_slope == 0 and \
            self.original_args.stair_height == 0 and self.original_args.sinusoidal_slope == 0:
            hc.flat.set_args(self.original_args, self.args)

        elif self.original_args.terrain_slope == 10:
            hc.slope_10.set_args(self.original_args, self.args)

        elif self.original_args.terrain_slope == 30:
            hc.slope_30.set_args(n_steps, self.original_args, self.args)

        elif self.original_args.terrain_slope == 45:
            hc.slope_45.set_args(self.original_args, self.args)

        elif self.original_args.terrain_slope == -15:
            hc.slope_ne15.set_args(self.original_args, self.args)

        elif self.original_args.terrain_slope == -30:
            hc.slope_ne30.set_args(self.original_args, self.args)

        elif self.original_args.sinusoidal_slope == 30:
            self.slope_state = hc.sine_30.set_args(self.original_args, self.args, slope, delta_slope, self.slope_state)

        elif self.original_args.sinusoidal_slope == 45:
            self.slope_state, print_qpos = hc.sine_45.set_args(self.original_args, self.args, slope, delta_slope, self.slope_state)

        elif self.original_args.stair_height == 6:
            hc.stairs_6.set_args(self.original_args, self.args)

        elif self.original_args.stair_height == 14:
            hc.stairs_14.set_args(self.original_args, self.args)

        else:
            raise("High level controller is not tuned for this setting")

        self.low_ctrl.set_action()

        # Debugging
        # if print_qpos:
        #     print(self.data.qpos)
        #     print(self.data.qvel)
        #     input("")
        # else:
        #     pass
