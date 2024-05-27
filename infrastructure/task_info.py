# Author: Nithin Vasishta
# Created: 4th August 2023
# Last Edited: 26th August

import json
import numpy as np
from utils.inertia_utils import calculate_inertia


class TaskInfo():

    def __init__(self, model, data, args):

        self.model = model
        self.data = data
        self.args = args

        self.robot = None
        self.terrain = None
        self.initialise_args()
        self.initialise_dynamics_info()


    def initialise_args(self):

        if not self.args.scaled_biped == -1:
            file = "utils/robot_info_scaled.json"
        else:
            file = "utils/robot_info.json"

        with open(file, "r") as file:
            robot = json.load(file)

        terrain = {}
        terrain["stairs_pos_up"] = None
        terrain["stairs_pos_down"] = None
        terrain["ground_height"] = self.ground_height
        terrain["ground_slope"] = lambda x: np.radians(self.args.terrain_slope)
        terrain["delta_ground_slope"] = lambda x: 0
        terrain["internal_sine_angle"] = lambda x: 0

        if self.args.stair_height != 0:
            terrain["stairs_pos_up"] = [0, -1, -2]
            terrain["stairs_pos_down"] = [-4, -5, -6]
            terrain["ground_height"] = self.get_stair_height

        elif self.args.sinusoidal_slope != 0:
            terrain["ground_height"] = self.get_sinusoidal_height
            terrain["ground_slope"] = self.get_sinusoidal_slope
            terrain["delta_ground_slope"] = self.get_sinusoidal_delta_slope
            terrain["internal_sine_angle"] = self.get_internal_sine_angle

        else:
            pass

        self.robot = robot
        self.terrain = terrain


    def initialise_dynamics_info(self):

        i_st = self.robot["body_start_id"]
        i_end = self.robot["body_end_id"] + 1
        mass = np.sum(self.model.body_mass[i_st:i_end])
        inertia = calculate_inertia(self.model, self.data, self.robot, "wrt_torso")

        self.robot["init_mass"] = mass       
        self.robot["init_inertia"] = inertia       


    def get_internal_sine_angle(self, x):

        freq = np.radians(self.args.sinusoidal_slope)
        return np.degrees(freq*(x) - np.radians(90))


    def get_sinusoidal_height(self, x, method=2):

        if method == 1:

            # Using sine function
            freq = np.radians(self.args.sinusoidal_slope)
            return np.sin(freq*(x) - np.radians(90))

        elif method == 2:

            # Using hfield values
            # Note: we set nrow/ncol to be odd

            nrow = self.model.hfield_nrow[0]
            ncol = self.model.hfield_ncol[0]

            origin_i = (ncol - 1) / 2 
            tile_length_x = 2*self.model.hfield_size[0][0] / (ncol-1)

            n_i = int(origin_i + np.ceil((x - tile_length_x/2) / tile_length_x))
            n_j = 0

            # Note: since we're moving in the negative direction 'next' results in the previous (n-1) value
            h_prev = self.model.hfield_data[n_i + ncol*n_j]
            h_next = self.model.hfield_data[max(0, n_i-1) + ncol*n_j]  # max(0, n-1) to handle edge case

            x_prev = (n_i - origin_i)*tile_length_x
            x_next = (max(0, n_i-1) - origin_i)*tile_length_x

            if x_prev == x_next:
                h =  h_prev

            else:
                h = (h_next - h_prev)/(x_next - x_prev)*(x - x_prev) + h_prev

            return h


    def get_sinusoidal_slope(self, x, method=2):


        if method == 1:

            # Using sine function
            freq = np.radians(self.args.sinusoidal_slope)
            return -1*freq*np.cos(freq*(x) - np.radians(90))
            # We multiply by -1 as we're walking in the -ve x direction

        elif method == 2:

            # Using hfield values
            # Note: we set nrow/ncol to be odd

            nrow = self.model.hfield_nrow[0]
            ncol = self.model.hfield_ncol[0]

            origin_i = (ncol - 1) / 2 
            tile_length_x = 2*self.model.hfield_size[0][0] / (ncol-1)

            n_i = int(origin_i + np.ceil((x - tile_length_x/2) / tile_length_x))
            n_j = 0

            # Note: since we're moving in the negative direction 'next' results in the previous (n-1) value
            h_prev = self.model.hfield_data[n_i + ncol*n_j]
            h_next = self.model.hfield_data[max(0, n_i-1) + ncol*n_j]  # max(0, n-1) to handle edge case

            x_prev = (n_i - origin_i)*tile_length_x
            x_next = (max(0, n_i-1) - origin_i)*tile_length_x

            if x_prev == x_next:
                return 0

            else:
                return (h_next - h_prev)/tile_length_x


    def get_sinusoidal_delta_slope(self, x):

        freq = np.radians(self.args.sinusoidal_slope)
        delta_slope = -1*freq*freq*np.sin(freq*(x) - np.radians(90))
        # We multiply by -1 as we're walking in the -ve x direction 
        # (since it's the negative direction, derivative of sin is -cos
        #  derivative of cos is sin. It's not d/dx, it's d/(-dx))
        return delta_slope


    def get_stair_height(self, x):

        h = self.args.stair_height/100

        if self.args.stair_type == 0:

            if x <= 0 and x > -1:
                return h

            elif x <= -1 and x > -2:
                return 2*h

            elif x <= -2 and x > -4:
                return 3*h
        
            elif x <= -4 and x > -5:
                return 2*h

            elif x <= -5 and x > -6:
                return h

            else:
                return 0

        else:
            raise("Not implemented")


    def ground_height(self, x):
        return -1*np.tan(np.radians(self.args.terrain_slope))*x
