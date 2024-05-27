# Author: Nithin Vasishta

import numpy as np
from typing import Literal

_method_types = Literal['sensor', 'past_positions', 'terrain']


class Kinematics():
    
    def __init__(self, model, data, task_info):

        self.model = model
        self.data = data
        self.task_info = task_info


    def estimate_slope(self, past_contact_pos):
        return 0


    def has_the_leg_moved(self, past_foot_pos, look_at_prev_timesteps = 10):

        dist_moved_list = []
        curr_pos = past_foot_pos[-1]
        for i in range(-1, -1*look_at_prev_timesteps - 1, -1):
            dist_moved_list.append(np.linalg.norm(curr_pos - past_foot_pos[i]))

        return np.max(dist_moved_list)


    def estimate_contact(self, method: _method_types = 'terrain', terrain_info = None, 
                            in_stair_manuever = 0):

        # Taking a shortcut
        # This should be replaced by a proper contact estimator

        if method == 'sensor':

            itouch_right = self.task_info.robot["touch_sensor_right_foot_id"]
            itouch_left = self.task_info.robot["touch_sensor_left_foot_id"]
            right_foot_contact = self.data.sensordata[itouch_right] > 0 
            left_foot_contact = self.data.sensordata[itouch_left] > 0 

            foot_contact_states = [right_foot_contact, left_foot_contact]

        elif method == 'terrain':

            ibody_torso = self.task_info.robot["body_torso_id"]
            torso_pos = self.data.xpos[ibody_torso]
            slope = self.task_info.terrain["ground_slope"](torso_pos[0])

            if slope > np.radians(30):
                isite_right = self.task_info.robot["site_right_foot_fwd_id"]
                isite_left = self.task_info.robot["site_left_foot_fwd_id"]

            elif slope < -np.radians(30):
                isite_right = self.task_info.robot["site_right_foot_bck_id"]
                isite_left = self.task_info.robot["site_left_foot_bck_id"]

            else:
                isite_right = self.task_info.robot["site_right_foot_mid_id"]
                isite_left = self.task_info.robot["site_left_foot_mid_id"]

            right_foot_pos = self.data.site_xpos[isite_right]
            left_foot_pos = self.data.site_xpos[isite_left]
            ground_height_r = terrain_info['ground_height'](right_foot_pos[0])
            ground_height_l = terrain_info['ground_height'](left_foot_pos[0])

            epsilon = 0.02
            foot_contact_states = np.array([right_foot_pos[2] - ground_height_r,
                                            left_foot_pos[2] - ground_height_l]) < epsilon

        else:
            raise("Not implemented")

        return foot_contact_states


    def end_effector_position(self):

        # This should be replaced by proper forward kinematics
        isite_right_foot = self.task_info.robot["site_right_foot_mid_id"]
        isite_left_foot = self.task_info.robot["site_left_foot_mid_id"]

        right_foot = self.data.site_xpos[isite_right_foot]
        left_foot = self.data.site_xpos[isite_left_foot]

        return (right_foot, left_foot)
