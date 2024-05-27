# Author: Nithin Vasishta
# Created: 29th July 2023

import numpy as np


def decide_phase(swing_class):

    init_epsilon_pos = 0.1
    init_epsilon_vel = 0
    target_epsilon_pre = 0.1
    target_epsilon_post = 0.1
    
    torso_velocity = swing_class.data.qvel[0]
    init_foot_pos = swing_class.init_foot_position + swing_class.hip_position        
    target_foot_pos = swing_class.estimated_target_world_frame
    terrain_info = swing_class.task_info.terrain

    for x in terrain_info["stairs_pos_up"]:

        if init_foot_pos[0] > x and init_foot_pos[0] < x + init_epsilon_pos and torso_velocity < init_epsilon_vel:

            # input("Init foot")
            info_stair = {"stair_pos_x": x, 
                          "stair_height": terrain_info['ground_height'](x)}

            return 2, info_stair


        elif target_foot_pos[0] <= x + target_epsilon_pre and init_foot_pos[0] > x:

            # input("Target foot")
            info_stair = {"stair_pos_x": x, 
                          "stair_height": terrain_info['ground_height'](x)}

            #######
            # rec = input("Debug ?")
            # if rec == "y":
            #     import ipdb; ipdb.set_trace()

            return 1, info_stair

        else:
            pass

    return False, None


def phase1_modify(swing_class, info_stair):
    
    # Reduce swing target distance of swing
    clearance = 0.10
    estimated_pos_x = swing_class.estimated_target_world_frame[0]

    stair_pos_x = info_stair["stair_pos_x"]

    delta_x = stair_pos_x + clearance - estimated_pos_x
    swing_class.target_foot_position[0] += delta_x

    #######
    # rec = input("1\nVel: {}\nInit swing x: {}\n".format(swing_class.data.qvel[0], swing_class.init_foot_position[0] \
    #                                             + swing_class.hip_position[0]))
    # if rec == "d":
    #     import ipdb; ipdb.set_trace()

    return True


def phase2_modify(swing_class, info_stair, method=1):
    
    # Place swing leg on staircase
    clearance_x = 0.10
    clearance_z = 0.03
    
    estimated_pos_x = swing_class.estimated_target_world_frame[0]
    estimated_pos_z = swing_class.estimated_target_world_frame[2]
    
    stair_pos_x = info_stair["stair_pos_x"]
    stair_pos_z = info_stair["stair_height"]

    delta_x = stair_pos_x - clearance_x - estimated_pos_x
    delta_z = stair_pos_z - clearance_z - estimated_pos_z

    swing_class.foot_clearance = 0.14 # Should be a function of X vel
    swing_class.target_foot_position[0] += delta_x
    swing_class.target_foot_position[2] += delta_z

    #########
    # rec = input("2\nVel: {}\nInit swing x: {}\n".format(swing_class.data.qvel[0], swing_class.init_foot_position[0] \
    #                                             + swing_class.hip_position[0]))
    # if rec == "d":
    #     import ipdb; ipdb.set_trace()

    return True


def phase3_modify(swing_class, info_stair):

    # Balance on staircase
    clearance_x = 0.14
    clearance_z = 0.03
    
    estimated_pos_x = swing_class.estimated_target_world_frame[0]
    estimated_pos_z = swing_class.estimated_target_world_frame[2]
    
    stair_pos_x = info_stair["stair_pos_x"]
    stair_pos_z = info_stair["stair_height"]

    delta_x = stair_pos_x - clearance_x - estimated_pos_x
    delta_z = stair_pos_z - clearance_z - estimated_pos_z

    swing_class.target_foot_position[0] += delta_x
    swing_class.target_foot_position[2] += delta_z

    #########
    # rec = input("3\nVel: {}\nInit swing x: {}\n".format(swing_class.data.qvel[0], swing_class.init_foot_position[0] \
    #                                             + swing_class.hip_position[0]))
    # if rec == "d":
    #     import ipdb; ipdb.set_trace()

    return True

