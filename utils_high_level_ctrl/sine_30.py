# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 13th September 2023

import numpy as np


def set_args(original_args, new_args, slope, delta_slope, slope_state):

    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired

    if vx < 0 and vy == 0 and dyw == 0:
        # Move in x direction
        # Finite state machine; No velocity tracking

        if slope >= 0:

            slope_state = 1

            if original_args.scaled_biped == -1:

                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 0.11
                new_args.pitch_desired = 0.37

            else:

                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.3
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
        
                

        elif slope >= np.radians(-15) and slope < np.radians(0) and delta_slope < 0:

            # if slope_state != 2:
            #     print(data.qpos)
            #     print(data.qvel)
            #     rec = input("2")

            slope_state = 2

            if original_args.scaled_biped == -1:

                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 0.24
                new_args.pitch_desired = 1.1

            else:

                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.4
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
        


        elif slope >= np.radians(-25) and slope < np.radians(-15) and delta_slope < 0:

            # if slope_state != 3:
            #     rec = input("3")

            slope_state = 3

            if original_args.scaled_biped == -1:

                new_args.mpc_weights_config = 4
                new_args.foot_clearance = 0.05
                new_args.vx_desired = 0.2
                new_args.pitch_desired = 1

            else:

                new_args.mpc_weights_config = 9
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.8
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
        


        elif slope >= np.radians(-30) and slope < np.radians(-25):

            # if slope_state != 4:
            #     print(data.qpos)
            #     print(data.qvel)
            #     print(low_ctrl.phase)
            #     rec = input("4")

            slope_state = 4

            if original_args.scaled_biped == -1:

                new_args.mpc_weights_config = 4
                new_args.foot_clearance = 0.05
                new_args.vx_desired = 0.35
                new_args.pitch_desired = 1.2

            else:

                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
        


        elif slope >= np.radians(-25) and slope < np.radians(-15) and delta_slope > 0:

            # if slope_state != 5:
            #     rec = input("5")

            slope_state = 5

            if original_args.scaled_biped == -1:

                new_args.mpc_weights_config = 4
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 0.2
                new_args.pitch_desired = 1

            else:

                new_args.mpc_weights_config = 9
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 1.5
                new_args.pitch_desired = 0.8
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
        
                
            
        elif slope >= np.radians(-15) and slope < np.radians(0) and delta_slope > 0:

            # if slope_state != 6:
            #     rec = input("6")

            slope_state = 5

            if original_args.scaled_biped == -1:

                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 0.1
                new_args.pitch_desired = 0.5

            else:

                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 0.7
                new_args.pitch_desired = 0.6
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
        
                

        else:
            input("Sine30: New slope condition. Please check")
            pass

    else:
        raise("Controller not tuned for this setting")

    return slope_state