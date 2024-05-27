# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 3rd September 2023

def set_args(original_args, new_args):

    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired

    
    if vx < 0 and vy == 0 and dyw == 0:

        # Move in x direction

        if original_args.scaled_biped == -1:

            new_args.vx_desired = 0.4
            new_args.pitch_desired = 0.39
            new_args.foot_clearance = 0.12

        elif original_args.scaled_biped == 2:
            
            new_args.mpc_weights_config = 8
            new_args.foot_clearance = 0.08
            new_args.vx_desired = 1
            new_args.pitch_desired = 0
            new_args.hip_height_desired = 0.75
            new_args.torso_height_desired = 0.8

    else:
        raise("Controller not tuned for this setting")
