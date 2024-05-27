# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 26th February 2024

def set_args(original_args, new_args):

    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired
    
    if vx <= 0 and vy == 0 and dyw == 0:
        # Move in x direction
        
        if original_args.scaled_biped == -1:
            new_args.mpc_weights_config = 4
            new_args.foot_clearance = 0.05
            new_args.vx_desired = 0.1
            new_args.pitch_desired = 0.4

        elif original_args.scaled_biped == 2:
            new_args.mpc_weights_config = 10
            new_args.foot_clearance = 0.08
            new_args.vx_desired = 0.4
            new_args.pitch_desired = 1.1
            new_args.hip_height_desired = 1.1
            new_args.torso_height_desired = 1.2
            new_args.position_gain_swing = 1000
            new_args.velocity_gain_swing = 200
            # new_args.gait_frequency = 10

        elif original_args.scaled_biped == 3:
            new_args.mpc_weights_config = 10
            new_args.foot_clearance = 0.08
            new_args.vx_desired = 0.2
            new_args.pitch_desired = 1.1
            new_args.hip_height_desired = 1.1
            new_args.torso_height_desired = 1.2
            new_args.position_gain_swing = 1050
            new_args.velocity_gain_swing = 200
            # new_args.gait_frequency = 10

        elif original_args.scaled_biped == 0:
            new_args.mpc_weights_config = 11
            new_args.foot_clearance = 0.08
            new_args.vx_desired = 0.4
            new_args.pitch_desired = 1
            new_args.hip_height_desired = 1.25
            new_args.torso_height_desired = 1.25
            # new_args.gait_frequency = 10

        else:
            raise("Controller for this scaled version of biped not available")

    elif vx > 0 and vy == 0 and dyw == 0:
        # Move in x direction
        
        if original_args.scaled_biped == 2:
            new_args.mpc_weights_config = 8.1
            new_args.foot_clearance = 0.08
            new_args.vx_desired = 2.5
            new_args.pitch_desired = 0.4
            new_args.hip_height_desired = 1
            new_args.torso_height_desired = 1.1
            # new_args.position_gain_swing = 1000
            # new_args.velocity_gain_swing = 200
            
    else:
        raise("Controller not tuned for this setting")