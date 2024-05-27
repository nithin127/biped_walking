# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 31st January 2024


def set_args(original_args, new_args):
    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired

    # raise("Controller not tuned for this setting")

    if vx < 0 and vy == 0 and dyw == 0:
        if original_args.scaled_biped == 2:
            new_args.mpc_weights_config = 8.245
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 0.7
            new_args.pitch_desired = 0.35
            new_args.hip_height_desired = 1

        else:
            raise ("Controller not tuned for this setting")

    elif vx == 0 and vy == 0 and dyw == 0:
        if original_args.scaled_biped == 2:
            new_args.mpc_weights_config = 8.245
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 1
            new_args.pitch_desired = 0.35
            new_args.hip_height_desired = 1

        else:
            raise ("Controller not tuned for this setting")

    elif vx > 0 and vy == 0 and dyw == 0:
        # Move in +ve x direction

        if original_args.scaled_biped == 2:
            new_args.mpc_weights_config = 8.2451
            new_args.foot_clearance = 0.1
            new_args.vx_desired = 1.2
            new_args.pitch_desired = 0.45
            new_args.hip_height_desired = 1

        else:
            raise ("Controller not tuned for this setting")

    else:
        raise ("Controller not tuned for this setting")
