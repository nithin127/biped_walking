# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 28th December 2023


def set_args(n_steps, original_args, new_args):
    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired

    if vx < 0 and vy == 0 and dyw == 0:
        # Move in x direction

        if original_args.scaled_biped == -1:
            new_args.vx_desired = 0

        else:
            new_args.mpc_weights_config = 8
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 1
            new_args.pitch_desired = 0.3

    elif vx == 0 and vy != 0 and dyw == 0:
        # Move laterally
        # raise("Controller not tuned for this setting")

        if original_args.scaled_biped != -1:
            raise ("Controller not tuned for this setting")

        if n_steps > 100:
            new_args.mpc_weights_config = 5.1
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 0.4
            new_args.vy_desired = 0.35 if original_args.vy_desired > 0 else -0.35
            new_args.pitch_desired = 0.3
            new_args.hip_height_desired = 0.69
            new_args.torso_height_desired = 0.72

        else:
            new_args.mpc_weights_config = 0
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 0
            new_args.vy_desired = 0
            new_args.pitch_desired = 0.3

    elif vy != 0 and dyw == 0:
        # Move diagonally
        # raise("Controller not tuned for this setting")

        if original_args.scaled_biped != -1:
            raise ("Controller not tuned for this setting")

        if n_steps > 1000:
            new_args.mpc_weights_config = 5
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 0.1
            new_args.vy_desired = 0.2 if original_args.vy_desired > 0 else -0.2
            new_args.pitch_desired = 0.4
            new_args.hip_height_desired = 0.7
            new_args.torso_height_desired = 0.7
            new_args.y_position_gain_raibert_controller = 1

        else:
            new_args.mpc_weights_config = 0
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 0
            new_args.vy_desired = 0
            new_args.pitch_desired = 0.3

    elif vx > 0 and vy == 0 and dyw == 0:
        # Move in +ve x direction

        if original_args.scaled_biped == -1:
            new_args.vx_desired = 0

        else:
            new_args.mpc_weights_config = 8
            new_args.foot_clearance = 0.12
            new_args.vx_desired = 2
            new_args.pitch_desired = 0.1

    else:
        raise ("Controller not tuned for this setting")
