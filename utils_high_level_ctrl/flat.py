# Author: Nithin Vasishta
# Date Created: 3rd September 2023
# Last Edited: 11th September 2023

def set_args(original_args, new_args):

    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired

    if vx == 0 and vy == 0 and dyw == 0:
        # Step in place

        if original_args.scaled_biped == -1:
            new_args.vx_desired = 0.7
            
        else:
            ## Eventually steps in place
            new_args.mpc_weights_config = 8
            new_args.vx_desired = 1.6

            ## Rotates a little
            # new_args.mpc_weights_config = 8
            # new_args.vx_desired = 1.6
            # new_args.pitch_desired = 1

    elif vx == 0 and vy == 0:
        # Rotate in place:
        # Think there is something wrong in the implementation
        # The robot is just not turning
        raise("Rotate in place: not tuned")
        new_args.mpc_weights_config = 7
        new_args.vx_desired = 0.7
        new_args.pitch_desired = 1
        new_args.yaw_rate_desired = -2

    elif vy == 0 and dyw == 0:
        # Move in x direction
        # Not exact velocity tracking
        if original_args.scaled_biped == -1:

            if vx < 0:
                new_args.vx_desired = max(-0.7, vx)
            else:
                new_args.vx_desired = min(max(1, vx), 3)

        else:
            new_args.mpc_weights_config = 8
            new_args.vx_desired = 1.3
        
    elif vx == 0 and dyw == 0:
        # Move in y direction
        if vy > 0:
            # Not exact velocity tracking
            new_args.mpc_weights_config = 6
            new_args.vx_desired = 0.7
            new_args.vy_desired = min(max(0.2, vy), 1.2)
            new_args.yaw_rate_desired = -1
            new_args.pitch_desired = 1

        else:
            # No velocity tracking
            new_args.mpc_weights_config = 6
            new_args.vx_desired = 0.77
            new_args.vy_desired = 0.4
            new_args.pitch_desired = 1

    elif dyw == 0:
        # Move diagonally
        # No velocity tracking
        if vx < 0 and vy < 0:
            new_args.mpc_weights_config = 6
            new_args.vx_desired = 0.7
            new_args.vy_desired = -0.4

        elif vx < 0 and vy > 0:
            new_args.mpc_weights_config = 6
            new_args.vx_desired = 0.7
            new_args.vy_desired = 0.4

        elif vx > 0 and vy < 0:
            new_args.mpc_weights_config = 6
            new_args.vx_desired = 1.4
            new_args.vy_desired = -0.4

        else:
            new_args.mpc_weights_config = 6
            new_args.vx_desired = 1.4
            new_args.vy_desired = 0.4

    elif vy == 0:
        # Move in circle
        raise("Move in circle: not tuned")

    else:
        raise("Controller not tuned for this setting")
