# Author: Nithin Vasishta
# Date Created: 22nd September 2023
# Last Edited: 22nd September 2023

import numpy as np


def set_args(original_args, new_args, slope, delta_slope, slope_state):
    vx = original_args.vx_desired
    vy = original_args.vy_desired
    dyw = original_args.yaw_rate_desired

    print_qpos = False

    # raise("Controller not tuned for this setting")

    if vx != 0 and vy == 0 and dyw == 0:
        # Move in x direction
        # Finite state machine; No velocity tracking

        if slope >= 0 and slope < np.radians(30):
            slope_state = 1

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 0.4
                new_args.pitch_desired = 0.35

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 8  # 8.2452 also works
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.3
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.3
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(30) and slope <= np.radians(45):
            slope_state = 1.5

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 0.25
                new_args.pitch_desired = 0.38

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 8.2452
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.3
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2
                new_args.position_gain_swing = 1000
                # new_args.velocity_gain_swing = 200

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.3
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-15) and slope < np.radians(0) and delta_slope < 0:
            if slope_state != 2:
                print_qpos = True
            #     print(data.qpos)
            #     print(data.qvel)
            #     rec = input("2")

            slope_state = 2

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 0.2
                new_args.pitch_desired = 1

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1.4
                new_args.pitch_desired = 0.45
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 1
                new_args.pitch_desired = 0.4
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-25) and slope < np.radians(-15) and delta_slope < 0:
            # if slope_state != 3:
            #     rec = input("3")

            slope_state = 3

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 4
                new_args.foot_clearance = 0.05
                new_args.vx_desired = 0.2
                new_args.pitch_desired = 1

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 9
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.8
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 9
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.8
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-30) and slope < np.radians(-25) and delta_slope < 0:
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

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-45) and slope < np.radians(-30):
            # if slope_state != 4:
            #     print(data.qpos)
            #     print(data.qvel)
            #     print(low_ctrl.phase)
            #     rec = input("4")

            slope_state = 4

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 12
                new_args.foot_clearance = 0.05
                new_args.vx_desired = 0.35
                new_args.pitch_desired = 1.2

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-30) and slope < np.radians(-25) and delta_slope > 0:
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

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 10
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 2
                new_args.pitch_desired = 0.9
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-25) and slope < np.radians(-15) and delta_slope > 0:
            # if slope_state != 5:
            #     rec = input("5")

            slope_state = 5

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 4
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 0.2
                new_args.pitch_desired = 1

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 9
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 1.5
                new_args.pitch_desired = 0.8
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 9
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 1.5
                new_args.pitch_desired = 0.8
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        elif slope >= np.radians(-15) and slope < np.radians(0) and delta_slope > 0:
            # if slope_state != 6:
            #     rec = input("6")

            slope_state = 5

            if original_args.scaled_biped == -1:
                new_args.mpc_weights_config = 0
                new_args.foot_clearance = 0.08
                new_args.vx_desired = 0.1
                new_args.pitch_desired = 0.5

            elif original_args.scaled_biped == 2 and original_args.random_sine == 2:
                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 0.7
                new_args.pitch_desired = 0.6
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            elif original_args.scaled_biped == 2:
                new_args.mpc_weights_config = 8
                new_args.foot_clearance = 0.12
                new_args.vx_desired = 0.7
                new_args.pitch_desired = 0.6
                new_args.hip_height_desired = 1.15
                new_args.torso_height_desired = 1.2

            else:
                raise ("Not tuned for this setting")

        else:
            input(f"Sine45: New slope condition: {np.degrees(slope)}. Please check")
            pass

    else:
        raise ("Controller not tuned for this setting")

    return slope_state, print_qpos
