# Author: Nithin Vasishta
# Created: 28th July 2023
# Last edited: 17th March 2024

import numpy as np

PI = np.pi


def get_KL(param_id):
    L_i = (
        np.eye(13) * 0
    )  # 13 = 3 com_rpy, 3 com_pos, 3 com_angular_vel, com_vel, 1_gravity
    K_i = (
        np.eye(3 * 2) * 0.01
    )  # 3 corresponds to num_ground_reaction_forces, 2 corresponds to num_legs

    if param_id == 1:
        # For stage 1 of stair manuever

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

    elif param_id == 2:
        # For stage 2 of stair manuever

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

    elif param_id == 3:
        # For stage 3 of stair manuever

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

    elif param_id == 4:
        # Downhill

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 12  # Yaw
        L_i[3, 3] = 20  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 2  # Z

    elif param_id == 5:
        # Slant walking 30 degree slope

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 12  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 15  # Z

    elif param_id == 5.1:
        # Lateral walking 30 degree slope

        L_i[0, 0] = 10  # Roll
        L_i[1, 1] = 12  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 15  # Z

    elif param_id == 6:
        # Sideways walking, increasing yaw component

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 18  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

    elif param_id == 7:
        # Rotate in place, doesn't work

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 30  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

    elif param_id == 8:
        # Scaled biped 2: flat and up slope

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

        K_i = 0.01 * K_i

    elif param_id == 8.1:
        # Scaled biped 2: backward up slope

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

        K_i = 0.01 * K_i

    elif param_id == 8.245:
        # Scaled biped 2: 45 degree biped

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 20  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

        K_i = 0.01 * K_i

    elif param_id == 8.2451:
        # Scaled biped 2: 45 degree biped (backward downslope)

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 12  # Pitch
        L_i[2, 2] = 45  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 8  # Z

        K_i = 0.01 * K_i

    elif param_id == 8.2452:
        # Scaled biped 2: 45 degree biped (sinusoidal)

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

        K_i = 0.01 * K_i

    elif param_id == 9:
        # Scaled biped 2: downslope

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 12  # Yaw
        L_i[3, 3] = 20  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 7  # Z

        K_i = 0.1 * K_i

    elif param_id == 10:
        # Scaled biped 2: downslope (25 to 30 degree)

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

        K_i = 0.01 * K_i

    elif param_id == 11:
        # Scaled biped 0: downslope

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 12  # Yaw
        L_i[3, 3] = 20  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 5  # Z

        K_i = 0.1 * K_i

    elif param_id == 12:
        # 45 degree downslope

        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 12  # Yaw
        L_i[3, 3] = 20  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 1  # Z

    else:
        L_i[0, 0] = 15  # Roll
        L_i[1, 1] = 8  # Pitch
        L_i[2, 2] = 8  # Yaw
        L_i[3, 3] = 2  # X
        L_i[4, 4] = 5  # Y
        L_i[5, 5] = 20  # Z

    return K_i, L_i


def get_desired_reference_values(phase, data, args, task_info, torso_pos, torso_rpy):
    # The reference trajectory will track these desired value

    if phase < PI:
        isite_foot = task_info.robot["site_left_foot_mid_id"]
    else:
        isite_foot = task_info.robot["site_right_foot_mid_id"]

    absolute_torso_roll_desired = 0
    relative_torso_roll_desired = absolute_torso_roll_desired - torso_rpy[0]

    absolute_torso_pitch_desired = args.pitch_desired
    relative_torso_pitch_desired = absolute_torso_pitch_desired - torso_rpy[1]

    # absolute_torso_yaw_desired = 0
    # relative_torso_yaw_desired = absolute_torso_yaw_desired - torso_rpy[2]
    relative_torso_yaw_desired = 0

    # absolute_torso_x_desired = 0
    # relative_torso_x_desired = absolute_torso_x_desired - torso_pos[0]
    relative_torso_x_desired = 0

    # absolute_torso_y_desired = 0
    # relative_torso_y_desired = absolute_torso_y_desired - torso_pos[1]
    relative_torso_y_desired = 0

    # absolute_torso_z_desired = torso_height_desired
    absolute_torso_z_desired = args.torso_height_desired

    if args.stair_height != 0:
        stance_leg_x = data.site_xpos[isite_foot][0]
        stair_height = task_info.terrain["ground_height"](stance_leg_x)
        absolute_torso_z_desired += stair_height

    elif args.sinusoidal_slope != 0:
        absolute_torso_z_desired += task_info.terrain["ground_height"](torso_pos[0])

    elif args.terrain_slope != 0:
        tan0 = np.tan(np.radians(args.terrain_slope))
        absolute_torso_z_desired += -1 * torso_pos[0] * tan0

    relative_torso_z_desired = absolute_torso_z_desired - torso_pos[2]
    # relative_torso_z_desired = 0

    desired_stats = {
        "relative_torso_roll_desired": relative_torso_roll_desired,
        "relative_torso_pitch_desired": relative_torso_pitch_desired,
        "relative_torso_yaw_desired": relative_torso_yaw_desired,
        "relative_torso_x_desired": relative_torso_x_desired,
        "relative_torso_y_desired": relative_torso_y_desired,
        "relative_torso_z_desired": relative_torso_z_desired,
        "vx_desired": args.vx_desired,
        "vy_desired": args.vy_desired,
        "yaw_rate_desired": args.yaw_rate_desired,
    }

    return desired_stats
