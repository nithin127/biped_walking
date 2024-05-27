# Author: Nithin Vasishta
# Created: 4th August 2023

import numpy as np
import mujoco as mj


def get_xml(args):
    if args.stair_height != 0:
        assert args.terrain_slope == 0
        assert args.sinusoidal_slope == 0

        if args.stair_height == 6 and args.stair_type == 0:
            xml_path = "assets/biped_stairs_6_0.xml"
        elif args.stair_height == 14 and args.stair_type == 0:
            xml_path = "assets/biped_stairs_14_0.xml"
        else:
            raise ("XML corresponding to stair height does not exist")

    elif args.terrain_slope == 10:
        assert args.sinusoidal_slope == 0
        xml_path = "assets/biped_10.xml"

    elif args.terrain_slope == 30:
        assert args.sinusoidal_slope == 0
        if args.scaled_biped != -1:
            if args.scaled_biped == 2:
                xml_path = "assets/biped_30_scaled_2.xml"
            else:
                raise ("XML not present")

        elif args.added_weight_percent == 0:
            xml_path = "assets/biped_30.xml"

        else:
            xml_path = "assets/biped_30_highTorqueLimit.xml"

    elif args.terrain_slope == 45:
        assert args.sinusoidal_slope == 0
        if args.scaled_biped != -1:
            if args.scaled_biped == 2:
                xml_path = "assets/biped_45_scaled_2.xml"
            else:
                raise ("XML not present")

        elif args.added_weight_percent == 0:
            xml_path = "assets/biped_45.xml"

        else:
            # xml_path = 'assets/biped_45_highTorqueLimit.xml'
            xml_path = "assets/biped_45.xml"

    elif args.terrain_slope == -15:
        assert args.sinusoidal_slope == 0
        xml_path = "assets/biped_ne15.xml"

    elif args.terrain_slope == -30:
        assert args.sinusoidal_slope == 0
        if args.scaled_biped != -1:
            if args.scaled_biped == 0:
                xml_path = "assets/biped_ne30_scaled_0.xml"

            elif args.scaled_biped == 2:
                xml_path = "assets/biped_ne30_scaled_2.xml"

            elif args.scaled_biped == 3:
                xml_path = "assets/biped_ne30_scaled_3.xml"

            else:
                raise ("XML not present")

        else:
            xml_path = "assets/biped_ne30.xml"

    else:
        if args.random_sine != 0:
            assert args.sinusoidal_slope != 0
        else:
            pass

        if args.added_weight_percent == 0 and args.scaled_biped == -1:
            xml_path = "assets/biped.xml"

        elif args.scaled_biped == 0:
            # no density change
            xml_path = "assets/biped_scaled_0.xml"

        elif args.scaled_biped == 1:
            # density doubled
            xml_path = "assets/biped_scaled_1.xml"

        elif args.scaled_biped == 2:
            # density of torso increased to get double biped mass
            xml_path = "assets/biped_scaled_2.xml"

        else:
            xml_path = "assets/biped_highTorqueLimit.xml"

    return xml_path


def add_sinusoidal_bump(row_i, freq, amp, phase_st, phase_end, tile_length_x):
    row_i = list(row_i)
    delta_x = (phase_end - phase_st) * amp / freq
    num_indices = int(delta_x / tile_length_x)

    row_iplus1 = []
    for i in range(num_indices + 1):
        x_i = tile_length_x * i
        row_iplus1.append(amp * np.sin(freq / amp * x_i + phase_st))

    diff = row_iplus1[-1] - row_i[0]
    row_iplus1 = [h - diff for h in row_iplus1[:-1]]

    return row_iplus1 + row_i


def add_slope(row_i, slope, num_indices, tile_length_x):
    row_i = list(row_i)

    row_iplus1 = []
    for i in range(num_indices + 1):
        x_i = tile_length_x * i
        row_iplus1.append(x_i * slope)

    row_iplus1 = row_iplus1[::-1]
    diff = row_iplus1[-1] - row_i[0]
    row_iplus1 = [h - diff for h in row_iplus1[:-1]]

    return row_iplus1 + row_i


def get_model(args):
    xml_path = get_xml(args)
    model = mj.MjModel.from_xml_path(xml_path)

    if args.sinusoidal_slope == 0:
        pass

    else:
        # We make sure that nrow and ncol are odd
        # so that (0,0) has a distinct value
        nrow = model.hfield_nrow[0]
        ncol = model.hfield_ncol[0]

        lenx = 2 * model.hfield_size[0][0]

        tile_length_x = lenx / ncol

        freq = np.radians(args.sinusoidal_slope)

        ## Build a single row of the terrain

        if args.random_sine == 0:
            row_i = []
            for j in range(ncol):
                x_i = (j - (ncol - 1) / 2) * tile_length_x
                row_i.append(np.sin(freq * x_i - np.radians(90)))

        else:
            row_i = [-1]

            if args.random_sine == 1:
                main_freq = freq
                terrain = [
                    ("slope", 0, 15),
                    ("sine", main_freq, 0.3, np.pi / 2, 3 * np.pi / 2),
                    ("sine", main_freq, 0.2, -np.pi, np.pi / 2),
                    ("slope", main_freq, 25),
                    ("sine", main_freq, 0.6, 0, np.pi),
                    ("sine", main_freq, 0.1, 0, 2 * np.pi),
                    ("sine", main_freq, 0.7, 3 * np.pi / 2, 2 * np.pi),
                    ("slope", 0, 25),
                ]
                terrain = terrain * 5

            elif args.random_sine == 2:
                main_freq = freq
                aux_freq1 = np.radians(15)
                # aux_freq2 = np.radians(30)
                terrain = [
                    ("slope", 0, 8),
                    ("sine", main_freq, 0.7, np.pi / 2, 3 * np.pi / 2),
                    ("sine", aux_freq1, 0.8, -np.pi / 2, np.pi / 2),
                    # ("slope", 0, 15),
                    # ("sine", aux_freq2, 0.3, np.pi, 3 * np.pi / 2),
                    # ("slope", aux_freq2, 25),
                    # ("sine", aux_freq2, 0.6, np.pi / 2, np.pi),
                    ("slope", 0, 8),
                ]
                terrain = terrain * 10

            else:
                raise ("Random sine ID not available")

            # We are building the terrain from the right
            # arguments: ("slope", slope_val, num_indices), ("sine", amplitude, phase_st, phase_end)

            for t in terrain:
                if t[0] == "slope":
                    row_i = add_slope(row_i, t[1], t[2], tile_length_x)
                else:
                    row_i = add_sinusoidal_bump(
                        row_i, t[1], t[2], t[3], t[4], tile_length_x
                    )

            # Flat for the remaining (if any indices left)
            if len(row_i) >= ncol:
                row_i = row_i[-ncol:]

            else:
                nleft = ncol - len(row_i)
                row_i = [row_i[0]] * nleft + row_i

        ## Prepare the final model

        terrain_map = row_i * nrow
        model.hfield_data = np.array(terrain_map).flatten()

    return model


def get_data(model):
    return mj.MjData(model)


def set_starting_config(args):
    # Default initialisation of args according to task
    if not args.scaled_biped == -1:
        if args.random_sine != 0:
            args.starting_config = 10

        elif args.sinusoidal_slope != 0:
            args.starting_config = 11

        elif args.terrain_slope == 30:
            args.starting_config = 12

        elif args.terrain_slope == 45:
            args.starting_config = 12.45

        elif args.terrain_slope == -15:
            args.starting_config = 14

        elif args.terrain_slope == -30:
            args.starting_config = 13

        else:
            args.starting_config = 9

    else:
        if args.stair_height != 0:
            args.starting_config = 3

        elif args.random_sine != 0:
            args.starting_config = 7

        elif args.sinusoidal_slope != 0:
            args.starting_config = 4

        elif args.terrain_slope == 30:
            args.starting_config = 1

        elif args.terrain_slope == 45:
            args.starting_config = 8

        elif args.terrain_slope == -15:
            args.starting_config = 5

        elif args.terrain_slope == -30:
            args.starting_config = 6

        else:
            # args.starting_config = -1
            pass


def initialise_position(model, data, args):
    if args.starting_config == 0:
        # Flat ground
        init_pos = np.array([0, 0, 0.8, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

        # init_vel[0] = self.args.vx_desired        # X vel
        # init_vel[1] = self.args.vy_desired        # Y vel

    elif args.starting_config == 1:
        # 30 degree slope
        init_pos = np.array([0, 0, 0.75, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

        # init_vel[0] = self.args.vx_desired        # X vel
        # init_vel[1] = self.args.vy_desired        # Y vel

    elif args.starting_config == 2:
        # 45 degree slope
        init_pos = np.array([0, 0, 0.75, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

        init_vel[0] = -0.2  # X vel
        init_vel[2] = 0.2  # Z vel

    elif args.starting_config == 3:
        # Stairs
        init_pos = np.array([0.3, 0, 0.8, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 4:
        # Sinusoidal slopes

        option = 1

        if option == 1:
            init_pos = np.array(
                [0, 0, -0.2, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
            )
            init_vel = np.zeros(model.nv)

        elif option == 2:
            init_pos = np.array(
                [-6, 0, 1.8, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
            )
            init_vel = np.zeros(model.nv)

        elif option == 3:
            # From first hill (30 degree slope)
            init_pos = [
                -6.00056216e00,
                -1.27074687e-01,
                1.80886288e00,
                9.96515647e-01,
                1.21348426e-02,
                8.23761070e-02,
                4.84640486e-03,
                -1.70038803e-02,
                -5.24307765e-01,
                -1.04373330e00,
                6.61708060e-02,
                -3.34880850e-01,
                -1.06427023e00,
            ]
            init_vel = [
                -0.81757649,
                -0.2106412,
                -0.24997668,
                0.50486062,
                -1.65974997,
                -0.41061333,
                0.50186096,
                -1.1910049,
                0.20788969,
                0.6409103,
                -3.94911024,
                -3.93505356,
            ]

        elif option == 4:
            # Drop from height (30 degree slope)
            init_pos = np.array(
                [0, 0, 0.8, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
            )
            init_vel = np.zeros(model.nv)

        elif option == 5:
            # From first hill (45 degree slope)
            init_pos = [
                -3.96949169,
                -0.06758544,
                1.79753724,
                0.99613579,
                -0.00536108,
                0.07293125,
                -0.04863937,
                -0.04976841,
                -0.4075139,
                -1.15405483,
                0.0228687,
                -0.57200725,
                -1.09645277,
            ]

            init_vel = [
                -0.73049018,
                0.17405263,
                -0.13320547,
                -0.08867497,
                -1.2269031,
                0.59011905,
                -0.28465794,
                -3.92828756,
                -4.64184544,
                0.18377789,
                -1.13447423,
                -1.46752972,
            ]

        else:
            raise ("Wrong option")

    elif args.starting_config == 5:
        # Negative 15 slope
        init_pos = np.array([0, 0, 0.8, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 6:
        # Negative 30 slope
        init_pos = np.array([0, 0, 0.9, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 7:
        # Random sine terrain
        init_pos = np.array([50, 0, -0.2, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 8:
        # 45 degree slope
        init_pos = np.array([0, 0, 0.8, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 9:
        # Scaled biped: flat ground

        option = 1

        if option == 1:
            init_pos = np.array(
                [48, 0, 1.265, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
            )
            init_vel = np.zeros(model.nv)

        else:
            init_pos = np.array([48, 0, 3.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
            init_vel = np.zeros(model.nv)

    elif args.starting_config == 10:
        # Scaled biped: random sinusoidal

        init_pos = np.array(
            [49.5, 0, 0.265, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
        )
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 11:
        # Scaled biped: sinusoidal slope

        option = 2

        if option == 1:
            # valley
            init_pos = np.array(
                [0, 0, 0.265, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
            )
            init_vel = np.zeros(model.nv)

        else:
            # peak
            init_pos = np.array(
                [-6, 0, 2.265, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05]
            )
            init_vel = np.zeros(model.nv)

    elif args.starting_config == 12:
        # Scaled biped: 30 degree incline

        init_pos = np.array([0, 0, 1.2, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 12.45:
        # Scaled biped: 45 degree incline

        init_pos = np.array([0, 0, 1.2, 1, 0, 0.04, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 13:
        # Scaled biped: -30 degree incline

        init_pos = np.array([0, 0, 1.4, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == 14:
        # Scaled biped: -15 degree incline

        init_pos = np.array([0, 0, 0.83, 0, 0, 0, 0, 0, -0.58, -1.13, 0, -0.7, -1.05])
        init_vel = np.zeros(model.nv)

    elif args.starting_config == -1:
        # Straight robot for Identifying offsets and lengths
        init_pos = np.zeros(model.nq)
        init_vel = np.zeros(model.nv)
        init_pos[2] = 2

    else:
        raise ("Mention valid starting_config")

    data.qpos = init_pos
    data.qvel = init_vel
