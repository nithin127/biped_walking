# Author: Nithin Vasishta
# Created : 4th July, 2023
# Last Edited: 8th September

import argparse

import mujoco as mj
import mujoco_viewer

from high_level_controller import HighLevelController
from infrastructure.task_info import TaskInfo

import utils.initialise_utils as init
from utils.inertia_utils import quat2euler


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("-x", "--scaled-biped", default=-1, type=int, help="Use the scaled version of the biped. -1: don't use scaled biped. 0: use same density. 1: double density. 2: only torso density increase to get double weight")

    parser.add_argument("-sc", "--starting-config", default=0, type=float, help="Which starting config to use")
    parser.add_argument("-ts", "--terrain-slope", default=0, type=int, help="Slope of the incline in degrees")
    parser.add_argument("-ss", "--sinusoidal-slope", default=0, type=int, help="Slope of the sinusoidal terrain in degrees")
    parser.add_argument("-rs", "--random-sine", default=0, type=int, help="Type of random sine. 0 indicates uniform sine")
    parser.add_argument("-sh", "--stair-height", default=0, type=int, help="Stair height in cm. This overrides terrain slope")
    parser.add_argument("-sty", "--stair-type", default=0, type=int, help="Stair type")

    parser.add_argument("-w", "--added-weight-percent", default=0, type=int, help="Percent of weight increase. Torso")
    parser.add_argument("-wk", "--added-weight-known", action="store_true", help="Do we know the added weight while planning")

    parser.add_argument("-gf", "--gait-frequency", default=3, type=float, help="Number of gait cycles per second")
    parser.add_argument("-mf", "--mpc-frequency", default=50, type=float, help="Number of time MPC controller for stance leg is executed per second")
    parser.add_argument("-sf", "--swing-frequency", default=1000, type=float, help="Number of times classical swing leg controller is executed per second")
    parser.add_argument("-lcf", "--lost-contact-frequency", default=1000, type=float, help="Number of times Lost Contact controller for stance leg is executed per second")

    parser.add_argument("-jc", "--joint-control", action="store_true", help="Torques directly calculated at joint level for swing leg")
    parser.add_argument("-nff", "--no-feedforward-torque", action="store_true", help="We will not account for feedforward torque in swing leg torque calculations")
    parser.add_argument("-ntr", "--no-torso-regulation", action="store_true", help="We will use torso regulation to stabilise the torso roll")

    parser.add_argument("-fc", "--foot-clearance", default=0.12, type=float, help="Height of foot during swing during swing trajectory")
    parser.add_argument("-hh", "--hip-height-desired", default=0.72, type=float, help="Hip height relative to the foot during swing")
    parser.add_argument("-th", "--torso-height-desired", default=0.75, type=float, help="Desired torso height relative to the ground")

    parser.add_argument("-vx", "--vx-desired", default=-0.3, type=float, help="Desired velocity in x direction (along the terrain slope)")
    parser.add_argument("-vy", "--vy-desired", default=0.0, type=float, help="Desired velocity in y direction (along the terrain slope)")
    parser.add_argument("-yaw", "--yaw-rate-desired", default=0.0, type=float, help="yaw rate")
    parser.add_argument("-pch", "--pitch-desired", default=0.3, type=float, help="Desired velocity in x direction")
    parser.add_argument("-ph", "--planning-horizon", default=15, type=int, help="Planning horizon for mpc")

    parser.add_argument("-mwc", "--mpc-weights-config", default=0, type=float, help="Config ID for mpc")
    parser.add_argument("-lcz", "--lost-contact-only-z", action="store_true", help="To use only z method or the one where we track x position as well")

    parser.add_argument("-kpsw", "--position-gain-swing", default=800, type=float, help="Position gain for PD control: Swing")
    parser.add_argument("-kvsw", "--velocity-gain-swing", default=100, type=float, help="Velocity gain for PD control: Swing")
    parser.add_argument("-kprx", "--x-position-gain-raibert-controller", default=0.2, type=float, help="X Position gain for Raibert Swing Controller")
    parser.add_argument("-kpry", "--y-position-gain-raibert-controller", default=0.2, type=float, help="Y Position gain for Raibert Swing Controller")
    parser.add_argument("-kplc", "--position-gain-lost-contact-controller", default=100, type=float, help="Position gain for Lost Contact Controller")
    parser.add_argument("-kvlc", "--velocity-gain-lost-contact-controller", default=5, type=float, help="Velocity gain for Lost Contact Controller")
    parser.add_argument("-xclc", "--pos-correction-coefficient-lost-contact-controller", default=0.03, type=float, help="Position correction coefficient for Lost Contact Controller")
    parser.add_argument("-aclc", "--ang-correction-coefficient-lost-contact-controller", default=0.03, type=float, help="Angular correction coefficient for Lost Contact Controller")

    parser.add_argument("-sk", "--skip-timesteps", default="0", type=int, help="Will skip timesteps before getting stats")
    parser.add_argument("-skr", "--skip-num-reset", default="1", type=int, help="Asks to plot after skipping x resets")
    parser.add_argument("-plt", "--plot", default="none", choices=["none", "step", "mode_change"], help="Plots at every mode_change, or doesn't plot")

    args = parser.parse_args()

    ## MuJoCo data structures
    model = init.get_model(args)
    data = init.get_data(model)

    if not args.scaled_biped == -1:
        args.mpc_weights_config = 8
        args.hip_height_desired = 1.15
        args.torso_height_desired = 1.2
        args.position_gain_swing = 800
        args.velocity_gain_swing = 100
        args.x_position_gain_raibert_controller = 0.15
        args.y_position_gain_raibert_controller = 0.15
        # args.gait_frequency = 4
    else:
        pass

    # Modify according to task
    init.set_starting_config(args)
    init.initialise_position(model, data, args)

    mj.mj_step(model, data)

    # import ipdb; ipdb.set_trace()

    ## Mujoco Viewer
    viewer = mujoco_viewer.MujocoViewer(model, data)

    viewer.cam.azimuth = 90
    viewer.cam.elevation = 0
    viewer.cam.distance =  4

    # viewer.cam.azimuth = 40
    # viewer.cam.elevation = -15
    # viewer.cam.distance =  4.5

    ## Init Controller
    task_info = TaskInfo(model, data, args)
    high_ctrl = HighLevelController(model, data, args, task_info)

    # Added Weight
    body_torso_id = task_info.robot["body_torso_id"]
    geom_torso_id = task_info.robot["geom_torso_id"]

    init_rgba = model.geom_rgba[geom_torso_id].copy()
    new_rgba = [0.9, 0, 0, 1]
    init_torso_mass = model.body_mass[body_torso_id]
    new_torso_mass = init_torso_mass + (args.added_weight_percent/100)*(sum(model.body_mass))

    # time_added = [10000*i + 1000 for i in range(10)]
    # time_removed = [10000*i + 6000 for i in range(10)]

    # time_added = [5000*i + 400 for i in range(10)]
    # time_removed = [5000*i + 2900 for i in range(10)]

    time_added = [1000]
    time_removed = []

    # time_added = []
    # time_removed = []

    ## Simulation loop
    n_steps = 0

    while viewer.is_alive:

        high_ctrl.set_action()            
        mj.mj_step(model, data)

        n_steps += 1
        # import ipdb; ipdb.set_trace()

        if n_steps in time_added:
            model.body_mass[body_torso_id] = new_torso_mass
            model.geom_rgba[geom_torso_id] = new_rgba

        elif n_steps in time_removed:
            model.body_mass[body_torso_id] = init_torso_mass
            model.geom_rgba[geom_torso_id] = init_rgba

        else:
            pass

        # camera follows the robot
        viewer.cam.lookat[0] = data.qpos[0]
        viewer.cam.lookat[1] = data.qpos[1]
        viewer.cam.lookat[2] = data.qpos[2] - 0.5
        viewer.render()

        rpy = quat2euler(data.xquat[1])

        if (abs(rpy) > 2.5).any():
            break;

    # close
    viewer.close()
