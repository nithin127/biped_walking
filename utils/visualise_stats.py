import ipdb
import numpy as np
import matplotlib.pyplot as plt

from collections import deque
from dataclasses import dataclass
from utils.inertia_utils import quat2euler
from icecream import ic

PI = np.pi


@dataclass
class dataVisualiser:
    torso_x = []
    torso_x = []
    torso_y = []
    torso_z = []
    torso_r = []
    torso_p = []
    torso_yaw = []

    torque_st_abductor = []
    torque_st_hip = []
    torque_st_knee = []
    torque_sw_abductor = []
    torque_sw_hip = []
    torque_sw_knee = []

    delta_h_foot = []

    vel_x = deque([], maxlen=200)

    def update_step(self, ctrl):
        self.torso_x.append(ctrl.data.xpos[1][0])
        self.torso_y.append(ctrl.data.xpos[1][1])
        self.torso_z.append(ctrl.data.xpos[1][2])

        torso_quat = ctrl.data.xquat[1]
        torso_rpy = quat2euler(torso_quat)

        self.torso_r.append(torso_rpy[0])
        self.torso_p.append(torso_rpy[1])
        self.torso_yaw.append(torso_rpy[2])

        self.vel_x.append(ctrl.data.qvel[0])

        self.torque_st_abductor.append(ctrl.torque_st[0])
        self.torque_st_hip.append(ctrl.torque_st[1])
        self.torque_st_knee.append(ctrl.torque_st[2])
        self.torque_sw_abductor.append(ctrl.torque_sw[0])
        self.torque_sw_hip.append(ctrl.torque_sw[1])
        self.torque_sw_knee.append(ctrl.torque_sw[2])

        self.print_step(ctrl)

    def update_mode_change(self, ctrl):
        # self.print_mode_change()
        pass

    def print_step(self, ctrl):
        # pass

        # print(self.info_st["ctrl"])
        # return

        print("Step: ", ctrl._n_steps)

        if ctrl.phase < PI:
            site_stance_foot = ctrl.task_info.robot["site_left_foot_mid_id"]
        #     print("Left stance: ", ctrl.phase)

        else:
            site_stance_foot = ctrl.task_info.robot["site_right_foot_mid_id"]
        #     print("Right stance: ", ctrl.phase)

        print(ctrl.info_st["ctrl"])
        # if ctrl.info_st["ctrl"] == "Lost Contact":
        #     rec = input("Debug")
        #     if rec == "y":
        #         # plt.plot(self.delta_h_foot)
        #         # plt.show()
        #         import ipdb; ipdb.set_trace()
        #     else:
        #         pass
        # else:
        #     pass

        torso_xpos = ctrl.data.qpos[0]
        stance_foot_pos = ctrl.data.site_xpos[site_stance_foot]
        terrain_slope = np.degrees(ctrl.task_info.terrain["ground_slope"](torso_xpos))
        # terrain_slope = np.degrees(ctrl.task_info.terrain['ground_slope'](stance_foot_pos[0]))
        terrain_height = ctrl.task_info.terrain["ground_height"](torso_xpos)
        terrain_height_foot = ctrl.task_info.terrain["ground_height"](
            stance_foot_pos[0]
        )

        # print("")
        # print("X vel: ", np.mean(self.vel_x))
        print("terrain_slope: ", terrain_slope)
        print("terrain height: ", terrain_height)
        delta_h = np.round(stance_foot_pos[2] - terrain_height_foot, 5)
        # self.delta_h_foot.append(delta_h)
        print("Site - Terrain height: ", delta_h)
        # ic(stance_foot_pos[2])
        # ic(terrain_height_foot)
        # print("terrain_height: ", terrain_height)
        # print("stance foot (x, z): ", np.round(stance_foot_pos[0], 4), np.round(stance_foot_pos[2], 4))

        # print("vel cmd: ", ctrl.args.vx_desired)
        # print("pitch cmd: ", ctrl.args.pitch_desired)

        # print("torso_height: ", ctrl.data.qpos[2])
        # print("rel height cmd: ", ctrl.args.torso_height_desired)
        # print("rel height act wrt foot: ", ctrl.data.qpos[2] - stance_foot_pos[2])
        # print("rel height act wrt terrain: ", ctrl.data.qpos[2] - terrain_height)

        # print("rel hip height cmd: ", ctrl.args.hip_height_desired)
        # print("rel hip height act: ", ctrl.data.site_xpos[0][2] - terrain_height)
        # print("Foot pos R: ", self.foot_pos_r)
        # print("Foot pos L: ", self.foot_pos_l)

        print("\n")
        # print("\n\n")

    def print_mode_change(self, ctrl):
        pass

    def plot_debug_step(self, ctrl):
        rec = input("Options {Debug: d, Plot: p} = ")

        if rec == "p":
            plot_mpc_result(
                self.info_st["x_ref"], self.info_st["x_res"], self.info_st["force"]
            )

        elif rec == "d":
            ipdb.set_trace()

        else:
            pass

    def plot_debug_mode_change(self, ctrl):
        rec = input("Options {Debug: d, Plot: p} = ")

        if rec == "p":
            plot_mpc_result(
                ctrl.info_st["x_ref"], ctrl.info_st["x_res"], ctrl.info_st["force"]
            )
            plot_torques(
                self.torque_st_abductor,
                self.torque_st_hip,
                self.torque_st_knee,
                self.torque_sw_abductor,
                self.torque_sw_hip,
                self.torque_sw_knee,
            )
            plot_torso_motion(
                self.torso_x,
                self.torso_y,
                self.torso_z,
                self.torso_r,
                self.torso_p,
                self.torso_yaw,
            )

        elif rec == "d":
            ipdb.set_trace()

        else:
            pass


def plot_torso_motion(torso_x, torso_y, torso_z, torso_r, torso_p, torso_yaw):
    plt.subplot(3, 2, 1)
    plt.plot(torso_x, "-b", label="torso_x")
    plt.legend()

    plt.subplot(3, 2, 2)
    plt.plot(torso_y, "-b", label="torso_y")
    plt.legend()

    plt.subplot(3, 2, 3)
    plt.plot(torso_z, "-b", label="torso_z")
    plt.legend()

    plt.subplot(3, 2, 4)
    plt.plot(torso_r, "-b", label="torso_r")
    plt.legend()

    plt.subplot(3, 2, 5)
    plt.plot(torso_p, "-b", label="torso_p")
    plt.legend()

    plt.subplot(3, 2, 6)
    plt.plot(torso_yaw, "-b", label="torso_yaw")
    plt.legend()

    plt.show()


def plot_torques(
    torque_st_abductor,
    torque_st_hip,
    torque_st_knee,
    torque_sw_abductor,
    torque_sw_hip,
    torque_sw_knee,
):
    x = np.arange(len(torque_st_hip))

    plt.plot(x, torque_st_abductor, "k-")
    plt.plot(x, torque_st_hip, "r-")
    plt.plot(x, torque_st_knee, "m-")
    plt.legend(["stance: abductor", "stance: hip", "stance: knee"])
    plt.show()

    plt.plot(x, torque_sw_abductor, "b-")
    plt.plot(x, torque_sw_hip, "c-")
    plt.plot(x, torque_sw_knee, "g-")
    plt.legend(["swing: abductor", "swing: hip", "swing: knee"])
    plt.show()


def plot_mpc_result(x_ref, x_res, U):
    x_pos = []
    x_pos_ref = []
    x_vel = []
    x_vel_ref = []

    y_pos = []
    y_pos_ref = []
    y_vel = []
    y_vel_ref = []

    z_pos = []
    z_pos_ref = []
    z_vel = []
    z_vel_ref = []

    pitch = []
    pitch_ref = []
    pitch_rate = []
    pitch_rate_ref = []

    roll = []
    roll_ref = []
    roll_rate = []
    roll_rate_ref = []

    yaw = []
    yaw_ref = []
    yaw_rate = []
    yaw_rate_ref = []

    U_x = []
    U_y = []
    U_z = []

    planning_horizon = int(len(U) / 6)

    for i in range(planning_horizon):
        x_pos.append(x_res[i * 13 + 3])
        x_pos_ref.append(x_ref[i * 13 + 3])
        x_vel.append(x_res[i * 13 + 9])
        x_vel_ref.append(x_ref[i * 13 + 9])

        y_pos.append(x_res[i * 13 + 4])
        y_pos_ref.append(x_ref[i * 13 + 4])
        y_vel.append(x_res[i * 13 + 10])
        y_vel_ref.append(x_ref[i * 13 + 10])

        z_pos.append(x_res[i * 13 + 5])
        z_pos_ref.append(x_ref[i * 13 + 5])
        z_vel.append(x_res[i * 13 + 11])
        z_vel_ref.append(x_ref[i * 13 + 11])

        pitch.append(x_res[i * 13 + 1])
        pitch_ref.append(x_ref[i * 13 + 1])
        pitch_rate.append(x_res[i * 13 + 7])
        pitch_rate_ref.append(x_ref[i * 13 + 7])

        roll.append(x_res[i * 13])
        roll_ref.append(x_ref[i * 13])
        roll_rate.append(x_res[i * 13 + 6])
        roll_rate_ref.append(x_ref[i * 13 + 6])

        yaw.append(x_res[i * 13 + 2])
        yaw_ref.append(x_ref[i * 13 + 2])
        yaw_rate.append(x_res[i * 13 + 8])
        yaw_rate_ref.append(x_ref[i * 13 + 8])

        # Assumes left foot swing
        U_x.append(U[i * 6])
        U_y.append(U[i * 6 + 1])
        U_z.append(U[i * 6 + 2])

    plt.subplot(6, 2, 1)
    plt.plot(x_pos, "-b", label="x_pos")
    plt.plot(x_pos_ref, "-r", label="x_pos_ref")
    plt.legend()

    plt.subplot(6, 2, 2)
    plt.plot(x_vel, "-b", label="x_vel")
    plt.plot(x_vel_ref, "-r", label="x_vel_ref")
    plt.legend()

    plt.subplot(6, 2, 3)
    plt.plot(y_pos, "-b", label="y_pos")
    plt.plot(y_pos_ref, "-r", label="x_pos_ref")
    plt.legend()

    plt.subplot(6, 2, 4)
    plt.plot(y_vel, "-b", label="y_vel")
    plt.plot(y_vel_ref, "-r", label="y_vel_ref")
    plt.legend()

    plt.subplot(6, 2, 5)
    plt.plot(z_pos, "-b", label="z_pos")
    plt.plot(z_pos_ref, "-r", label="z_pos_ref")
    plt.legend()

    plt.subplot(6, 2, 6)
    plt.plot(z_vel, "-b", label="z_vel")
    plt.plot(z_vel_ref, "-r", label="z_vel_ref")
    plt.legend()

    plt.subplot(6, 2, 7)
    plt.plot(pitch, "-b", label="pitch")
    plt.plot(pitch_ref, "-r", label="pitch_ref")
    plt.legend()

    plt.subplot(6, 2, 8)
    plt.plot(pitch_rate, "-b", label="pitch_rate")
    plt.plot(pitch_rate_ref, "-r", label="pitch_rate_ref")
    plt.legend()

    plt.subplot(6, 2, 9)
    plt.plot(roll, "-b", label="roll")
    plt.plot(roll_ref, "-r", label="roll_ref")
    plt.legend()

    plt.subplot(6, 2, 10)
    plt.plot(roll_rate, "-b", label="roll_rate")
    plt.plot(roll_rate_ref, "-r", label="roll_rate_ref")
    plt.legend()

    plt.subplot(6, 2, 11)
    plt.plot(yaw, "-b", label="yaw")
    plt.plot(yaw_ref, "-r", label="yaw_ref")
    plt.legend()

    plt.subplot(6, 2, 12)
    plt.plot(yaw_rate, "-b", label="yaw_rate")
    plt.plot(yaw_rate_ref, "-r", label="yaw_rate_ref")
    plt.legend()

    plt.show()
