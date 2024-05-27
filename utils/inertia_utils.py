# Author: Nithin Vasishta
# Last Modified: 4th August 2023


import numpy as np
from scipy.spatial.transform import Rotation as R


def quat2euler(quat):
    # SciPy defines quaternion as [x, y, z, w]
    # MuJoCo defines quaternion as [w, x, y, z]
    _quat = np.concatenate([quat[1:], quat[:1]])
    r = R.from_quat(_quat)

    # roll-pitch-yaw is the same as rotating w.r.t
    # the x, y, z axis in the world frame
    euler = r.as_euler('xyz', degrees=False)

    return euler


def calculate_inertia(model, data, robot_info, calculation_type="wrt_torso", consider_full_mass=True):

	i_st = robot_info["body_start_id"]
	i_end = robot_info["body_end_id"] + 1
		
	if calculation_type == "torso_only":
		# We only consider the inertia at torso
		Ixx, Iyy, Izz = model.body_inertia[robot_info["body_torso_id"]]
		Rmat = data.ximat[1].reshape((3,3))
		inertia_tensor = Rmat@np.diag((Ixx, Iyy, Izz))@Rmat.T

		if consider_full_mass:
			full_mass = sum(model.body_mass[i_st:i_end])
			inertia_tensor = inertia_tensor*full_mass/model.body_mass[1]

	else:

		if calculation_type == "wrt_torso":
			p_ref = data.xpos[robot_info["body_torso_id"]]
		elif calculation_type == "wrt_com":
			p_ref = np.sum([data.xipos[i]*model.body_mass[i] for i in range(i_st, i_end)], axis=0)/sum(model.body_mass[i_st:i_end])
		else: #wrt_world
			p_ref = np.array([0, 0, 0])

		# Note: data.xpos -> body frame pos; data.xipos -> body com pos. Both are same for torso
		inertia_tensor = np.zeros((3,3))
		for i in range(i_st, i_end):
			delta_p = data.xipos[i] - p_ref
			Ixx, Iyy, Izz = model.body_inertia[i]
			Ixx += model.body_mass[i]*(delta_p[1]**2 + delta_p[2]**2)
			Iyy += model.body_mass[i]*(delta_p[2]**2 + delta_p[0]**2)
			Izz += model.body_mass[i]*(delta_p[0]**2 + delta_p[1]**2)
			Rmat = data.ximat[i].reshape((3,3))
			inertia_tensor += Rmat@np.diag((Ixx, Iyy, Izz))@Rmat.T

	return inertia_tensor
