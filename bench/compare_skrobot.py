#!/usr/bin/env python
import skrobot
from skrobot.coordinates import CascadedCoords, Coordinates
import _tinyfk as tinyfk
import numpy as np
import time

# common setting
urdf_path = "../data/fetch.urdf"
rtree = tinyfk.RobotModel(urdf_path)

link_names = ["shoulder_pan_link", "shoulder_lift_link",
              "upperarm_roll_link", "elbow_flex_link",
              "forearm_roll_link", "wrist_flex_link",
              "wrist_roll_link"]
elink_names = [
        "l_gripper_finger_link", 
        "r_gripper_finger_link", 
        "wrist_flex_link",
        "wrist_roll_link",
        "shoulder_lift_link",
        "upperarm_roll_link"];

joint_names = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint",
        "upperarm_roll_joint", 
        "elbow_flex_joint", 
        "forearm_roll_joint", 
        "wrist_flex_joint", 
        "wrist_roll_joint"]

rarm_jids = rtree.get_joint_ids(joint_names)
elinks = rtree.get_link_ids(elink_names)
av = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1] 
av_seq = [av] * 10
N_itr = 100

# bench tinyfk
ts = time.time()
for i in range(N_itr):
    P, J = rtree.solve_forward_kinematics(av_seq, elinks, rarm_jids, False, False, True)
print("tinyfk jac computation : {0} sec".format(time.time() - ts))

robot_model = skrobot.models.Fetch()
joint_table = {j.name : j for j in robot_model.joint_list}
link_table = {l.name : l for l in robot_model.link_list}
joint_list = [joint_table[name] for name in joint_names]
link_list = [j.child_link for j in joint_list]

# bench skrobot
def set_joint_angles(av):
    return [j.joint_angle(a) for j, a in zip(joint_list, av)]

endcoords_list = [skrobot.coordinates.CascadedCoords(parent=link_table[name]) 
        for name in elink_names]
world_coordinate = CascadedCoords()

import time
ts = time.time()
for i in range(N_itr):
    for av in av_seq:
        set_joint_angles(av)
        for ec in endcoords_list:
            J_sk = robot_model.calc_jacobian_from_link_list(
                    ec,
                    link_list,
                    transform_coords=world_coordinate,
                    rotation_axis=False)
print("skrobot jac computation : {0} sec".format(time.time() - ts))
