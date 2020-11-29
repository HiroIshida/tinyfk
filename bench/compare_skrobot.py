#!/usr/bin/env python
import skrobot
from skrobot.coordinates import CascadedCoords, Coordinates
import _tinyfk as tinyfk
import numpy as np
import time

# common setting
urdf_path = "../data/fetch_description/fetch.urdf"
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
elink_names.extend(["l_gripper_finger_link"]*100)

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
av = [0.0]*7
av_seq = [av] * 100000
N_itr = 1

ts = time.time()
rtree._solve_forward_kinematics(av_seq, elinks, rarm_jids, False, False, False)
print("tinyfk jac computation : {0} sec".format(time.time() - ts))
