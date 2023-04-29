import json
import numpy as np
import skrobot
from skrobot.model import Link
from skrobot.coordinates import CascadedCoords, Coordinates, make_cascoords
from skrobot.coordinates.math import rpy_matrix, rpy_angle

robot_model = skrobot.models.PR2()
joint_list = [
    robot_model.r_shoulder_pan_joint, robot_model.r_shoulder_lift_joint,
    robot_model.r_upper_arm_roll_joint, robot_model.r_elbow_flex_joint,
    robot_model.r_forearm_roll_joint, robot_model.r_wrist_flex_joint,
    robot_model.r_wrist_roll_joint]
joint_names = [j.name for j in joint_list]

mylink = Link(pos=[0.1, 0.1, 0.1], rot=[0.1, 0.2, 0.3], name="mylink")
robot_model.r_upper_arm_link.assoc(mylink, mylink)
link_list = [
    robot_model.r_shoulder_pan_link, robot_model.r_shoulder_lift_link,
    robot_model.r_upper_arm_roll_link, robot_model.r_elbow_flex_link,
    robot_model.r_forearm_roll_link, robot_model.r_wrist_flex_link,
    robot_model.r_wrist_roll_link, robot_model.base_link, robot_model.r_upper_arm_link, mylink]

joint_angles = [0.564, 0.35, -0.74, -0.7, -0.7, -0.17, -0.63]
for j, a in zip(joint_list, joint_angles):
    j.joint_angle(a)

x, y, z, r, p, y = 0.3, 0.5, 0.7, 0.1, 0.2, 0.3
co = Coordinates(pos=[x, y, z], rot=rpy_matrix(y, p, r))
robot_model.newcoords(co)

angle_vector = joint_angles + [x, y, z, r, p, y]

# compute pose of links
world_coordinate = CascadedCoords()

def extract_pose(co): return np.hstack(
    (co.worldpos(), co.worldcoords().rpy_angle()[0]))


pose_list = [extract_pose(co).tolist() for co in link_list]

# compute jacobian of links
world_coordinate = CascadedCoords()
J = [robot_model.calc_jacobian_from_link_list(
    link,
    [j.child_link for j in joint_list],
    transform_coords=world_coordinate,
    rotation_axis=True).tolist() for link in link_list]

ground_truth = {
    "angle_vector": angle_vector,
    "joint_names": joint_names,
    "link_names": [l.name for l in link_list],
    "pose_list": pose_list,
}

with open("test_data.json", 'w') as f:
    json.dump(ground_truth, f)
