import json
import numpy as np
import skrobot
from skrobot.coordinates import CascadedCoords, Coordinates
from skrobot.coordinates.math import rpy_matrix, rpy_angle

robot_model = skrobot.models.urdf.RobotModelFromURDF(urdf_file=skrobot.data.fetch_urdfpath())
joint_table = {j.name : j for j in robot_model.joint_list}

joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

joint_list = [joint_table[name] for name in joint_names]
link_list = [j.child_link for j in joint_list]

# set torso, arm
joint_angles = [0.2] + [-0.5]*7
for j, a in zip(joint_list, joint_angles):
    j.joint_angle(a)

# set base position 
x, y, theta = 0.3, 0.5, 0.7
co = Coordinates(pos = [x, y, 0.0], rot=rpy_matrix(theta, 0.0, 0.0))
robot_model.newcoords(co)

angle_vector = joint_angles + [x, y, theta]

# compute pose of links
world_coordinate = CascadedCoords()
extract_pose = lambda link: np.hstack((link.worldpos(), link.worldcoords().rpy_angle()[0]))
pose_list = [extract_pose(link).tolist() for link in link_list]

# compute jacobian of links
cascoords_list = [CascadedCoords(link) for link in link_list]
world_coordinate = CascadedCoords()
J = [robot_model.calc_jacobian_from_link_list(cascoords, link_list, transform_coords=world_coordinate, rotation_axis=True).tolist() for cascoords in cascoords_list]


rarm_end_coords = skrobot.coordinates.CascadedCoords(
        parent=robot_model.gripper_link, 
        name='rarm_end_coords')

ground_truth = {
        "angle_vector" : angle_vector, 
        "joint_names" : joint_names, 
        "link_names" : [l.name for l in link_list],
        "pose_list" : pose_list,
        "jacobian_list" : J
        }

with open("test_data.json", 'w') as f:
    json.dump(ground_truth, f, indent=4)
