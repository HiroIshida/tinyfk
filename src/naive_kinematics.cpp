/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

// inefficient methods which will be used only in test

#include "tinyfk.hpp"

namespace tinyfk {

void NaiveRobotModel::get_link_pose(size_t link_id,
                                    urdf::Pose &out_tf_rlink_to_elink,
                                    bool with_base) const {
  // h : here , e: endeffector , r: root, p: parent
  // e.g. hlink means here_link and rlink means root_link

  urdf::LinkSharedPtr hlink = links_[link_id];
  urdf::Pose tf_hlink_to_elink; // unit transform by default

  while (true) {
    // transform from parent to child links are computed by combining
    // three transforms: tf_here_to_joint, tf_joint_to_joint,
    // tf_joint_to_parent, in order.

    const urdf::JointSharedPtr &pjoint = hlink->parent_joint;
    if (pjoint == nullptr) {
      if (with_base) {
        tf_hlink_to_elink = pose_transform(base_pose_.pose_, tf_hlink_to_elink);
      }
      break;
    }

    urdf::Pose tf_plink_to_hlink;
    const urdf::Pose &tf_plink_to_pjoint =
        pjoint->parent_to_joint_origin_transform;

    if (pjoint->type == urdf::Joint::FIXED) {
      tf_plink_to_hlink = tf_plink_to_pjoint;
    } else {
      double angle = joint_angles_[pjoint->id];
      urdf::Pose tf_pjoint_to_hlink = pjoint->transform(angle);
      tf_plink_to_hlink =
          pose_transform(tf_plink_to_pjoint, tf_pjoint_to_hlink);
    }
    urdf::Pose tf_plink_to_elink =
        pose_transform(tf_plink_to_hlink, tf_hlink_to_elink);

    // update here node
    tf_hlink_to_elink = std::move(tf_plink_to_elink);
    hlink = hlink->getParent();
  }
  out_tf_rlink_to_elink = tf_hlink_to_elink;
}

Eigen::MatrixXd
NaiveRobotModel::get_jacobian(size_t elink_id,
                              const std::vector<size_t> &joint_ids,
                              bool with_rpy, bool with_base) {
  size_t n_pose_dim = (with_rpy ? 6 : 3);
  size_t n_joints = joint_ids.size();
  size_t n_dof = (with_base ? n_joints + 3 : n_joints);
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n_pose_dim, n_dof);

  double dx = 1e-7;
  std::vector<double> q0 = this->get_joint_angles(joint_ids);
  urdf::Pose pose0, pose1;
  this->get_link_pose(elink_id, pose0, with_base);
  for (size_t i = 0; i < n_joints; i++) {
    int jid = joint_ids[i];

    this->set_joint_angle(jid, q0[i] + dx);
    this->get_link_pose(elink_id, pose1, with_base);
    this->set_joint_angle(jid, q0[i]); // must to set to the original

    urdf::Vector3 &pos0 = pose0.position;
    urdf::Vector3 &pos1 = pose1.position;

    J(0, i) = (pos1.x - pos0.x) / dx;
    J(1, i) = (pos1.y - pos0.y) / dx;
    J(2, i) = (pos1.z - pos0.z) / dx;
    if (with_rpy) {
      urdf::Vector3 &&rpy0 = pose0.rotation.getRPY();
      urdf::Vector3 &&rpy1 = pose1.rotation.getRPY();
      urdf::Vector3 rpy_diff = rpy1 - rpy0;
      J(3, i) = rpy_diff.x / dx;
      J(4, i) = rpy_diff.y / dx;
      J(5, i) = rpy_diff.z / dx;
    }
  }

  if (with_base) {
    for (size_t i = 0; i < 3; i++) {
      std::array<double, 3> &tmp = base_pose_.pose3d_;
      tmp[i] += dx;
      this->set_base_pose(tmp[0], tmp[1], tmp[2]);
      this->get_link_pose(elink_id, pose1, true);
      tmp[i] -= dx;
      this->set_base_pose(tmp[0], tmp[1], tmp[2]);

      urdf::Vector3 &pos0 = pose0.position;
      urdf::Vector3 &pos1 = pose1.position;
      J(0, n_joints + i) = (pos1.x - pos0.x) / dx;
      J(1, n_joints + i) = (pos1.y - pos0.y) / dx;
      J(2, n_joints + i) = (pos1.z - pos0.z) / dx;
      if (with_rpy) {
        urdf::Vector3 &&rpy0 = pose0.rotation.getRPY();
        urdf::Vector3 &&rpy1 = pose1.rotation.getRPY();
        urdf::Vector3 rpy_diff = rpy1 - rpy0;
        J(3, n_joints + i) = rpy_diff.x / dx;
        J(4, n_joints + i) = rpy_diff.y / dx;
        J(5, n_joints + i) = rpy_diff.z / dx;
      }
    }
  }
  return J;
}

} // namespace tinyfk
