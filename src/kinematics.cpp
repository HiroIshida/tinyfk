/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include "tinyfk.hpp"
#include "urdf_model/pose.h"
#include <Eigen/Dense>
#include <cmath>

urdf::Vector3 rpy_derivative(const urdf::Vector3 &rpy,
                             const urdf::Vector3 &axis) {
  urdf::Vector3 drpy_dt;
  double a2 = -rpy.y;
  double a3 = -rpy.z;
  drpy_dt.x = cos(a3) / cos(a2) * axis.x - sin(a3) / cos(a2) * axis.y;
  drpy_dt.y = sin(a3) * axis.x + cos(a3) * axis.y;
  drpy_dt.z = -cos(a3) * sin(a2) / cos(a2) * axis.x +
              sin(a3) * sin(a2) / cos(a2) * axis.y + axis.z;
  return drpy_dt;
}

namespace tinyfk {

void CacheUtilizedRobotModel::get_link_pose(size_t link_id,
                                            urdf::Pose &out_tf_rlink_to_elink,
                                            bool usebase) const {
  urdf::Pose const *pose_ptr = transform_cache_.get_cache(link_id);
  if (pose_ptr) {
    out_tf_rlink_to_elink = *pose_ptr;
    return;
  }
  this->get_link_pose_inner(link_id, out_tf_rlink_to_elink, usebase);
}

void CacheUtilizedRobotModel::get_link_pose_inner(
    size_t link_id, urdf::Pose &out_tf_rlink_to_elink, bool usebase) const {
  urdf::LinkSharedPtr hlink = links_[link_id];

  urdf::Pose tf_rlink_to_blink;
  if (usebase) {
    tf_rlink_to_blink = base_pose_;
  }

  transform_stack_.reset();
  while (true) {

    urdf::LinkSharedPtr plink = hlink->getParent();
    if (plink == nullptr) {
      break;
    } // hit the root link

    urdf::Pose const *tf_rlink_to_blink_ptr =
        transform_cache_.get_cache(hlink->id);
    if (tf_rlink_to_blink_ptr) {
      tf_rlink_to_blink = *tf_rlink_to_blink_ptr;
      break;
    }

    urdf::Pose tf_plink_to_hlink;
    { // compute tf_plink_to_hlink
      const urdf::JointSharedPtr &pjoint = hlink->parent_joint;
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
    }

    // update
    transform_stack_.push(LinkIdAndPose{
        hlink->id, std::move(tf_plink_to_hlink)}); // TODO(HiroIshida): move?
    hlink = plink;
  }

  urdf::Pose tf_rlink_to_plink = std::move(tf_rlink_to_blink);
  while (!transform_stack_.empty()) {

    const auto &pose_id_pair = transform_stack_.top();
    const urdf::Pose &tf_plink_to_hlink = pose_id_pair.pose;
    const size_t hid = pose_id_pair.id;
    transform_stack_.pop();
    urdf::Pose tf_rlink_to_hlink =
        pose_transform(tf_rlink_to_plink, tf_plink_to_hlink);
    transform_cache_.set_cache(hid, tf_rlink_to_hlink);
    tf_rlink_to_plink = std::move(tf_rlink_to_hlink);
  }
  out_tf_rlink_to_elink = std::move(tf_rlink_to_plink);
}

Eigen::MatrixXd
CacheUtilizedRobotModel::get_jacobian(size_t elink_id,
                                      const std::vector<size_t> &joint_ids,
                                      bool with_rot, bool with_base) {
  int dim_jacobi = (with_rot ? 6 : 3);
  int dim_dof = joint_ids.size() + (with_base ? 6 : 0);

  // compute values shared through the loop
  urdf::Pose tf_rlink_to_elink;
  this->get_link_pose(elink_id, tf_rlink_to_elink, with_base);
  urdf::Vector3 &epos = tf_rlink_to_elink.position;
  urdf::Rotation &erot = tf_rlink_to_elink.rotation;

  urdf::Vector3 erpy; // will be used only with_rot
  if (with_rot) {
    erpy = erot.getRPY();
  }

  urdf::Pose tf_rlink_to_blink, tf_blink_to_rlink, tf_blink_to_elink;
  urdf::Vector3 rpy_rlink_to_blink;
  if (with_base) {
    this->get_link_pose(this->root_link_id_, tf_rlink_to_blink,
                        true); // confusing but root_link_id -> base_link
    tf_blink_to_rlink = tf_rlink_to_blink.inverse();
    rpy_rlink_to_blink = tf_rlink_to_blink.rotation.getRPY();
    tf_blink_to_elink = pose_transform(tf_blink_to_rlink, tf_rlink_to_elink);
  }

  // Jacobian computation
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(dim_jacobi, dim_dof);

  for (size_t i = 0; i < joint_ids.size(); i++) {
    int jid = joint_ids[i];
    if (rptable_.isRelevant(jid, elink_id)) {
      const urdf::JointSharedPtr &hjoint = joints_[jid];
      size_t type = hjoint->type;
      if (type == urdf::Joint::FIXED) {
        assert(type != urdf::Joint::FIXED && "fixed type is not accepted");
      }
      urdf::LinkSharedPtr clink =
          hjoint->getChildLink(); // rotation of clink and hlink is same. so
                                  // clink is ok.

      urdf::Pose tf_rlink_to_clink;
      this->get_link_pose(clink->id, tf_rlink_to_clink, with_base);

      urdf::Rotation &crot = tf_rlink_to_clink.rotation;
      urdf::Vector3 &&world_axis = crot * hjoint->axis; // axis w.r.t root link
      urdf::Vector3 dpos;
      if (type == urdf::Joint::PRISMATIC) {
        dpos = world_axis;
      } else { // revolute or continuous
        urdf::Vector3 &cpos = tf_rlink_to_clink.position;
        urdf::Vector3 vec_clink_to_elink = {epos.x - cpos.x, epos.y - cpos.y,
                                            epos.z - cpos.z};
        cross_product(world_axis, vec_clink_to_elink, dpos);
      }
      jacobian(0, i) = dpos.x;
      jacobian(1, i) = dpos.y;
      jacobian(2, i) = dpos.z;
      if (with_rot) { // (compute rpy jacobian)
        if (type == urdf::Joint::PRISMATIC) {
          // jacobian for rotation is all zero
        } else {
          urdf::Vector3 drpy_dt = rpy_derivative(erpy, world_axis);
          jacobian(3, i) = drpy_dt.x;
          jacobian(4, i) = drpy_dt.y;
          jacobian(5, i) = drpy_dt.z;
        }
      }
    }
  }

  if (with_base) {
    const size_t dim_dof = joint_ids.size();

    jacobian(0, dim_dof + 0) = 1.0;
    jacobian(1, dim_dof + 1) = 1.0;
    jacobian(2, dim_dof + 2) = 1.0;

    // we resort to numerical method to base pose jacobian (just because I don't
    // have time)
    // TODO(HiroIshida): compute using analytical method.
    constexpr double eps = 1e-7;
    for (size_t rpy_idx = 0; rpy_idx < 3; rpy_idx++) {
      const size_t idx_col = dim_dof + 3 + rpy_idx;

      auto rpy_tweaked = rpy_rlink_to_blink;
      rpy_tweaked[rpy_idx] += eps;

      urdf::Pose tf_rlink_to_blink_tweaked = tf_rlink_to_blink;
      tf_rlink_to_blink_tweaked.rotation.setFromRPY(
          rpy_tweaked.x, rpy_tweaked.y, rpy_tweaked.z);
      urdf::Pose tf_rlink_to_elink_tweaked =
          pose_transform(tf_rlink_to_blink_tweaked, tf_blink_to_elink);
      auto pose_out = tf_rlink_to_elink_tweaked;

      const auto pos_diff = pose_out.position - tf_rlink_to_elink.position;
      jacobian(0, idx_col) = pos_diff.x / eps;
      jacobian(1, idx_col) = pos_diff.y / eps;
      jacobian(2, idx_col) = pos_diff.z / eps;
      if (with_rot) {
        auto erpy_tweaked = pose_out.rotation.getRPY();
        const auto erpy_diff = erpy_tweaked - erpy;
        jacobian(3, idx_col) = erpy_diff.x / eps;
        jacobian(4, idx_col) = erpy_diff.y / eps;
        jacobian(5, idx_col) = erpy_diff.z / eps;
      }
    }
  }
  return jacobian;
}

}; // namespace tinyfk
