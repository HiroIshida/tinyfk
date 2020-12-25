/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

// inefficient methods which will be used only in test

#include "tinyfk.hpp"

namespace tinyfk
{

  void RobotModel::get_link_point(
      unsigned int link_id, urdf::Pose& out_tf_rlink_to_elink, bool basealso) const
  {
    // h : here , e: endeffector , r: root, p: parent
    // e.g. hlink means here_link and rlink means root_link
    
    urdf::LinkSharedPtr hlink = _links[link_id];
    urdf::Pose tf_hlink_to_elink; // unit transform by default

    while(true){
      // transform from parent to child links are computed by combining 
      // three transforms: tf_here_to_joint, tf_joint_to_joint, tf_joint_to_parent, in order.

      const urdf::JointSharedPtr& pjoint = hlink->parent_joint;
      if(pjoint == nullptr){
        if(basealso){tf_hlink_to_elink = pose_transform(_base_pose._pose, tf_hlink_to_elink);}
          break;
      }

      urdf::Pose tf_plink_to_hlink;
      const urdf::Pose& tf_plink_to_pjoint = pjoint->parent_to_joint_origin_transform;

      if(pjoint->type==urdf::Joint::FIXED){
        tf_plink_to_hlink = tf_plink_to_pjoint;
      }else{
        double angle = _joint_angles[pjoint->id];
        urdf::Pose tf_pjoint_to_hlink = pjoint->transform(angle);
        tf_plink_to_hlink = pose_transform(tf_plink_to_pjoint, tf_pjoint_to_hlink);
      }
      urdf::Pose tf_plink_to_elink = pose_transform(tf_plink_to_hlink, tf_hlink_to_elink);

      // update here node
      tf_hlink_to_elink = std::move(tf_plink_to_elink);
      hlink = hlink->getParent(); 
    }
    out_tf_rlink_to_elink = tf_hlink_to_elink;
  }

  std::array<Eigen::MatrixXd, 2> RobotModel::get_jacobians_withcache(
      const std::vector<unsigned int>& elink_ids,
      const std::vector<unsigned int>& joint_ids, 
      bool with_rot, bool with_base) const
  {
    int dim_pose = (with_rot ? 6 : 3);
    int dim_dof = joint_ids.size() + (with_base ? 3 : 0);
    int dim_feature = elink_ids.size();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_pose, elink_ids.size());
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(dim_pose * dim_feature, dim_dof);
    auto P_ = TinyMatrix(P);
    auto J_ = TinyMatrix(J);

    this->_solve_batch_forward_kinematics(elink_ids, joint_ids,
        with_rot, with_base, P_, J_);
    std::array<Eigen::MatrixXd, 2> ret = {J, P};
    return ret;
  }

  Eigen::MatrixXd RobotModel::get_jacobian_naive(
      unsigned int elink_id, const std::vector<unsigned int>& joint_ids, bool rotalso, bool basealso) 
  {
    unsigned int n_pose_dim = (rotalso ? 6 : 3);
    unsigned int n_joints = joint_ids.size();
    unsigned int n_dof = (basealso ? n_joints + 3 : n_joints);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n_pose_dim, n_dof);

    double dx = 1e-7;
    std::vector<double> q0 = this->get_joint_angles(joint_ids);
    urdf::Pose pose0, pose1;
    this->get_link_point(elink_id, pose0, basealso);
    for(unsigned int i=0; i<n_joints; i++){
      int jid = joint_ids[i];

      this->set_joint_angle(jid, q0[i] + dx);
      this->get_link_point(elink_id, pose1, basealso);
      this->set_joint_angle(jid, q0[i]); // must to set to the original

      urdf::Vector3& pos0 = pose0.position;
      urdf::Vector3& pos1 = pose1.position;

      J(0, i) = (pos1.x - pos0.x)/dx;
      J(1, i) = (pos1.y - pos0.y)/dx;
      J(2, i) = (pos1.z - pos0.z)/dx;
      if(rotalso){
        urdf::Vector3&& rpy0 = pose0.rotation.getRPY(); 
        urdf::Vector3&& rpy1 = pose1.rotation.getRPY();
        urdf::Vector3 rpy_diff = rpy1 - rpy0;
        J(3, i) = rpy_diff.x/dx;
        J(4, i) = rpy_diff.y/dx;
        J(5, i) = rpy_diff.z/dx;
      }
    }

    if(basealso){
      for(unsigned int i=0; i<3; i++){
        std::array<double, 3>& tmp = _base_pose._pose3d;
        tmp[i] += dx;
        this->set_base_pose(tmp);
        this->get_link_point(elink_id, pose1, true);
        tmp[i] -= dx;
        this->set_base_pose(tmp);

        urdf::Vector3& pos0 = pose0.position;
        urdf::Vector3& pos1 = pose1.position;
        J(0, n_joints+i) = (pos1.x - pos0.x)/dx;
        J(1, n_joints+i) = (pos1.y - pos0.y)/dx;
        J(2, n_joints+i) = (pos1.z - pos0.z)/dx;
        if(rotalso){
          urdf::Vector3&& rpy0 = pose0.rotation.getRPY(); 
          urdf::Vector3&& rpy1 = pose1.rotation.getRPY();
          urdf::Vector3 rpy_diff = rpy1 - rpy0;
          J(3, n_joints+i) = rpy_diff.x/dx;
          J(4, n_joints+i) = rpy_diff.y/dx;
          J(5, n_joints+i) = rpy_diff.z/dx;
        }
      }
    }
    return J;
  }

}
