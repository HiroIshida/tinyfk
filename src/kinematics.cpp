/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include "tinyfk.hpp"
#include <cmath>

urdf::Vector3 rpy_derivative(const urdf::Vector3& rpy, const urdf::Vector3& axis)
{
  urdf::Vector3 drpy_dt;
  double a1 = -rpy.x;
  double a2 = -rpy.y;
  double a3 = -rpy.z;
  drpy_dt.x = cos(a3)/cos(a2)*axis.x - sin(a3)/cos(a2)*axis.y;
  drpy_dt.y = sin(a3)*axis.x + cos(a3)*axis.y;
  drpy_dt.z = -cos(a3)*sin(a2)/cos(a2)*axis.x + sin(a3)*sin(a2)/cos(a2)*axis.y + axis.z;
  return drpy_dt;
}

namespace tinyfk
{

  void RobotModel::get_link_point_withcache(
      size_t link_id, urdf::Pose& out_tf_rlink_to_elink,
      bool usebase
      ) const
  {
    // one may consider this procedure is redundant becaues 
    // _tf_cache.get_cache() in _get_link_point_creating_cache does
    // the same thing. However, because we cannot avoid additional procedures 
    // in _get_link_point_creating_cache like while-loop and comparison of 
    // counter etc, directly calling here and return immediately leads to 
    // much efficiency.
    urdf::Pose* pose_ptr = tf_cache_.get_cache(link_id);
    if(pose_ptr){
      out_tf_rlink_to_elink = *pose_ptr;
      return;
    }
    // If cache does not found, get link pose creating cache 
    this->_get_link_point_creating_cache(link_id, out_tf_rlink_to_elink, usebase);
  }

  void RobotModel::_get_link_point_creating_cache(
      size_t link_id, urdf::Pose& out_tf_rlink_to_elink,
      bool usebase
      ) const
  {
    // the first part basically compute transforms between adjacent link pair
    // starting from the specifid endeffector link. The copmuted tfs are stored into 
    // _nasty_stack.hid_stack and _nasty_stack.tf_stack
    // maybe bit complicated because I use _tf_caceh
    urdf::LinkSharedPtr hlink = links_[link_id];

    // tf rlink_to_blink is set to a unit transform or _base_pose according to if usebase is enabled.
    // If a cached transform from root link to here link, then tf_rlink_to_blink is overwrite to the 
    // cached value.
    urdf::Pose tf_rlink_to_blink; 
    if(usebase){tf_rlink_to_blink = base_pose_.pose_;}

    int counter = -1;
    while(true){

      urdf::LinkSharedPtr plink = hlink->getParent();
      if(plink == nullptr){break;} // hit the root link

      urdf::Pose* tf_rlink_to_blink_ptr = tf_cache_.get_cache(hlink->id); 
      if(tf_rlink_to_blink_ptr){
        tf_rlink_to_blink = *tf_rlink_to_blink_ptr;
        break;
      }

      urdf::Pose tf_plink_to_hlink;
      {// compute tf_plink_to_hlink
        const urdf::JointSharedPtr& pjoint = hlink->parent_joint;
        const urdf::Pose& tf_plink_to_pjoint = pjoint->parent_to_joint_origin_transform;

        if(pjoint->type==urdf::Joint::FIXED){
          tf_plink_to_hlink = tf_plink_to_pjoint;
        }else{
          double angle = joint_angles_[pjoint->id];
          urdf::Pose tf_pjoint_to_hlink = pjoint->transform(angle);
          tf_plink_to_hlink = pose_transform(tf_plink_to_pjoint, tf_pjoint_to_hlink);
        }
      }

      // update
      counter++; //counter must be here
      nasty_stack_.hid_stack_[counter] = hlink->id;
      nasty_stack_.tf_stack_[counter] = std::move(tf_plink_to_hlink);
      hlink = std::move(plink);
    }

    // the second part then, compute tf_root_to_here bt iteration 
    // note that counter inclimented in the first is directry used here
    urdf::Pose tf_rlink_to_plink = std::move(tf_rlink_to_blink);
    while(counter >= 0){
      int hid = nasty_stack_.hid_stack_[counter];
      urdf::Pose& tf_plink_to_hlink = nasty_stack_.tf_stack_[counter];
      urdf::Pose tf_rlink_to_hlink = pose_transform(tf_rlink_to_plink, tf_plink_to_hlink);

      tf_cache_.set_cache(hid, tf_rlink_to_hlink);
      tf_rlink_to_plink = std::move(tf_rlink_to_hlink);
      counter--;
      if (counter == -1){break;}
    }
    out_tf_rlink_to_elink = std::move(tf_rlink_to_plink);
  }

  void get_base_jacobian(const urdf::Vector3& epos_local, TinyMatrix& base_jacobian, bool with_rot){
    /*
     * [1, 0,-y]
     * [0, 1, x]
     * [0, 0, 0]
     *
     * with_rot
     * [0, 0, 0]
     * [0, 0, 0]
     * [0, 0, 1]
     */
    int dim_pose = (with_rot ? 6 : 3);
    base_jacobian(0, 0) = 1.0;
    base_jacobian(1, 1) = 1.0;
    base_jacobian(0, 2) = -epos_local.y;
    base_jacobian(1, 2) = epos_local.x;
    if(with_rot){
      base_jacobian(5, 2) = 1.0;
      // NOTE : thanks to the definition of rpy, if rotation axis is [0, 0, 1]
      // then drpy/dt = [0, 0, 1], so we don't have to multiply kinematic matrix here.
    }
  }

  std::array<Eigen::MatrixXd, 2> RobotModel::get_jacobians_withcache(
      const std::vector<size_t>& elink_ids,
      const std::vector<size_t>& joint_ids, 
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

  void RobotModel::_solve_batch_forward_kinematics(
      std::vector<size_t> elink_ids, const std::vector<size_t>& joint_ids,
      bool with_rot, bool with_base, TinyMatrix& pose_arr, TinyMatrix& jacobian_arr) const
  {
    int dim_jacobi = (with_rot ? 6 : 3);
    int dim_pose = (with_rot ? 6 : 3);
    int dim_dof = joint_ids.size() + (with_base ? 3 : 0);

    for(int i=0; i< elink_ids.size(); i++){
      int elink_id = elink_ids[i];
      TinyMatrix pose = pose_arr.slice(i);
      TinyMatrix jacobian = jacobian_arr.block(dim_jacobi*i, 0, dim_jacobi, dim_dof);
      _solve_forward_kinematics(elink_id, joint_ids, with_rot, with_base, pose, jacobian);
    }
  }

  // lower level jacobian function, which directly iterate over poitner
  void RobotModel::_solve_forward_kinematics(
      int elink_id, const std::vector<size_t>& joint_ids,
      bool with_rot, bool with_base, TinyMatrix& pose, TinyMatrix& jacobian) const
  {

    // Forward kinematics computation 
    // tf_rlink_to_elink and epos, erot, erpy will be also used in jacobian computation
    urdf::Pose tf_rlink_to_elink;
    this->get_link_point_withcache(elink_id, tf_rlink_to_elink, with_base); 
    urdf::Vector3& epos = tf_rlink_to_elink.position;
    urdf::Rotation& erot = tf_rlink_to_elink.rotation;
    urdf::Vector3 erpy; // will be used only with_rot
    if(with_rot){
      erpy = erot.getRPY();
    }
    pose[0] = epos.x; pose[1] = epos.y; pose[2] = epos.z;
    if(with_rot){
      pose[3] = erpy.x; pose[4] = erpy.y; pose[5] = erpy.z;
    }

    // Jacobian computation
    int dim_jacobi = (with_rot ? 6 : 3);
    for(int i=0; i<joint_ids.size(); i++){
      int jid = joint_ids[i];
      if(rptable_.isRelevant(jid, elink_id)){
        const urdf::JointSharedPtr& hjoint = joints_[jid];
        size_t type = hjoint->type;
        if(type == urdf::Joint::FIXED){
           assert(type!=urdf::Joint::FIXED && "fixed type is not accepted");
        }
        urdf::LinkSharedPtr clink = hjoint->getChildLink(); // rotation of clink and hlink is same. so clink is ok.

        urdf::Pose tf_rlink_to_clink;
        this->get_link_point_withcache(clink->id, tf_rlink_to_clink, with_base);

        urdf::Rotation& crot = tf_rlink_to_clink.rotation;
        urdf::Vector3&& world_axis = crot * hjoint->axis; // axis w.r.t root link
        urdf::Vector3 dpos;
        if(type == urdf::Joint::PRISMATIC){
          dpos = world_axis;
        }else{//revolute or continuous
          urdf::Vector3& cpos = tf_rlink_to_clink.position;
          urdf::Vector3 vec_clink_to_elink = {epos.x - cpos.x, epos.y - cpos.y, epos.z - cpos.z};
          cross_product(world_axis, vec_clink_to_elink, dpos);
        }
        jacobian(0, i) = dpos.x;
        jacobian(1, i) = dpos.y;
        jacobian(2, i) = dpos.z;
        if(with_rot){ // (compute rpy jacobian)
          if(type == urdf::Joint::PRISMATIC){
            // jacobian for rotation is all zero
          }else{
            urdf::Vector3 drpy_dt = rpy_derivative(erpy, world_axis);
            jacobian(3, i) = drpy_dt.x;
            jacobian(4, i) = drpy_dt.y;
            jacobian(5, i) = drpy_dt.z;
          }
        }
      }
    }

    if(with_base){
      // NOTE that epos is wrt global not wrt root link!
      // so we first compute epos w.r.t root link then take a 
      // cross product of [0, 0, 1] and local = {-local.y, local.x, 0}
      const std::array<double, 3>& basepose3d = base_pose_.pose3d_;
      urdf::Vector3 epos_local = epos - urdf::Vector3(basepose3d[0], basepose3d[1], 0);
      int dim_dof = joint_ids.size();
      TinyMatrix base_jacobian = jacobian.block(0, dim_dof, dim_jacobi, 3);
      get_base_jacobian(epos_local, base_jacobian, with_rot);
    }
  }


}; //end namepsace tinyfk
