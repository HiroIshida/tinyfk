/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include "tinyfk.hpp"

void RobotModel::get_link_point_withcache(
    unsigned int link_id, urdf::Pose& out_tf_rlink_to_elink,
    bool usebase
    ) const
{
  // the first part basically compute transforms between adjacent link pair
  // starting from the specifid endeffector link. The copmuted tfs are stored into 
  // _nasty_stack.hid_stack and _nasty_stack.tf_stack
  // maybe bit complicated because I use _tf_caceh
  urdf::LinkSharedPtr hlink = _links[link_id];
  urdf::Pose tf_hlink_to_elink; 

  // tf rlink_to_blink is set to a unit transform or _base_pose according to if usebase is enabled.
  // If a cached transform from root link to here link, then tf_rlink_to_blink is overwrite to the 
  // cached value.
  urdf::Pose tf_rlink_to_blink; 
  if(usebase){tf_rlink_to_blink = _base_pose._pose;}

  int counter = -1;
  while(true){

    urdf::LinkSharedPtr plink = hlink->getParent();
    if(plink == nullptr){break;} // hit the root link

    urdf::Pose* tf_rlink_to_blink_ptr = _tf_cache.get_cache(hlink->id); 
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
        double angle = _joint_angles[pjoint->id];
        urdf::Pose tf_pjoint_to_hlink = pjoint->transform(angle);
        tf_plink_to_hlink = pose_transform(tf_plink_to_pjoint, tf_pjoint_to_hlink);
      }
    }

    // update
    counter++; //counter must be here
    _nasty_stack._hid_stack[counter] = hlink->id;
    _nasty_stack._tf_stack[counter] = std::move(tf_plink_to_hlink);
    hlink = std::move(plink);
  }

  // the second part then, compute tf_root_to_here bt iteration 
  // note that counter inclimented in the first is directry used here
  urdf::Pose tf_rlink_to_plink = std::move(tf_rlink_to_blink);
  while(counter >= 0){
    int hid = _nasty_stack._hid_stack[counter];
    urdf::Pose& tf_plink_to_hlink = _nasty_stack._tf_stack[counter];
    urdf::Pose tf_rlink_to_hlink = pose_transform(tf_rlink_to_plink, tf_plink_to_hlink);

    _tf_cache.set_cache(hid, tf_rlink_to_hlink);
    tf_rlink_to_plink = std::move(tf_rlink_to_hlink);
    counter--;
    if (counter == -1){break;}
  }
  out_tf_rlink_to_elink = std::move(tf_rlink_to_plink);
}

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
    bool rotalso, bool basealso) const
{
  unsigned int n_pose_dim = (rotalso ? 6 : 3);
  unsigned int n_dof = (basealso ? joint_ids.size() + 3 : joint_ids.size());
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(n_pose_dim*elink_ids.size(), n_dof);
  Eigen::MatrixXd elink_points = Eigen::MatrixXd::Zero(n_pose_dim*elink_ids.size(), n_pose_dim);

  // reserve
  urdf::Vector3 dpos; // d : diff 
  urdf::Vector3 drot; // d : diff 
  urdf::Pose tf_rlink_to_elink;

  for(unsigned int j=0; j<elink_ids.size(); j++){
    int elink_id = elink_ids[j];
    this->get_link_point_withcache(elink_id, tf_rlink_to_elink, basealso); 
    urdf::Vector3& epos = tf_rlink_to_elink.position;
    urdf::Rotation& erot = tf_rlink_to_elink.rotation;

    for(unsigned int i=0; i<joint_ids.size(); i++){
      int jid = joint_ids[i];
      if(!_abtable.isAncestorLink(jid, elink_id)){
        const urdf::JointSharedPtr& hjoint = _joints[jid];
        unsigned int type = hjoint->type;
        if(type == urdf::Joint::FIXED){
            throw std::invalid_argument("fixed type is not accepted");
        }
        urdf::LinkSharedPtr clink = hjoint->getChildLink(); // rotation of clink and hlink is same. so clink is ok.

        urdf::Pose tf_rlink_to_clink;
        this->get_link_point_withcache(clink->id, tf_rlink_to_clink, basealso);


        urdf::Rotation& crot = tf_rlink_to_clink.rotation;
        urdf::Vector3&& world_axis = crot * hjoint->axis; // axis w.r.t root link
        if(type == urdf::Joint::PRISMATIC){
          dpos = world_axis;
        }else{//revolute or continuous
          urdf::Vector3& cpos = tf_rlink_to_clink.position;
          urdf::Vector3 vec_clink_to_elink = {epos.x - cpos.x, epos.y - cpos.y, epos.z - cpos.z};
          cross_product(world_axis, vec_clink_to_elink, dpos);
        }
        J(n_pose_dim*j+0, i) = dpos.x;
        J(n_pose_dim*j+1, i) = dpos.y;
        J(n_pose_dim*j+2, i) = dpos.z;
        elink_points(j, 0) = epos.x; elink_points(j, 1) = epos.y; elink_points(j, 2) = epos.z;

        if(rotalso){ // (compute rpy jacobian)
          if(type == urdf::Joint::PRISMATIC){
              for(int i=3; i<=6; i++){
                  J(n_pose_dim*j + i) = 0.0;
              }
          }else{
              J(n_pose_dim*j+3, i) = world_axis.x;
              J(n_pose_dim*j+4, i) = world_axis.y;
              J(n_pose_dim*j+5, i) = world_axis.z;
          }
          urdf::Vector3 erpy = erot.getRPY();
          elink_points(j, 3) = erpy.x; elink_points(j, 4) = erpy.y; elink_points(j, 5) = erpy.z;
        }
      }
      if(basealso){
        // NOTE that epos is wrt global not wrt root link!
        // so we first compute epos w.r.t root link then take a 
        // cross product of [0, 0, 1] and local = {-local.y, local.x, 0}
        const std::array<double, 3>& basepose3d = _base_pose._pose3d;
        urdf::Vector3 epos_local = epos - urdf::Vector3(basepose3d[0], basepose3d[1], 0);

        J(n_pose_dim*j+0, joint_ids.size() + 0) = 1.0; // dx/dx
        J(n_pose_dim*j+0, joint_ids.size() + 2) = -epos_local.y; // dx/dtheta
        J(n_pose_dim*j+1, joint_ids.size() + 1) = 1.0; // dy/dy
        J(n_pose_dim*j+1, joint_ids.size() + 2) = epos_local.x; //dy/dtheta
        if(rotalso){
          J(n_pose_dim*j+5, joint_ids.size() + 2) = 1.0; // world_axis = [0, 0, 1]
        }
      }
    }
  }
  std::array<Eigen::MatrixXd, 2> ret = {J, elink_points};
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
