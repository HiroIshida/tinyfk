/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include "tinyfk.hpp"
#include "urdf_model/pose.h"
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace tinyfk {

KinematicModel::KinematicModel(const std::string &xml_string) {
  if (xml_string.empty()) {
    throw std::runtime_error("xml string is empty");
  }
  urdf::ModelInterfaceSharedPtr robot_urdf_interface =
      urdf::parseURDF(xml_string);

  // numbering link id
  std::vector<urdf::LinkSharedPtr> links;
  std::unordered_map<std::string, int> link_ids;
  int lid = 0;
  for (const auto &map_pair : robot_urdf_interface->links_) {
    std::string name = map_pair.first;
    urdf::LinkSharedPtr link = map_pair.second;
    link_ids[name] = lid;
    link->id = lid;
    links.push_back(link);
    lid++;
  }
  size_t N_link = lid; // starting from 0 and finally ++ increment, so it'S ok

  // construct joints and joint_ids, and numbering joint id
  std::vector<urdf::JointSharedPtr> joints;
  std::unordered_map<std::string, int> joint_ids;
  int jid = 0;
  for (auto &map_pair : robot_urdf_interface->joints_) {
    std::string jname = map_pair.first;
    urdf::JointSharedPtr joint = map_pair.second;
    size_t jtype = joint->type;

    if (jtype == urdf::Joint::REVOLUTE || jtype == urdf::Joint::CONTINUOUS ||
        jtype == urdf::Joint::PRISMATIC) {
      joints.push_back(joint);
      joint_ids[jname] = jid;
      joint->id = jid;
      jid++;
    } else if (jtype == urdf::Joint::FIXED) {
    } else {
      throw std::invalid_argument("unsuported joint type is detected");
    }
  }

  // set joint->_child_link.
  for (urdf::JointSharedPtr joint : joints) {
    std::string clink_name = joint->child_link_name;
    int clink_id = link_ids[clink_name];
    urdf::LinkSharedPtr clink = links[clink_id];
    joint->setChildLink(clink);
  }

  int num_dof = joint_ids.size();
  std::vector<double> joint_angles(num_dof, 0.0);

  transform_stack_ = SizedStack<LinkIdAndTransform>(N_link);
  transform_cache_ = SizedCache<Eigen::Affine3d>(N_link);
  root_link_id_ = link_ids[robot_urdf_interface->root_link_->name];
  links_ = links;
  link_ids_ = link_ids;
  joints_ = joints;
  joint_ids_ = joint_ids;
  num_dof_ = num_dof;
  joint_angles_ = joint_angles;

  // add COM of each link as new link
  {
    // NOTE: due to my bad design (add_new_link update internal state)
    // this procedure must come after initialization of member variables
    std::vector<urdf::LinkSharedPtr> com_dummy_links;
    for (const auto &link : links) {
      if (link->inertial == nullptr) {
        continue;
      }
      const auto com_dummy_link_name = link->name + "_com";
      Transform new_link_pose;
      // new_link_pose.position.x = link->inertial->origin.position.x;
      // new_link_pose.position.y = link->inertial->origin.position.y;
      // new_link_pose.position.z = link->inertial->origin.position.z;
      // FIXME: remove this conversion
      Eigen::Vector3d origin_position = link->inertial->origin.translation();
      new_link_pose.position.x = origin_position(0);
      new_link_pose.position.y = origin_position(1);
      new_link_pose.position.z = origin_position(2);
      const auto new_link =
          this->add_new_link(com_dummy_link_name, link->id, new_link_pose);
      // set new link's inertial as the same as the parent
      // except its origin is zero
      new_link->inertial = link->inertial;
      new_link->inertial->origin = Eigen::Affine3d::Identity();
      com_dummy_links.push_back(new_link);
    }
    this->com_dummy_links_ = com_dummy_links;
  }

  this->set_base_pose(Transform()); // initial base pose
}

void KinematicModel::set_joint_angles(const std::vector<size_t> &joint_ids,
                                      const std::vector<double> &joint_angles) {
  this->_set_joint_angles(joint_ids, joint_angles);
  transform_cache_.clear();
}

void KinematicModel::_set_joint_angles(
    const std::vector<size_t> &joint_ids,
    const std::vector<double> &joint_angles) {
  for (size_t i = 0; i < joint_ids.size(); i++) {
    joint_angles_[joint_ids[i]] = joint_angles[i];
  }
}

// FIXME: remove conversion
void KinematicModel::_set_base_pose(Transform pose) {
  this->base_pose_ = urdf_pose_to_eigen_affine3d(pose);
}

void KinematicModel::clear_cache() { transform_cache_.clear(); }

void KinematicModel::set_init_angles() {
  std::vector<double> joint_angles(num_dof_, 0.0);
  joint_angles_ = joint_angles;
  transform_cache_.clear();
}

std::vector<double>
KinematicModel::get_joint_angles(const std::vector<size_t> &joint_ids) const {
  std::vector<double> angles(joint_ids.size());
  for (size_t i = 0; i < joint_ids.size(); i++) {
    int idx = joint_ids[i];
    angles[i] = joint_angles_[idx];
  }
  return angles;
}

std::vector<size_t>
KinematicModel::get_joint_ids(std::vector<std::string> joint_names) const {
  int n_joint = joint_names.size();
  std::vector<size_t> joint_ids(n_joint);
  for (int i = 0; i < n_joint; i++) {
    auto iter = joint_ids_.find(joint_names[i]);
    if (iter == joint_ids_.end()) {
      throw std::invalid_argument("no joint named " + joint_names[i]);
    }
    joint_ids[i] = iter->second;
  }
  return joint_ids;
}

std::vector<AngleLimit>
KinematicModel::get_joint_limits(const std::vector<size_t> &joint_ids) const {
  const size_t n_joint = joint_ids.size();
  std::vector<AngleLimit> limits(n_joint, AngleLimit());
  for (size_t i = 0; i < n_joint; i++) {

    const auto &joint = joints_[joint_ids[i]];
    auto &limit = limits[i];
    limit.first = joint->limits->lower;
    limit.second = joint->limits->upper;
  }
  return limits;
}

std::vector<size_t>
KinematicModel::get_link_ids(std::vector<std::string> link_names) const {
  int n_link = link_names.size();
  std::vector<size_t> link_ids(n_link);
  for (int i = 0; i < n_link; i++) {
    auto iter = link_ids_.find(link_names[i]);
    if (iter == link_ids_.end()) {
      throw std::invalid_argument("no link named " + link_names[i]);
    }
    link_ids[i] = iter->second;
  }
  return link_ids;
}

urdf::LinkSharedPtr
KinematicModel::add_new_link(const std::string &link_name, size_t parent_id,
                             const std::array<double, 3> &position,
                             const std::array<double, 3> &rpy) {
  Transform pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.rotation.setFromRPY(rpy[0], rpy[1], rpy[2]);
  return this->add_new_link(link_name, parent_id, pose);
}

urdf::LinkSharedPtr KinematicModel::add_new_link(const std::string &link_name,
                                                 size_t parent_id,
                                                 const Transform &pose) {
  bool link_name_exists = (link_ids_.find(link_name) != link_ids_.end());
  if (link_name_exists) {
    std::string message = "link name " + link_name + " already exists";
    throw std::invalid_argument("link name : " + link_name + " already exists");
  }

  auto fixed_joint = std::make_shared<urdf::Joint>();

  fixed_joint->parent_to_joint_origin_transform =
      urdf_pose_to_eigen_affine3d(pose);
  fixed_joint->type = urdf::Joint::FIXED;

  int link_id = links_.size();
  auto new_link = std::make_shared<urdf::Link>();
  new_link->parent_joint = fixed_joint;
  new_link->setParent(links_[parent_id]);
  new_link->name = link_name;
  new_link->id = link_id;

  link_ids_[link_name] = link_id;
  links_.push_back(new_link);
  links_[parent_id]->child_links.push_back(new_link);
  transform_cache_.extend();

  this->update_rptable(); // set _rptable

  return new_link;
}

void KinematicModel::update_rptable() {
  // this function usually must come in the end of a function

  // we must recreate from scratch
  int n_link = link_ids_.size();
  int n_dof = joint_ids_.size();
  auto rptable = RelevancePredicateTable(n_link, n_dof);

  for (urdf::JointSharedPtr joint : joints_) {
    int joint_id = joint_ids_.at(joint->name);
    urdf::LinkSharedPtr clink = joint->getChildLink();
    std::stack<urdf::LinkSharedPtr> link_stack;
    link_stack.push(clink);
    while (!link_stack.empty()) {
      auto here_link = link_stack.top();
      link_stack.pop();
      rptable.table_[here_link->id][joint_id] = true;
      for (auto &link : here_link->child_links) {
        link_stack.push(link);
      }
    }
  }
  rptable_ = rptable;
}

std::string load_urdf(const std::string &urdf_path) {
  std::string xml_string;
  std::fstream xml_file(urdf_path, std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  return xml_string;
}

Eigen::Affine3d urdf_pose_to_eigen_affine3d(const urdf::Pose &pose) {
  Eigen::Affine3d affine;
  Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                              pose.rotation.z);
  affine = Eigen::Translation3d(position) * rotation;
  return affine;
}

urdf::Pose eigen_affine3d_to_urdf_pose(const Eigen::Affine3d &affine) {
  urdf::Pose pose;
  pose.position.x = affine.translation().x();
  pose.position.y = affine.translation().y();
  pose.position.z = affine.translation().z();
  Eigen::Quaterniond rotation(affine.rotation());
  pose.rotation.w = rotation.w();
  pose.rotation.x = rotation.x();
  pose.rotation.y = rotation.y();
  pose.rotation.z = rotation.z();
  return pose;
}

}; // end namespace tinyfk
