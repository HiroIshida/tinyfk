/*
Copyright (c) 2020 Hirokazu Ishida
This software is released under the MIT License, see LICENSE.
tinyfk: https://github.com/HiroIshida/tinyfk
*/

#include <fstream>
#include <cmath>
#include "tinyfk.hpp"

RobotModel::RobotModel(const std::string& urdf_file){
  std::string xml_string;
  std::fstream xml_file(urdf_file, std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  urdf::ModelInterfaceSharedPtr robot_urdf_interface = urdf::parseURDF(xml_string);

  // numbering link id 
  std::vector<urdf::LinkSharedPtr> links;
  std::unordered_map<std::string, int> link_ids;
  int lid = 0;
  for(const auto& map_pair: robot_urdf_interface->links_){
    std::string name = map_pair.first;
    urdf::LinkSharedPtr link = map_pair.second;
    link_ids[name] = lid;
    link->id = lid;
    links.push_back(link);
    lid++;
  }
  unsigned int N_link = lid; // starting from 0 and finally ++ increment, so it'S ok

  // construct joints and joint_ids, and numbering joint id
  std::vector<urdf::JointSharedPtr> joints;
  std::unordered_map<std::string, int> joint_ids;
  int jid = 0;
  for(auto& map_pair: robot_urdf_interface->joints_){
    std::string jname = map_pair.first;
    urdf::JointSharedPtr joint = map_pair.second;
    unsigned int jtype = joint->type;

    if(
        jtype==urdf::Joint::REVOLUTE || 
        jtype==urdf::Joint::CONTINUOUS ||
        jtype==urdf::Joint::PRISMATIC 
      ){
      joints.push_back(joint);
      joint_ids[jname] =  jid;
      joint->id = jid;
      jid ++;
    }else if(jtype==urdf::Joint::FIXED){
    }else{
      throw std::invalid_argument("unsuported joint type is detected");
    }
  }

  // set joint->_child_link. 
  for(urdf::JointSharedPtr joint : joints){
    std::string clink_name = joint->child_link_name;
    int clink_id = link_ids[clink_name];
    urdf::LinkSharedPtr clink = links[clink_id];
    joint->setChildLink(clink);
  }

  int num_dof = joint_ids.size();
  std::vector<double> joint_angles(num_dof, 0.0);

  // consturct rptable

  _urdf_file = urdf_file;
  _nasty_stack = NastyStack(N_link);
  _tf_cache = TransformCache(N_link);
  _root_link = robot_urdf_interface->root_link_;
  _links = links;
  _link_ids = link_ids;
  _joints = joints;
  _joint_ids = joint_ids;
  _num_dof = num_dof;
  _joint_angles = joint_angles;
  this->_update_rptable(); // update _rptable
}

void RobotModel::set_joint_angles(
    const std::vector<unsigned int>& joint_ids, const std::vector<double>& joint_angles){
    for(unsigned int i=0; i<joint_ids.size(); i++){
      _joint_angles[joint_ids[i]] = joint_angles[i];
    }
    _tf_cache.clear();
}

void RobotModel::set_init_angles(){
  std::vector<double> joint_angles(_num_dof, 0.0);
  _joint_angles = joint_angles;
  _tf_cache.clear();
}

std::vector<double> RobotModel::get_joint_angles(const std::vector<unsigned int>& joint_ids) const
{
  std::vector<double> angles(joint_ids.size());
  for(unsigned int i=0; i<joint_ids.size(); i++){
    int idx = joint_ids[i];
    angles[i] = _joint_angles[idx];
  }
  return angles;
}

std::vector<unsigned int> RobotModel::get_joint_ids(std::vector<std::string> joint_names) const
{
  int n_joint = joint_names.size();
  std::vector<unsigned int> joint_ids(n_joint);
  for(int i=0; i<n_joint; i++){
    auto iter = _joint_ids.find(joint_names[i]);
    if(iter==_joint_ids.end()){
      throw std::invalid_argument("no joint named " + joint_names[i]);
    }
    joint_ids[i] = iter->second;
  }
  return joint_ids;
}

std::vector<unsigned int> RobotModel::get_link_ids(std::vector<std::string> link_names) const
{
  int n_link = link_names.size();
  std::vector<unsigned int> link_ids(n_link);
  for(int i=0; i<n_link; i++){
    auto iter = _link_ids.find(link_names[i]);
    if(iter==_link_ids.end()){
      throw std::invalid_argument("no link named " + link_names[i]);
    }
    link_ids[i] = iter->second;
  }
  return link_ids;
}

