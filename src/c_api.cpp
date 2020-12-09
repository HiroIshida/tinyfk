#include "c_api.h"
#include "tinyfk.hpp"
#include <string>

void* capi_create_robot_model(char* urdf_file_){
  std::string urdf_file(urdf_file_);
  auto robot_model = static_cast<void*>(new RobotModel(urdf_file));
  return robot_model;
}

void capi_clear_cache(void* robot_model_){
  auto robot_model = static_cast<RobotModel*>(robot_model_);
  robot_model->clear_cache();
}

unsigned int capi_get_link_id(void* robot_model_, char* link_name){
  auto robot_model = static_cast<RobotModel*>(robot_model_);
  return robot_model->get_link_id(std::string(link_name));
}

unsigned int capi_get_joint_id(void* robot_model_, char* joint_name){
  auto robot_model = static_cast<RobotModel*>(robot_model_);
  return robot_model->get_joint_id(std::string(joint_name));
}

void capi_set_joint_angle(void* robot_model_, unsigned int joint_id, double angle){
  auto robot_model = static_cast<RobotModel*>(robot_model_);
  robot_model->set_joint_angle(joint_id, angle);
}

void capi_get_link_point(
    void* robot_model_, unsigned int link_id, 
    bool with_rot,
    bool with_base, 
    double* pose_carr)
{

  auto robot_model = static_cast<RobotModel*>(robot_model_);

  urdf::Pose pose;
  robot_model->get_link_point_withcache(link_id, pose, with_base);
  pose_carr[0] = pose.position.x;
  pose_carr[1] = pose.position.y;
  pose_carr[2] = pose.position.z;
  if(with_rot){
    pose_carr[3] = pose.rotation.x;
    pose_carr[4] = pose.rotation.y;
    pose_carr[5] = pose.rotation.z;
    pose_carr[6] = pose.rotation.w;
  }
}

void hello(){
  std::cout << "hello world" << std::endl; 
}
