#include "c_api.h"
#include "tinyfk.hpp"
#include <string>

void* capi_create_robot_model(char* urdf_file_){
  std::string urdf_file(urdf_file_);
  auto robot_model = static_cast<void*>(new RobotModel(urdf_file));
  return robot_model;
}

unsigned int capi_get_link_id(void* robot_model_, char* link_name){
  auto robot_model = static_cast<RobotModel*>(robot_model_);
  return robot_model->get_link_id(std::string(link_name));
}

unsigned int capi_get_joint_id(void* robot_model_, char* joint_name){
  auto robot_model = static_cast<RobotModel*>(robot_model_);
  return robot_model->get_joint_id(std::string(joint_name));
}

void hello(){
  std::cout << "hello world" << std::endl; 
}
