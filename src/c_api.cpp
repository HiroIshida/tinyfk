#include "c_api.h"
#include "tinyfk.hpp"
#include <string>

void* c_create_robotic_tree(char* urdf_file_){
  std::string urdf_file(urdf_file_);
  auto crtree = static_cast<void*>(new RobotModel(urdf_file));
  return crtree;
}

void hello(){
  std::cout << "hello world" << std::endl; 
}
