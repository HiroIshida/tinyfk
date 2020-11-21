#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include "core.hpp"

using namespace std;

bool isNear(double x, double y){
  return (abs(x - y) < 1e-5);
}


int main(){
  // loading test data
  ifstream test_data("../test/test_data.json");
  nlohmann::json js;
  test_data >> js;
  vector<double> angle_vector = js["angle_vector"];
  vector<string> joint_names = js["joint_names"];
  vector<string> link_names = js["link_names"];
  vector<vector<double>> pose_list = js["pose_list"];

  // test main
  std::string urdf_file = "../data/fetch_description/fetch.urdf";
  auto robot = RobotModel(urdf_file);
  auto joint_ids = robot.get_joint_ids(joint_names);
  auto link_ids = robot.get_link_ids(link_names);

  for(int i=0; i<angle_vector.size(); i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  bool base_also = true;
  urdf::Pose pose;
  for(int i=0; i<link_ids.size(); i++){
    int link_id = link_ids[i];
    robot.get_link_point_withcache(link_id, pose, base_also);
    if(
        !isNear(pose.position.x, pose_list[i][0]) || 
        !isNear(pose.position.y, pose_list[i][1]) || 
        !isNear(pose.position.z, pose_list[i][2])  
      ){
      std::cout << "poition of " << link_names[i] << "does not match"<< std::endl; 
    }

    auto rpy = pose.rotation.getRPY();
    if( // z -> y -> x ... definition of rpy is different? 
        !isNear(rpy.z, pose_list[i][3]) || 
        !isNear(rpy.y, pose_list[i][4]) || 
        !isNear(rpy.x, pose_list[i][5])
      ){

      std::cout << rpy.x << " " << rpy.y << " " << rpy.z << std::endl; 
      std::cout << pose_list[i][3] << " " << pose_list[i][4] << " " << pose_list[i][5] << std::endl; 
      std::cout << "rpy of " << link_names[i] << "does not match"<< std::endl; 
    }
  }
  std::cout << "[PASS] get_link_point_withcache" << std::endl; 
}
