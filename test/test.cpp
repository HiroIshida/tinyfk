#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include "core.hpp"

using namespace std;
using namespace Eigen;

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

  for(int i=0; i<8; i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[8], angle_vector[9], angle_vector[10]);
  bool base_also = true;
  urdf::Pose pose, pose_naive;
  for(int i=0; i<link_ids.size(); i++){
    int link_id = link_ids[i];
    robot.get_link_point_withcache(link_id, pose, base_also);

    if(
        !isNear(pose.position.x, pose_list[i][0]) || 
        !isNear(pose.position.y, pose_list[i][1]) || 
        !isNear(pose.position.z, pose_list[i][2])  
      ){
      std::cout << "poition of " << link_names[i] << " does not match"<< std::endl; 
    }

    auto rpy = pose.rotation.getRPY();
    if( 
        // NOTE that rpy of URDFdom is (r, p, y) order, but (y, p, r) in skrobot
        !isNear(rpy.z, pose_list[i][3]) || 
        !isNear(rpy.y, pose_list[i][4]) || 
        !isNear(rpy.x, pose_list[i][5])
      ){
      std::cout << "rpy of " << link_names[i] << " does not match"<< std::endl; 
    }
  }
  std::cout << "[PASS] get_link_point_withcache" << std::endl; 

  // Now we comapre jacobian computed by finite diff with the analytical one 
  for(int i=0; i<8; i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[8], angle_vector[9], angle_vector[10]);
  int gripper_id = robot._link_ids["gripper_link"];
  robot._tf_cache.clear();
  for(int i; i< 8; i++){
    std::cout << "============TESTING===============" << std::endl; 
    int link_id = link_ids[i];
    std::cout << link_names[i] << std::endl; 
    vector<unsigned int> link_ids_ = {link_id};
    auto tmp = robot.get_jacobians_withcache(link_ids_, joint_ids, false, true);
    auto J_ = tmp[0];
    MatrixXd J_numerical = robot.get_jacobian_naive(link_id, joint_ids, false, true);
    std::cout << J_ - J_numerical << std::endl; 
  }

}
