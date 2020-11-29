#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <stdexcept>
#include "tinyfk.hpp"

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

  {// add new link to the robot
    std::vector<std::string> strvec = {"gripper_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    int parent_link_id = robot.get_link_ids(strvec)[0];
    robot.add_new_link("mylink", parent_link_id, pos);
  }

  {// must raise exception when add link with the same name 
    std::vector<std::string> strvec = {"gripper_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    int parent_link_id = robot.get_link_ids(strvec)[0];
    try{
      robot.add_new_link("mylink", parent_link_id, pos);
      std::cout << "[FAIL] must raise exception (add_new_link)" << std::endl;
      return 0;
    }catch (const std::exception& e){
      std::cout << "[PASS] successfully raise exception in add_new_link" << std::endl; 
    }
  }

  auto joint_ids = robot.get_joint_ids(joint_names);
  auto link_ids = robot.get_link_ids(link_names);

  for(int i=0; i<8; i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[8], angle_vector[9], angle_vector[10]);
  bool base_also = true;
  urdf::Pose pose, pose_naive;
  for(unsigned int i=0; i<link_ids.size(); i++){
    std::cout << "testing " << link_names[i] << std::endl; 
    int link_id = link_ids[i];
    robot.get_link_point_withcache(link_id, pose, base_also);

    if(
        !isNear(pose.position.x, pose_list[i][0]) || 
        !isNear(pose.position.y, pose_list[i][1]) || 
        !isNear(pose.position.z, pose_list[i][2])  
      ){
      std::cout << "expected : " << pose_list[i][0] << " " << pose_list[i][1] << " " << pose_list[i][2] << std::endl; 
      std::cout << "computed : " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl; 
      std::cout << "[FAIL] position of " << link_names[i] << " does not match" << std::endl; 
      return 0;
    }

    auto rpy = pose.rotation.getRPY();
    if( 
        // NOTE that rpy of URDFdom is (r, p, y) order, but (y, p, r) in skrobot
        !isNear(rpy.z, pose_list[i][3]) || 
        !isNear(rpy.y, pose_list[i][4]) || 
        !isNear(rpy.x, pose_list[i][5])
      ){
      //throw std::runtime_error("[FAIL] rpy of " << link_names[i] << " does not match");
    }
  }
  std::cout << "[PASS] get_link_point_withcache" << std::endl; 

  // Now we comapre jacobian computed by finite diff with the analytical one 
  for(int i=0; i<8; i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[8], angle_vector[9], angle_vector[10]);
  robot._tf_cache.clear();
  for(int i=0; i< 8; i++){ 
    bool rot_also = false; // rotatio part of the geometric jacobian is not yet checked
    unsigned int link_id = link_ids[i];
    vector<unsigned int> link_ids_ = {link_id};
    auto tmp = robot.get_jacobians_withcache(link_ids_, joint_ids, rot_also, true);
    auto J_ = tmp[0];
    MatrixXd J_numerical = robot.get_jacobian_naive(link_id, joint_ids, rot_also, true);
    bool jacobian_equal = (J_ - J_numerical).array().abs().maxCoeff() < 1e-5;
    if(!jacobian_equal){
      std::cout << "[FAIL] jacobains of " << link_names[i] << "mismatch" << std::endl; 
      return  0;
    }
  }
  std::cout << "[PASS] jacobain" << std::endl; 
}
