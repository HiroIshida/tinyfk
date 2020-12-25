#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <stdexcept>
#include "tinyfk.hpp"

using namespace std;
using namespace Eigen;
using namespace tinyfk;

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

  int n_joints = joint_names.size();
  int n_links = link_names.size();

  // test main
  std::string urdf_file = "../data/pr2.urdf";
  auto robot = RobotModel(urdf_file);

  {// add new link to the robot
    std::vector<std::string> strvec = {"r_upper_arm_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    int parent_link_id = robot.get_link_ids(strvec)[0];
    robot.add_new_link("mylink", parent_link_id, pos);
  }

  {// must raise exception when add link with the same name 
    std::vector<std::string> strvec = {"r_upper_arm_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    int parent_link_id = robot.get_link_ids(strvec)[0];
    try{
      robot.add_new_link("mylink", parent_link_id, pos);
      std::cout << "[FAIL] must raise exception (add_new_link)" << std::endl;
      return -1;
    }catch (const std::exception& e){
      std::cout << "[PASS] successfully raise exception in add_new_link" << std::endl; 
    }
  }
  {// check if rptable is propery updated
    std::vector<std::string> joint_names = {"torso_lift_joint", "r_wrist_flex_joint"};
    std::vector<std::string> link_names = {"mylink", "head_pan_link", "fl_caster_rotation_link"};
    auto joint_ids = robot.get_joint_ids(joint_names);
    auto link_ids = robot.get_link_ids(link_names);
    if(!robot._rptable.isRelevant(joint_ids[0], link_ids[0])){
      std::cout << "[FAIL] torso_lift_joint must move mylink" << std::endl;
      return -1;
    }else{
      std::cout << "[PASS] rptable" << std::endl;
    }
    if(robot._rptable.isRelevant(joint_ids[1], link_ids[0])){
      std::cout << "[FAIL] r_wrist_flex_joint must not move mylink" << std::endl;
      return -1;
    }else{
      std::cout << "[PASS] rptable" << std::endl;
    }
    if(!robot._rptable.isRelevant(joint_ids[0], link_ids[1])){
      std::cout << "[FAIL] torso_lift_joint must move head_pan_link" << std::endl;
      return -1;
    }else{
      std::cout << "[PASS] rptable" << std::endl;
    }
    if(robot._rptable.isRelevant(joint_ids[0], link_ids[2])){
      std::cout << "[FAIL] torso_lift_joint must not move fl_caster_rotation_link" << std::endl;
      return -1;
    }else{
      std::cout << "[PASS] rptable" << std::endl;
    }
  }

  auto joint_ids = robot.get_joint_ids(joint_names);
  auto link_ids = robot.get_link_ids(link_names);

  for(int i=0; i<n_joints; i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[n_joints+0], angle_vector[n_joints+1], angle_vector[n_joints+2]);
  bool base_also = true;
  urdf::Pose pose, pose_naive;
  for(unsigned int i=0; i<n_links; i++){
    std::cout << "testing " << link_names[i] << std::endl; 
    int link_id = link_ids[i];
    robot.get_link_point_withcache(link_id, pose, base_also);
    std::cout << "expected : " << pose_list[i][0] << " " << pose_list[i][1] << " " << pose_list[i][2] << std::endl; 
    std::cout << "computed : " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl; 

    if(
        !isNear(pose.position.x, pose_list[i][0]) || 
        !isNear(pose.position.y, pose_list[i][1]) || 
        !isNear(pose.position.z, pose_list[i][2])  
      ){
      std::cout << "[FAIL] position of " << link_names[i] << " does not match" << std::endl; 
      return -1;
    }else{
      std::cout << "[PASS] match" << std::endl; 
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
  for(int i=0; i<n_joints; i++){
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[n_joints], angle_vector[n_joints+1], angle_vector[n_joints+2]);
  robot._tf_cache.clear();
  for(int i=0; i< link_names.size(); i++){ 
    bool rot_also = true; // rotatio part of the geometric jacobian is not yet checked
    int link_id = link_ids[i];
    vector<unsigned int> link_ids_ = {link_id};
    auto tmp = robot.get_jacobians_withcache(link_ids_, joint_ids, rot_also, true);
    auto J_analytical_old = tmp[0];
    auto tmpo = robot.get_jacobians_withcache_new(link_ids_, joint_ids, rot_also, true);
    auto J_analytical = tmpo[0];
    bool jacobian_equal = (J_analytical - J_analytical_old).array().abs().maxCoeff() < 1e-5;
    if(!jacobian_equal){
      std::cout << "analytical :\n" << J_analytical << std::endl; 
      std::cout << "analytical_old :\n" << J_analytical_old << std::endl; 
      std::cout << "[FAIL] jacobains of " << link_names[i] << "mismatch" << std::endl; 
      return  -1;
    }
  }
  std::cout << "[PASS] fk solve compare" << std::endl; 


}
