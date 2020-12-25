#include <tinyfk.hpp>
#include <iostream>
#include <fstream>

using namespace tinyfk;

int main(){
  int N = 100000;
  std::string urdf_file = "../data/fetch.urdf";
  std::vector<std::string> link_names = {
    "l_gripper_finger_link", 
    "r_gripper_finger_link", 
    "wrist_flex_link",
    "wrist_roll_link",
    "shoulder_lift_link",
    "upperarm_roll_link"};
  std::cout << "\n start benchmarking" << std::endl; 

  auto robot = RobotModel(urdf_file);
  std::vector<std::string> joint_names = {// all joints to drive fetch arm
        "torso_lift_joint",
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint"};
  auto joint_ids = robot.get_joint_ids(joint_names);
  auto link_ids = robot.get_link_ids(link_names);
  std::vector<double> angle_vector = {0, 0, 0, 0, 0, 0, 0, 0};
  {// bench tinyfk FK : without using cache
    clock_t start = clock();
    urdf::Pose out;
    for(int i=0; i<N; i++){
      for(int lid : link_ids){
        robot.get_link_point(lid, out, false);
      }
    }
    clock_t end = clock();
    std::cout << "tinyfk.FK_naive : " << end - start << std::endl;
  }

  {// bench tinyfk FK : with cache
    clock_t start = clock();
    urdf::Pose out;
    for(int i=0; i<N; i++){
      robot.set_joint_angles(joint_ids, angle_vector); // this clear cached TFs
      for(int lid : link_ids){
        robot.get_link_point_withcache(lid, out, false);
      }
    }
    clock_t end = clock();
    std::cout << "tinyfk.FK_with_cache : " << end - start << std::endl;
  }
  {
    clock_t start = clock();
    urdf::Pose out;
    for(int i=0; i<N; i++){
      robot.set_joint_angles(joint_ids, angle_vector); // this clear cached TFs
      for(int j=0; j<link_ids.size(); j++){
        robot.get_jacobians_withcache(link_ids, joint_ids, true, true);
      }
    }
    clock_t end = clock();
    std::cout << "tinyfk.jaocbian_with_cache : " << end - start << std::endl;
  }
}
