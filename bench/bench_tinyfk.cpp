#include <tinyfk.hpp>
#include <iostream>
#include <fstream>

int main(){
  int N = 100000;
  std::string urdf_file = "../data/fetch_description/fetch.urdf";
  std::vector<std::string> link_names = {
    "l_gripper_finger_link", 
    "r_gripper_finger_link", 
    "wrist_flex_link",
    "wrist_roll_link",
    "shoulder_lift_link",
    "upperarm_roll_link"};

  for(int i=0; i<100; i++){
    link_names.push_back("l_gripper_finger_link");
  }
  std::cout << "\n start benchmarking" << std::endl; 

  auto robot = RobotModel(urdf_file);
  std::vector<std::string> joint_names = {// all joints to drive fetch arm
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "upperarm_roll_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint"};
  auto joint_ids = robot.get_joint_ids(joint_names);
  auto link_ids = robot.get_link_ids(link_names);
  std::vector<double> angle_vector = {0, 0, 0, 0, 0, 0, 0};

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
    std::cout << "tinyfk.FK_with_cache : " << double(end - start)/1e6 << std::endl;
  }
}
