#include "kdl_parser.hpp"
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <tinyfk.hpp>
#include <iostream>
#include <fstream>

void urdf2kdl(const std::string& urdf_file, KDL::Tree& tree){
  std::string xml_string;
  std::fstream xml_file(urdf_file, std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  auto robot_urdf_interface = urdf::parseURDF(xml_string);
  kdl_parser::treeFromUrdfModel(*robot_urdf_interface, tree);
}

// http://wiki.ros.org/pr2_mechanism/Tutorials/Coding%20a%20realtime%20Cartesian%20controller%20with%20KDL
int main(){

  int N = 1000;
  std::string urdf_file = "../data/fetch_description/fetch.urdf";
  std::vector<std::string> link_names = {
    "l_gripper_finger_link", 
    "r_gripper_finger_link", 
    "wrist_flex_link",
    "wrist_roll_link",
    "shoulder_lift_link",
    "upperarm_roll_link"};
  std::cout << "\n start benchmarking" << std::endl; 

  {// bench KDL FK
    KDL::Tree tree;
    urdf2kdl(urdf_file, tree);
    KDL::Chain chain;
    tree.getChain("base_link", "r_gripper_finger_link", chain);

    auto segmap = tree.getSegments();
    for(auto& elem : segmap){
      //std::cout << elem.first << std::endl; 
    }
    auto solver = KDL::TreeFkSolverPos_recursive(tree);
    int n_dof = tree.getNrOfJoints();
    auto jntarr = KDL::JntArray(n_dof);

    KDL::Frame frame;
    KDL::Jacobian  J;
    {
      clock_t start = clock();
      for(int i=0; i<N; i++){
        for(auto& lname : link_names){
          int ret = solver.JntToCart(jntarr, frame, lname); 
        }
      }
      clock_t end = clock();
      std::cout << "\n KDL.FK : " << end - start << std::endl;
    }
  }

  {// bench tinyfk FK
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
  }
}
