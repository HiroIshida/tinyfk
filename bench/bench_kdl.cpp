#include "kdl_parser/kdl_parser.hpp"
#include "urdf_model/joint.h"
#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"
#include <fstream>
#include <iostream>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

void urdf2kdl(const std::string &urdf_file, KDL::Tree &tree) {
  std::string xml_string;
  std::fstream xml_file(urdf_file, std::fstream::in);
  while (xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();
  auto robot_urdf_interface = urdf::parseURDF(xml_string);
  kdl_parser::treeFromUrdfModel(*robot_urdf_interface, tree);
}

// http://wiki.ros.org/pr2_mechanism/Tutorials/Coding%20a%20realtime%20Cartesian%20controller%20with%20KDL
int main() {
  int N = 100000;
  std::string urdf_file = "../data/fetch.urdf";
  std::vector<std::string> link_names = {
      "l_gripper_finger_link", "r_gripper_finger_link", "wrist_flex_link",
      "wrist_roll_link",       "shoulder_lift_link",    "upperarm_roll_link"};
  std::cout << "\n start benchmarking" << std::endl;

  KDL::Tree tree;
  urdf2kdl(urdf_file, tree);
  KDL::Chain chain;
  tree.getChain("base_link", "r_gripper_finger_link", chain);

  auto segmap = tree.getSegments();
  for (auto &elem : segmap) {
    // std::cout << elem.first << std::endl;
  }
  auto solver = KDL::TreeFkSolverPos_recursive(tree);
  int n_dof = tree.getNrOfJoints();
  auto jntarr = KDL::JntArray(n_dof);

  KDL::Frame frame;
  KDL::Jacobian J;
  {
    clock_t start = clock();
    for (int i = 0; i < N; i++) {
      for (auto &lname : link_names) {
        int ret = solver.JntToCart(jntarr, frame, lname);
      }
    }
    clock_t end = clock();
    std::cout << "\n KDL.FK : " << end - start << std::endl;
  }
}
