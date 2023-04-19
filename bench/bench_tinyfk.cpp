#include <fstream>
#include <iostream>
#include <memory>
#include <tinyfk.hpp>

using namespace tinyfk;

void benchmark_fk(KinematicsModel &kin, size_t n_iter,
                  const std::string &bench_name) {
  std::vector<std::string> link_names = {
      "l_gripper_finger_link", "r_gripper_finger_link", "wrist_flex_link",
      "wrist_roll_link",       "shoulder_lift_link",    "upperarm_roll_link"};
  std::cout << "\n start benchmarking" << std::endl;

  std::vector<std::string> joint_names = {
      // all joints to drive fetch arm
      "torso_lift_joint",    "shoulder_pan_joint", "shoulder_lift_joint",
      "upperarm_roll_joint", "elbow_flex_joint",   "forearm_roll_joint",
      "wrist_flex_joint",    "wrist_roll_joint"};
  const auto joint_ids = kin.get_joint_ids(joint_names);
  const auto link_ids = kin.get_link_ids(link_names);

  const std::vector<double> q = {0, 0, 0, 0, 0, 0, 0, 0};
  {
    clock_t start = clock();
    urdf::Pose out;
    for (size_t i = 0; i < n_iter; i++) {
      kin.set_joint_angles(joint_ids, q); // this clear cached TFs
      for (int lid : link_ids) {
        kin.get_link_pose(lid, out, false);
      }
    }
    clock_t end = clock();
    std::cout << bench_name << " fk : " << end - start << std::endl;
  }

  {
    clock_t start = clock();
    urdf::Pose out;
    for (size_t i = 0; i < n_iter; i++) {
      kin.set_joint_angles(joint_ids, q); // this clear cached TFs
      for (size_t j = 0; j < link_ids.size(); j++) {
        for (size_t lid : link_ids) {
          kin.get_jacobian(lid, joint_ids, RotationType::RPY, true);
        }
      }
    }
    clock_t end = clock();
    std::cout << bench_name << " jacobian : " << end - start << std::endl;
  }
}

int main() {
  const std::string urdf_file = "../data/fetch.urdf";
  const auto urdf_string = load_urdf(urdf_file);
  auto kin = KinematicsModel(urdf_string);

  const size_t N = 100000;
  benchmark_fk(kin, N, "with_cache");
}
