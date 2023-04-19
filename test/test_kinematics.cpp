#include "tinyfk.hpp"
#include "urdf_model/pose.h"
#include <cmath>
#include <fstream>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;
using namespace Eigen;
using namespace tinyfk;

bool isNear(double x, double y) { return (abs(x - y) < 1e-5); }

Eigen::MatrixXd compute_numerical_jacobian_with_base(
    KinematicsModel &kin, size_t link_id, const std::vector<size_t> &joint_ids,
    const std::vector<double> &angle_vector, const urdf::Pose &base_pose,
    RotationType rot_type) {

  const auto set_configuration = [&](const std::vector<double> &q) {
    for (size_t i = 0; i < joint_ids.size(); ++i) {
      kin.set_joint_angle(joint_ids[i], q[i]);
    }
    urdf::Pose pose;
    pose.position.x = q[joint_ids.size()];
    pose.position.y = q[joint_ids.size() + 1];
    pose.position.z = q[joint_ids.size() + 2];

    pose.rotation.setFromRPY(q[joint_ids.size() + 3], q[joint_ids.size() + 4],
                             q[joint_ids.size() + 5]);

    kin.set_base_pose(pose);
  };

  constexpr double eps = 1e-6;

  const auto get_tweaked_q = [&](const std::vector<double> &q, size_t idx) {
    auto q_out = q;
    q_out[idx] += eps;
    return q_out;
  };

  std::vector<double> q0;
  {
    for (size_t i = 0; i < joint_ids.size(); ++i) {
      q0.push_back(angle_vector[i]);
    }
    q0.push_back(base_pose.position.x);
    q0.push_back(base_pose.position.y);
    q0.push_back(base_pose.position.z);

    const auto rpy = base_pose.rotation.getRPY();
    q0.push_back(rpy.x);
    q0.push_back(rpy.y);
    q0.push_back(rpy.z);
  }

  urdf::Pose pose0, pose1;
  urdf::Vector3 rpy0, rpy1;
  set_configuration(q0);
  kin.get_link_pose(link_id, pose0, true);
  rpy0 = pose0.rotation.getRPY();

  const size_t dim_jacobi = 3 + (rot_type == RotationType::RPY) * 3 +
                            (rot_type == RotationType::XYZW) * 4;
  Eigen::MatrixXd J(dim_jacobi, q0.size());

  for (size_t idx = 0; idx < q0.size(); idx++) {
    const auto q1 = get_tweaked_q(q0, idx);
    set_configuration(q1);
    kin.get_link_pose(link_id, pose1, true);
    J(0, idx) = (pose1.position.x - pose0.position.x) / eps;
    J(1, idx) = (pose1.position.y - pose0.position.y) / eps;
    J(2, idx) = (pose1.position.z - pose0.position.z) / eps;

    if (rot_type == RotationType::RPY) {
      rpy1 = pose1.rotation.getRPY();
      J(3, idx) = (rpy1.x - rpy0.x) / eps;
      J(4, idx) = (rpy1.y - rpy0.y) / eps;
      J(5, idx) = (rpy1.z - rpy0.z) / eps;
    }
    if (rot_type == RotationType::XYZW) {
      J(3, idx) = (pose1.rotation.x - pose0.rotation.x) / eps;
      J(4, idx) = (pose1.rotation.y - pose0.rotation.y) / eps;
      J(5, idx) = (pose1.rotation.z - pose0.rotation.z) / eps;
      J(6, idx) = (pose1.rotation.w - pose0.rotation.w) / eps;
    }
  };
  return J;
}

TEST(KINEMATICS, AllTest) {
  // loading test data
  ifstream test_data("../test/data/test_data.json");
  nlohmann::json js;
  test_data >> js;
  vector<double> angle_vector = js["angle_vector"];
  vector<string> joint_names = js["joint_names"];
  vector<string> link_names = js["link_names"];
  vector<vector<double>> pose_list = js["pose_list"];

  size_t n_joints = joint_names.size();
  size_t n_links = link_names.size();

  // test main
  const std::string urdf_file = "../data/pr2.urdf";
  const auto xml_string = load_urdf(urdf_file);
  auto kin = KinematicsModel(xml_string);
  auto kin2 = KinematicsModel(xml_string);

  { // add new link to the robot
    std::vector<std::string> strvec = {"r_upper_arm_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    std::array<double, 3> rot = {0.3, 0.2, 0.1};
    int parent_link_id = kin.get_link_ids(strvec)[0];
    kin.add_new_link("mylink", parent_link_id, pos, rot);
    kin2.add_new_link("mylink", parent_link_id, pos, rot);
  }

  { // must raise exception when add link with the same name
    std::vector<std::string> strvec = {"r_upper_arm_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    std::array<double, 3> rot = {0.3, 0.2, 0.1};
    int parent_link_id = kin.get_link_ids(strvec)[0];
    EXPECT_THROW(kin.add_new_link("mylink", parent_link_id, pos, rot),
                 std::exception);
  }

  { // check if rptable is propery updated
    std::vector<std::string> joint_names = {"torso_lift_joint",
                                            "r_wrist_flex_joint"};
    std::vector<std::string> link_names = {"mylink", "head_pan_link",
                                           "fl_caster_rotation_link"};
    auto joint_ids = kin.get_joint_ids(joint_names);
    auto link_ids = kin.get_link_ids(link_names);
    EXPECT_TRUE(kin.rptable_.isRelevant(joint_ids[0], link_ids[0]));
    EXPECT_FALSE(kin.rptable_.isRelevant(joint_ids[1], link_ids[0]));
    EXPECT_TRUE(kin.rptable_.isRelevant(joint_ids[0], link_ids[1]));
    EXPECT_FALSE(kin.rptable_.isRelevant(joint_ids[0], link_ids[2]));
  }

  auto joint_ids = kin.get_joint_ids(joint_names);
  auto link_ids = kin.get_link_ids(link_names);

  auto base_pose = urdf::Pose();
  base_pose.position.x = angle_vector[n_joints + 0];
  base_pose.position.y = angle_vector[n_joints + 1];
  base_pose.position.z = angle_vector[n_joints + 2];
  base_pose.rotation.setFromRPY(angle_vector[n_joints + 3],
                                angle_vector[n_joints + 4],
                                angle_vector[n_joints + 5]);

  for (size_t i = 0; i < n_joints; i++) {
    kin.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  kin.set_base_pose(base_pose);

  bool base_also = true;
  urdf::Pose pose, pose_naive;
  for (size_t i = 0; i < n_links; i++) {
    int link_id = link_ids[i];

    kin.get_link_pose(link_id, pose, base_also);
    EXPECT_TRUE(isNear(pose.position.x, pose_list[i][0]));
    EXPECT_TRUE(isNear(pose.position.y, pose_list[i][1]));
    EXPECT_TRUE(isNear(pose.position.z, pose_list[i][2]));

    auto rpy = pose.rotation.getRPY();
    EXPECT_TRUE(isNear(rpy.z, pose_list[i][3]));
    EXPECT_TRUE(isNear(rpy.y, pose_list[i][4]));
    EXPECT_TRUE(isNear(rpy.x, pose_list[i][5]));
  }

  // compare numerical and analytical jacobian
  kin.set_base_pose(base_pose);
  kin2.set_base_pose(base_pose);

  kin.transform_cache_.clear();
  std::vector<RotationType> rot_types = {RotationType::IGNORE,
                                         RotationType::RPY, RotationType::XYZW};
  for (auto rot_type : rot_types) {
    for (size_t i = link_names.size() - 1; i < link_names.size(); i++) {
      size_t link_id = link_ids[i];
      auto J_numerical = compute_numerical_jacobian_with_base(
          kin2, link_id, joint_ids, angle_vector, base_pose, rot_type);
      auto J_analytical = kin.get_jacobian(link_id, joint_ids, rot_type, true);
      bool jacobian_equal =
          (J_analytical - J_numerical).array().abs().maxCoeff() < 1e-5;
      std::cout << J_numerical << std::endl;
      std::cout << "" << std::endl;
      std::cout << J_analytical << std::endl;
      EXPECT_TRUE(jacobian_equal);
    }
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
