#include "tinyfk.hpp"
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
  std::string urdf_file = "../data/pr2.urdf";
  auto robot = construct_from_urdfpath(urdf_file);

  { // add new link to the robot
    std::vector<std::string> strvec = {"r_upper_arm_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    std::array<double, 3> rot = {0.3, 0.2, 0.1};
    int parent_link_id = robot.get_link_ids(strvec)[0];
    robot.add_new_link("mylink", parent_link_id, pos, rot);
  }

  { // must raise exception when add link with the same name
    std::vector<std::string> strvec = {"r_upper_arm_link"};
    std::array<double, 3> pos = {0.1, 0.1, 0.1};
    std::array<double, 3> rot = {0.3, 0.2, 0.1};
    int parent_link_id = robot.get_link_ids(strvec)[0];
    EXPECT_THROW(robot.add_new_link("mylink", parent_link_id, pos, rot),
                 std::exception);
  }

  { // check if rptable is propery updated
    std::vector<std::string> joint_names = {"torso_lift_joint",
                                            "r_wrist_flex_joint"};
    std::vector<std::string> link_names = {"mylink", "head_pan_link",
                                           "fl_caster_rotation_link"};
    auto joint_ids = robot.get_joint_ids(joint_names);
    auto link_ids = robot.get_link_ids(link_names);
    EXPECT_TRUE(robot.rptable_.isRelevant(joint_ids[0], link_ids[0]));
    EXPECT_FALSE(robot.rptable_.isRelevant(joint_ids[1], link_ids[0]));
    EXPECT_TRUE(robot.rptable_.isRelevant(joint_ids[0], link_ids[1]));
    EXPECT_FALSE(robot.rptable_.isRelevant(joint_ids[0], link_ids[2]));
  }

  auto joint_ids = robot.get_joint_ids(joint_names);
  auto link_ids = robot.get_link_ids(link_names);

  for (size_t i = 0; i < n_joints; i++) {
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[n_joints + 0], angle_vector[n_joints + 1],
                      angle_vector[n_joints + 2]);

  bool base_also = true;
  urdf::Pose pose, pose_naive;
  for (size_t i = 0; i < n_links; i++) {
    int link_id = link_ids[i];

    robot.get_link_pose(link_id, pose, base_also);
    EXPECT_TRUE(isNear(pose.position.x, pose_list[i][0]));
    EXPECT_TRUE(isNear(pose.position.y, pose_list[i][1]));
    EXPECT_TRUE(isNear(pose.position.z, pose_list[i][2]));

    auto rpy = pose.rotation.getRPY();
    EXPECT_TRUE(isNear(rpy.z, pose_list[i][3]));
    EXPECT_TRUE(isNear(rpy.y, pose_list[i][4]));
    EXPECT_TRUE(isNear(rpy.x, pose_list[i][5]));
  }

  // Now we comapre jacobian computed by finite diff with the analytical one
  for (size_t i = 0; i < n_joints; i++) {
    robot.set_joint_angle(joint_ids[i], angle_vector[i]);
  }
  robot.set_base_pose(angle_vector[n_joints], angle_vector[n_joints + 1],
                      angle_vector[n_joints + 2]);
  robot.transform_cache_.clear();
  for (size_t i = 0; i < link_names.size(); i++) {
    bool rot_also =
        true; // rotatio part of the geometric jacobian is not yet checked
    int link_id = link_ids[i];
    vector<int> link_ids_ = {link_id};
    auto J_numerical =
        robot.get_jacobian_naive(link_id, joint_ids, rot_also, true);
    auto tmpo =
        robot.get_jacobians_withcache(link_ids, joint_ids, rot_also, true);
    auto J_analytical_whole = tmpo[0];
    auto J_analytical_block = J_analytical_whole.block(6 * i, 0, 6, 10);

    bool jacobian_equal =
        (J_analytical_block - J_numerical).array().abs().maxCoeff() < 1e-5;
    EXPECT_TRUE(jacobian_equal);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
