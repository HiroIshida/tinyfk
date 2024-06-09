#include "tinyfk.hpp"
#include <gtest/gtest.h>

using namespace tinyfk;

TEST(TEST_PARSE, AllTest) {
  const std::string urdf_file = "../data/pr2.urdf";
  const auto xml_string = load_urdf(urdf_file);
  auto kin = KinematicModel(xml_string);

  // continuous, revolute, prismatic
  auto ids = kin.get_joint_ids(
      {"r_forearm_roll_joint", "r_shoulder_pan_joint", "torso_lift_joint"});
  auto pos_limits = kin.get_joint_position_limits(ids);
  auto vel_limits = kin.get_joint_velocity_limits(ids);
  auto eff_limits = kin.get_joint_effort_limits(ids);

  // check that limit of r_forearm_roll_joint is continuous
  // <limit effort="30" velocity="3.6"/>
  EXPECT_EQ(pos_limits[0].first, -std::numeric_limits<double>::infinity());
  EXPECT_EQ(pos_limits[0].second, std::numeric_limits<double>::infinity());
  EXPECT_EQ(vel_limits[0], 3.6);
  EXPECT_EQ(eff_limits[0], 30);

  // check that limit of r_shoulder_pan_joint is
  // <limit effort="30" lower="-2.2853981634" upper="0.714601836603"
  // velocity="2.088"/>
  EXPECT_EQ(pos_limits[1].first, -2.2853981634);
  EXPECT_EQ(pos_limits[1].second, 0.714601836603);
  EXPECT_EQ(pos_limits[1].second, 0.714601836603);
  EXPECT_EQ(vel_limits[1], 2.088);
  EXPECT_EQ(eff_limits[1], 30);

  // check that limit of torso_lift_joint is
  // <limit effort="10000" lower="0.0" upper="0.33" velocity="0.013"/>
  EXPECT_EQ(pos_limits[2].first, 0.0);
  EXPECT_EQ(pos_limits[2].second, 0.33);
  EXPECT_EQ(vel_limits[2], 0.013);
  EXPECT_EQ(eff_limits[2], 10000);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
