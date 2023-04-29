#include "urdf_model/joint.h"
#include "urdf_model/pose.h"
#include "urdf_parser/urdf_parser.h"
#include <gtest/gtest.h>

int main() {
  // test inverse
  urdf::Pose pose12, pose13;
  pose12.position.x = 0.1;
  pose12.position.y = 0.1;
  pose12.position.y = 0.1;
  pose12.rotation.setFromRPY(0.1, 0.2, 0.3);

  pose13.position.x = 0.2;
  pose13.position.y = 0.2;
  pose13.position.y = 0.2;
  pose13.rotation.setFromRPY(0.2, 0.3, 0.4);

  urdf::Pose pose21, pose23;
  pose21 = pose12.inverse();
  pose23 = pose_transform(pose21, pose13);

  urdf::Pose pose13_again = pose_transform(pose12, pose23);
  const auto pos_diff = pose13_again.position - pose13.position;
  const auto rpy_diff =
      pose13_again.rotation.getRPY() - pose13.rotation.getRPY();

  EXPECT_NEAR(pos_diff.x, 0.0, 1e-6);
  EXPECT_NEAR(pos_diff.y, 0.0, 1e-6);
  EXPECT_NEAR(pos_diff.z, 0.0, 1e-6);

  EXPECT_NEAR(rpy_diff.x, 0.0, 1e-6);
  EXPECT_NEAR(rpy_diff.y, 0.0, 1e-6);
  EXPECT_NEAR(rpy_diff.z, 0.0, 1e-6);
}
