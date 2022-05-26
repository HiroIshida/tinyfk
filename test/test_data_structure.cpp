#include <data_structure.hpp>
#include <gtest/gtest.h>

TEST(DataStructureTest, SizedStack) {
  tinyfk::SizedStack<int> s(100);
  s.top();
  assert(s.empty());
  s.push(1);
  s.push(2);
  EXPECT_EQ(s.size(), 2);
  EXPECT_EQ(s.top(), 2);
  s.pop();
  s.pop();
  EXPECT_TRUE(s.empty());
  s.reset();
  s.push(100);
  s.push(200);
  EXPECT_EQ(s.top(), 200);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
