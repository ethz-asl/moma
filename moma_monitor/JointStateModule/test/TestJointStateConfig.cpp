//
// Created by acey on 24.10.23.
//

#include "gtest/gtest.h"
#include "JointStateConfig.h"

class JointStateConfigTest : public ::testing::Test {
protected:
  JointStateConfig config_{[](){}};
};

TEST_F(JointStateConfigTest, basic) {
  config_.setValue<int>(JointStateConfig::DIAL_COUNT, 2);

  auto node = config_.save();
  JointStateConfig config2([](){});
  config2.load(node);

  EXPECT_EQ(2, config2.getValue<int>(JointStateConfig::DIAL_COUNT));
  EXPECT_EQ(JointStateConfig::DIAL_MIN_DEFAULT, config2.getValue<double>("dial_min_0"));
  EXPECT_EQ(JointStateConfig::DIAL_MAX_DEFAULT, config2.getValue<double>("dial_max_0"));
  EXPECT_EQ(JointStateConfig::DIAL_MIN_DEFAULT, config2.getValue<double>("dial_min_1"));
  EXPECT_EQ(JointStateConfig::DIAL_MAX_DEFAULT, config2.getValue<double>("dial_max_1"));
}

