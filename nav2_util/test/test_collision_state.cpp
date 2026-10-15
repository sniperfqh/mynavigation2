#include <gtest/gtest.h>
#include <limits>
#include "nav2_util/collision_state.hpp"

TEST(CollisionState, InitialStatesTimeoutInvalidAndRecovery)
{
  nav2_util::CollisionState cache;
  const auto origin = nav2_util::CollisionState::Clock::now();
  EXPECT_TRUE(cache.snapshot(origin).stopped());
  EXPECT_STREQ(cache.snapshot(origin).reason(), "initial");
  for (uint8_t state = 0; state <= 4; ++state)
  {
    cache.update(state, origin);
    EXPECT_EQ(cache.snapshot(origin).effective, state);
    EXPECT_EQ(cache.snapshot(origin).stopped(), state <= 1);
  }
  EXPECT_EQ(cache.snapshot(origin + std::chrono::milliseconds(499)).effective, 4);
  EXPECT_EQ(cache.snapshot(origin + std::chrono::milliseconds(500)).effective, 0);
  EXPECT_STREQ(cache.snapshot(origin + std::chrono::milliseconds(500)).reason(), "timeout");
  cache.update(255, origin);
  EXPECT_EQ(cache.snapshot(origin).effective, 0);
  EXPECT_STREQ(cache.snapshot(origin).reason(), "invalid");
  cache.update(3, origin);
  EXPECT_DOUBLE_EQ(cache.snapshot(origin).ratio(), 0.65);
  cache.reset();
  EXPECT_FALSE(cache.snapshot(origin).received);
  EXPECT_THROW(cache.configure(0.0), std::invalid_argument);
  EXPECT_THROW(cache.configure(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}

TEST(CollisionState, TaskBasedCapsDirectionsCurvatureAndNoRepeatedScaling)
{
  nav2_util::CollisionState cache;
  const std::vector<double> low{-1.5, 0.0, -2.0};
  const std::vector<double> high{1.5, 0.0, 2.0};
  cache.update(3);
  auto target = nav2_util::collisionVelocityTarget({0.3, 0.0, 0.15}, 0.4, cache.snapshot(), low, high);
  EXPECT_NEAR(target[0], 0.26, 1e-12);
  EXPECT_NEAR(target[2], 0.13, 1e-12);
  target = nav2_util::collisionVelocityTarget({0.05, 0.0, 0.1}, 0.4, cache.snapshot(), low, high);
  EXPECT_DOUBLE_EQ(target[0], 0.05);
  EXPECT_DOUBLE_EQ(target[2], 0.1);
  cache.update(2);
  target = nav2_util::collisionVelocityTarget({-0.4, 0.0, 0.2}, 0.4, cache.snapshot(), low, high);
  EXPECT_NEAR(target[0], -0.14, 1e-12);
  EXPECT_NEAR(target[2], 0.07, 1e-12);
  target = nav2_util::collisionVelocityTarget({0.0, 0.0, 0.4}, 0.4, cache.snapshot(), low, high);
  EXPECT_NEAR(target[2], 0.14, 1e-12);
  EXPECT_DOUBLE_EQ(nav2_util::collisionVelocityTarget({0.4, 0.0, 0.4}, 0.0, cache.snapshot(), low, high)[2], 0.0);
  for (uint8_t state : {0, 1})
  {
    cache.update(state);
    target = nav2_util::collisionVelocityTarget({0.4, 0.0, 0.4}, 0.4, cache.snapshot(), low, high);
    EXPECT_EQ(target, (std::array<double, 3>{0.0, 0.0, 0.0}));
  }
  cache.update(4);
  EXPECT_DOUBLE_EQ(nav2_util::collisionVelocityTarget({2.0, 0.0, 0.0}, 2.0, cache.snapshot(), low, high)[0], 1.5);
}

TEST(CollisionState, FeedbackCannotBypassSelectedTarget)
{
  EXPECT_DOUBLE_EQ(nav2_util::collisionFeedbackTarget(0.4, 0.26), 0.26);
  EXPECT_DOUBLE_EQ(nav2_util::collisionFeedbackTarget(-0.4, -0.14), -0.14);
  EXPECT_DOUBLE_EQ(nav2_util::collisionFeedbackTarget(-0.1, 0.26), 0.0);
  EXPECT_DOUBLE_EQ(nav2_util::collisionFeedbackTarget(0.1, -0.14), 0.0);
  EXPECT_DOUBLE_EQ(nav2_util::collisionFeedbackTarget(0.4, 0.0), 0.0);
}
