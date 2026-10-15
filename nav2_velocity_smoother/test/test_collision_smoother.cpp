#include <gtest/gtest.h>
#include "nav2_velocity_smoother/velocity_smoother.hpp"

class CollisionSmoother : public nav2_velocity_smoother::VelocitySmoother
{
public:
  explicit CollisionSmoother(bool enabled = true) : VelocitySmoother(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("collision_state_enabled", enabled), rclcpp::Parameter("jerk_limited_smoothing", true), rclcpp::Parameter("feedback", "OPEN_LOOP"), rclcpp::Parameter("smoothing_frequency", 100.0), rclcpp::Parameter("max_velocity", std::vector<double>{1.5, 0.0, 2.0}), rclcpp::Parameter("min_velocity", std::vector<double>{-1.5, 0.0, -2.0})}))
  {
  }

  void start()
  {
    on_configure(rclcpp_lifecycle::State());
    on_activate(rclcpp_lifecycle::State());
    timer_->cancel();
  }

  void finish()
  {
    on_deactivate(rclcpp_lifecycle::State());
    on_cleanup(rclcpp_lifecycle::State());
  }

  void task(double speed, bool percentage = false)
  {
    auto message = std::make_shared<nav2_msgs::msg::SpeedLimit>();
    message->speed_limit = speed;
    message->percentage = percentage;
    speedLimitCallback(message);
  }

  geometry_msgs::msg::Twist step(int state, double speed = 0.4, double angular = 0.2)
  {
    if (state >= 0)
    {
      collision_state_.update(static_cast<uint8_t>(state));
    }
    auto message = std::make_shared<geometry_msgs::msg::Twist>();
    message->linear.x = speed;
    message->angular.z = angular;
    inputCommandCallback(message);
    jerk_last_tick_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(10);
    jerk_clock_initialized_ = true;
    smootherTimer();
    return last_cmd_;
  }

  void expire()
  {
    collision_state_.update(4, std::chrono::steady_clock::now() - std::chrono::milliseconds(501));
  }

  double acceleration() const
  {
    return jerk_profiles_[0].acceleration();
  }
};

class CollisionSmootherTest : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(CollisionSmootherTest, CapsStopAndResumePreserveSCurve)
{
  auto node = std::make_shared<CollisionSmoother>();
  node->start();
  node->task(0.4);
  EXPECT_DOUBLE_EQ(node->step(-1).linear.x, 0.0);
  geometry_msgs::msg::Twist output;
  double previous_acceleration = node->acceleration();
  for (int state : {4, 3, 2, 1, 4, 0, 4})
  {
    for (int i = 0; i < 200; ++i)
    {
      output = node->step(state);
      EXPECT_LE(std::abs(node->acceleration()), 2.5 + 1e-6);
      EXPECT_LE(std::abs(node->acceleration() - previous_acceleration), 0.09);
      previous_acceleration = node->acceleration();
      EXPECT_LE(std::abs(output.linear.x), 1.5);
    }
    const double expected = state == 4 ? 0.4 : state == 3 ? 0.26 : state == 2 ? 0.14 : 0.0;
    EXPECT_NEAR(output.linear.x, expected, 1e-5);
    EXPECT_NEAR(output.angular.z, expected / 2.0, 1e-5);
  }
  const auto stopped_transition = node->step(1);
  EXPECT_GT(stopped_transition.linear.x, 0.1);
  node->finish();
}

TEST_F(CollisionSmootherTest, InvalidTimeoutTaskEndReverseAndLowerInput)
{
  auto node = std::make_shared<CollisionSmoother>();
  node->start();
  node->task(0.4);
  geometry_msgs::msg::Twist output;
  for (int i = 0; i < 200; ++i)
  {
    output = node->step(2, -0.4, 0.0);
  }
  EXPECT_NEAR(output.linear.x, -0.14, 1e-5);
  for (int i = 0; i < 200; ++i)
  {
    output = node->step(255);
  }
  EXPECT_NEAR(output.linear.x, 0.0, 1e-5);
  for (int i = 0; i < 200; ++i)
  {
    output = node->step(3, 0.015, 0.0);
  }
  EXPECT_NEAR(output.linear.x, 0.015, 1e-5);
  node->expire();
  for (int i = 0; i < 200; ++i)
  {
    output = node->step(-1);
  }
  EXPECT_NEAR(output.linear.x, 0.0, 1e-5);
  node->task(0.0);
  for (int i = 0; i < 200; ++i)
  {
    output = node->step(4);
  }
  EXPECT_NEAR(output.linear.x, 0.0, 1e-5);
  EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("collision_state_timeout", 1.0)).successful);
  node->finish();
}

TEST_F(CollisionSmootherTest, DisabledDoesNotGate)
{
  auto node = std::make_shared<CollisionSmoother>(false);
  node->start();
  node->task(0.4);
  geometry_msgs::msg::Twist output;
  for (int i = 0; i < 200; ++i)
  {
    output = node->step(1);
  }
  EXPECT_NEAR(output.linear.x, 0.4, 1e-5);
  node->finish();
}
