#include <gtest/gtest.h>
#include "nav2_controller/controller_server.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_core/terminal_aware_controller.hpp"
#include "nav2_core/exceptions.hpp"

class TrackingController : public nav2_core::Controller
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string, std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void cleanup() override
  {
  }
  void activate() override
  {
  }
  void deactivate() override
  {
  }
  void setPlan(const nav_msgs::msg::Path &) override
  {
  }
  void setSpeedLimit(const double &, const bool &) override
  {
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *) override
  {
    return geometry_msgs::msg::TwistStamped();
  }
};

class TerminalController : public TrackingController, public nav2_core::TerminalAwareController
{
public:
  bool isTerminalStopLatched() override
  {
    return false;
  }
  bool isTerminalPositionAccurate() override
  {
    return false;
  }
};

class CollisionController : public nav2_controller::ControllerServer
{
public:
  CollisionController() : ControllerServer(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("progress_checker.movement_time_allowance", 0.05)}))
  {
  }
  void prepare(bool enabled = true, bool terminal = true)
  {
    collision_state_enabled_ = enabled;
    current_controller_ = "test_controller";
    if (terminal)
    {
      controllers_[current_controller_] = std::make_shared<TerminalController>();
    }
    else
    {
      controllers_[current_controller_] = std::make_shared<TrackingController>();
    }
    progress_checker_ = std::make_shared<nav2_controller::SimpleProgressChecker>();
    progress_checker_->initialize(shared_from_this(), "progress_checker");
  }
  void state(uint8_t value)
  {
    collision_state_.update(value);
  }
  void check()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.w = 1.0;
    checkProgress(pose);
  }
  bool paused() const
  {
    return collision_progress_paused_;
  }
};

class CollisionProgressTest : public testing::Test
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

TEST_F(CollisionProgressTest, StopWaitsPastDeadlineAndResumeRestartsChecker)
{
  auto node = std::make_shared<CollisionController>();
  node->prepare();
  node->state(4);
  EXPECT_NO_THROW(node->check());
  node->state(1);
  EXPECT_NO_THROW(node->check());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_NO_THROW(node->check());
  EXPECT_TRUE(node->paused());
  node->state(3);
  EXPECT_NO_THROW(node->check());
  EXPECT_FALSE(node->paused());
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  EXPECT_THROW(node->check(), nav2_core::PlannerException);
}

TEST_F(CollisionProgressTest, InitialInvalidAndExpiredStatePause)
{
  auto node = std::make_shared<CollisionController>();
  node->prepare();
  EXPECT_NO_THROW(node->check());
  EXPECT_TRUE(node->paused());
  node->state(255);
  EXPECT_NO_THROW(node->check());
  node->state(4);
  EXPECT_NO_THROW(node->check());
  std::this_thread::sleep_for(std::chrono::milliseconds(510));
  EXPECT_NO_THROW(node->check());
  EXPECT_TRUE(node->paused());
  node->state(4);
  EXPECT_NO_THROW(node->check());
}

TEST_F(CollisionProgressTest, DisabledOrNonterminalControllerStillTimesOut)
{
  for (int kind = 0; kind < 2; ++kind)
  {
    auto node = std::make_shared<CollisionController>();
    node->prepare(kind != 0, kind == 0);
    node->state(1);
    EXPECT_NO_THROW(node->check());
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    EXPECT_THROW(node->check(), nav2_core::PlannerException);
    EXPECT_FALSE(node->paused());
  }
}
