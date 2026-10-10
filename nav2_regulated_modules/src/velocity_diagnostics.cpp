// 独立进程只观测速度链，不参与控制或发布运动指令。
#include <cmath>
#include <filesystem>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "byd_custom_msgs/msg/motion_state.hpp"
#include "byd_custom_msgs/msg/control_res.hpp"
#include "byd_custom_msgs/msg/chassis_control.hpp"
#include "spdlog_wrapper.hpp"

class VelocityDiagnostics : public rclcpp::Node
{
public:
  VelocityDiagnostics() : Node("velocity_diagnostics")
  {
    const auto file_hz = declare_parameter("velocity_file_log_frequency", 100.0);
    const auto console_hz = declare_parameter("velocity_console_log_frequency", 1.0);
    stale_seconds_ = declare_parameter("velocity_stale_timeout", 0.2);
    mode_ = declare_parameter("operation_mode", std::string("fixed_path"));
    const auto directory = declare_parameter("log_dir", std::string("/tmp/nav2_logs"));
    if (!std::isfinite(file_hz) || file_hz <= 0.0 || file_hz > 1000.0 || !std::isfinite(console_hz) || console_hz <= 0.0 || console_hz > file_hz || !std::isfinite(stale_seconds_) || stale_seconds_ <= 0.0)
    {
      throw std::invalid_argument("Invalid velocity logging frequencies or stale timeout");
    }
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console->set_level(spdlog::level::info);
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(console);
    const auto path = directory + "/nav2_regulated_modules/velocity_diagnostics.log";
    try
    {
      std::filesystem::create_directories(std::filesystem::path(path).parent_path());
      auto file = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 10 * 1024 * 1024, 5);
      file->set_level(spdlog::level::debug);
      sinks.push_back(file);
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(get_logger(), "File diagnostics disabled: %s", error.what());
    }
    spdlog::init_thread_pool(8192, 1);
    log_ = std::make_shared<spdlog::async_logger>("velocity_diagnostics", sinks.begin(), sinks.end(), spdlog::thread_pool(), spdlog::async_overflow_policy::overrun_oldest);
    spdlog::register_logger(log_);
    log_->set_level(spdlog::level::debug);
    log_->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    spdlog::flush_every(std::chrono::seconds(1));
    const bool collision = declare_parameter("use_collision_monitor", false);
    const auto smoother_topic = declare_parameter("smoother_output_topic", std::string("cmd_vel"));
    const auto feedback_topic = declare_parameter("smoother_feedback_topic", std::string("/motion_state"));
    const auto controller_feedback = declare_parameter("controller_feedback_topic", std::string("/odometry"));
    log_->info("velocity diagnostics mode={} file={} file_enabled={} file_hz={} console_hz={} collision={} smoother_feedback={} feedback_mode={}", mode_, path, sinks.size() > 1, file_hz, console_hz, collision, feedback_topic, declare_parameter("smoother_feedback_mode", std::string("CLOSED_LOOP")));
    if (mode_ != "remote")
    {
      addTwist("controller_out/smoother_in", "cmd_vel_nav");
      addTwist("smoother_out", smoother_topic);
      if (collision)
      {
        addTwist("collision_out/final_cmd", "cmd_vel");
      }
      auto chassis = addStage("chassis_control_in", declare_parameter("chassis_input_topic", std::string("/downstream/chassis_control")));
      subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::ChassisControl>(chassis->topic, rclcpp::SensorDataQoS(), [this, chassis](byd_custom_msgs::msg::ChassisControl::ConstSharedPtr msg)
      {
        chassis->op = msg->op;
        update(chassis, msg->linear_velocity, msg->angular_velocity);
      }));
      const auto chassis_output = declare_parameter("chassis_output_topic", std::string("/control_to_uart"));
      if (chassis_output != "/control_to_uart")
      {
        auto stage = addStage("chassis_control_out", chassis_output);
        subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::ControlRes>(stage->topic, rclcpp::SensorDataQoS(), [this, stage](byd_custom_msgs::msg::ControlRes::ConstSharedPtr msg)
        {
          update(stage, msg->v, msg->w);
        }));
      }
      addFeedback("controller_feedback", controller_feedback);
      addFeedback("smoother_feedback", feedback_topic);
    }
    addFeedback("chassis_feedback", "/motion_state");
    auto uart = addStage(mode_ == "remote" ? "remote_out" : "uart_out", "/control_to_uart");
    subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::ControlRes>(uart->topic, rclcpp::SensorDataQoS(), [this, uart](byd_custom_msgs::msg::ControlRes::ConstSharedPtr msg)
    {
      update(uart, msg->v, msg->w);
    }));
    file_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / file_hz)), [this]()
    {
      snapshot(spdlog::level::debug);
    });
    console_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / console_hz)), [this]()
    {
      snapshot(spdlog::level::info);
    });
  }
private:
  struct Stage
  {
    std::string name;
    std::string topic;
    double v = 0.0;
    double w = 0.0;
    int op = -1;
    uint64_t count = 0;
    bool finite = false;
    std::chrono::steady_clock::time_point received;
  };
  std::shared_ptr<Stage> addStage(const std::string & name, const std::string & topic)
  {
    auto stage = std::make_shared<Stage>();
    stage->name = name;
    stage->topic = topic;
    stages_.push_back(stage);
    return stage;
  }
  void update(const std::shared_ptr<Stage> & stage, double v, double w)
  {
    stage->v = v;
    stage->w = w;
    stage->finite = std::isfinite(v) && std::isfinite(w);
    stage->received = std::chrono::steady_clock::now();
    ++stage->count;
  }
  void addTwist(const std::string & name, const std::string & topic)
  {
    auto stage = addStage(name, topic);
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(topic, rclcpp::SensorDataQoS(), [this, stage](geometry_msgs::msg::Twist::ConstSharedPtr msg)
    {
      update(stage, msg->linear.x, msg->angular.z);
    }));
  }
  void addFeedback(const std::string & name, const std::string & topic)
  {
    auto stage = addStage(name, topic);
    if (topic.substr(topic.find_last_of('/') + 1) == "motion_state")
    {
      subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::MotionState>(topic, rclcpp::SensorDataQoS(), [this, stage](byd_custom_msgs::msg::MotionState::ConstSharedPtr msg)
      {
        update(stage, msg->v_car, msg->w_car);
      }));
    }
    else
    {
      subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(topic, rclcpp::SensorDataQoS(), [this, stage](nav_msgs::msg::Odometry::ConstSharedPtr msg)
      {
        update(stage, msg->twist.twist.linear.x, msg->twist.twist.angular.z);
      }));
    }
  }
  void snapshot(spdlog::level::level_enum level)
  {
    const auto steady = std::chrono::steady_clock::now();
    const auto ros_time = now().seconds();
    for (const auto & stage : stages_)
    {
      const double age = stage->count ? std::chrono::duration<double>(steady - stage->received).count() : -1.0;
      const auto status = !stage->count ? "missing" : !stage->finite ? "invalid" : age > stale_seconds_ ? "stale" : "valid";
      log_->log(level, "velocity ros_time={:.9f} mode={} stage={} topic={} vx={:.6f} wz={:.6f} count={} age={:.6f} status={} op={}", ros_time, mode_, stage->name, stage->topic, stage->v, stage->w, stage->count, age, status, stage->op);
    }
  }
  // 单线程执行器串行执行订阅和采样，无需占用控制节点的锁。
  std::string mode_;
  double stale_seconds_ = 0.2;
  std::vector<std::shared_ptr<Stage>> stages_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr file_timer_;
  rclcpp::TimerBase::SharedPtr console_timer_;
  std::shared_ptr<spdlog::async_logger> log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<VelocityDiagnostics>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(rclcpp::get_logger("velocity_diagnostics"), "%s", error.what());
    rclcpp::shutdown();
    spdlog::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  spdlog::shutdown();
  return 0;
}
