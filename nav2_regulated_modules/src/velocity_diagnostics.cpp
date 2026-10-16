// 独立速度链观测进程。按“解析后的话题＋消息类型”复用订阅，再按频率输出最新值快照。
// 快照频率不是输入接收频率；missing 的零占位不是停车，count 不增长表示没有新样本。
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
  // 校验快照频率，建立异步滚动日志与按模式选取的订阅。
  // 文件失败仍保留终端输出，8192 条异步队列拥塞时覆盖旧记录，不能保证逐消息无丢失。
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
      const auto chassis_topic = declare_parameter("chassis_input_topic", std::string("/downstream/chassis_control"));
      auto chassis = addStage("chassis_control_in", chassis_topic, "ChassisControl");
      subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::ChassisControl>(chassis_topic, rclcpp::SensorDataQoS(), [this, chassis](byd_custom_msgs::msg::ChassisControl::ConstSharedPtr msg)
      {
        chassis->op = msg->op;
        update(chassis, msg->linear_velocity, msg->angular_velocity);
      }));
      const auto chassis_output = declare_parameter("chassis_output_topic", std::string("/control_to_uart"));
      if (chassis_output != "/control_to_uart")
      {
        addControlRes("chassis_control_out", chassis_output);
      }
      addFeedback("controller_feedback", controller_feedback);
      addFeedback("smoother_feedback", feedback_topic);
    }
    addFeedback("chassis_feedback", "/motion_state");
    addControlRes(mode_ == "remote" ? "remote_out" : "uart_out", "/control_to_uart");
    for (size_t i = 0; i < stages_.size(); ++i)
    {
      const auto & stage = stages_[i];
      log_->info("velocity source id={} stage={} topic={} type={}", i, stage->name, stage->topic, stage->type);
    }
    // 两个墙钟定时器分别以 debug／info 级别记录同一类快照；sink 的级别门限决定文件与终端接收哪些帧。
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
  // 一个已解析“话题＋类型”的最新快照，可合并多个业务角色；count=0 表示还没有样本。
  // received 使用 steady_clock；v/w 与其他字段在当前单线程执行器串行更新和读取。
  struct Stage
  {
    std::string name;
    std::string topic;
    std::string type;
    bool subscribed = false;
    double v_lift = 0.0;
    double lift_height = 0.0;
    double w_shelf = 0.0;
    double yaw_shelf = 0.0;
    double v = 0.0;
    double w = 0.0;
    int op = -1;
    uint64_t count = 0;
    bool finite = false;
    std::chrono::steady_clock::time_point received;
  };
  // 先解析话题全名，再按“同话题＋同类型”复用 Stage，并合并其业务角色名。
  // 复用后 count、age 与有效性来自同一订阅，防止同源被误报成不同采样。
  std::shared_ptr<Stage> addStage(const std::string & name, const std::string & topic, const std::string & type)
  {
    const auto resolved = get_node_topics_interface()->resolve_topic_name(topic);
    for (const auto & existing : stages_)
    {
      if (existing->topic == resolved && existing->type == type)
      {
        existing->name += "," + name;
        return existing;
      }
    }
    auto stage = std::make_shared<Stage>();
    stage->name = name;
    stage->topic = resolved;
    stage->type = type;
    stages_.push_back(stage);
    return stage;
  }
  // 保存最新线／角速度、有限性和单调接收时间并递增 count。
  // 即使非法样本也会递增计数并标为 invalid，不能将日志中的零初值当作有效反馈。
  void update(const std::shared_ptr<Stage> & stage, double v, double w)
  {
    stage->v = v;
    stage->w = w;
    stage->finite = std::isfinite(v) && std::isfinite(w);
    stage->received = std::chrono::steady_clock::now();
    ++stage->count;
  }
  // 为尚未订阅的 Stage 建立 SensorDataQoS Twist 订阅，读取 linear.x/angular.z。
  void addTwist(const std::string & name, const std::string & topic)
  {
    auto stage = addStage(name, topic, "Twist");
    if (stage->subscribed)
    {
      return;
    }
    stage->subscribed = true;
    subscriptions_.push_back(create_subscription<geometry_msgs::msg::Twist>(topic, rclcpp::SensorDataQoS(), [this, stage](geometry_msgs::msg::Twist::ConstSharedPtr msg)
    {
      update(stage, msg->linear.x, msg->angular.z);
    }));
  }
  // 按解析前话题末段是否为 motion_state 选择 MotionState 或 Odometry 读取方式。
  // 这是当前实现的类型约定，不是通过 ROS 图自动发现消息类型。
  void addFeedback(const std::string & name, const std::string & topic)
  {
    const bool motion = topic.substr(topic.find_last_of('/') + 1) == "motion_state";
    auto stage = addStage(name, topic, motion ? "MotionState" : "Odometry");
    if (stage->subscribed)
    {
      return;
    }
    stage->subscribed = true;
    if (motion)
    {
      subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::MotionState>(topic, rclcpp::SensorDataQoS(), [this, stage](byd_custom_msgs::msg::MotionState::ConstSharedPtr msg)
      {
        update(stage, msg->v_car, msg->w_car);
        stage->v_lift = msg->v_lift;
        stage->lift_height = msg->lift_height;
        stage->w_shelf = msg->w_shelf;
        stage->yaw_shelf = msg->yaw_shelf;
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
  // 订阅最终或支路 ControlRes，仅观察 v/w；不会向底盘回发命令。
  void addControlRes(const std::string & name, const std::string & topic)
  {
    auto stage = addStage(name, topic, "ControlRes");
    if (stage->subscribed)
    {
      return;
    }
    stage->subscribed = true;
    subscriptions_.push_back(create_subscription<byd_custom_msgs::msg::ControlRes>(topic, rclcpp::SensorDataQoS(), [this, stage](byd_custom_msgs::msg::ControlRes::ConstSharedPtr msg)
    {
      update(stage, msg->v, msg->w);
    }));
  }
  // 按来源输出最新值、count、age 和 missing／invalid／stale／valid，MotionState 另附货架反馈。
  // age 用单调时钟，ros_time 用节点时钟；高频快照可能重复 count，不代表收到同频新数据。
  void snapshot(spdlog::level::level_enum level)
  {
    const auto steady = std::chrono::steady_clock::now();
    const auto ros_time = now().seconds();
    std::string frame;
    for (size_t i = 0; i < stages_.size(); ++i)
    {
      const auto & stage = stages_[i];
      const double age = stage->count ? std::chrono::duration<double>(steady - stage->received).count() : -1.0;
      const auto status = !stage->count ? "missing" : !stage->finite ? "invalid" : age > stale_seconds_ ? "stale" : "valid";
      frame += fmt::format(" [src={} stage={} topic={} vx={:.6f} wz={:.6f} count={} age={:.6f} status={} op={}", i, stage->name, stage->topic, stage->v, stage->w, stage->count, age, status, stage->op);
      if (stage->type == "MotionState")
      {
        frame += fmt::format(" v_lift={:.6f} lift_height={:.6f} w_shelf={:.6f} yaw_shelf={:.6f}", stage->v_lift, stage->lift_height, stage->w_shelf, stage->yaw_shelf);
      }
      frame += "]";
    }
    log_->log(level, "velocity ros_time={:.9f} mode={} schema=2{}", ros_time, mode_, frame);
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

// 独立速度链观测进程。按“解析后的话题＋消息类型”复用订阅，再按频率输出最新值快照。
// 返回码由调用环境用于区分成功／失败，进程启动本身不能作为业务功能验收。
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
