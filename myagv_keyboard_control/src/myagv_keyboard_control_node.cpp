#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>

#include "byd_custom_msgs/msg/control_res.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

class TerminalMode
{
public:
  TerminalMode()
  {
    if (!isatty(STDIN_FILENO)) {
      throw std::runtime_error("stdin is not a terminal");
    }

    if (tcgetattr(STDIN_FILENO, &original_termios_) == -1) {
      throw std::runtime_error("failed to read terminal attributes");
    }

    original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_flags_ == -1) {
      throw std::runtime_error("failed to read terminal flags");
    }

    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1) {
      throw std::runtime_error("failed to configure terminal attributes");
    }
    termios_changed_ = true;

    if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      restore();
      throw std::runtime_error("failed to configure non-blocking terminal input");
    }
    flags_changed_ = true;
  }

  ~TerminalMode()
  {
    restore();
  }

  TerminalMode(const TerminalMode &) = delete;
  TerminalMode & operator=(const TerminalMode &) = delete;

private:
  void restore()
  {
    if (flags_changed_) {
      (void)fcntl(STDIN_FILENO, F_SETFL, original_flags_);
      flags_changed_ = false;
    }
    if (termios_changed_) {
      (void)tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      termios_changed_ = false;
    }
  }

  termios original_termios_{};
  int original_flags_{0};
  bool termios_changed_{false};
  bool flags_changed_{false};
};

enum class Motion
{
  STOP,
  FORWARD,
  REVERSE,
  LEFT,
  RIGHT
};

}  // namespace

class MyAgvKeyboardControl : public rclcpp::Node
{
public:
  MyAgvKeyboardControl()
  : Node("myagv_keyboard_control")
  {
    const auto output_topic = declare_parameter<std::string>("output_topic", "/control_to_uart");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.2);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);

    validateParameters();

    publisher_ = create_publisher<byd_custom_msgs::msg::ControlRes>(output_topic, 10);
    last_direction_key_ = std::chrono::steady_clock::now();

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&MyAgvKeyboardControl::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing byd_custom_msgs/msg/ControlRes to %s at %.1f Hz",
      output_topic.c_str(), publish_rate_);
    RCLCPP_INFO(
      get_logger(),
      "Controls: W/Up forward, S/Down reverse, A/Left turn left, D/Right turn right, "
      "Space/X stop, Q quit");
    RCLCPP_INFO(
      get_logger(),
      "Speeds: linear %.3f m/s, angular %.3f rad/s, command timeout %.3f s",
      linear_speed_, angular_speed_, command_timeout_);
  }

  void stop()
  {
    setMotion(Motion::STOP, false);
    publishCommand();
  }

private:
  void validateParameters() const
  {
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than zero");
    }
    if (!std::isfinite(linear_speed_) || linear_speed_ < 0.0) {
      throw std::invalid_argument("linear_speed must be finite and non-negative");
    }
    if (!std::isfinite(angular_speed_) || angular_speed_ < 0.0) {
      throw std::invalid_argument("angular_speed must be finite and non-negative");
    }
    if (!std::isfinite(command_timeout_) || command_timeout_ < 0.0) {
      throw std::invalid_argument("command_timeout must be finite and non-negative");
    }
  }

  void timerCallback()
  {
    readKeyboard();
    applyCommandTimeout();
    publishCommand();
  }

  void readKeyboard()
  {
    char key = 0;
    while (true) {
      const ssize_t bytes_read = read(STDIN_FILENO, &key, 1);
      if (bytes_read == 1) {
        processKey(key);
        continue;
      }
      if (bytes_read == -1 && errno == EINTR) {
        continue;
      }
      if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to read keyboard input: %s", strerror(errno));
      }
      break;
    }
  }

  void processKey(char key)
  {
    if (escape_state_ == 1) {
      escape_state_ = key == '[' ? 2 : 0;
      return;
    }

    if (escape_state_ == 2) {
      escape_state_ = 0;
      switch (key) {
        case 'A':
          commandMotion(Motion::FORWARD);
          return;
        case 'B':
          commandMotion(Motion::REVERSE);
          return;
        case 'C':
          commandMotion(Motion::RIGHT);
          return;
        case 'D':
          commandMotion(Motion::LEFT);
          return;
        default:
          return;
      }
    }

    if (key == '\x1b') {
      escape_state_ = 1;
      return;
    }

    const char normalized = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    switch (normalized) {
      case 'w':
        commandMotion(Motion::FORWARD);
        break;
      case 's':
        commandMotion(Motion::REVERSE);
        break;
      case 'a':
        commandMotion(Motion::LEFT);
        break;
      case 'd':
        commandMotion(Motion::RIGHT);
        break;
      case 'x':
      case ' ':
        setMotion(Motion::STOP, true);
        break;
      case 'q':
        setMotion(Motion::STOP, true);
        publishCommand();
        RCLCPP_INFO(get_logger(), "Quit requested; published stop command");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  void commandMotion(Motion motion)
  {
    last_direction_key_ = std::chrono::steady_clock::now();
    setMotion(motion, true);
  }

  void setMotion(Motion motion, bool log_change)
  {
    if (motion_ == motion) {
      return;
    }

    motion_ = motion;
    switch (motion_) {
      case Motion::FORWARD:
        current_v_ = linear_speed_;
        current_w_ = 0.0;
        break;
      case Motion::REVERSE:
        current_v_ = -linear_speed_;
        current_w_ = 0.0;
        break;
      case Motion::LEFT:
        current_v_ = 0.0;
        current_w_ = angular_speed_;
        break;
      case Motion::RIGHT:
        current_v_ = 0.0;
        current_w_ = -angular_speed_;
        break;
      case Motion::STOP:
        current_v_ = 0.0;
        current_w_ = 0.0;
        break;
    }

    if (log_change) {
      RCLCPP_INFO(get_logger(), "Command changed: v=%.3f, w=%.3f", current_v_, current_w_);
    }
  }

  void applyCommandTimeout()
  {
    if (motion_ == Motion::STOP || command_timeout_ == 0.0) {
      return;
    }

    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_direction_key_).count();
    if (elapsed > command_timeout_) {
      setMotion(Motion::STOP, false);
      RCLCPP_WARN(get_logger(), "Keyboard command timed out; stopping chassis");
    }
  }

  void publishCommand()
  {
    byd_custom_msgs::msg::ControlRes msg;
    msg.v = current_v_;
    msg.w = current_w_;
    msg.v_lift = 0.0;
    msg.w_rotation = 0.0;
    publisher_->publish(msg);
  }

  TerminalMode terminal_mode_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double publish_rate_{50.0};
  double linear_speed_{0.2};
  double angular_speed_{0.5};
  double command_timeout_{0.5};
  double current_v_{0.0};
  double current_w_{0.0};
  Motion motion_{Motion::STOP};
  int escape_state_{0};
  std::chrono::steady_clock::time_point last_direction_key_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MyAgvKeyboardControl>();
    rclcpp::spin(node);
    if (rclcpp::ok()) {
      node->stop();
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("myagv_keyboard_control"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
