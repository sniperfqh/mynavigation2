// 中文注释：实现终端键盘读取、目标速度状态机、加减速斜坡和 MYAGV 底盘指令周期发布。
#include <algorithm>
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

// 中文注释：ControlRes 是底盘最终控制消息，rclcpp 提供节点、参数、日志、定时器和发布器接口。
#include "byd_custom_msgs/msg/control_res.hpp"
#include "rclcpp/rclcpp.hpp"

// 中文注释：匿名命名空间限制终端工具、运动枚举和速度算法只在本编译单元内可见。
namespace
{

// 中文注释：以 RAII 方式打开并配置终端，析构时恢复原始属性和文件状态标志。
class TerminalMode
{
public:
  // 中文注释：优先打开参数指定设备；失败且标准输入为终端时，复制标准输入描述符作为回退。
  explicit TerminalMode(const std::string & input_device) {
    input_fd_ = open(input_device.c_str(), O_RDONLY | O_NOCTTY);
    if (input_fd_ == -1 && isatty(STDIN_FILENO)) {
      input_fd_ = dup(STDIN_FILENO);
    }
    // 中文注释：没有可读终端时立即失败，避免节点运行却永远收不到安全控制按键。
    if (input_fd_ == -1) {
      throw std::runtime_error("failed to open input device '" + input_device + "' and stdin is not a terminal");
    }

    // 中文注释：保存终端原始属性，后续退出时必须完整恢复。
    if (tcgetattr(input_fd_, &original_termios_) == -1) {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to read terminal attributes");
    }

    // 中文注释：保存原文件状态标志，以便在析构时撤销非阻塞模式。
    original_flags_ = fcntl(input_fd_, F_GETFL, 0);
    if (original_flags_ == -1) {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to read terminal flags");
    }

    // 中文注释：关闭规范行缓冲和回显，使单个按键无需回车即可被节点读取。
    termios raw = original_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    // 中文注释：VMIN／VTIME 均为零，read() 在没有字符时不会阻塞定时器线程。
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    // 中文注释：应用原始输入模式；失败时关闭文件描述符并终止节点构造。
    if (tcsetattr(input_fd_, TCSANOW, &raw) == -1) {
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to configure terminal attributes");
    }
    termios_changed_ = true;

    // 中文注释：叠加 O_NONBLOCK，保证一个控制周期内只消费当前已经到达的按键字节。
    if (fcntl(input_fd_, F_SETFL, original_flags_ | O_NONBLOCK) == -1) {
      restore();
      close(input_fd_);
      input_fd_ = -1;
      throw std::runtime_error("failed to configure non-blocking terminal input");
    }
    flags_changed_ = true;
  }

  // 中文注释：先恢复终端状态，再关闭本对象持有的描述符，避免污染用户 Shell。
  ~TerminalMode() {
    restore();
    if (input_fd_ != -1) {
      close(input_fd_);
      input_fd_ = -1;
    }
  }

  // 中文注释：终端描述符具有唯一所有权，禁止复制对象导致重复恢复或重复关闭。
  TerminalMode(const TerminalMode &) = delete;
  TerminalMode & operator=(const TerminalMode &) = delete;

  // 中文注释：向键盘读取循环提供已经配置为非阻塞模式的终端描述符。
  int inputFd() const {
    return input_fd_;
  }

private:
  // 中文注释：按实际完成的配置步骤逆向恢复；可被构造失败路径和析构安全重复调用。
  void restore() {
    if (flags_changed_) {
      (void)fcntl(input_fd_, F_SETFL, original_flags_);
      flags_changed_ = false;
    }
    if (termios_changed_) {
      (void)tcsetattr(input_fd_, TCSANOW, &original_termios_);
      termios_changed_ = false;
    }
  }

  // 中文注释：保存终端描述符、原始 termios、原文件标志和两类配置是否已经生效。
  int input_fd_{-1};
  termios original_termios_{};
  int original_flags_{0};
  bool termios_changed_{false};
  bool flags_changed_{false};
};

class SCurvePlanner {
public:
    /**
     * @brief 构造函数
     * @param dt 控制循环的时间步长（秒），例如 0.01 (100Hz)
     * @param jm 最大加加速度 (m/s^3)
     * @param am 最大加速度 (m/s^2)
     * @param sm 最大速度 (m/s)
     */
    SCurvePlanner(double dt, double sm, 
                  double norm_accel_amax, double norm_accel_jmax, 
                  double norm_decel_amax, double norm_decel_jmax)
                //   double emrg_amax, double emrg_jmax)
        : dt_(dt), sm_(sm), 
          am_(norm_accel_amax), jm_(norm_accel_jmax),
          norm_accel_amax_(norm_accel_amax), norm_accel_jmax_(norm_accel_jmax), 
          norm_decel_amax_(norm_decel_amax), norm_decel_jmax_(norm_decel_jmax), 
          emrg_amax_(8.0), emrg_jmax_(8.0),
          sc_(0.0), ac_(0.0), jc_(0.0), d_(0.0) {}

    void reset(){
        sc_ = 0.0;
        ac_ = 0.0;
        jc_ = 0.0;
        d_  = 0.0;
    }

    /**
     * @brief 设置目标方向
     * @param d 前进后退指标：+1(前进), 0(停止), -1(后退)
     */
    void setDirection(double d, bool emrgstate = false) {
        d_ = std::max(-1.0, std::min(1.0, d)); // 确保 d 在 [-1, 0, 1] 之间
        // 2. 【核心逻辑】换向过零保护
        // 如果当前速度 sc_ 和目标速度 target_vel 符号相反（说明正在换向）
        // 并且当前速度绝对值大于极小阈值（说明还没完全停稳）
        // 则强制将目标速度设为 0，让 S型曲线先完成刹车
        if (sc_ * d_ < 0.0 && std::abs(sc_) > 1e-6) {
            d_ = 0.0; 
        }
        if(prestate_emrg == true && emrgstate == false){
            am_ = norm_accel_amax_;   
            jm_ = norm_accel_jmax_;   
            prestate_emrg = false;
        }
        if(prestate_emrg == false && emrgstate == true){
            am_ = emrg_amax_;  
            jm_ = emrg_jmax_;   
            prestate_emrg = true;
        }
    }

    /**
     * @brief 更新状态（需要在控制循环中周期调用）
     * @return 返回当前规划出的速度 sc
     */
    double update() {
        double vg = d_ * sm_; // 目标速度

        // 1. 如果已经到达目标状态（速度为0且方向为0，或速度等于目标速度且加速度为0）
        if (std::abs(sc_ - vg) < 1e-6 ) {
            sc_ = vg;
            ac_ = 0.0;
            jc_ = 0.0;
            return sc_;
        }
        
        if (std::abs(ac_) < 1e-6) {
            // 【情况 A】当前加速度为 0
            // 此时无法通过 v 和 a 的点乘判断，必须根据“速度误差”来决定：
            // 如果当前速度和目标速度方向一致，且还没达到目标速度 -> 准备起步加速
            // 否则（方向相反，或需要减速） -> 准备刹车减速
            bool is_speeding_up = (sc_ * vg >= 0.0) && (std::abs(sc_) < std::abs(vg));
            
            if (is_speeding_up) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        } else {
            // 【情况 B】当前加速度不为 0
            // 通过速度(sc_)和加速度(ac_)的点乘判断：
            // 同向 (点乘 > 0) 为加速，反向 (点乘 < 0) 为减速
            if (sc_ * ac_ >= 0.0) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        }

        // 2. 计算速度误差
        double vel_err = vg - sc_;
        double dr = vel_err/(std::abs(vel_err)+1e-9);

        // 3. 决定加加速度 jc 的方向
        // 如果当前加速度方向与速度误差方向一致，且加速度过大，需要减小加速度
        // 如果当前加速度方向与速度误差方向相反，或者加速度不够，需要增加加速度
        
        if ( dr*ac_>0.0 && std::abs( dr*vel_err - 0.5 * dr*ac_ * std::abs(ac_/jm_) )<= 1e-4) {
            // 加速阶段或匀速阶段
            jc_ = -dr*jm_; // 趋近目标，加速度减小
        } else {
            // 减加速度阶段（加加速度方向与目标速度方向相反）
            jc_ =  dr*jm_; // 目标较远，加速度增大
        }

        // 4. 积分更新（欧拉法）
        ac_ += jc_ * dt_;
        
        // 5. 限制加速度范围
        ac_ = std::max(-am_, std::min(am_, ac_));
        if( dr*ac_>0.0 && dr*jc_<0.0){
          if(dr > 0) ac_ = std::max(ac_, 0.0);
          if(dr < 0) ac_ = std::min(ac_, 0.0);
        }

        sc_ += ac_ * dt_;

        // 6. 限制速度范围并处理过零
        if (dr >= 1e-6) {
            sc_ = std::min(sc_, vg);
        } else if (dr <= -1e-6) {
            sc_ = std::max(sc_, vg);
        }
        else{
          sc_ = vg;
        }
        sc_ = std::max(-sm_, std::min(sm_, sc_));
        return sc_;
    }

    // 获取当前状态（方便调试）
    double getVelocity() const { return sc_; }
    double getAcceleration() const { return ac_; }
    double getJerk() const { return jc_; }

private:
    
    bool prestate_emrg = false;  // 上一状态是否紧急
    double dt_;   // 时间步长
    double sm_;   // 最大速度
    double am_;   // 最大加速度
    double jm_;   // 最大加加速度

    double norm_accel_amax_;  // 正常加速最大加速度
    double norm_accel_jmax_;  // 正常加速最大加加速度
    double norm_decel_amax_;  // 正常减速最大加速度
    double norm_decel_jmax_;  // 正常减速最大加加速度
    double emrg_amax_;  // 紧急最大加速度
    double emrg_jmax_;  // 紧急最大加加速度

    double sc_;   // 当前速度     speed curent
    double ac_;   // 当前加速度
    double jc_;   // 当前加加速度
    double d_;       // 方向指标
};

// 中文注释：运动状态只表达停止、直行和原地旋转，差速底盘不同时输出线速度与角速度目标。
enum class Motion
{
  STOP,
  FORWARD,
  REVERSE,
  LEFT,
  RIGHT
};

// 中文注释：速度绝对值低于该阈值时按零处理，避免浮点残差阻塞模式切换。
constexpr double kVelocityEpsilon = 1e-9;

// 中文注释：在一个 dt 周期内把 current 向 target 推进，并根据增速或减速选择不同变化率。
double approachVelocity(const double current, const double target, const double acceleration_limit, const double deceleration_limit, const double dt) {
  // 中文注释：已经到达目标时直接返回目标值，消除长期累积的浮点微小误差。
  if (std::abs(target - current) <= kVelocityEpsilon) {
    return target;
  }

  // 中文注释：换向和速度幅值下降均使用减速度限制，只有同方向增大幅值才使用加速度限制。
  const bool changing_direction = current * target < 0.0;
  const bool increasing_magnitude = std::abs(target) > std::abs(current);
  const double rate_limit = changing_direction || !increasing_magnitude ? deceleration_limit : acceleration_limit;
  const double max_delta = rate_limit * dt;
  // 中文注释：分别限制向上和向下变化量，并用 min／max 防止单周期越过目标。
  if (target > current) {
    return std::min(current + max_delta, target);
  }
  return std::max(current - max_delta, target);
}

}  // namespace
// 中文注释：匿名命名空间结束，下面定义对 ROS 2 暴露的节点类。

// 中文注释：键盘遥控主节点负责参数校验、按键解析、速度平滑和 ControlRes 发布。
class MyAgvKeyboardControl : public rclcpp::Node
{
public:
  // 中文注释：声明全部启动参数，创建终端、发布器和固定频率 Wall Timer。
  MyAgvKeyboardControl() : Node("myagv_keyboard_control") {
    // 中文注释：输入输出参数确定终端数据源和底盘控制 Topic。
    const auto input_device = declare_parameter<std::string>("input_device", "/dev/tty");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/control_to_uart");
    // 中文注释：速度、斜坡和松键超时参数在启动时读取，当前不支持运行期动态更新。
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    linear_speed_ = declare_parameter<double>("linear_speed", 0.2);
    angular_speed_ = declare_parameter<double>("angular_speed", 0.5);
    linear_accel_limit_ = declare_parameter<double>("linear_accel_limit", 0.4);
    linear_decel_limit_ = declare_parameter<double>("linear_decel_limit", 0.8);
    angular_accel_limit_ = declare_parameter<double>("angular_accel_limit", 1.0);
    angular_decel_limit_ = declare_parameter<double>("angular_decel_limit", 2.0);
    linear_accel_jerk_limit_ = declare_parameter<double>("linear_accel_jerk_limit", 0.4);
    linear_decel_jerk_limit_ = declare_parameter<double>("linear_decel_jerk_limit", 0.8);
    angular_accel_jerk_limit_ = declare_parameter<double>("angular_accel_jerk_limit", 1.0);
    angular_decel_jerk_limit_ = declare_parameter<double>("angular_decel_jerk_limit", 2.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.5);

    // 中文注释：在创建定时器和硬件输出接口前拒绝非法数值，避免除零或失控速度。
    validateParameters();

    double dt = 1.0/publish_rate_;
    linear_planner_ = std::make_unique<SCurvePlanner>(dt, 
      linear_speed_, linear_accel_limit_, linear_accel_jerk_limit_, linear_decel_limit_, linear_decel_jerk_limit_);
    angular_planner_ = std::make_unique<SCurvePlanner>(dt, 
      angular_speed_, angular_accel_limit_, angular_accel_jerk_limit_, angular_decel_limit_, angular_decel_jerk_limit_);

    // 中文注释：打开真实终端并创建深度为 10 的底盘控制发布器。
    terminal_mode_ = std::make_unique<TerminalMode>(input_device);
    publisher_ = create_publisher<byd_custom_msgs::msg::ControlRes>(output_topic, 10);
    // 中文注释：初始化方向键时间和速度积分时间，首个周期不会得到异常大的 dt。
    last_direction_key_ = std::chrono::steady_clock::now();
    last_update_time_ = last_direction_key_;

    // 中文注释：由发布频率计算 Wall Timer 周期，每次回调依次读取、超时、平滑并发布。
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(period, std::bind(&MyAgvKeyboardControl::timerCallback, this));

    // 中文注释：启动日志打印接口、按键、目标速度和加减速限制，便于联调确认参数是否生效。
    RCLCPP_INFO(get_logger(), "Publishing byd_custom_msgs/msg/ControlRes to %s at %.1f Hz", output_topic.c_str(), publish_rate_);
    RCLCPP_INFO(get_logger(), "Controls: W/Up forward, S/Down reverse, A/Left turn left, D/Right turn right, " "Space/X stop, Q quit");
    RCLCPP_INFO(get_logger(), "Speeds: linear %.3f m/s, angular %.3f rad/s, command timeout %.3f s", linear_speed_, angular_speed_, command_timeout_);
    RCLCPP_INFO(get_logger(), "Rate limits: linear accel/decel %.3f/%.3f m/s^2, angular accel/decel " "%.3f/%.3f rad/s^2", linear_accel_limit_, linear_decel_limit_, angular_accel_limit_, angular_decel_limit_);
  }

  // 中文注释：提供安全收口入口，立即清零当前值和目标值并发布一帧停车指令。
  void stop() {
    hardStop();
    publishCommand();
  }

private:
  // 中文注释：校验所有数值参数的有限性和取值范围，失败时通过异常终止节点启动。
  void validateParameters() const {
    // 中文注释：发布频率必须为正，否则无法计算有效定时器周期。
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
      throw std::invalid_argument("publish_rate must be greater than zero");
    }
    // 中文注释：目标速度允许为零，但不能为负数、无穷大或 NaN；方向由运动状态决定。
    if (!std::isfinite(linear_speed_) || linear_speed_ < 0.0) {
      throw std::invalid_argument("linear_speed must be finite and non-negative");
    }
    if (!std::isfinite(angular_speed_) || angular_speed_ < 0.0) {
      throw std::invalid_argument("angular_speed must be finite and non-negative");
    }
    // 中文注释：四个变化率必须严格为正，确保速度能够从任意状态收敛到目标。
    if (!std::isfinite(linear_accel_limit_) || linear_accel_limit_ <= 0.0) {
      throw std::invalid_argument("linear_accel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(linear_decel_limit_) || linear_decel_limit_ <= 0.0) {
      throw std::invalid_argument("linear_decel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_accel_limit_) || angular_accel_limit_ <= 0.0) {
      throw std::invalid_argument("angular_accel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_decel_limit_) || angular_decel_limit_ <= 0.0) {
      throw std::invalid_argument("angular_decel_limit must be finite and greater than zero");
    }
    if (!std::isfinite(linear_accel_jerk_limit_) || linear_accel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("linear_accel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(linear_decel_jerk_limit_) || linear_decel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("linear_decel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_accel_jerk_limit_) || angular_accel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("angular_accel_jerk_limit must be finite and greater than zero");
    }
    if (!std::isfinite(angular_decel_jerk_limit_) || angular_decel_jerk_limit_ <= 0.0) {
      throw std::invalid_argument("angular_decel_jerk_limit must be finite and greater than zero");
    }
    // 中文注释：超时允许为零，零表示关闭松键自动停车，其余值必须为有限非负数。
    if (!std::isfinite(command_timeout_) || command_timeout_ < 0.0) {
      throw std::invalid_argument("command_timeout must be finite and non-negative");
    }
  }

  // 中文注释：单个控制周期的数据流为“读键盘→判定松键超时→计算 dt→平滑速度→发布”。
  void timerCallback() {
    readKeyboard();
    const auto now = std::chrono::steady_clock::now();
    applyCommandTimeout(now);
    
    // 线性速度规划
    // 中文注释：使用稳态时钟计算真实周期，并把异常延迟限制到最多两个标称周期。
    // const double elapsed = std::chrono::duration<double>(now - last_update_time_).count();
    // last_update_time_ = now;
    // const double dt = std::clamp(elapsed, 0.0, 2.0 / publish_rate_);
    // // RCLCPP_INFO(get_logger(), "msg print time: dt=%.3f .", dt);
    // updateSmoothedCommand(dt);             

    // S型曲线速度规划           
    current_v_ = linear_planner_->update();           // S型曲线速度规划
    current_w_ = angular_planner_->update();          // S型曲线速度规划
    publishCommand();
  }

  // 中文注释：在非阻塞终端上一次性读取所有已到达字节，并把每个字节交给按键状态机。
  void readKeyboard() {
    char key = 0;
    while (true) {
      const ssize_t bytes_read = read(terminal_mode_->inputFd(), &key, 1);
      // 中文注释：成功读取一个字节后继续循环，避免方向键转义序列跨多个控制周期处理。
      if (bytes_read == 1) {
        processKey(key);
        continue;
      }
      // 中文注释：系统调用被信号中断时立即重试，不把 EINTR 当作终端故障。
      if (bytes_read == -1 && errno == EINTR) {
        continue;
      }
      // 中文注释：EAGAIN／EWOULDBLOCK 只表示当前没有新字符，其他错误按两秒节流记录。
      if (bytes_read == -1 && errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to read keyboard input: %s", strerror(errno));
      }
      break;
    }
  }

  // 中文注释：解析普通 WASD、停车／退出键以及方向键的 ESC [ A/B/C/D 三字节序列。
  void processKey(char key) {
    // 中文注释：收到 ESC 后等待左方括号，非标准序列直接复位。
    if (escape_state_ == 1) {
      escape_state_ = key == '[' ? 2 : 0;
      return;
    }

    // 中文注释：方向键末字节 A/B/C/D 分别映射为前进、后退、右转和左转。
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

    // 中文注释：ESC 是方向键转义序列起点，本字节不直接改变运动目标。
    if (key == '\x1b') {
      escape_state_ = 1;
      return;
    }

    // 中文注释：普通字符统一转为小写，使 WASD 和 wasd 行为一致。
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
        // 中文注释：Space／X 只把目标切到零，实际速度仍按减速度限制平滑下降。
        setMotion(Motion::STOP, true);
        break;
      case 'q':
        // 中文注释：Q 是显式紧急退出入口，绕过斜坡立即清零、发布停车并关闭 ROS。
        hardStop();
        publishCommand();
        RCLCPP_INFO(get_logger(), "Quit requested; published stop command");
        rclcpp::shutdown();
        break;
      default:
        break;
    }
  }

  // 中文注释：方向键输入同时刷新松键超时时间，并更新期望运动状态。
  void commandMotion(Motion motion) {
    last_direction_key_ = std::chrono::steady_clock::now();
    setMotion(motion, true);
  }

  // 中文注释：把离散运动状态转换为互斥的线速度和角速度目标。
  void setMotion(Motion motion, bool log_change) {
    // 中文注释：重复键只刷新 commandMotion() 中的时间，不重复打印目标变化日志。
    if (motion_ == motion) {
      return;
    }

    motion_ = motion;
    // 中文注释：直行目标只设置 v，原地转向目标只设置 w，停止目标两轴均为零。
    switch (motion_) {
      case Motion::FORWARD:
        target_v_ = linear_speed_;
        target_w_ = 0.0;
        linear_planner_->setDirection(1.0);
        angular_planner_->setDirection(0.0);
        break;
      case Motion::REVERSE:
        target_v_ = -linear_speed_;
        target_w_ = 0.0;
        linear_planner_->setDirection(-1.0);
        angular_planner_->setDirection(0.0);
        break;
      case Motion::LEFT:
        target_v_ = 0.0;
        target_w_ = angular_speed_;
        linear_planner_->setDirection(0.0);
        angular_planner_->setDirection(1.0);
        break;
      case Motion::RIGHT:
        target_v_ = 0.0;
        target_w_ = -angular_speed_;
        linear_planner_->setDirection(0.0);
        angular_planner_->setDirection(-1.0);
        break;
      case Motion::STOP:
        target_v_ = 0.0;
        target_w_ = 0.0;
        linear_planner_->setDirection(0.0);
        angular_planner_->setDirection(0.0);
        break;
    }

    // 中文注释：人工切换记录目标值；超时触发停车时可关闭重复日志。
    if (log_change) {
      RCLCPP_INFO(get_logger(), "Target changed: v=%.3f, w=%.3f", target_v_, target_w_);
    }
  }

  // 中文注释：同时清零状态、目标和当前输出，用于 Q 退出及显式安全收口。
  void hardStop() {
    motion_ = Motion::STOP;
    target_v_ = 0.0;
    target_w_ = 0.0;
    current_v_ = 0.0;
    current_w_ = 0.0;
  }

  // 中文注释：判断当前输出是否必须先回零，覆盖同轴反向和直行／原地转向互切。
  bool transitionRequiresStop() const {
    const bool linear_sign_change = current_v_ * target_v_ < 0.0;
    const bool angular_sign_change = current_w_ * target_w_ < 0.0;
    const bool linear_to_angular = std::abs(current_v_) > kVelocityEpsilon && std::abs(target_w_) > kVelocityEpsilon;
    const bool angular_to_linear = std::abs(current_w_) > kVelocityEpsilon && std::abs(target_v_) > kVelocityEpsilon;
    return linear_sign_change || angular_sign_change || linear_to_angular || angular_to_linear;
  }

  // 中文注释：根据当前周期 dt 更新实际输出；需要过零时暂时把两轴有效目标都设为零。
  void updateSmoothedCommand(const double dt) {
    double effective_target_v = target_v_;
    double effective_target_w = target_w_;
    if (transitionRequiresStop()) {
      effective_target_v = 0.0;
      effective_target_w = 0.0;
    }

    // 中文注释：线速度与角速度分别应用各自的加速和减速限制。
    current_v_ = approachVelocity(current_v_, effective_target_v, linear_accel_limit_, linear_decel_limit_, dt);
    current_w_ = approachVelocity(current_w_, effective_target_w, angular_accel_limit_, angular_decel_limit_, dt);
  }

  // 中文注释：把“长时间未收到方向键”解释为松键，并仅把目标切零以触发平滑停车。
  void applyCommandTimeout(const std::chrono::steady_clock::time_point now) {
    // 中文注释：已经停止或配置为零超时时，不执行自动停车判定。
    if (motion_ == Motion::STOP || command_timeout_ == 0.0) {
      return;
    }

    const auto elapsed = std::chrono::duration<double>(now - last_direction_key_).count();
    // 中文注释：超时只触发一次状态变化，后续周期继续把当前速度平滑推进到零。
    if (elapsed > command_timeout_) {
      setMotion(Motion::STOP, false);
      RCLCPP_WARN(get_logger(), "Keyboard command timed out; smoothly decelerating chassis");
    }
  }

  // 中文注释：把当前平滑后的 v／w 写入底盘消息，升降和旋转机构通道固定清零。
  void publishCommand() {
    byd_custom_msgs::msg::ControlRes msg;
    msg.v = current_v_;
    msg.w = current_w_;
    msg.v_lift = 0.0;
    msg.w_rotation = 0.0;
    publisher_->publish(msg);
    // RCLCPP_INFO(get_logger(), "msg.v: v=%.3f, msg.w: w=%.3f", current_v_, current_w_);
    // RCLCPP_INFO(get_logger(), "current : v=%.3f, acc=%.3f, jerk=%0.3f", linear_planner_->getVelocity(), linear_planner_->getAcceleration(), linear_planner_->getJerk());
  }

  // 中文注释：ROS 资源与终端资源的生命周期均由智能指针管理。
  std::unique_ptr<TerminalMode> terminal_mode_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 中文注释：启动参数缓存；节点运行期间控制循环直接读取这些成员。
  double publish_rate_{50.0};
  double linear_speed_{0.2};
  double angular_speed_{0.5};
  double linear_accel_limit_{0.4};
  double linear_decel_limit_{0.8};
  double angular_accel_limit_{1.0};
  double angular_decel_limit_{2.0};
  double linear_accel_jerk_limit_{0.4};
  double linear_decel_jerk_limit_{0.8};
  double angular_accel_jerk_limit_{1.0};
  double angular_decel_jerk_limit_{2.0};
  double command_timeout_{0.5};

  std::unique_ptr<SCurvePlanner> linear_planner_;
  std::unique_ptr<SCurvePlanner> angular_planner_;

  // 中文注释：target_* 是按键期望值，current_* 是经过斜坡限制后实际发布的值。
  double target_v_{0.0};
  double target_w_{0.0};
  double current_v_{0.0};
  double current_w_{0.0};
  // 中文注释：保存离散运动状态、方向键转义序列阶段和两个稳态时钟时间点。
  Motion motion_{Motion::STOP};
  int escape_state_{0};
  std::chrono::steady_clock::time_point last_direction_key_{};
  std::chrono::steady_clock::time_point last_update_time_{};
};

// 中文注释：初始化并运行键盘节点；正常分支负责停车，异常分支负责记录故障并返回非零退出码。
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0; // 记录退出码
  try {
    // 中文注释：节点构造失败时进入 catch；构造成功后由单线程执行器处理定时器。
    auto node = std::make_shared<MyAgvKeyboardControl>();
    rclcpp::spin(node);
    // 中文注释：执行器在 ROS 仍有效时返回，额外发布一帧立即停车指令。
    RCLCPP_WARN(node->get_logger(), "Node shutting down, publishing final STOP command...");
    node->stop();
  } catch (const std::exception & error) {
    // 中文注释：终端或参数初始化异常返回非零退出码，便于 Launch 和运维系统识别失败。
    RCLCPP_FATAL(rclcpp::get_logger("myagv_keyboard_control"), "%s", error.what());
    exit_code = 1;
  }
  // 中文注释：正常路径释放 ROS 2 全局资源并返回成功。
  rclcpp::shutdown();
  return exit_code;
}
