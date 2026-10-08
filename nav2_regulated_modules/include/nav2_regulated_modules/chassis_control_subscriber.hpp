// 底盘控制输入与 S 曲线限速接口。遥控指令先按运动状态和超时规则校验，再平滑为底盘输出；本文件同时定义配置与内部状态。

#ifndef NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_
#define NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "byd_custom_msgs/msg/chassis_control.hpp"
#include "byd_custom_msgs/msg/control_res.hpp"
#include "nav2_regulated_modules/motion_state_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_regulated_modules
{

// 对单轴速度按加速度与加加速度约束渐变；线速度和角速度各使用一个实例。
class SCurvePlanner {
public:
    // dt 为控制周期；sm 为速度上限，后续四项分别限制正常加速与减速的幅值和变化率。
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

    // 清空控制或检查器历史状态，避免跨任务沿用上次进度。
    void reset(){
        sc_ = 0.0;
        ac_ = 0.0;
        jc_ = 0.0;
        d_  = 0.0;
    }

    // 以当前速度初始化 S 曲线，并清零加速度；用于切换控制阶段。
    void init(double sc){
        sc_ = sc;
        ac_ = 0.0;
    }

    // 设置目标速度方向和加减速约束；速度反向时先收敛到零。
    void setDirection(double sm, double am, bool emrgstate = false) {
        if(std::abs(sm) < 1e-3){
            d_ = 0.0;
        } else {
            sm_ = std::abs(sm);
            am_ = std::abs(am);
            d_ = std::max(-1.0, std::min(1.0, sm/sm_));
        }
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

    // 每周期推进一次 S 曲线状态，并返回新的目标速度。
    double update() {
        double vg = d_ * sm_;

        if (std::abs(sc_ - vg) <= 1e-6 ) {
            sc_ = vg;
            ac_ = 0.0;
            jc_ = 0.0;
            return sc_;
        }
        
        if (std::abs(ac_) < 1e-6) {
            bool is_speeding_up = (sc_ * vg >= 0.0) && (std::abs(sc_) < std::abs(vg));
            
            if (is_speeding_up) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        } else {
            if (sc_ * ac_ >= 0.0) {
                am_ = norm_accel_amax_;
                jm_ = norm_accel_jmax_;
            } else {
                am_ = norm_decel_amax_;
                jm_ = norm_decel_jmax_;
            }
        }

        double vel_err = vg - sc_;
        double dr = 0;
        if( vel_err < -1e-6 ) dr = -1.0;
        if( vel_err >  1e-6 ) dr =  1.0;

        
        if ( dr*ac_>=0.0 && std::abs(vel_err)-0.5*std::abs(ac_*ac_/jm_) <= 1e-4) {
            if( std::abs(dr) > 1e-6){
              jc_ =  -dr*jm_;
            } else {
              if (ac_ > 0.0) jc_ = -jm_;  
              if (ac_ < 0.0) jc_ =  jm_;  
            }
        } else {
            if( std::abs(dr) > 1e-6){
              jc_ =  dr*jm_;
            }
            else{
              if (ac_ > 0.0) jc_ =  jm_;
              if (ac_ < 0.0) jc_ = -jm_;
            }
            
        }

        double tmp = ac_;
        ac_ += jc_ * dt_;
        
        ac_ = std::max(-am_, std::min(am_, ac_));
        if( dr*ac_>=0.0 && dr*jc_<=0.0 )
        {
          if( jc_<0.0 ) {
            ac_ = std::max(ac_, 0.0);
          }
          if( jc_>0.0 ) {
            ac_ = std::min(ac_, 0.0);
          }
        }

        sc_ += 0.5*(tmp + ac_)*dt_ ;

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

    // 读取本轮 S 曲线输出速度。
    double getVelocity() const { return sc_; }
    // 读取本轮 S 曲线输出加速度。
    double getAcceleration() const { return ac_; }
    // 读取本轮 S 曲线输出加加速度。
    double getJerk() const { return jc_; }

private:
    
    bool prestate_emrg = false;
    double dt_;
    double sm_;
    double am_;
    double jm_;

    double norm_accel_amax_;
    double norm_accel_jmax_;
    double norm_decel_amax_;
    double norm_decel_jmax_;
    double emrg_amax_;
    double emrg_jmax_;

    double sc_;
    double ac_;
    double jc_;
    double d_;
};

// 底盘指令入口、输出与安全限幅；速度和加速度参数分别对应线运动及旋转。
struct ChassisControlConfig
{
  std::string input_topic{"/downstream/chassis_control"};
  std::string output_topic{"/control_to_uart"};
  double motion_state_timeout{0.2};
  double command_timeout{0.15};
  double publish_rate{100.0};
  double default_linear_speed_max{1.5};
  double linear_speed_max = 0.3;
  double default_angular_speed_max{0.5};
  double angular_speed_max = 0.3;
  double default_linear_accel_max{2.0};
  double linear_accel_max{3.0};
  double linear_decel_max{4.0};
  double default_angular_accel_max{1.5};
  double angular_accel_max{2.0};
  double angular_decel_max{3.0};
  double linear_accel_jerk_max{6.0};
  double linear_decel_jerk_max{8.0};
  double angular_accel_jerk_max{4.0};
  double angular_decel_jerk_max{6.0};
};

class ChassisControlSubscriber
{
  public:
  // 创建底盘控制订阅、运动状态检查和周期输出。
  ChassisControlSubscriber(nav2_util::LifecycleNode & node, MotionStateSubscriber & motion_state_subscriber, ChassisControlConfig config);
  // 激活订阅和周期输出；重新进入运行态前清理旧指令。
  void activate();
  // 停用控制输出并发布停车指令，防止旧目标继续生效。
  void deactivate();
  // 清空控制或检查器历史状态，避免跨任务沿用上次进度。
  void reset();

private:
  // 缓存最近一次已解析指令；超时和运动状态检查先于周期输出。
  struct TargetCommand
  {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    double linear_acceleration{1.0};
    double angular_acceleration{1.0};
    uint8_t operation{0};
  };

  // 解析底盘控制消息并更新目标运动状态；无效输入不应绕过安全停车。
  void onChassisControl(const byd_custom_msgs::msg::ChassisControl::ConstSharedPtr message);
  // 在互斥锁下处理超时、运动状态及加减速，再发布本周期控制值。
  void processControlCommand();
  // 清除锁保护的控制目标与时间状态；调用方须已持有互斥锁。
  void clearControlStateLocked();
  // 将限幅后的线角速度转换为底盘控制消息并发布。
  void publishControl(double linear_velocity, double angular_velocity);
  // 通过统一发布路径输出零线速度和零角速度。
  void publishZero();
  // 校验控制参数的数值范围及话题设置；失败时拒绝进入运行态。
  bool validateConfig() const;

  nav2_util::LifecycleNode & node_;
  MotionStateSubscriber & motion_state_subscriber_;
  ChassisControlConfig config_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  TargetCommand target_command_;
  std::chrono::steady_clock::time_point last_command_tmie_;
  bool active_{false};
  bool has_command_{false};
  bool is_remote_control_{false};
  bool publisher_conflict_{false};

  SCurvePlanner linear_planner_;
  SCurvePlanner angular_planner_;

  rclcpp::Subscription<byd_custom_msgs::msg::ChassisControl>::SharedPtr subscription_;
  rclcpp::Publisher<byd_custom_msgs::msg::ControlRes>::SharedPtr publisher_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__CHASSIS_CONTROL_SUBSCRIBER_HPP_
