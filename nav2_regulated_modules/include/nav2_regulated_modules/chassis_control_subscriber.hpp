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

class SCurvePlanner {
public:
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

    void init(double sc){
        sc_ = sc;
        ac_ = 0.0;
    }

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

    double getVelocity() const { return sc_; }
    double getAcceleration() const { return ac_; }
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

struct ChassisControlConfig
{
  std::string input_topic{"/downstream/chassis_control"};
  std::string output_topic{"/control_to_uart"};
  double motion_state_timeout{0.2};
  double command_timeout{0.15};
  double publish_rate{50.0};
  double default_linear_speed_max{0.5};
  double linear_speed_max{1.0};
  double default_angular_speed_max{0.5};
  double angular_speed_max{0.8};
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
  ChassisControlSubscriber(nav2_util::LifecycleNode & node, MotionStateSubscriber & motion_state_subscriber, ChassisControlConfig config);
  void activate();
  void deactivate();
  void reset();

private:
  struct TargetCommand
  {
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    double linear_acceleration{1.0};
    double angular_acceleration{1.0};
    uint8_t operation{0};
  };

  void onChassisControl(const byd_custom_msgs::msg::ChassisControl::ConstSharedPtr message);
  void processControlCommand();
  void clearControlStateLocked();
  void publishControl(double linear_velocity, double angular_velocity);
  void publishZero();
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
