// 声明常驻遥控与双 Action 的生命周期协调器，按任务类型路由规划、跟踪及限速，交接等待旧请求收口和新鲜停稳反馈。

#ifndef NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
#define NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_

#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include "lifecycle_msgs/srv/get_state.hpp"
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "byd_custom_msgs/action/charging.hpp"
#include "byd_custom_msgs/action/navigation_service.hpp"
#include "byd_custom_msgs/msg/navi_segment.hpp"
#include "byd_custom_msgs/msg/qr_code.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav2_regulated_modules/chassis_control_subscriber.hpp"
#include "nav2_regulated_modules/control_module.hpp"
#include "nav2_regulated_modules/motion_state_subscriber.hpp"
#include "nav2_regulated_modules/navigation_state.hpp"
#include "nav2_regulated_modules/planning_module.hpp"
#include "nav2_regulated_modules/tf_diagnostics.hpp"
#include "nav2_regulated_modules/detail/autonomous_fixed_path.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/collision_state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "spdlog_wrapper.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nav2_regulated_modules
{

class RegulatedNavigator : public nav2_util::LifecycleNode
{
  public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigationService = byd_custom_msgs::action::NavigationService;
  using Charging = byd_custom_msgs::action::Charging;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  using ComputePoseHandle = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowPath>;
  using NavigatePoseHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using NavigationServiceHandle = rclcpp_action::ServerGoalHandle<NavigationService>;
  using ChargingHandle = rclcpp_action::ServerGoalHandle<Charging>;

  // 充电任务阶段：AWAIT_QR 等待二维码缓存出现新鲜观测；FOLLOWING 已下发 map 路径等待执行结果。
  enum class ChargingPhase
  {
    IDLE,
    AWAIT_QR,
    FOLLOWING,
  };

  // 仅声明运行参数；ROS 实体在 configure 建立，业务入口在 activate 后才允许接收。
  // 构造中的 false 等值是节点回退，不覆盖 Launch 从项目 YAML 注入的配置。
  explicit RegulatedNavigator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // 读取并校验配置，再创建客户端、Action 服务、订阅与定时器。
  // 两个上层 Action 在配置阶段建立，接收 Goal 时核对激活状态、执行通道和底盘准入条件。
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  // 激活固定路径可视化与底盘支路，重置 collision 接收历史，再开放业务入口。
  // 这只表示生命周期就绪，不生成任务，也不证明定位／轮速已经新鲜。
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  // 先关闭业务许可，再停用底盘定时器并请求取消导航任务。
  // 固定路径可视化停用、bond 销毁与实际机械停稳是不同事件。
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  // 释放配置阶段创建的 ROS 句柄、反馈缓存与 TF 对象。
  // cleanup 是资源清理，不是控制权移交或完整的下游停稳等待。
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  // 停用底盘支路并调用任务取消流程；是否发零速取决于当前任务状态。
  // 不能仅凭本回调返回 SUCCESS 推断 UART 和轮速已完成停车。
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  // 进入 WAIT_START 并初始化单次规划任务的等待状态，准备定位、旧任务终态和停稳条件。
  void beginFixedAutonomy();
  // 驱动首次规划、原路径执行和暂停恢复，监控状态心跳、定位质量、障碍等待及无进展超时。
  void monitorFixedAutonomy();
  // 验证规划坐标系及请求目标误差，保存原路径、分段和指纹，随后等待首段执行条件。
  void freezeAutonomousPath(const nav_msgs::msg::Path & path);
  // 按单调游标截取当前原始路径段，检查偏离距离及旧跟踪终态，再发送带新执行标识的 FollowPath。
  void followFrozenSegment(bool resume);
  // 在当前段完成后复核段末位姿与停稳条件，决定切入下一段或完成上层单点 Action。
  void finishFrozenSegment();
  // 解析控制器的执行状态心跳，用任务执行标识过滤旧状态，更新定位暂停和障碍等待依据。
  void onFixedExecutionStatus(const std_msgs::msg::String::SharedPtr message);
  // 要求源时间年龄在 −0.05～0.2 秒内，平面线速度≤0.01 m/s、角速度≤0.05 rad/s，且反馈速度均为有限值。
  bool fixedVelocityStopped();
  // 原始冻结路径及其分段保持几何一致，指纹用于检测执行前的意外变化。
  nav_msgs::msg::Path frozen_path_;
  std::vector<detail::FixedSegment> frozen_segments_;
  std::size_t frozen_segment_ = 0;
  std::size_t frozen_cursor_ = 0;
  std::size_t frozen_dispatch_first_ = 0;
  uint64_t frozen_fingerprint_ = 0;
  // 单次规划锁存与在途跟踪计数分别约束规划次数和段间取消握手。
  bool frozen_plan_requested_ = false;
  bool frozen_follow_inflight_ = false;
  bool frozen_localization_wait_ = false;
  std::size_t frozen_outstanding_follows_ = 0;
  // 每段派发使用递增执行标识，控制器状态心跳必须与当前派发匹配。
  int64_t frozen_execution_id_ = 0;
  int64_t frozen_last_dispatch_id_ = 0;
  std::string frozen_phase_;
  double frozen_wait_since_ = 0.0;
  double frozen_status_since_ = 0.0;
  double frozen_obstacle_since_ = -1.0;
  // 等待窗口分开记录障碍、定位稳定与短暂停状态，避免一种等待覆盖另一种超时。
  std::optional<double> frozen_inhibited_since_;
  std::optional<double> frozen_stable_since_;
  std::optional<std::chrono::steady_clock::time_point> frozen_soft_pause_since_;
  std::string frozen_localization_reason_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr frozen_status_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr frozen_path_pub_;

  // 仅接收通过统一准入且位姿有效的单点 Goal，behavior_tree 必须为空。
  // 接收回调只做准入校验，规划在 Goal 排队并完成控制权交接后触发。
  rclcpp_action::GoalResponse handlePoseGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigateToPose::Goal> goal);
  // 仅当句柄属于当前单点任务时置取消标记，随后由监控周期执行取消流程。
  // 返回 ACCEPT 表示接受取消请求，不是下游 FollowPath 的终态或停稳确认。
  rclcpp_action::CancelResponse handlePoseCancel(const std::shared_ptr<NavigatePoseHandle> goal);
  // 接受后的 Goal 先进入待执行队列并请求取消旧任务，交接完成才递增代次并启动单次规划。
  // 旧子 Goal 以异步方式取消，旧回调靠代次过滤，不把句柄重置视为取消握手完成。
  void handlePoseAccepted(const std::shared_ptr<NavigatePoseHandle> goal);
  // 固定路径 Goal 要求节点激活、底盘准入、分段非空、全段同向且 max_speed 有限为正。
  // 几何连续性检查在接受后的 prepareFixedPath，故部分几何错误会以已接受后 ABORTED 结束。
  rclcpp_action::GoalResponse handleNavigationServiceGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const NavigationService::Goal> goal);
  // 当前固定路径句柄才设置 cancel_requested，监控周期随后处理。
  // 返回 ACCEPT 不表示下游取消完成，finish=false 也不能替代轮速停稳证据。
  rclcpp_action::CancelResponse handleNavigationServiceCancel(const std::shared_ptr<NavigationServiceHandle> goal);
  // 构造几何路径、等待 FollowPath，再接管旧任务并计算总长与最小段限速。
  // 车头姿态由切线和方向生成，当前定位不被重置；超过全局速度上限时钳位，不因此自动拒绝 Goal。
  void handleNavigationServiceAccepted(const std::shared_ptr<NavigationServiceHandle> goal);
  // 仅接受节点激活、固定通道就绪、底盘准入、task_id 非空且起点终点坐标有限的充电请求；
  // 已有导航任务或待执行导航 Goal 时拒绝，与导航准入 admitUnified 形成双向互斥。
  // 这里只校验请求，接受后到 handleChargingAccepted 才保存句柄与更新任务快照。
  rclcpp_action::GoalResponse handleChargingGoal(const rclcpp_action::GoalUUID & uuid, const std::shared_ptr<const Charging::Goal> goal);
  // 取消直接把充电目标摘回 IDLE、停充电侧定时器并把在途 FollowPath 置取消，再以 canceled 终结。
  // 返回 ACCEPT 后取消即完成，不存在需延迟消费的取消标记。
  rclcpp_action::CancelResponse handleChargingCancel(const std::shared_ptr<ChargingHandle> goal);
  // 保存充电句柄并进入 AWAIT_QR：配对一次性截止定时器（charging_qr_wait_timeout_）与消费定时器。
  // 消费定时器非阻塞轮询二维码缓存，认领到新鲜观测后转入 startChargingFollow 下发执行。
  void handleChargingAccepted(const std::shared_ptr<ChargingHandle> goal);
  // AWAIT_QR 截止定时器回调：等待窗口内未认领到新鲜二维码即以失败收口。
  void handleChargingQrTimeout(const std::shared_ptr<ChargingHandle> goal);
  // AWAIT_QR 消费定时器回调：每拍检查缓存，出现新鲜二维码即认领一次并转入路径下发；不 sleep。
  void handleChargingQrPoll(const std::shared_ptr<ChargingHandle> goal);
  // 用二维码坐标修正 map 帧终点、生成充电路径并异步下发 FollowPath；
  // 进度由 feedback 回调实时上报，结果回调走 finalizeChargingGoal 收口。
  void startChargingFollow(const std::shared_ptr<ChargingHandle> goal);
  // 临界收口：仅当 goal 仍是活动充电任务时终结一次（摘回 IDLE、停定时器、作废在途 FollowPath），
  // 迟到或重复的收口请求静默返回，保证动作 Goal 不会被二次终结。
  void finalizeChargingGoal(const std::shared_ptr<ChargingHandle> goal, bool success, const std::string & message);
  // 内部取消充电任务：接管保护、生命周期停用／清理与导航任务收口共用，
  // 无活动充电目标时静默返回；以 abort 终结（外部取消才报 canceled）。
  void cancelChargingTask(const std::string & reason);
  // 充电任务处于任意在途阶段（含 AWAIT_QR）：车辆被占用，导航 Goal 准入据此互斥。
  bool chargingTaskActive() const;
  // 充电 FollowPath 执行中：底盘统一出口应把固定通道当作当前导航速度源。
  bool chargingFollowActive() const;
  // 充电跟随的跟踪轮次，供底盘出口的速度缓存做代次匹配。
  uint64_t chargingFollowGeneration() const;
  // 在固定通道速度基准话题发布绝对值 SpeedLimit：下发跟随前置 charging_task_speed_，
  // 收口时置 0 复位——固定通道 velocity_smoother 的 collision 门控以任务速度为基准，
  // 速度为 0 时会把控制器输出全部清零（车不动），充电流程必须像导航任务一样显式声明。
  void publishChargingSpeedLimit(double speed);
  // 从起点到终点沿直线按 step 均匀插值，并在终点方向额外延长 extension 距离。
  // 两输入点约定为同一帧（充电起终点都在 map 帧），输出路径按 global_frame 自动盖章。
  // 路径 z 置 0，车头航向由行进方向生成；两端点重合或参数非法时不生成路径，
  // 这里不进行碰撞可行性验证，只承担几何采样，后续由充电执行流程消费。
  std::optional<nav_msgs::msg::Path> generateChargingPath(const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & target) const;
  // 无阻塞读取最新二维码快照，用二维码坐标只修正输入的 map 帧终点：转 QR 系把横纵轴中
  // 较小分量置零再转回 map 帧写出；起点不做处理（仍在 map 帧）。快照锁内复核消息年龄
  // （≤qr_max_age 秒），过期或定位不可用返回 false；不等待、不阻塞。
  bool correctChargingTargetToMap(geometry_msgs::msg::Point & target);
  // 检查有限坐标、非重合端点、相邻连接与段类型，再为直线／贝塞尔生成统一采样。
  // 路径位于 global_frame，输出 z=0；按整条方向编码车头姿态，不在这里进行全程碰撞可行性验证。
  std::optional<nav_msgs::msg::Path> prepareFixedPath(const std::vector<byd_custom_msgs::msg::NaviSegment> & segments);
  // 将两端点扩成共线的四控制点，两个内点分别位于线段的三分之一和三分之二。
  // 因此同一贝塞尔采样器也能处理直线，函数名沿用既有拼写。
  void liner2to4point(const nav_msgs::msg::Path & input_path, nav_msgs::msg::Path & output_path);
  // 以 t∈[0,1] 对四控制点求三次贝塞尔 XY 位置，z 固定为 0。
  // use_result=false 时返回单位姿态供弧长估计；true 时由一阶导数求切线航向。调用方保证四个控制点和参数有效。
  geometry_msgs::msg::PoseStamped bezier3(const nav_msgs::msg::Path & input_path, double t, bool use_result);
  // 按相邻离散点二维距离累计弧长，单位 m，首点累计值为 0。
  // 这是离散近似，不是三次曲线解析弧长。
  void computeArcLengths(const std::vector<geometry_msgs::msg::PoseStamped> & poses, std::vector<double> & arc_lengths);
  // 在累计弧长表中定位目标长度，再在线性邻区间反查 t。
  // 首尾边界钳位到参数范围，重复长度区间不能用作有效除数。
  double findTfromArcLength(const std::vector<double> & arc_lengths, const std::vector<double> & ts, double target_length);
  // 先密集采样估计累计弧长，再按 interval（m）反查 t 输出近似等距路径。
  // 保留精确末点并合并近重复邻点；等距指空间弧长，不是控制器时间间隔。
  void generateBezierUniformPoints(const nav_msgs::msg::Path & input_path, double interval, nav_msgs::msg::Path & output_path);
  // 查询当前位姿，为半自主单点 Action 计算时间、ETA、恢复次数和剩余距离。
  // 固定路径进度另在 FollowPath 反馈回调发布，不能把此定时器频率当作它的固定反馈频率。
  void publishFeedback();
  // 为固定路径发布 percentage=false 的正绝对限速，单位 m/s。
  // 倒车方向在 Path 姿态和控制器符号中表达，不能发布负 SpeedLimit 代替方向。
  void publishSpeedLimit();

  // 查询半自主执行通道的规划、跟踪 Action 与生命周期就绪状态。
  // 服务可发现只证明通信入口存在，不证明地图、TF 或底盘反馈有效。
  bool dependenciesReady();
  // 仅在半自主单点任务有效且没有在途规划时提交一次请求。
  // generation 标识任务，plan_sequence 标识该任务的规划轮次；两者均匹配才接受回调。
  void startPlanning();
  // 同时比较 generation 和规划 sequence，拒绝旧任务或旧规划轮次的异步结果。
  // 它不检查机器人位置或路径质量，只承担回调归属判断。
  bool isCurrentPlan(uint64_t generation, uint64_t sequence) const;
  // 接收到有效的规划结果后直接冻结原路径并进入分段执行。
  void onPathReady(const nav_msgs::msg::Path & path);
  // 规划失败交由任务类型的恢复入口处理，半自主单点任务将结束为失败。
  // 失败处理保持原规划算法及已冻结几何的约束。
  void handlePlanningFailure(const std::string & reason);

  // 按任务类别选择控制器与 GoalChecker，再异步发送完整 Path。
  // FollowPath 的响应、反馈和结果均校验 generation／follow_sequence；固定路径绝对限速在接受后发布。
  void sendFollowPath(const nav_msgs::msg::Path & path);
  // 发布固定路径中心线，并复用共享边界显示。
  void publishFixedPath(const nav_msgs::msg::Path & path);
  // 为固定路径或半自主冻结路径发布左右边界及采样点，不修改输入路径。
  // 边界宽度仅供显示，不作为碰撞检测区或实际行驶误差限制。
  void publishPathBoundaries(const nav_msgs::msg::Path & path);
  // 比较任务代次与跟踪轮次；同一任务换路径也会使旧 sequence 失效。
  // 同一任务可能经历分段或原路径恢复，必须同时核对 follow_sequence，防止旧跟踪结果干扰当前段。
  bool isCurrentFollow(uint64_t generation, uint64_t sequence) const;
  // 向速度链入口连续发布三帧零 Twist，增加停车请求被下游收到的机会。
  // 不检查发布确认或实际轮速；若没有发布器直接返回，不承诺已经机械停稳。
  void stopRobot();
  // 在 velocity_mutex 保护下缓存控制器最新 Twist 与曾接收标记。
  // 缓存用于日志，不代表与平滑器反馈来自严格同一控制周期。
  void onControllerVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  // 记录平滑后 Twist 供诊断；不再对速度限幅或二次发布。
  // 旧快照可持续存在，消费者应另行理解采样新鲜度。
  void onSmoothedVelocity(const geometry_msgs::msg::Twist::SharedPtr velocity);
  // 缓存速度反馈，供速度观测和 autonomous_fixed 连续停稳判断；不作为地图定位。
  void onVelocityOdometry(const nav_msgs::msg::Odometry::SharedPtr odometry);
  void onVelocityMotionState(const byd_custom_msgs::msg::MotionState::SharedPtr motion);
  // 记录二维码标签与相对位姿，并将原始 mm／度单位归一化为 m／弧度。
  // 缓存只是本机最新观测，不代替地图定位，也不证明车身已经对齐充电桩。
  void qrCodeMsgCallback(const byd_custom_msgs::msg::QrCode::SharedPtr msg);
  // 先在锁内复制三路快照，再在锁外写日志，减少回调持锁时间。
  // 没有任何样本时不打印；独立订阅的 latest 值不能当作同步传感器帧。
  void logVelocityChain();

  // 在恢复轮次允许时取消子 Goal、发送零速并请求清空代价地图。
  // 清图请求异步发送；等待时间到后再尝试恢复，不在此确认清图或实际停车完成。
  void startRecovery(const std::string & reason);
  // 固定路径恢复复用 active_path，半自主恢复使用冻结路径当前段。
  // 是否还属于当前有效任务由 task_ 与上层状态机决定；不从已结束任务恢复。
  void resumeCurrentTask();
  // 依次处理统一交接、取消、冻结路径监控、固定路径恢复、定位与进度检查。
  // collision 仅暂停固定路径无进展计时，不跳过前面的取消与定位处理。
  void monitorTask();
  // 只有规划请求及两通道 FollowPath 的在途计数全部归零，才认为旧子任务已收口。
  bool unifiedDrained() const;
  // 检查交接后的有效 MotionState，接收年龄≤反馈超时且线角速度幅值<0.001，同时要求导航输出满足静默窗口。
  bool unifiedStopped() const;
  // 在接收 Goal 前检查生命周期、交接状态、底盘许可与定位，空闲启动还要求旧请求收口和实测停稳。
  bool admitUnified(bool lattice);
  // 检查所选通道的 Action 可发现性及最近一秒内的 Active 状态，Lattice 还要求规划服务可用。
  bool executionReady(bool lattice) const;
  // 先轮询状态并处理取消，再等待旧子任务收口和停稳，最后启动仍有效的待执行 Goal。返回 true 表示本周期由交接流程接管。
  bool monitorUnified();
  // 异步轮询两条执行通道的生命周期状态，超时请求从客户端移除，失效缓存按未激活处理。
  void pollExecutionStates();
  // 只处理尚未启动的 Goal，已请求取消的句柄返回 CANCELED；内部撤销返回 ABORTED，固定路径句柄收口后清空待执行路径。
  void finishPending(bool client_cancel);
  // 将已取消任务的上层句柄报告为 CANCELED 或 ABORTED，再清理任务快照。正常激活期间由交接监控调用。
  void finishCancellation();
  void startFixedPath(const std::shared_ptr<NavigationServiceHandle> & goal,
    nav_msgs::msg::Path path);
  void startPose(const std::shared_ptr<NavigatePoseHandle> & goal);
  // 按当前任务选择 FollowPath 客户端，单点任务进入 lattice_exec，业务固定路径进入 fixed_path_exec。
  rclcpp_action::Client<FollowPath>::SharedPtr followClient() const;
  // 为当前执行通道选择 SpeedLimit 发布器，使两条通道的任务限速分别维护。
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speedPublisher() const;

  using StateService = lifecycle_msgs::srv::GetState;
  struct ExecutionNode
  {
    rclcpp::Client<StateService>::SharedPtr client;
    bool active = false;
    std::optional<int64_t> request;
    std::chrono::steady_clock::time_point last_request;
    std::chrono::steady_clock::time_point last_response;
  };
  std::vector<ExecutionNode> fixed_execution_;
  std::vector<ExecutionNode> lattice_execution_;
  std::shared_ptr<NavigatePoseHandle> pending_pose_;
  std::shared_ptr<NavigationServiceHandle> pending_fixed_;
  nav_msgs::msg::Path pending_path_;
  std::chrono::steady_clock::time_point pending_since_;
  std::chrono::steady_clock::time_point stopping_since_{};
  std::string cancellation_reason_;
  std::size_t outstanding_plans_ = 0;
  rclcpp_action::Client<FollowPath>::SharedPtr lattice_follow_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr lattice_stop_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr lattice_speed_pub_;
  // 半自主单点任务检查 TF 源时间戳，固定路径查询成功后按当前节点时间记录有效性。
  // 新鲜定位仍不等于独立物理精度验收。
  bool lookupCurrentPose(geometry_msgs::msg::PoseStamped & pose);

  // 对已持有句柄发送 async_cancel_goal，再重置本地句柄；可选递增轮次使旧回调失效。
  // 未等待取消响应／结果，不能称为下游全部终止；尚未拿到句柄的请求也不由这些分支直接取消。
  void cancelSubGoals(bool invalidate_callbacks);
  // 进入取消状态、废弃旧回调、请求停车，再按上层句柄状态返回 canceled 或 abort。
  // 只有客户端正在取消的句柄报告 canceled；一般内部取消走 abort，随后清空任务。
  void cancelTask(const std::string & reason);
  // 向当前对应 Action 报告成功；固定路径另发 progress=1、finish=true 后清理任务。
  // 上层成功由下游结果触发，不直接证明外部精度验收、定位覆盖或真实底盘制动距离合格。
  void succeedTask();
  // 废弃子 Goal 并向现有上层句柄报告失败，再清理任务快照。
  // 失败流程请求 stopRobot 并取消子任务，后续启动仍需经过交接监控的请求收口与实测停稳检查。
  void failTask(const std::string & reason);
  // 清除任务与诊断历史；固定路径结束时发布 0 SpeedLimit 清除该任务基准。
  // 0 在插件接口中的语义与 collision_task_speed 清理需区分，不代表独立停车命令。
  void resetTask();

  // 业务状态与生命周期不是两个独立控制源：active_ 是上层回调放行条件，任务阶段另由 task_ 表示。
  bool active_ { false };
  bool planning_active_ { false };
  bool has_last_pose_ { false };
  bool cancel_requested_ { false };
  bool collision_state_enabled_ = false;
  nav2_util::CollisionState collision_state_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr collision_state_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr collision_params_handler_;
  bool collision_progress_paused_ = false;
  // 代次／轮次只用于当前异步回调归属，不是 ROS Action UUID，也不提供分布式取消确认。
  uint64_t task_generation_ { 0 };
  uint64_t plan_sequence_ { 0 };
  uint64_t follow_sequence_ { 0 };
  // 取消响应仅代表接受取消；须等待每个已发送 FollowPath 的拒绝或终态结果。
  std::size_t fixed_path_outstanding_follows_{0};
  NavigationTask task_;
  std::unique_ptr<MotionStateSubscriber> motion_state_subscriber_;
  std::unique_ptr<ChassisControlSubscriber> chassis_control_subscriber_;
  PlanningModule planning_module_;
  ControlModule control_module_;

  std::string global_frame_;
  std::string robot_base_frame_;
  std::string navigation_service_action_;
  std::string charging_action_;
  // 充电直线路径的采样步幅（m）与终点方向延长量（m）。
  double charging_path_step_ { 0.04 };
  double charging_path_extension_ { 0.3 };
  // 充电等待二维码的截止时长（s）与二维码消息允许的最大年龄（s）。
  double charging_qr_wait_timeout_ { 2.0 };
  double charging_qr_max_age_ { 0.5 };
  std::string fixed_path_visualization_topic_;
  std::string fixed_path_boundaries_topic_;
  std::string fixed_path_controller_id_;
  std::string fixed_path_goal_checker_id_;
  std::string charging_controller_id_;
  std::string charging_checker_id_;
  // 充电跟随的固定通道速度基准（m/s）：固定通道 velocity_smoother 的 collision 门控基准，
  // 应不低于 ChargingController 的 desired_linear_vel，否则跟踪段线速度会被截低。
  double charging_task_speed_ { 0.1 };
  std::string controller_cmd_vel_topic_;
  std::string smoothed_cmd_vel_topic_;
  std::string velocity_odom_topic_;
  std::string speed_limit_topic_;
  // 下列超时／周期为 s；距离阈值为 m，角度跳变阈值为 rad，频率字段为 Hz。
  double server_timeout_ { 5.0 };
  double cancel_timeout_ { 2.0 };
  double smoothing_duration_ { 2.0 };
  double feedback_frequency_ { 5.0 };
  double costmap_wait_duration_ { 0.8 };
  double passed_goal_radius_ { 0.7 };
  double localization_timeout_ { 0.3 };
  bool enable_localization_jump_detection_ { false };
  double max_translation_jump_ { 0.3 };
  double max_rotation_jump_ { 0.35 };
  double progress_min_translation_ { 0.1 };
  double autonomous_progress_min_rotation_ = 0.2;
  double autonomous_goal_progress_distance_ = 0.01;
  std::optional<double> autonomous_progress_heading_error_;
  double autonomous_progress_goal_distance_ = 0.0;
  bool autonomous_progress_terminal_ = false;
  double autonomous_progress_reference_heading_ = 0.0;
  bool autonomous_terminal_progress_enabled_ = false;
  double autonomous_terminal_xy_tolerance_ = 0.03;
  double autonomous_terminal_distance_ = 0.15;
  double localization_recovery_timeout_ { 10.0 };
  double localization_stable_duration_ { 0.5 };
  int max_recovery_rounds_ { 2 };
  bool check_smoother_collisions_ { true };
  double current_speed_ { 0.0 };
  double fixed_path_step_ { 0.1 };
  double fixed_path_max_speed_ { 1.5 };
  double fixed_path_boundary_half_width_ { 0.4 };
  double velocity_log_frequency_ { 1.0 };

  rclcpp::Time last_valid_tf_time_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time localization_lost_time_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time localization_stable_since_ { 0, 0, RCL_ROS_TIME };
  rclcpp::Time recovery_ready_time_ { 0, 0, RCL_ROS_TIME };
  geometry_msgs::msg::PoseStamped last_pose_;
  // 保留的历史原始路径缓存，当前规划成功后通过 onPathReady 直接冻结结果。
  nav_msgs::msg::Path pending_raw_path_;
  geometry_msgs::msg::Twist latest_controller_velocity_;
  geometry_msgs::msg::Twist latest_smoothed_velocity_;
  nav_msgs::msg::Odometry latest_velocity_odometry_;
  // 仅保护诊断速度缓存，不是整个任务状态机的全局互斥锁。
  std::mutex velocity_mutex_;
  bool has_controller_velocity_ { false };
  bool has_smoothed_velocity_ { false };
  bool has_velocity_odometry_ { false };

  rclcpp_action::Client<ComputePathToPose>::SharedPtr compute_pose_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_pose_server_;
  rclcpp_action::Server<NavigationService>::SharedPtr navigation_service_server_;
  rclcpp_action::Server<Charging>::SharedPtr charging_server_;
  // 等待二维码的截止定时器（charging_qr_wait_timeout_）；AWAIT_QR 超时未等到新鲜观测即失败收口。
  rclcpp::TimerBase::SharedPtr charging_qr_timeout_timer_;
  // AWAIT_QR 的二维码消费定时器：非阻塞轮询缓存，认领到新鲜观测即转入路径下发，定期空转后复位。
  rclcpp::TimerBase::SharedPtr charging_qr_poll_timer_;
  // 充电任务阶段；接受、二维码消费、跟随时限与取消分处不同回调，切换受 charging_mutex_ 保护。
  ChargingPhase charging_phase_ { ChargingPhase::IDLE };
  rclcpp::Client<ClearCostmap>::SharedPtr clear_local_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr clear_global_client_;

  // 这些句柄只在接受响应到达后可用；尚在等待接受的请求与已经持有句柄的任务需区分。
  ComputePoseHandle::SharedPtr active_compute_pose_goal_;
  FollowHandle::SharedPtr active_follow_goal_;
  std::shared_ptr<NavigatePoseHandle> active_pose_goal_;
  std::shared_ptr<NavigationServiceHandle> active_navigation_service_goal_;
  // 充电状态机锁：保护 active_charging_goal_、charging_phase_、charging_follow_goal_ 的认领与收口。
  // 节点跑在 MultiThreadedExecutor 上，截止/消费定时器、FollowPath 回调与取消应答可能并发，
  // 只有持锁把目标从 active 摘除的一方允许终结动作 Goal，避免同一 Goal 被重复终结。
  // const 查询（chargingTaskActive 等）也要持锁，故声明为 mutable。
  mutable std::mutex charging_mutex_;
  std::shared_ptr<ChargingHandle> active_charging_goal_;
  // 充电正在跟踪的 FollowPath 句柄与跟踪轮次；结果/反馈/取消用它区分当前跟踪与迟到结果。
  FollowHandle::SharedPtr charging_follow_goal_;
  uint64_t charging_follow_sequence_ { 0 };

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr smoothed_velocity_sub_;
  rclcpp::SubscriptionBase::SharedPtr velocity_odom_sub_;
  rclcpp::Subscription<byd_custom_msgs::msg::QrCode>::SharedPtr qr_code_sub_;  //二维码信息订阅
  // 仅保护二维码观测缓存；传感器回调与消费者不同线程，不把变速箱状态放进同一锁。
  std::mutex qr_code_mutex_;
  // 已解码到 m／弧度的最新二维码观测，由 qrCodeMsgCallback 写入、充电续态在锁内读取。
  byd_custom_msgs::msg::QrCode current_qr_code_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_cmd_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr fixed_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr fixed_path_boundaries_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  rclcpp::TimerBase::SharedPtr velocity_log_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<TfDiagnostics> tf_diagnostics_;
};

}
// namespace nav2_regulated_modules

#endif  // NAV2_REGULATED_MODULES__REGULATED_NAVIGATOR_HPP_
