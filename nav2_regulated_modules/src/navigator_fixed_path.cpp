#include "nav2_regulated_modules/regulated_navigator.hpp"

#include <chrono>

#include "nav2_regulated_modules/navigation_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_regulated_modules
{

// 中文注释：校验固定 Path，并把每个 Pose 统一变换到导航全局坐标系后返回新路径。  增加固定路径的离散点生成
void RegulatedNavigator::prepareFixedPath(const nav_msgs::msg::Path & input, nav_msgs::msg::Path &output) {
  if (input.poses.size() < 2 || input.header.frame_id.empty()) {
    LOG_ERROR("固定路径至少需要两个有效路径点，并且必须提供坐标系");
    output = nav_msgs::msg::Path();
    return;
  }

  output.header.frame_id = global_frame_;
  output.header.stamp = now();
  output.poses.clear();
  output.poses.reserve(input.poses.size()); // 注意：插值后点数会变多，这里只是预分配基础空间

  for (const auto & input_pose : input.poses) {
    geometry_msgs::msg::PoseStamped source = input_pose;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = input.header.frame_id;
    }
    if (!navigation_utils::validPose(source)) {
      LOG_ERROR("固定路径包含非有限坐标或无效四元数");
      output = nav_msgs::msg::Path();
      return;
    }

    try {
      auto target = tf_buffer_->transform(source, global_frame_, tf2::durationFromSec(0.1));
      target.header.frame_id = global_frame_;
      target.header.stamp = output.header.stamp;
      output.poses.push_back(target);
    } catch (const tf2::TransformException & error) {
      LOG_ERROR("固定路径无法转换到 {}：{}", global_frame_, error.what());
      output = nav_msgs::msg::Path();
      return;
    }
  }

  // 新增逻辑：路径插值处理
  // 检查变换后的路径是否满足插值前提（至少2个点）
  if (output.poses.size() >= 2) {
    nav_msgs::msg::Path interpolated_path;
    interpolated_path.header = output.header; // 保持Header一致
    interpolated_path.poses.push_back(output.poses.front());

    nav_msgs::msg::Path segment;
    segment.header = output.header;
    constexpr double DISTANCE_TOLERANCE = 1e-3;

    // 依次遍历 output 的两个连续点
    for (size_t i = 0; i < output.poses.size() - 1; ++i) {
        // 提取相邻的两个点，构造一个临时的双点 Path
        segment.poses.clear();
        segment.poses.push_back(output.poses[i]);
        segment.poses.push_back(output.poses[i + 1]);

        // 使用 liner2to4point 将两个点转换为四个点
        nav_msgs::msg::Path four_points;
        liner2to4point(segment, four_points);

        // 使用 generateBezierUniformPoints 生成贝塞尔序列点
        // 注意：请根据实际接口传入合适的 interval 参数，这里假设一个默认值或从参数获取
        nav_msgs::msg::Path bezier_points;
        generateBezierUniformPoints(four_points, fixed_path_step_, bezier_points);

        // 定义一个极小的距离阈值（例如 1 毫米），用于判断两个点是否重合
        // 获取前一段路径的尾点和当前段的头点
        const auto& prev_pose = interpolated_path.poses.back();
        const auto& curr_pose = bezier_points.poses.front();
        // 计算两点间的欧氏距离
        double dx = prev_pose.pose.position.x - curr_pose.pose.position.x;
        double dy = prev_pose.pose.position.y - curr_pose.pose.position.y;
        double distance_sq = dx * dx + dy * dy;
        // 如果距离小于阈值，说明是同一个拼接点，移除前一段的尾点（避免重复）
        if (distance_sq < DISTANCE_TOLERANCE * DISTANCE_TOLERANCE) interpolated_path.poses.pop_back();

        // 使用 std::move 将结果高效堆入 interpolated_path 的尾部，避免深拷贝
        interpolated_path.poses.insert(interpolated_path.poses.end(), std::make_move_iterator(bezier_points.poses.begin()), std::make_move_iterator(bezier_points.poses.end()));
    }

    // 用插值后的路径替换原有的 output
    output = std::move(interpolated_path);
    // 使用 const auto& 避免拷贝对象，提高性能
    for (const auto& pose_stamped : output.poses) {
        // 通过 pose_stamped.pose.position 访问具体的 x, y 坐标
        double x = pose_stamped.pose.position.x;
        double y = pose_stamped.pose.position.y;
        LOG_INFO("Path Point -> x: {}, y: {}", x, y);
    }
  }
}

// 中文注释：只在固定路径模式和激活状态接受结构有效的完整 Path，TF 转换留到 Accepted 回调处理。
rclcpp_action::GoalResponse RegulatedNavigator::handleFixedPathGoal(const rclcpp_action::GoalUUID &, const std::shared_ptr<const FollowFixedPath::Goal> goal) {
  if (operation_mode_ != NavigationMode::FIXED_PATH) {
    LOG_WARN("当前模式不接受固定路径 Action Goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!active_ || goal->path.poses.size() < 2 || goal->path.header.frame_id.empty()) {
    LOG_WARN("拒绝固定路径 Goal：节点未激活、路径点少于两个或顶层坐标系为空");
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (const auto & input_pose : goal->path.poses) {
    auto pose = input_pose;
    if (pose.header.frame_id.empty()) {pose.header.frame_id = goal->path.header.frame_id;}
    if (!navigation_utils::validPose(pose)) {
      LOG_WARN("拒绝固定路径 Goal：路径包含非有限坐标或无效四元数");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 中文注释：固定路径取消只设置请求标志，避免在 Action 回调中重入下游取消和任务复位。
rclcpp_action::CancelResponse RegulatedNavigator::handleFixedPathCancel(const std::shared_ptr<FixedPathHandle> goal) {
  if (goal == active_fixed_path_goal_) {
    cancel_requested_ = true;
    LOG_DEBUG("收到固定路径取消请求，generation={}", task_.generation);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// 中文注释：先完成新路径变换和下游可用性检查，再抢占旧任务并启动新的固定路径 Action。
void RegulatedNavigator::handleFixedPathAccepted(const std::shared_ptr<FixedPathHandle> goal) {
  nav_msgs::msg::Path prepared_path;
  prepareFixedPath(goal->get_goal()->path, prepared_path);
  if (prepared_path.poses.empty()) {
    auto result = std::make_shared<FollowFixedPath::Result>();
    result->success = false;
    goal->abort(result);
    LOG_ERROR("固定路径 Goal 已接受但路径坐标变换失败");
    return;
  }
  if (!follow_client_->wait_for_action_server(std::chrono::duration<double>(server_timeout_))) {
    auto result = std::make_shared<FollowFixedPath::Result>();
    result->success = false;
    goal->abort(result);
    LOG_ERROR("固定路径 Goal 已接受但 FollowPath Action Server 不可用");
    return;
  }

  preemptCurrentTask();
  active_fixed_path_goal_ = goal;
  task_ = NavigationTask();
  task_.generation = ++task_generation_;
  task_.type = TaskType::FIXED_PATH;
  task_.goal = prepared_path.poses.back();
  task_.active_path = std::move(prepared_path);
  task_.start_time = now();
  task_.last_progress_time = task_.start_time;
  task_.last_progress_pose = geometry_msgs::msg::PoseStamped();
  LOG_INFO("接受固定路径 Action，generation={}，frame={}，路径点数={}", task_.generation, task_.active_path.header.frame_id, task_.active_path.poses.size());
  sendFollowPath(task_.active_path);
}

// 中文注释：恢复固定路径时重发已保存 Path；自主任务则重新进入规划链。
void RegulatedNavigator::resumeCurrentTask() {
  if (task_.type == TaskType::FIXED_PATH) {
    if (task_.active_path.poses.empty()) {
      failTask("没有可恢复的固定路径");
      return;
    }
    LOG_INFO("恢复固定路径任务，generation={}，路径点数={}", task_.generation, task_.active_path.poses.size());
    sendFollowPath(task_.active_path);
    return;
  }
  LOG_INFO("恢复自主导航任务，generation={}，重新进入规划链", task_.generation);
  startPlanning(false);
}

// 直线插值生成四点
void RegulatedNavigator::liner2to4point(const nav_msgs::msg::Path &input_path, nav_msgs::msg::Path &output_path){
    // 提取起点和终点
    const auto& p0 = input_path.poses[0].pose.position;
    const auto& p3 = input_path.poses[1].pose.position;

    // 创建一个新的包含 4 个点的路径
    output_path.header = input_path.header;
    output_path.poses.clear();
    output_path.poses.reserve(4);

    // 1. 添加起点 P0
    output_path.poses.push_back(input_path.poses[0]);

    // 2. 计算并添加第一个四分点 P1 (1/3 处)
    geometry_msgs::msg::PoseStamped p1_pose;
    p1_pose.header = input_path.poses[0].header;
    p1_pose.pose.position.x = p0.x + (p3.x - p0.x) / 3.0;
    p1_pose.pose.position.y = p0.y + (p3.y - p0.y) / 3.0;
    p1_pose.pose.position.z = 0.0;
    p1_pose.pose.orientation.w = 1.0; // 默认无旋转
    output_path.poses.push_back(p1_pose);

    // 3. 计算并添加第二个四分点 P2 (2/3 处)
    geometry_msgs::msg::PoseStamped p2_pose;
    p2_pose.header = input_path.poses[0].header;
    p2_pose.pose.position.x = p0.x + 2.0 * (p3.x - p0.x) / 3.0;
    p2_pose.pose.position.y = p0.y + 2.0 * (p3.y - p0.y) / 3.0;
    p2_pose.pose.position.z = 0.0;
    p2_pose.pose.orientation.w = 1.0; // 默认无旋转
    output_path.poses.push_back(p2_pose);

    // 4. 添加终点 P3
    output_path.poses.push_back(input_path.poses[1]);
}

// 3阶bezier曲线单点坐标计算
geometry_msgs::msg::PoseStamped RegulatedNavigator::bezier3(const nav_msgs::msg::Path &input_path, double t, bool useresult = false) {
    assert(input_path.poses.size() == 4);
    
    double u = 1.0 - t;
    double b0 = u * u * u;
    double b1 = 3.0 * u * u * t;
    double b2 = 3.0 * u * t * t;
    double b3 = t * t * t;

    const auto& p0 = input_path.poses[0].pose.position;
    const auto& p1 = input_path.poses[1].pose.position;
    const auto& p2 = input_path.poses[2].pose.position;
    const auto& p3 = input_path.poses[3].pose.position;

    geometry_msgs::msg::PoseStamped result_pose;
    result_pose.header = input_path.header; // 继承输入的 header
    
    // 提取控制点坐标并计算贝塞尔曲线
    result_pose.pose.position.x = b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x;                                 
    result_pose.pose.position.y = b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y;
                                  
    // z轴强制赋值为0
    result_pose.pose.position.z = 0.0;
    
    // 默认姿态 (无旋转)
    if(useresult == false){
        result_pose.pose.orientation.x = 0.0;
        result_pose.pose.orientation.y = 0.0;
        result_pose.pose.orientation.z = 0.0;
        result_pose.pose.orientation.w = 1.0;
    }
    else{
        // 2. 计算三阶贝塞尔曲线在 t 处的一阶导数（切向量）
        // 公式: P'(t) = 3(1-t)^2(P1-P0) + 6(1-t)t(P2-P1) + 3t^2(P3-P2)

        double dx = 3.0 * u * u * (p1.x - p0.x) + 6.0 * u * t * (p2.x - p1.x) + 3.0 * t * t * (p3.x - p2.x);             
        double dy = 3.0 * u * u * (p1.y - p0.y) + 6.0 * u * t * (p2.y - p1.y) + 3.0 * t * t * (p3.y - p2.y);

        // 3. 计算 Yaw 角（绕 Z 轴的旋转）
        double yaw = std::atan2(dy, dx);

        // 4. 将 Yaw 角转换为四元数 (Roll=0, Pitch=0, Yaw=yaw)
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize(); // 确保是单位四元数

        // 5. 赋值给结果
        result_pose.pose.orientation = tf2::toMsg(q);
        result_pose.header = input_path.poses[0].header;
    }

    return result_pose;
}

// 计算3阶bezier曲线的长度
void RegulatedNavigator::computeArcLengths(const std::vector<geometry_msgs::msg::PoseStamped> &poses, std::vector<double> &arc_lengths) {
    arc_lengths.resize(poses.size(), 0.0);
    for (size_t i = 1; i < poses.size(); ++i) {
        double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
        double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
        arc_lengths[i] = arc_lengths[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
}

// 在arc_lengths中找到对应s_target的参数t，线性插值
double RegulatedNavigator::findTfromArcLength(const std::vector<double> &arc_lengths, const std::vector<double> &ts, double s_target) {
    if (s_target <= arc_lengths.front()) return ts.front();
    if (s_target >= arc_lengths.back()) return ts.back();

    // 二分查找区间
    size_t low = 0, high = arc_lengths.size() - 1;
    while (low <= high) {
        size_t mid = low + (high - low) / 2;
        if (arc_lengths[mid] < s_target) {
            low = mid + 1;
        } else {
            if (mid == 0 || arc_lengths[mid - 1] < s_target) {
                size_t i = mid - 1;
                double s0 = arc_lengths[i];
                double s1 = arc_lengths[i + 1];
                double t0 = ts[i];
                double t1 = ts[i + 1];
                double t = t0 + (s_target - s0) / (s1 - s0) * (t1 - t0);
                return t;
            }
            high = mid - 1;
        }
    }
    return ts.back();
}

// 依据四点生成3阶bezier曲线固定步长序列点
void RegulatedNavigator::generateBezierUniformPoints(const nav_msgs::msg::Path &input_path, double interval, nav_msgs::msg::Path &output_path) {
    // 检查控制点数量是否为4
    if (input_path.poses.size() != 4) {
        LOG_ERROR("路径控制点数量 input_path.poses.size()={}, !=4", input_path.poses.size());
        output_path = nav_msgs::msg::Path(); // 返回空 Path
        return;
    }

    // 1. 先粗略计算一下四个控制点连线的总长度（用于估算采样点数）
    double rough_total_len = 0.0;
    for (size_t i = 1; i < input_path.poses.size(); ++i) {
        double dx = input_path.poses[i].pose.position.x - input_path.poses[i - 1].pose.position.x;
        double dy = input_path.poses[i].pose.position.y - input_path.poses[i - 1].pose.position.y;
        rough_total_len += std::sqrt(dx * dx + dy * dy);
    }

    // 2. 根据总长度和步长动态计算高密度采样点数
    // 除以 3 是为了保证线性插值查找 t 时的精度
    int dense_num = static_cast<int>(std::ceil(rough_total_len / (interval * 3.0)));

    // 3. 设置安全下限，防止 interval 过大或控制点重合导致采样点太少
    if (dense_num < 10) dense_num = 10;

    std::vector<geometry_msgs::msg::PoseStamped> dense_poses;
    std::vector<double> ts;
    dense_poses.reserve(dense_num);
    ts.reserve(dense_num);

    for (int i = 0; i < dense_num; ++i) {
        double t = double(i) / (dense_num - 1);
        ts.push_back(t);
        dense_poses.push_back(bezier3(input_path, t));
    }

    std::vector<double> arc_lengths;
    computeArcLengths(dense_poses, arc_lengths);
    double total_len = arc_lengths.back();

    int sample_count = static_cast<int>(std::floor(total_len / interval)) + 1;
    std::vector<geometry_msgs::msg::PoseStamped> uniform_poses;
    uniform_poses.reserve(sample_count + 1);

    for (int i = 0; i < sample_count; ++i) {
        double s = i * interval;
        if (s > total_len) s = total_len; // 防止越界
        double t = findTfromArcLength(arc_lengths, ts, s);
        uniform_poses.push_back(bezier3(input_path, t, true));
    }

    // 确保最后一个点就是终点
    const auto& last_pt = uniform_poses.back().pose.position;
    const auto& end_pt = input_path.poses.back().pose.position;
    if ((last_pt.x != end_pt.x) || (last_pt.y != end_pt.y)) {
        uniform_poses.push_back(bezier3(input_path, 1.0, true));
    }

    // 构建输出路径
    output_path.header = input_path.header;
    output_path.poses = std::move(uniform_poses);
}

}  // namespace nav2_regulated_modules