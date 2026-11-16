#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""模拟行为解析模块：按接口文档周期发布 /car_op_status（std_msgs/msg/Int32）。

只发布状态，不做任何碰撞判定、不生成速度命令。可选用 --mock-motion 同时发布
/motion_state（byd_custom_msgs/msg/MotionState），让状态消费端脱离整车也能跑通。

接口文档约定：发布周期 100 ms，数据类型 std_msgs/msg/Int32。

枚举即 9 个设计状态：枚举值 0–8 ↔ 设计状态号 1–9（差 1）。空载/载货、
正走/倒走已内建在每个设计状态里，不需要额外的方向位或载货位。
消费端取值 +1 即设计状态号，细节见本包 doc/OBSTACLE_STATUS.md。
"""
import argparse
import math
import sys
import time


# /car_op_status 的整数取值即线上传输的 Int32 载荷。取值 0–8，对应设计状态 1–9（差 1）。
# 顺序与 doc/collision_zones/gen_zones.py 的 STATES 一致，空载/载货/正走/倒走已内建。
class CarGmstatus:
    IDLE_FORWARD = 0     # 状态1 空载正走
    IDLE_BACKWARD = 1    # 状态2 空载倒走
    GRAB_APPROACH = 2    # 状态3 正走抓料框
    CARGO_BACKWARD = 3   # 状态4 搬货倒走
    CARGO_FORWARD = 4    # 状态5 搬货正走
    STATIONARY = 5       # 状态6 静止
    SPIN_IN_PLACE = 6    # 状态7 原地旋转
    DOCK_CHARGE = 7      # 状态8 去充电（屏蔽前方）
    UNDOCK_LEAVE = 8     # 状态9 充电结束离开（屏蔽后方）

# 枚举值 → 设计状态号（设计文档 1–9）。
OP_TO_STATE = {v: v + 1 for v in range(9)}

# 枚举值 → 中文名（取自设计文档），仅用于日志。
NAME_OF = {
    CarGmstatus.IDLE_FORWARD: '空载正走',
    CarGmstatus.IDLE_BACKWARD: '空载倒走',
    CarGmstatus.GRAB_APPROACH: '正走抓料框',
    CarGmstatus.CARGO_BACKWARD: '搬货倒走',
    CarGmstatus.CARGO_FORWARD: '搬货正走',
    CarGmstatus.STATIONARY: '静止',
    CarGmstatus.SPIN_IN_PLACE: '原地旋转',
    CarGmstatus.DOCK_CHARGE: '去充电',
    CarGmstatus.UNDOCK_LEAVE: '充电结束离开',
}

# 缺省周期：按接口文档的 100 ms。
DEFAULT_PERIOD = 0.1


def build_schedule(args):
    """返回 [(枚举值, 该段保持秒数), ...]，构成一次完整循环。"""
    if args.mode == 'hold':
        return [(args.value, float('inf'))]
    if args.mode == 'script':
        return load_script(args.file)
    order = list(range(9))   # 9 个设计状态全排进周期
    return [(v, args.hold) for v in order]


def load_script(stream):
    """CSV：每行 `枚举值,保持秒数`，# 开头与空行忽略。

    stream 是 --file 用 FileType 打开的文件对象，报错时取其 name 用于定位。
    """
    path = getattr(stream, 'name', '<script>')
    schedule = []
    with stream:
        for lineno, raw in enumerate(stream, 1):
            line = raw.split('#', 1)[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) != 2:
                raise ValueError(f'{path}:{lineno}: 需要 "枚举值,保持秒数"')
            value, hold = int(parts[0]), float(parts[1])
            if not math.isfinite(hold) or hold <= 0:
                raise ValueError(f'{path}:{lineno}: 保持秒数须为有限正数')
            schedule.append((value, hold))
    if not schedule:
        raise ValueError(f'{path}: 没有任何有效行')
    return schedule


# 设计状态各自的朝向：状态号 → dir（gen_zones 权威）。
_DIR_OF = {
    1: 1,   # 空载正走
    2: -1,  # 空载倒走
    3: 1,   # 正走抓料框（固定档，v 无意义但给正向以便轨迹可看）
    4: -1,  # 搬货倒走
    5: 1,   # 搬货正走
    6: 0,   # 静止
    7: 0,   # 原地旋转
    8: 1,   # 去充电
    9: -1,  # 充电结束离开
}


def mock_motion_for(value, args, elapsed):
    """给出该枚举值下模拟的 (v_car, w_car, lift_height)。

    状态号 = value + 1。方向／载货已由设计状态决定，这里只按朝向给一个
    合理的 v 符号、按形状给 w，让消费端的档位判据能走到，不代表整车动力学。
    """
    state = OP_TO_STATE[value]
    d = _DIR_OF[state]
    if state == 7:          # 原地旋转：只转不走
        return 0.0, args.mock_w * (1 if args.direction >= 0 else -1), 0.0
    if d == 0:              # 静止
        return 0.0, 0.0, 0.0
    return (_DIR_OF[state] * args.mock_v), 0.0, 0.0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=['cycle', 'hold', 'script'], default='cycle')
    parser.add_argument('--value', type=int, default=CarGmstatus.SPIN_IN_PLACE,
                        help='--mode hold 时固定发布的枚举值 0–8（设计状态号 − 1）')
    parser.add_argument('--file', type=argparse.FileType('r', encoding='utf-8'),
                        help='--mode script 时的 CSV 路径')
    parser.add_argument('--hold', type=float, default=3.0,
                        help='--mode cycle 时每个枚举值的保持秒数')
    parser.add_argument('--period', type=float, default=DEFAULT_PERIOD,
                        help='发布周期，秒；接口文档为 0.1')
    parser.add_argument('--topic', default='/car_op_status')
    parser.add_argument('--mock-motion', action='store_true',
                        help='同时模拟发布 /motion_state，让消费端脱离整车可跑')
    parser.add_argument('--motion-topic', default='/motion_state')
    parser.add_argument('--mock-v', type=float, default=0.3,
                        help='--mock-motion 下带 v 的状态发布的 |v_car|')
    parser.add_argument('--mock-w', type=float, default=1.0,
                        help='--mock-motion 下原地旋转发布的 |w_car|')
    parser.add_argument('--direction', type=int, choices=[1, -1], default=1,
                        help='原地旋转时旋转方向（1 逆时针 / -1 顺时针）；正负由状态自带')
    parser.add_argument('--no-repeat', action='store_true',
                        help='周期跑完一轮后退出，便于脚本化验证')
    parser.add_argument('--yield-to-other-publishers', action='store_true',
                        help='检测到另一个同话题发布者时暂停自己，外部源退出后恢复；'
                             '供 launch 的默认状态源让位给手动状态窗口')
    # 由 launch 启动时会追加 --ros-args … --params-file，交给 ROS 处理，
    # 不当作本脚本的参数（否则 argparse 直接退出）。
    args, _ros_argv = parser.parse_known_args(argv)

    if not math.isfinite(args.period) or args.period <= 0:
        parser.error('--period 须为有限正数')
    if args.mode == 'script' and args.file is None:
        parser.error('--mode script 需要 --file')
    try:
        schedule = build_schedule(args)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))

    # rclpy 只在这里导入：--help 与参数校验不依赖 ROS 环境。
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int32

    rclpy.init(args=None)
    node = Node('car_op_status_sim')

    # 与项目内已有 /collision_state 发布习惯一致：BEST_EFFORT／VOLATILE／KeepLast(1)。
    status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
    status_pub = node.create_publisher(Int32, args.topic, status_qos)

    motion_pub = None
    motion_cls = None
    if args.mock_motion:
        from byd_custom_msgs.msg import MotionState
        motion_cls = MotionState
        # 与 myworld_bringup/odom_to_motion_state.py 的 profile 保持一致。
        motion_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        motion_pub = node.create_publisher(MotionState, args.motion_topic, motion_qos)

    state = {'index': 0, 'segment_start': time.monotonic(), 'cycles': 0,
             'yielded': False}
    total = len(schedule)

    def publish():
        now = time.monotonic()
        # 默认状态源让位：自己也是一个发布者，所以 >1 表示有外部源。
        # 外部源退出后重新接回，并重置当前段计时，避免补发已过期的切换。
        if args.yield_to_other_publishers:
            if node.count_publishers(args.topic) > 1:
                if not state['yielded']:
                    state['yielded'] = True
                    node.get_logger().info(
                        f'检测到另一个 {args.topic} 发布者，暂停默认状态源')
                return
            if state['yielded']:
                state['yielded'] = False
                state['segment_start'] = now
                node.get_logger().info(
                    f'外部 {args.topic} 发布者已退出，恢复默认状态源')
        value, hold = schedule[state['index']]
        if now - state['segment_start'] >= hold:
            state['index'] += 1
            state['segment_start'] = now
            if state['index'] >= total:
                state['cycles'] += 1
                if args.no_repeat:
                    node.get_logger().info(
                        f'周期跑完 {total} 段，--no-repeat 生效，退出')
                    raise KeyboardInterrupt
                state['index'] = 0
            value, hold = schedule[state['index']]

        if value not in NAME_OF:
            node.get_logger().warn(
                f'枚举值 {value} 不在 CarGmstatus 定义内，仍照发（消费端会告警）')

        status_pub.publish(Int32(data=int(value)))

        detail = ''
        if motion_pub is not None:
            v, w, lift = mock_motion_for(value, args, now - state['segment_start'])
            msg = motion_cls()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.v_car = float(v)
            msg.w_car = float(w)
            msg.v_lift = 0.0
            msg.lift_height = float(lift)
            msg.w_shelf = 0.0
            msg.yaw_shelf = 0.0
            motion_pub.publish(msg)
            detail = f'  motion: v={v:+.2f} w={w:+.2f} lift={lift:.2f}'

        node.get_logger().info(
            f'{args.topic} <- {value} ({NAME_OF.get(value, "未定义")})'
            f'  第{state["cycles"] + 1}轮 段{state["index"] + 1}/{total}{detail}',
            throttle_duration_sec=0.0)

    timer = node.create_timer(args.period, publish)
    node.get_logger().info(
        f'开始发布：mode={args.mode} 周期={args.period * 1000:.0f}ms '
        f'共{total}段 mock_motion={args.mock_motion}')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # SIGTERM／Ctrl+C 正常收尾；不打印 traceback。
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
