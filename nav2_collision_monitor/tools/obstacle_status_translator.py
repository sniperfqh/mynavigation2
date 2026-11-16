#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_status_translator.py — 把激光落点翻译成避障状态枚举。

接口协议
--------
主话题 `/collision_state`（std_msgs/msg/UInt8）——与 Nav2 规控栈对齐：
  订阅者 = regulated_navigator / controller_server / velocity_smoother
  （nav2_util::CollisionState，BEST_EFFORT + VOLATILE depth 1）。
  取值：0/1 停车，2 限速到任务速度 35%，3 限速到 65%，4 不限制；
  超过 0.5 s 没收到 → 按 0（停车）处理。

兼容话题 `/obstacle_status_int`（std_msgs/msg/Int32）——原运控接口，同值同发。
  可用 `--legacy-status-topic ''` 关闭。

  周期：默认 25 Hz；需减速 40 Hz；需停车 80 Hz。

枚举：
  0 ERROR     异常：雷达未启动 / 500 ms 无有效雷达数据 / 动态框未就绪 → 停车
  1 STOP      减速至停车   ← 落点在 L3 停止框内
  2 SLOWDOWN  减速         ← 落点在 L2 减速框内（regulated 限速 35%）
  3 WARNING   报警         ← 落点在 L1 预警框内（regulated 限速 65%）
  4 CLEAR     无障碍物     ← 三级框内均无落点

实现
----
不依赖 Nav2 collision_monitor：它的状态是边沿触发（只在 polygon 变化时发），
无法在启动时给出 CLEAR，也不便于按 25/40/80 Hz 主动上报。本节点自己做检测：

  1. 订阅 `/scan`，按每条 LaserScan 自己的时间戳查 TF，把落点投影到动态框所在
     坐标系（默认 base_link）；投影后先丢掉落在车体矩形内的点（雷达扫到的
     自身车体/支架/另一颗雷达，由 --profile 与 --body-margin 控制）；
  2. 订阅 `car_op_zones/{stop,slow_l2,slow_l1}`（TRANSIENT_LOCAL，由
     car_op_zones.py --drive-monitor 按 /car_op_status + /motion_state 动态生成）；
  3. 对三级多边形做点在多边形内判断，按 L3 > L2 > L1 优先级翻译成枚举；
  4. 按当前枚举选择发布周期（STOP 80 Hz / SLOWDOWN 40 Hz / 其余 25 Hz），
     状态变化时立即补发一帧；
  5. 把 velocity_smoother 的 `/cmd_vel_collision_in` 原样透传到 `/cmd_vel`，
     只转发、不改速。真实运控按枚举自行减速／停车。

雷达或动态框失联超时 → ERROR（失联时的安全档）。TF 查不到时该帧丢弃；若连续
超过 --scan-timeout 没有任何一帧处理成功，同样判 ERROR。
"""
import argparse
import os
import time

# 每档的发布周期（秒）
DEFAULT_PERIOD = 1.0 / 25.0
SLOWDOWN_PERIOD = 1.0 / 40.0
STOP_PERIOD = 1.0 / 80.0


class ObStatus:
    ERROR = 0
    STOP = 1
    SLOWDOWN = 2
    WARNING = 3
    CLEAR = 4


NAME_OF = {
    ObStatus.ERROR: 'ERROR',
    ObStatus.STOP: 'STOP',
    ObStatus.SLOWDOWN: 'SLOWDOWN',
    ObStatus.WARNING: 'WARNING',
    ObStatus.CLEAR: 'CLEAR',
}


def _default_zones_dir():
    """源码树 → 安装目录，与 car_op_zones.py 的查找顺序一致。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, '..', '..', 'doc', 'collision_zones')),
        os.path.normpath(os.path.join(here, '..', 'share', 'nav2_collision_monitor',
                                      'collision_zones')),
        os.path.normpath(os.path.join(here, '..', '..', 'share', 'nav2_collision_monitor',
                                      'collision_zones')),
    ]
    for path in candidates:
        if os.path.isfile(os.path.join(path, 'gen_zones.py')):
            return path
    return candidates[0]


_DEFAULT_ZONES_DIR = _default_zones_dir()


def load_gen_zones(zones_dir):
    """导入尺寸设计文件；只用标准库，不需要 ROS。"""
    import importlib.util
    path = os.path.join(zones_dir, 'gen_zones.py')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'找不到 {path}')
    spec = importlib.util.spec_from_file_location('gen_zones', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _parse_rename(value):
    rename = {}
    for pair in str(value or '').split(','):
        if ':' in pair:
            old, new = (part.strip() for part in pair.split(':', 1))
            if old and new:
                rename[old] = new
    return rename


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--status-topic', default='/collision_state',
                        help='主状态话题；默认与 regulated 的 collision_state_topic 一致')
    parser.add_argument('--status-type', choices=['uint8', 'int32'], default='uint8',
                        help='主状态话题的消息类型；regulated 栈订阅 UInt8')
    parser.add_argument('--legacy-status-topic', default='/obstacle_status_int',
                        help='同时发布的 Int32 旧接口；传空串关闭')
    parser.add_argument('--stop-topic', default='/car_op_zones/stop')
    parser.add_argument('--slow-l2-topic', default='/car_op_zones/slow_l2')
    parser.add_argument('--slow-l1-topic', default='/car_op_zones/slow_l1')
    parser.add_argument('--zone-frame', default='base_link',
                        help='动态框所在坐标系（car_op_zones.py --frame-id），'
                             '须与雷达同属机器人本体、由 /tf_static 提供外参')
    parser.add_argument('--cmd-in-topic', default='/cmd_vel_collision_in')
    parser.add_argument('--cmd-out-topic', default='/cmd_vel')
    parser.add_argument('--no-passthrough', dest='passthrough',
                        action='store_false',
                        help='只发状态，不转发速度（collision_monitor 仍接管 cmd_vel 时用）')
    parser.add_argument('--min-points', type=int, default=3,
                        help='框内至少多少个落点才判该档，抑制噪点')
    parser.add_argument('--scan-timeout', type=float, default=0.5,
                        help='超过多久没有有效雷达帧判 ERROR（秒）')
    parser.add_argument('--zone-timeout', type=float, default=1.0,
                        help='动态框超过多久未更新判 ERROR（秒）')
    parser.add_argument('--cmd-timeout', type=float, default=0.5,
                        help='cmd 输入超过多久没更新时补发一次零速（秒）')
    parser.add_argument('--tf-retry-window', type=float, default=0.2,
                        help='scan 时间戳的 TF 还没到时最多重试多久（秒），之后改用最新 TF')
    parser.add_argument('--range-max', type=float, default=30.0,
                        help='超过该距离的雷达点直接忽略（米）')
    parser.add_argument('--zones-dir', default=_DEFAULT_ZONES_DIR,
                        help='doc/collision_zones 的位置（含 gen_zones.py）')
    parser.add_argument('--profile', choices=('sim', 'real'), default='real',
                        help='车体尺寸档，用于屏蔽雷达扫到的自身车体回波：'
                             'sim=0.52×0.36 m，real=1.2×0.8 m')
    parser.add_argument('--body-margin', type=float, default=0.05,
                        help='车体屏蔽框在车体外缘外再放出的余量（米）')
    parser.add_argument('--no-body-filter', dest='body_filter', action='store_false',
                        help='不屏蔽车体自身回波（默认屏蔽）')
    parser.add_argument('--rate-clear', type=float, default=1.0 / DEFAULT_PERIOD)
    parser.add_argument('--rate-warning', type=float, default=1.0 / DEFAULT_PERIOD)
    parser.add_argument('--rate-slowdown', type=float, default=1.0 / SLOWDOWN_PERIOD)
    parser.add_argument('--rate-stop', type=float, default=1.0 / STOP_PERIOD)
    # ROS 参数/remap（--ros-args ...）由 rclpy.init 消费，argparse 只认自己的选项。
    args, _ros_argv = parser.parse_known_args(argv)

    if args.min_points < 1:
        parser.error('--min-points 必须 >= 1')
    if args.body_margin < 0.0:
        parser.error('--body-margin 必须 >= 0')

    import numpy as np
    import rclpy
    from geometry_msgs.msg import PolygonStamped, Twist
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Int32, UInt8
    from tf2_msgs.msg import TFMessage
    import tf2_ros

    rclpy.init(args=None)
    node = Node('obstacle_status_translator')

    # 车体自身回波屏蔽。实车 2D 雷达装在车体上，会扫到自己的车体、支架和
    # 另一颗雷达；而 L3 停止框按定义就是「车体外缘 + 净空」，这些点天然落在
    # L3 内，单帧噪点就能让枚举在 4/CLEAR 与 1/STOP 之间反复跳。这里把投影
    # 到 zone_frame 后位于车体矩形（外扩 --body-margin）内的点直接丢掉。
    # 物理上障碍物不可能在车体内部，不影响真实近场检测。
    body_exclusion = None
    if args.body_filter:
        try:
            zones = load_gen_zones(args.zones_dir)
            if hasattr(zones, 'apply_profile'):
                zones.apply_profile(args.profile)
            elif args.profile != 'sim':
                raise RuntimeError(
                    f'{args.zones_dir}/gen_zones.py 不支持尺寸档，无法用 {args.profile} 屏蔽车体')
            m = args.body_margin
            body_exclusion = (zones.R_F + m, zones.R_B + m, zones.R_S + m)
            node.get_logger().info(
                f'车体自身回波屏蔽（{args.profile}）: x[-{body_exclusion[1]:.2f}, '
                f'{body_exclusion[0]:.2f}] y±{body_exclusion[2]:.2f} m')
        except (FileNotFoundError, RuntimeError, ValueError) as exc:
            node.get_logger().warn(f'未加载 gen_zones.py，车体回波不屏蔽: {exc}')

    # ---- QoS ----
    # /scan 由 ros_gz_bridge / 传感器驱动发布，BEST_EFFORT 兼容可靠发布端。
    scan_qos = qos_profile_sensor_data
    # 动态框发布端是 RELIABLE + TRANSIENT_LOCAL，晚启动也能立即拿到最新一帧。
    zone_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
    cmd_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

    # 主接口与 regulated 对齐：UInt8。regulated 的订阅端是 BEST_EFFORT，
    # RELIABLE 发布端对它是兼容的；同时保证 `ros2 topic echo`（默认 reliable）能收到。
    status_msg_cls = UInt8 if args.status_type == 'uint8' else Int32
    status_pub = node.create_publisher(status_msg_cls, args.status_topic, status_qos)
    legacy_pub = None
    if args.legacy_status_topic and not (
            args.status_type == 'int32' and args.status_topic == args.legacy_status_topic):
        legacy_pub = node.create_publisher(Int32, args.legacy_status_topic, status_qos)
    cmd_pub = None
    if args.passthrough:
        cmd_pub = node.create_publisher(Twist, args.cmd_out_topic, cmd_qos)

    # ---- TF ----
    # 动态框与雷达都在机器人本体上（默认 base_link <- base_scan），只需要静态外参。
    # 只订阅 /tf_static 而不挂 TransformListener，避免 50 Hz 的 /tf 回调拖慢
    # 25/40/80 Hz 的发布节拍；静态变换对任意时间戳都有效。
    tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
    tf_static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                               reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_tf_static(msg):
        for transform in msg.transforms:
            try:
                tf_buffer.set_transform_static(transform, 'obstacle_status_translator')
            except tf2_ros.TransformException as exc:
                node.get_logger().warn(f'忽略无法写入的静态 TF: {exc}')

    node.create_subscription(TFMessage, '/tf_static', on_tf_static, tf_static_qos)

    levels = {
        ObStatus.STOP: {'topic': args.stop_topic, 'poly': None, 'bbox': None,
                        'last': 0.0, 'count': 0},
        ObStatus.SLOWDOWN: {'topic': args.slow_l2_topic, 'poly': None, 'bbox': None,
                            'last': 0.0, 'count': 0},
        ObStatus.WARNING: {'topic': args.slow_l1_topic, 'poly': None, 'bbox': None,
                           'last': 0.0, 'count': 0},
    }

    state = {
        'last_frame_ok': 0.0,   # 最近一次成功处理（拿到 TF + 判定）的 monotonic 时间
        'last_scan_msg': 0.0,   # 最近一次收到 scan 的时间（仅诊断）
        'pending_scan': None,   # 等待按时间戳查 TF 的激光帧
        'pending_since': 0.0,
        'last_cmd': 0.0,
        'cmd_zeros_sent': True,
        'status': None,
        'status_since': 0.0,
        'last_publish': 0.0,
        'period': DEFAULT_PERIOD,
        'tf_fail_streak': 0,
        'warned_zone_frame': False,
    }

    def zone_cb(msg, level):
        info = levels[level]
        info['last'] = time.monotonic()
        frame = msg.header.frame_id or args.zone_frame
        if frame != args.zone_frame:
            if not state['warned_zone_frame']:
                state['warned_zone_frame'] = True
                node.get_logger().warn(
                    f'{info["topic"]} 的坐标系是 {frame}，与 --zone-frame '
                    f'{args.zone_frame} 不一致；该层将被跳过')
            info['poly'] = None
            info['bbox'] = None
            return
        pts = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts) < 3:
            info['poly'] = None
            info['bbox'] = None
            return
        poly = np.asarray(pts, dtype=float)
        info['poly'] = poly
        info['bbox'] = (poly[:, 0].min(), poly[:, 0].max(),
                        poly[:, 1].min(), poly[:, 1].max())

    # 闭包按值绑定层号，避免循环变量晚绑定。
    subs = []
    for level in (ObStatus.STOP, ObStatus.SLOWDOWN, ObStatus.WARNING):
        topic = levels[level]['topic']
        subs.append(node.create_subscription(
            PolygonStamped, topic, lambda msg, lv=level: zone_cb(msg, lv), zone_qos))

    def points_in_polygon(points, polygon):
        """向量化射线法；points (N,2) -> bool (N,)。"""
        x = points[:, 0][:, None]
        y = points[:, 1][:, None]
        x1 = polygon[:, 0][None, :]
        y1 = polygon[:, 1][None, :]
        x2 = np.roll(polygon[:, 0], -1)[None, :]
        y2 = np.roll(polygon[:, 1], -1)[None, :]
        crosses = ((y1 > y) != (y2 > y))
        # 水平边不参与交点计算。
        denom = np.where(crosses, y2 - y1, 1.0)
        x_int = (x2 - x1) * (y - y1) / denom + x1
        inside = crosses & (x < x_int)
        return inside.any(axis=1)

    def count_in_level(points_xy, level):
        info = levels[level]
        poly = info['poly']
        if poly is None or points_xy.shape[0] == 0:
            return 0
        xmin, xmax, ymin, ymax = info['bbox']
        # 包围盒预筛，避免对每个点做完整射线法。
        mask = ((points_xy[:, 0] >= xmin) & (points_xy[:, 0] <= xmax) &
                (points_xy[:, 1] >= ymin) & (points_xy[:, 1] <= ymax))
        if not mask.any():
            return 0
        candidates = points_xy[mask]
        return int(points_in_polygon(candidates, poly).sum())

    RETRY = object()   # 该 scan 的 TF 还没进 buffer，下个 tick 再试

    def scan_to_zone_frame(msg):
        """把一帧 LaserScan 的有效落点投到 zone_frame。

        成功返回 (N,2) 数组；TF 暂时没到返回 RETRY；无法处理返回 None。
        全程不阻塞：按 scan 时间戳查不到就留到下一 tick（10 ms 后）重试，
        避免单线程执行器被 TF 等待拖慢发布周期。
        """
        source = msg.header.frame_id or 'base_scan'
        ranges = np.asarray(msg.ranges, dtype=float)
        if ranges.size == 0:
            return None
        angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
        lo = max(float(msg.range_min), 0.01)
        hi = min(float(msg.range_max), args.range_max)
        valid = np.isfinite(ranges) & (ranges >= lo) & (ranges <= hi)
        if not valid.any():
            return np.empty((0, 2))
        r = ranges[valid]
        a = angles[valid]
        points = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)

        if source != args.zone_frame:
            try:
                tf = tf_buffer.lookup_transform(
                    args.zone_frame, source, Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.0))
            except tf2_ros.ExtrapolationException as exc:
                # TF 帧通常比 scan 晚几毫秒发布；窗口内先挂起，下个 tick 再查。
                if time.monotonic() - state['pending_since'] < args.tf_retry_window:
                    return RETRY
                state['tf_fail_streak'] += 1
                if state['tf_fail_streak'] % 20 == 1:
                    node.get_logger().warn(
                        f'按 scan 时间戳查 {args.zone_frame}<-{source} 超过 '
                        f'{args.tf_retry_window:.2f}s 仍不可用: {exc}；改用最新 TF')
                try:
                    tf = tf_buffer.lookup_transform(
                        args.zone_frame, source, Time(), timeout=Duration(seconds=0.0))
                except tf2_ros.TransformException:
                    return None
            except tf2_ros.TransformException as exc:
                # 本仿真里 base_scan -> base_link 是静态的；时间戳基准一旦没对齐，
                # 按最新可用帧兜底，保证不因单帧抖动丢检测。
                state['tf_fail_streak'] += 1
                if state['tf_fail_streak'] % 20 == 1:
                    node.get_logger().warn(
                        f'按 scan 时间戳查 {args.zone_frame}<-{source} 失败 '
                        f'(连续 {state["tf_fail_streak"]} 帧): {exc}；改用最新 TF')
                try:
                    tf = tf_buffer.lookup_transform(
                        args.zone_frame, source, Time(), timeout=Duration(seconds=0.0))
                except tf2_ros.TransformException:
                    return None
            else:
                state['tf_fail_streak'] = 0

            q = tf.transform.rotation
            t = tf.transform.translation
            # yaw（本场景只有平面运动）
            yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = np.cos(yaw), np.sin(yaw)
            rotated = np.empty_like(points)
            rotated[:, 0] = c * points[:, 0] - s * points[:, 1] + t.x
            rotated[:, 1] = s * points[:, 0] + c * points[:, 1] + t.y
            points = rotated

        if body_exclusion is not None and points.shape[0] > 0:
            front, rear, side = body_exclusion
            inside_body = ((points[:, 0] >= -rear) & (points[:, 0] <= front) &
                           (np.abs(points[:, 1]) <= side))
            if inside_body.any():
                points = points[~inside_body]
        return points

    def evaluate(msg):
        points = scan_to_zone_frame(msg)
        if points is RETRY:
            return RETRY, {}
        if points is None:
            return None, {}
        counts = {level: count_in_level(points, level)
                  for level in (ObStatus.STOP, ObStatus.SLOWDOWN, ObStatus.WARNING)}
        if counts[ObStatus.STOP] >= args.min_points:
            status = ObStatus.STOP
        elif counts[ObStatus.SLOWDOWN] >= args.min_points:
            status = ObStatus.SLOWDOWN
        elif counts[ObStatus.WARNING] >= args.min_points:
            status = ObStatus.WARNING
        else:
            status = ObStatus.CLEAR
        return status, counts

    def on_scan(msg):
        # 只登记最新一帧；投影放在 tick 里做，给 TF 一点追帧时间。
        now = time.monotonic()
        state['last_scan_msg'] = now
        state['pending_scan'] = msg
        state['pending_since'] = now

    node.create_subscription(LaserScan, args.scan_topic, on_scan, scan_qos)

    def process_pending():
        msg = state['pending_scan']
        if msg is None:
            return
        status, counts = evaluate(msg)
        if status is RETRY:
            return
        state['pending_scan'] = None
        if status is None:
            return
        state['last_frame_ok'] = time.monotonic()
        for level, info in levels.items():
            info['count'] = counts.get(level, 0)
        state['scan_status'] = status

    def on_cmd(msg):
        now = time.monotonic()
        state['last_cmd'] = now
        state['cmd_zeros_sent'] = False
        if cmd_pub is not None:
            # 只透传，不改速；减速/停车由 regulated / 运控按状态枚举执行。
            cmd_pub.publish(msg)

    if args.passthrough:
        node.create_subscription(Twist, args.cmd_in_topic, on_cmd, cmd_qos)

    def rate_for(status):
        if status == ObStatus.STOP:
            return max(1e-3, args.rate_stop)
        if status == ObStatus.SLOWDOWN:
            return max(1e-3, args.rate_slowdown)
        if status == ObStatus.WARNING:
            return max(1e-3, args.rate_warning)
        return max(1e-3, args.rate_clear)

    def current_status(now):
        """失联判断优先于检测结果：雷达/动态框不能用时一律 ERROR。"""
        for level, info in levels.items():
            if info['poly'] is None or now - info['last'] > args.zone_timeout:
                return ObStatus.ERROR, f'动态框 {levels[level]["topic"]} 未就绪/超时'
        if now - state['last_frame_ok'] > args.scan_timeout:
            return ObStatus.ERROR, f'{args.scan_timeout:.2f}s 内无有效雷达帧'
        return state.get('scan_status', ObStatus.ERROR), ''

    def publish_status(status, reason, now):
        status_pub.publish(status_msg_cls(data=status))
        if legacy_pub is not None:
            legacy_pub.publish(Int32(data=status))
        period = 1.0 / rate_for(status)
        state['last_publish'] = now
        # 定时器周期即为当前档位的发布周期：用精确周期定时器而不是"10 ms tick +
        # 到期判断"，后者会因浮点/调度抖动退化成 20 Hz 左右。
        if abs(period - state['period']) > 1e-9:
            state['period'] = period
            timer.timer_period_ns = int(period * 1e9)
            timer.reset()
        if status != state['status']:
            if status == ObStatus.ERROR:
                node.get_logger().warn(
                    f'{args.status_topic} <- 0 (ERROR): {reason}')
            else:
                detail = ' '.join(
                    f'{NAME_OF[lv]}={levels[lv]["count"]}'
                    for lv in (ObStatus.STOP, ObStatus.SLOWDOWN, ObStatus.WARNING))
                node.get_logger().info(
                    f'{args.status_topic} <- {status} ({NAME_OF[status]}) '
                    f'[{detail}] 周期 {1.0 / period:.0f} Hz')
            state['status'] = status
            state['status_since'] = now

    def tick():
        # 定时器周期由当前档位决定，因此每次回调都发一帧（状态变化也立即体现）。
        process_pending()
        now = time.monotonic()
        status, reason = current_status(now)
        publish_status(status, reason, now)

        # 速度透传看门狗：上游停止发 cmd 时补一帧零速，避免运控保持旧速度。
        if (cmd_pub is not None and state['last_cmd'] > 0.0
                and not state['cmd_zeros_sent']
                and now - state['last_cmd'] > args.cmd_timeout):
            cmd_pub.publish(Twist())
            state['cmd_zeros_sent'] = True
            node.get_logger().warn(
                f'{args.cmd_timeout:.2f}s 未收到 {args.cmd_in_topic}，已补发零速')

    # 周期随状态切换：CLEAR/WARNING 25 Hz、SLOWDOWN 40 Hz、STOP 80 Hz。
    timer = node.create_timer(state['period'], tick)

    status_targets = f'{args.status_topic} ({status_msg_cls.__name__})'
    if legacy_pub is not None:
        status_targets += f' + {args.legacy_status_topic} (Int32)'
    node.get_logger().info(
        f'就绪: {args.scan_topic} + {args.stop_topic}/{args.slow_l2_topic}/'
        f'{args.slow_l1_topic} -> {status_targets} 25/40/80 Hz'
        + (f' | 透传 {args.cmd_in_topic} -> {args.cmd_out_topic}'
           if cmd_pub is not None else ' | 仅发状态'))

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
