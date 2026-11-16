#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 /car_op_status 翻译成设计文档里的三级碰撞边界，并发布为 PolygonStamped。

数字一个都不在本文件里重抄：全部 import doc/collision_zones/gen_zones.py，
取它的 STATES[*]["layers"] 与 shape_pts()。design 改了，本工具跟着改，不会各写一份。

尺寸档由 --profile 选择：sim=0.52×0.36 m 仿真车体，real=1.2×0.8 m 实车外廓。
默认 real（漏传参数时按更保守的实车尺寸，避免实车上用仿真小框）。

两条输出路径，互相独立：
    · 可视化（默认）：collision_stop_zone / collision_slowdown_zone /
      collision_slowdown_l1，恰好是 collision_boundary_visualizer 默认订阅的三个话题。
    · 真实防护（--drive-monitor）：car_op_zones/{stop,slow_l2,slow_l1}，
      配合 params/car_op_zones_demo.yaml 里 type: "polygon" + polygon_sub_topic 使用。

两个已知的接口缺口（详见 doc/OBSTACLE_STATUS.md）：
    · 枚举不带方向位，NAV 的正走／倒走只能靠 /motion_state 的 v_car 符号推
    · 枚举不载货位，空载／载货只能靠 lift_height 推
枚举之外的 8 去充电／9 充电结束离开照收，但每次都告警。
"""
import argparse
import math
import os
import re
import sys
import time

def _default_zones_dir():
    """按「源码树 → 安装目录」的顺序找 gen_zones.py 所在目录。

    源码树：tools/ 上溯两级即仓库根目录，其下 doc/collision_zones。
    安装后：CMakeLists 把 gen_zones.py 与 spec_table.md 装到 share/<pkg>/collision_zones。
    用 ros2 run 时走第二种，直接跑源码时走第一种。
    """
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
    return candidates[0]   # 都找不到就报第一个，错误信息里给出期望路径


_DEFAULT_ZONES_DIR = _default_zones_dir()


def load_gen_zones(zones_dir):
    """从给定目录导入样例生成器；它只用标准库，不需要 ROS。"""
    import importlib.util
    path = os.path.join(zones_dir, 'gen_zones.py')
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'找不到 {path}；用 --zones-dir 指定 doc/collision_zones 的位置')
    spec = importlib.util.spec_from_file_location('gen_zones', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# 接口枚举值 → 设计文档状态号。枚举 0–8 ↔ 设计状态 1–9，一一对应、差 1。
OP_TO_STATE = {v: v + 1 for v in range(9)}
OP_TO_NAME = {0: '空载正走', 1: '空载倒走', 2: '正走抓料框', 3: '搬货倒走',
              4: '搬货正走', 5: '静止', 6: '原地旋转', 7: '去充电', 8: '充电结束离开'}

# 设计状态号 → 分档轴。'v' 按 |v_car|，'w' 按 |w_car|，'fixed' 单档。
STATE_AXIS = {1: 'v', 2: 'v', 3: 'fixed', 4: 'v', 5: 'v', 6: 'fixed', 7: 'w', 8: 'v', 9: 'v'}

LV_KEYS = ('L3 停止', 'L2 减速', 'L1 预警')


class Resolver:
    """把 (枚举值, v_car, w_car) 解析成设计状态号与档位号。

    枚举值 0–8 ↔ 设计状态 1–9，一一对应。空载/载货、正走/倒走已内建在设计
    状态里，不需要再从 v_car／lift_height 推导方向或载货。
    档位号按该状态的 axis 由 |v_car| 或 |w_car| 选出。
    """

    def __init__(self, zones, stop_speed=0.02):
        self.z = zones
        self.stop_speed = stop_speed

    def resolve(self, op, v_car, lift_height):
        """返回 (状态号, 是否载货, 说明字符串列表)。"""
        notes = []
        state = OP_TO_STATE.get(op)
        if state is None:
            notes.append(f'枚举值 {op} 未定义，按静止（状态6）处理')
            return 6, False, notes

        # 载货与否由设计状态本身决定（搬货走 = cargo，空载走 / 静止 / 旋转 = 空载）。
        cargo = self.z.STATES[state - 1].get('cargo') is not None
        return state, cargo, notes

    def tier(self, state, v_car, w_car, cargo=False):
        """返回 (档位号, 说明)。超出设计区间时夹到最外档并告警。"""
        axis = STATE_AXIS[state]
        if axis == 'fixed':
            return 0, []
        value = abs(v_car) if axis == 'v' else abs(w_car)
        if not math.isfinite(value):
            return 0, [f'{axis} 非有限值，取第 0 档']
        state_obj = self.z.STATES[state - 1]
        tiers = state_obj['tiers']
        bounds = _tier_bounds(tiers)
        if bounds is None:
            return 0, []

        index = None
        for candidate, (lo, hi) in enumerate(bounds):
            if lo <= value < hi:
                index = candidate
                break
        notes = []
        if index is None:
            if value < bounds[0][0]:
                index = 0
                notes.append(f'|{axis}|={value:.3f} 低于第 0 档下界，取第 0 档')
            else:
                index = len(bounds) - 1
                notes.append(f'|{axis}|={value:.3f} 超出最高档上界 {bounds[-1][1]}，'
                             f'夹到第 {len(bounds) - 1} 档（设计未覆盖该速度）')

        # 载货旋转的兜底：设计文档明确要求 L3 半径 ≥ 货框外接半径（CARGO_R ≈ 0.694 m），
        # 否则转起来货框四角会捅出停止层。档位是按 |w| 选的，慢速档的半径不够，
        # 这里往上抬到第一个够大的档，并说明抬了。
        if cargo and state_obj['kind'] == 'circle':
            need = self.z.CARGO_R
            if state_obj['layers'][index][0] < need:
                for candidate in range(index + 1, len(bounds)):
                    if state_obj['layers'][candidate][0] >= need:
                        notes.append(
                            f'载货旋转：第 {index} 档 L3 半径 '
                            f'{state_obj["layers"][index][0]:.2f} m 小于货框外接半径 '
                            f'{need:.3f} m，抬到第 {candidate} 档')
                        index = candidate
                        break
                else:
                    notes.append(
                        f'载货旋转：全部档位的 L3 半径都小于货框外接半径 {need:.3f} m，'
                        f'停在最高档 —— 该状态下货框四角无保护')
        return index, notes


def _tier_bounds(tiers):
    """把 gen_zones 的档位标签（如 "0.0–0.3"／"固定"）解析成数值区间。

    gen_zones 的标签用的是 en dash（U+2013），这里容错处理。
    """
    if not tiers or tiers[0] == '固定':
        return None
    bounds = []
    for label in tiers:
        cleaned = label.replace('–', '-').replace('—', '-')
        parts = cleaned.split('-')
        if len(parts) != 2:
            return None
        try:
            bounds.append((float(parts[0]), float(parts[1])))
        except ValueError:
            return None
    return bounds


def polygon_points(zones, state, tier, lv):
    """该状态该档该层的多边形顶点，直接取自样例生成器。

    gen_zones.shape_pts 收的是 state 字典、且参数序是 (state, lv, tier)，
    这里统一成对外的 (状态号, 档, 层) 顺序，避免调用点记错。
    """
    return zones.shape_pts(zones.STATES[state - 1], lv, tier)


def polygon_stamped(zones, state, tier, lv, stamp, frame_id):
    from geometry_msgs.msg import PolygonStamped
    from geometry_msgs.msg import Point32
    msg = PolygonStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for x, y in polygon_points(zones, state, tier, lv):
        p = Point32()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        msg.polygon.points.append(p)
    return msg


def describe(zones, state, tier):
    """一行摘要：状态名 + 档位 + 三级净空（直接引用 gen_zones 的净空函数）。"""
    st = zones.STATES[state - 1]
    kind = st['kind']
    parts = []
    for lv in range(3):
        if kind == 'circle':
            parts.append(f'{zones.fmt(zones.cl_circle(st["layers"], tier, lv))}')
        else:
            parts.append(zones.cl_rect_str(st['layers'], tier, lv,
                                           zones.EDGE_CARGO if st.get('cargo') else zones.EDGE_IDLE))
    tier_label = st['tiers'][tier] if tier < len(st['tiers']) else '?'
    return (f'状态{state} {st["name"]}（{tier_label}）  '
            f'L3 {parts[0]} | L2 {parts[1]} | L1 {parts[2]}  [行进向/反向/侧向 净空 m]')


# ---------------- 自检：发布的顶点 ↔ layers ↔ spec_table.md ---------------- #

_ROW = re.compile(r'^\|\s*(\d+)\s*\|.*?\|\s*`([a-z_]+)`\s*\|\s*(.*?)\s*\|.*?\|(.*)$')


def parse_spec(path):
    """把 spec_table.md 解析成 {状态号: [(档位标签, [三层净空串, ...]), ...]}。

    注意：原地旋转行的「分档轴」列写的是 |w|  (rad/s)，竖线会被按分隔符切开，
    所以三列净空一律从右侧取，不能按固定下标从左数。
    """
    rows = {}
    with open(path, encoding='utf-8') as stream:
        for raw in stream:
            if not raw.startswith('|'):
                continue
            cells = [c.strip() for c in raw.strip().strip('|').split('|')]
            if len(cells) < 8 or not cells[0].isdigit():
                continue
            state_no = int(cells[0])
            tier_label = cells[3]
            rows.setdefault(state_no, []).append((tier_label, cells[-3:]))
    return rows


def self_check(zones, spec_path, verbose=True):
    """断言：发出的顶点 == layers，且 layers 的净空 == spec_table.md 的文字。"""
    failures = []

    def fail(msg):
        failures.append(msg)

    # 1) 顶点 ↔ layers：对全部 9 状态 × 全部档 × 3 层逐点核对
    checked = 0
    for index, st in enumerate(zones.STATES):
        for tier in range(len(st['tiers'])):
            for lv in range(3):
                pts = polygon_points(zones, index + 1, tier, lv)
                if len(pts) < 3:
                    fail(f'状态{index + 1} 档{tier} L{lv}: 顶点不足 3 个')
                    continue
                if st['kind'] == 'circle':
                    r = st['layers'][tier][lv]
                    for x, y in pts:
                        if abs(math.hypot(x, y) - r) > 1e-9:
                            fail(f'状态{index + 1} 档{tier} L{lv}: '
                                 f'顶点半径 {math.hypot(x, y):.6f} != {r}')
                            break
                else:
                    F, B, S = st['layers'][tier][lv]
                    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
                    want = ((F, -B) if st['dir'] > 0 else (B, -F))
                    if abs(max(xs) - want[0]) > 1e-9 or abs(min(xs) - want[1]) > 1e-9:
                        fail(f'状态{index + 1} 档{tier} L{lv}: x 跨度 '
                             f'[{min(xs):.4f},{max(xs):.4f}] != [{want[1]:.4f},{want[0]:.4f}]')
                    if abs(max(ys) - S) > 1e-9 or abs(min(ys) + S) > 1e-9:
                        fail(f'状态{index + 1} 档{tier} L{lv}: y 跨度不对')
                checked += 1

    # 2) layers ↔ spec_table.md：文字与数值同源
    spec_rows = 0
    if spec_path and os.path.isfile(spec_path):
        table = parse_spec(spec_path)
        for state_no, rows in sorted(table.items()):
            st = zones.STATES[state_no - 1]
            if len(rows) != len(st['tiers']):
                fail(f'状态{state_no}: spec_table 有 {len(rows)} 行，'
                     f'design 有 {len(st["tiers"])} 档')
            for tier, (label, cells) in enumerate(rows):
                if tier >= len(st['tiers']):
                    break
                for lv in range(3):
                    cell = cells[lv]
                    if st['kind'] == 'circle':
                        m = re.search(r'净空\s*(-?\d+\.\d+)', cell)
                        if not m:
                            fail(f'状态{state_no} 档{tier} L{lv}: 无法解析 "{cell}"')
                            continue
                        want = zones.fmt(zones.cl_circle(st['layers'], tier, lv))
                        if m.group(1) != want:
                            fail(f'状态{state_no} 档{tier} L{lv}: spec "{m.group(1)}" '
                                 f'!= design "{want}"')
                    else:
                        nums = re.findall(r'-?\d+\.\d+', cell)
                        if len(nums) != 3:
                            fail(f'状态{state_no} 档{tier} L{lv}: 期望 3 个净空，得 {cell!r}')
                            continue
                        edge = zones.EDGE_CARGO if st.get('cargo') else zones.EDGE_IDLE
                        want = [zones.fmt(v) for v in zones.cl_rect(st['layers'], tier, lv, edge)]
                        if nums != want:
                            fail(f'状态{state_no} 档{tier} {LV_KEYS[lv]}: '
                                 f'spec {nums} != design {want}')
                    spec_rows += 1
    elif spec_path:
        fail(f'找不到 spec_table：{spec_path}')

    if verbose:
        print(f'  顶点核对：{checked} 组（9 状态 × 各档 × 3 层）')
        print(f'  spec 核对：{spec_rows} 个单元格')
    if failures:
        for msg in failures:
            print(f'  FAIL {msg}')
        print(f'self_check: {len(failures)} 处不一致')
        return False
    print('self_check: ok')
    return True


# ---------------- 运行 ---------------- #

def make_markers(zones, state, tier, stamp, frame_id):
    """按设计文档锁定的配色画线框 + 标签。

    自己发 markers 而不是靠 collision_boundary_visualizer，是因为它按话题名猜颜色，
    collision_slowdown_zone 与 collision_slowdown_l1 都含 "slow" → 同色同标签，
    L1/L2 分不出来。这里能还原设计图的绿／橙／红。
    """
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray

    colors = ((0.898, 0.224, 0.208),   # L3 #e53935
              (0.984, 0.549, 0.000),   # L2 #fb8c00
              (0.263, 0.627, 0.278))   # L1 #43a047
    st = zones.STATES[state - 1]
    array = MarkerArray()

    for lv in range(3):
        pts = polygon_points(zones, state, tier, lv)
        line = Marker()
        line.header.stamp = stamp
        line.header.frame_id = frame_id
        line.ns = 'car_op_zones'
        line.id = lv
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.02
        line.color.r, line.color.g, line.color.b = colors[lv]
        line.color.a = 0.95
        line.frame_locked = True
        for x, y in list(pts) + [pts[0]]:   # 闭合
            p = Point()
            p.x, p.y, p.z = float(x), float(y), 0.0
            line.points.append(p)
        array.markers.append(line)

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = frame_id
        text.ns = 'car_op_zones_label'
        text.id = 10 + lv
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.16
        text.color.r, text.color.g, text.color.b = colors[lv]
        text.color.a = 1.0
        text.frame_locked = True
        # 贴在多边形最前端的角上，按方向翻符号
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        front = max(xs) if st['dir'] >= 0 else min(xs)
        text.pose.position.x = float(front)
        text.pose.position.y = float(max(ys))
        text.pose.position.z = 0.0
        if st['kind'] == 'circle':
            clearance = zones.cl_circle(st['layers'], tier, lv)
            text.text = f'{LV_KEYS[lv]}  R={st["layers"][tier][lv]:.2f}  净空 {clearance:.2f}'
        else:
            edge = zones.EDGE_CARGO if st.get('cargo') else zones.EDGE_IDLE
            clearance = zones.cl_rect(st['layers'], tier, lv, edge)
            text.text = (f'{LV_KEYS[lv]}  净空 {clearance[0]:.2f}/{clearance[1]:.2f}'
                         f'/{clearance[2]:.2f}')
        array.markers.append(text)

    # 状态标题
    title = Marker()
    title.header.stamp = stamp
    title.header.frame_id = frame_id
    title.ns = 'car_op_zones_title'
    title.id = 20
    title.type = Marker.TEXT_VIEW_FACING
    title.action = Marker.ADD
    title.scale.z = 0.20
    title.color.r = title.color.g = title.color.b = 1.0
    title.color.a = 1.0
    title.frame_locked = True
    title.pose.position.x = 0.0
    title.pose.position.y = 0.0
    title.pose.position.z = 0.6
    title.text = f'状态{state} {st["name"]}'
    array.markers.append(title)

    return array


def run(args):
    zones = load_gen_zones(args.zones_dir)
    if hasattr(zones, 'apply_profile'):
        zones.apply_profile(args.profile)
    elif args.profile != 'sim':
        raise RuntimeError(
            f'{args.zones_dir}/gen_zones.py 不支持尺寸档，无法切到 {args.profile}；'
            '请更新该文件后重试')

    spec_name = 'spec_table_real.md' if args.profile == 'real' else 'spec_table.md'
    spec = args.spec or os.path.join(args.zones_dir, spec_name)
    if args.self_check:
        return 0 if self_check(zones, spec) else 1

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy, qos_profile_sensor_data)
    from std_msgs.msg import Int32

    from byd_custom_msgs.msg import MotionState
    from geometry_msgs.msg import PolygonStamped

    rclpy.init(args=None)
    node = Node('car_op_zones')

    # transient_local：既兼容 collision_monitor 的默认 volatile 订阅，
    # 也兼容 collision_boundary_visualizer 要求的 transient_local。
    # 效果是后启动的 RViz／监控节点立刻能拿到最后一帧。
    poly_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # 输入侧与发布端对齐：/car_op_status 是 BEST_EFFORT。
    op_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.VOLATILE)

    viz_topics = ['collision_stop_zone', 'collision_slowdown_zone', 'collision_slowdown_l1']
    demo_topics = [f'car_op_zones/{n}' for n in ('stop', 'slow_l2', 'slow_l1')]

    viz_pubs = [node.create_publisher(PolygonStamped, t, poly_qos) for t in viz_topics]
    demo_pubs = ([node.create_publisher(PolygonStamped, t, poly_qos) for t in demo_topics]
                 if args.drive_monitor else [])
    marker_pub = None
    if args.markers:
        from visualization_msgs.msg import MarkerArray
        marker_pub = node.create_publisher(MarkerArray, args.marker_topic, poly_qos)

    state = {'op': None, 'v': 0.0, 'w': 0.0, 'lift': 0.0,
             'motion_seen': False, 'resolved': None, 'tier': None}
    started = time.monotonic()
    motion_warned = {'done': False}

    resolver = Resolver(zones, args.stop_speed)

    def on_op(msg):
        state['op'] = int(msg.data)

    def on_motion(msg):
        state['motion_seen'] = True
        state['v'] = float(msg.v_car)
        state['w'] = float(msg.w_car)
        state['lift'] = float(msg.lift_height)

    node.create_subscription(Int32, args.op_topic, on_op, op_qos)
    # 与 regulated 的 MotionStateSubscriber（SensorDataQoS / BEST_EFFORT）保持一致：
    # BEST_EFFORT 订阅端同时兼容实车运控的 BEST_EFFORT 和仿真的 RELIABLE 发布端。
    node.create_subscription(
        MotionState, args.motion_topic, on_motion, qos_profile_sensor_data)

    published = {'last': None}

    def tick():
        if state['op'] is None:
            return
        if (not state['motion_seen'] and not motion_warned['done']
                and time.monotonic() - started > 2.0):
            motion_warned['done'] = True
            node.get_logger().warn(
                f'2 秒内未收到 {args.motion_topic}，避障框固定在第 0 档；'
                '实车请确认运控发布 MotionState（BEST_EFFORT）')
        op = state['op']
        resolved, cargo, notes = resolver.resolve(op, state['v'], state['lift'])
        tier, tier_notes = resolver.tier(resolved, state['v'], state['w'], cargo)
        stamp = node.get_clock().now().to_msg()

        # 三层到话题的映射：viz 列表顺序为 stop(L3)、slowdown(L2)、l1(L1)，
        # 与 NAME_OF_LV 的 lv 索引一致。
        for lv, pub in enumerate(viz_pubs):
            pub.publish(polygon_stamped(zones, resolved, tier, lv, stamp, args.frame_id))
        for lv, pub in enumerate(demo_pubs):
            pub.publish(polygon_stamped(zones, resolved, tier, lv, stamp, args.frame_id))
        if marker_pub is not None:
            marker_pub.publish(make_markers(zones, resolved, tier, stamp, args.frame_id))

        key = (op, resolved, tier)
        if key != published['last'] or args.verbose:
            published['last'] = key
            for note in notes + tier_notes:
                node.get_logger().warn(note)
            node.get_logger().info(
                f'{args.op_topic}={op} ({OP_TO_NAME.get(op, "未定义")}) -> '
                f'{describe(zones, resolved, tier)}')

    node.create_timer(args.period, tick)
    node.get_logger().info(
        f'就绪（尺寸档 {args.profile}，车体半长前/后/半宽 '
        f'{zones.R_F:.2f}/{zones.R_B:.2f}/{zones.R_S:.2f} m）：'
        f'{args.op_topic} + {args.motion_topic} -> '
        f'{len(viz_pubs)} 路可视化'
        + (f' + {len(demo_pubs)} 路监控' if demo_pubs else '')
        + (f' + markers({args.marker_topic})' if marker_pub else ''))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # SIGTERM／Ctrl+C 正常收尾；不打印 traceback。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--zones-dir', default=_DEFAULT_ZONES_DIR,
                        help='doc/collision_zones 的位置（含 gen_zones.py）')
    parser.add_argument('--profile', choices=('sim', 'real'), default='real',
                        help='尺寸档：sim=仿真车体 0.52×0.36 m，'
                             'real=实车外廓 1.2×0.8 m（默认 real，更保守）')
    parser.add_argument('--spec', help='spec_table 路径，缺省取 zones-dir 下与档位对应的文件')
    parser.add_argument('--self-check', action='store_true',
                        help='只核对顶点与 spec_table.md，不启动 ROS')
    parser.add_argument('--op-topic', default='/car_op_status')
    parser.add_argument('--motion-topic', default='/motion_state')
    parser.add_argument('--marker-topic', default='/car_op_zones/markers')
    parser.add_argument('--frame-id', default='base_link')
    parser.add_argument('--period', type=float, default=0.1,
                        help='重发周期，秒；与状态发布同频即可')
    parser.add_argument('--stop-speed', type=float, default=0.02,
                        help='|v_car| 低于此值视为静止档（保留参数，仅作展示）')
    parser.add_argument('--drive-monitor', action='store_true',
                        help='额外发布 car_op_zones/* 供 demo YAML 的真实 collision_monitor 订阅')
    parser.add_argument('--no-markers', dest='markers', action='store_false',
                        help='不发布自带配色的 MarkerArray')
    parser.add_argument('--verbose', action='store_true',
                        help='状态没变也打印（默认只在变化时打印）')
    # ROS 参数/remap（--ros-args ...）由 rclpy.init 消费，argparse 只认自己的选项。
    args, _ros_argv = parser.parse_known_args(argv)

    if not math.isfinite(args.period) or args.period <= 0:
        parser.error('--period 须为有限正数')

    try:
        return run(args)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
