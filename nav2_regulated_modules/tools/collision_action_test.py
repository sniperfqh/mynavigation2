#!/usr/bin/env python3
# 连接已有仿真的 collision 注入与 1.5 m/s Action 例子；不直接发布底盘速度或合成定位。
# periodic 独立按 50 Hz 切换状态，cruise 等待速度条件；只有相同区间、相同任务内的合格样本才能组成证据。
# 退出只取消自身 UUID 并观察停稳，工具 Ctrl+C 同时改变状态与任务，不能代替独立状态源断流验收。
"""在现有仿真中周期注入 collision 状态；只取消本工具的 Action。"""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import time


# 保存要求的连续墙钟时长 seconds，初始无起点、无上次观察时刻。
class StageWindow:
    """单调时钟上的连续证据窗，缺帧或不满足条件立即重置。"""
    # 保存要求的连续墙钟时长 seconds，初始无起点、无上次观察时刻。
    def __init__(self, seconds):
        self.seconds = seconds
        self.start = None
        self.previous = None

    # 条件为假或观察 gap>0.25 s 时重置起点；连续有效累计到 seconds 才返回 true。
    # now 来自单调时钟，不把长时间没观察也计作有效保持。
    def update(self, now, valid):
        if not valid or (self.previous is not None and now - self.previous > 0.25):
            self.start = None
        self.previous = now
        if not valid:
            return False
        if self.start is None:
            self.start = now
        return now - self.start >= self.seconds


# 要求五个状态保持时长都为有限正数，保存各段时长与整周期总长 s。
class PeriodicSchedule:
    states = (4, 3, 2, 1, 0)

    # 要求五个状态保持时长都为有限正数，保存各段时长与整周期总长 s。
    def __init__(self, durations):
        if len(durations) != 5 or any(not math.isfinite(v) or v <= 0 for v in durations):
            raise ValueError('五个状态保持时间必须为有限正数')
        self.durations = tuple(durations)
        self.period = sum(durations)

    # 从 elapsed 墙钟秒计算周期号、区间号和状态；边界时刻进入下一段。
    # 状态顺序固定 4、3、2、1、0，回调延迟不会改变周期 epoch。
    def at(self, elapsed):
        elapsed = max(0., elapsed)
        cycle = int(elapsed // self.period)
        offset = elapsed - cycle * self.period
        for index, duration in enumerate(self.durations):
            if offset < duration:
                return cycle, index, self.states[index]
            offset -= duration
        return cycle + 1, 0, 4


# 初始化发布次数、首尾时刻和间隔分布；没有至少两帧时不能计算实测频率。
class PublishStats:
    bounds = (.010, .015, .025, .040, .100, float('inf'))

    # 初始化发布次数、首尾时刻和间隔分布；没有至少两帧时不能计算实测频率。
    def __init__(self):
        self.count = 0
        self.first = self.last = None
        self.minimum = float('inf')
        self.maximum = 0.
        self.histogram = [0] * len(self.bounds)

    # 记录相邻发布调用的单调间隔，并累计到互斥的时间桶。
    # 统计的是本机 publisher.publish 调用时序，不是端到端接收抖动。
    def add(self, now):
        if self.last is not None:
            gap = now - self.last
            self.minimum = min(self.minimum, gap)
            self.maximum = max(self.maximum, gap)
            for i, bound in enumerate(self.bounds):
                if gap <= bound:
                    self.histogram[i] += 1
                    break
        else:
            self.first = now
        self.last = now
        self.count += 1

    # 实测频率 = (次数−1)÷首尾时间差，同时返回间隔极值、均值和直方图。
    # 单帧或无帧时相关值为空，不用 configured_hz 代替测量。
    def result(self):
        span = (self.last - self.first) if self.count > 1 else 0.
        return dict(configured_hz=50., count=self.count,
                    measured_hz=(self.count - 1) / span if span else None,
                    mean_interval_s=span / (self.count - 1) if span else None,
                    min_interval_s=self.minimum if self.count > 1 else None,
                    max_interval_s=self.maximum if self.count > 1 else None,
                    interval_histogram=dict(zip(['≤10ms', '10–15ms', '15–25ms', '25–40ms', '40–100ms', '>100ms'], self.histogram)))


# 以周期／状态／腿／UUID／方向／活动标志固定证据归属，并记录切入初速度。
# 平台、停车和恢复分别维护连续窗口，不能合并不同状态区间的样本。
class IntervalEvidence:
    """按状态区间与任务分段；观察结论绝不控制发布时序。"""
    # 以周期／状态／腿／UUID／方向／活动标志固定证据归属，并记录切入初速度。
    # 平台、停车和恢复分别维护连续窗口，不能合并不同状态区间的样本。
    def __init__(self, key, now, initial_v, initial_w=0.):
        self.key = key
        cycle, index, state, leg, goal_uuid, direction, active = key
        self.record = dict(cycle=cycle, index=index, state=state, leg=leg, goal_uuid=goal_uuid,
                           direction=direction, active_goal=active, start=now,
                           status='UNAVAILABLE', command_status='UNAVAILABLE',
                           wheel_status='UNAVAILABLE', initial_v=initial_v, initial_w=initial_w,
                           samples=0, eligible_samples=0, eligible_hold_s=0., recovered=False)
        self.window = StageWindow(1 if state <= 1 else .3)
        self.wheels = StageWindow(1)
        self.demand = StageWindow(1)
        self.recovery = StageWindow(.2)
        self.last_sample = None

    # 仅处理当前区间内未处理的新 UART 样本；先核对数据、方向、原始需求与离终点距离，再累计平台／停车。
    # 0.3 s 平台和 1 s 停稳分别判定，轮速停车与命令停车各自保存。
    def sample(self, stamp, uart, raw, wheel, remaining, valid):
        if stamp == self.last_sample or stamp < self.record['start']:
            return
        self.last_sample = stamp
        r = self.record
        r['samples'] += 1
        state, direction = r['state'], r['direction']
        finite = all(math.isfinite(x) for x in (*uart, raw, *wheel, remaining))
        straight = abs(uart[1]) <= .05
        cap = 1.5 * {4: 1., 3: .65, 2: .35, 1: 0., 0: 0.}[state]
        eligible = valid and finite and r['active_goal'] and remaining >= .5
        if state > 1:
            eligible = eligible and straight and direction * raw >= cap + (.05 if state < 4 else -.05)
            good = eligible and abs(direction * uart[0] - cap) <= .05
        else:
            good = eligible and abs(uart[0]) <= .01 and abs(uart[1]) <= .05
        if eligible:
            r['eligible_samples'] += 1
        self.demand.update(stamp, eligible)
        if eligible and self.demand.start is not None:
            r['eligible_hold_s'] = max(r['eligible_hold_s'], stamp - self.demand.start)
        if self.window.update(stamp, good):
            r['command_status'] = 'PASS'
        elif state <= 1:
            r['command_status'] = 'UNAVAILABLE'
        wheel_good = eligible and abs(wheel[0]) <= .01 and abs(wheel[1]) <= .05
        if self.wheels.update(stamp, wheel_good):
            r['wheel_status'] = 'PASS'
        elif state <= 1:
            r['wheel_status'] = 'UNAVAILABLE'
        if self.recovery.update(stamp, eligible and direction * uart[0] > .1 and direction * raw > .1):
            r['recovered'] = True

    # 按窗口证据给出 PASS／FAIL／UNAVAILABLE，分别标记区间内制动与进入区间前已静止。
    # 状态 4 未达正常平台仍是不可评价；证据不足不能强行判成碰撞响应失败。
    def finish(self, now, reason):
        r = dict(self.record, end=now, split_reason=reason)
        enough = now - r['start'] >= 2 and r['eligible_hold_s'] >= 1
        if r['command_status'] == 'PASS':
            r['status'] = 'PASS'
        elif enough and r['state'] != 4:
            r['command_status'] = r['status'] = 'FAIL'
        if r['state'] <= 1:
            r['response'] = ('制动停车' if abs(r['initial_v']) > .05 else
                             '转向停止' if abs(r['initial_w']) > .05 else '保持停稳')
            if r['status'] == 'PASS' and r['wheel_status'] != 'PASS':
                r['status'] = 'FAIL' if enough else 'UNAVAILABLE'
        else:
            cap = 1.5 * {4: 1., 3: .65, 2: .35}[r['state']]
            r['response'] = '减速' if abs(r['initial_v']) > cap + .05 else '速度平台／限速保持'
        return r


# 使用方向化速度检查平台，要求直行、足够离终点且原始请求仍高于减速上限。
# 状态 0／1 检查零线角速度，不用已进入终点制动的样本冒充 collision 减速。
def stage_condition(state, uart, raw, remaining, direction=1):
    v, w = uart
    if not all(math.isfinite(x) for x in (*uart, raw, remaining)):
        return False
    if remaining < 0.5:
        return False
    if state <= 1:
        return abs(v) <= 0.01 and abs(w) <= 0.05
    cap = 1.5 * {4: 1.0, 3: 0.65, 2: 0.35}[state]
    return abs(direction*v - cap) <= 0.05 and abs(w) <= 0.05 and direction*raw >= cap + (0.05 if state != 4 else -0.05)


# 检查 UART 与轮速零值及有限性，轮速必须在 0.25 s 内新鲜。
# 只有已确认自身任务终态时，才允许“最后零 UART＋持续新鲜零轮速”的静默退出证据。
def stationary_after_terminal(now, uart, wheel, terminal):
    if uart is None or wheel is None or now-wheel[0]>.25:
        return False
    if not all(math.isfinite(v) for v in (*uart[1:],*wheel[1:])):
        return False
    zero=all(abs(v)<=limit for values in (uart[1:],wheel[1:]) for v,limit in zip(values,(.01,.05)))
    return zero and (now-uart[0]<=.25 or terminal)


# 从原始 CSV 读取 UART，生成不依赖绘图库的 SVG／HTML，保留正负方向和状态上限。
# 预览可降采样，CSV 仍是完整原始证据；缺 UART 时不伪造曲线。
def write_plot(directory):
    # Standalone SVG/HTML avoids a plotting dependency; preserve every sample in CSV.
    points = []
    with (directory / 'velocity.csv').open() as stream:
        for row in csv.DictReader(stream):
            if row['topic'] == '/control_to_uart':
                points.append((float(row['elapsed']), float(row['v']), int(row['state']), int(row.get('direction', 1))))
    if not points:
        return
    width, height = 1100, 360
    duration = max(points[-1][0], 1)
    # 将时间和带符号速度投影为 SVG 坐标；这只生成预览点串，不改变 CSV 原始数据。
    def line(values):
        return ' '.join(f'{50 + t / duration * 1000:.1f},{175 - v / 1.7 * 140:.1f}' for t, v in values)
    # Keep the preview bounded for indefinite runs; CSV remains lossless.
    stride = max(1, len(points) // 10000)
    selected = points[::stride] + points[-1:]
    target = [(t, d * {4: 1.5, 3: .975, 2: .525, 1: 0., 0: 0.}[s]) for t, _, s, d in selected]
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">'
           '<rect width="100%" height="100%" fill="white"/><path d="M50 30V320M50 175H1050" fill="none" stroke="black"/>'
           '<text x="2" y="52">+1.5</text><text x="25" y="180">0</text><text x="2" y="302">−1.5</text>'
           '<text x="50" y="20">v (m/s): blue=control_to_uart, orange=collision target cap</text>'
           f'<polyline points="{line(target)}" fill="none" stroke="orange"/>'
           f'<polyline points="{line([(t,v) for t,v,_,_ in selected])}" fill="none" stroke="#1769aa"/>'
           f'<text x="50" y="340">0 s</text><text x="970" y="340">{duration:.1f} s</text></svg>')
    (directory / 'velocity.svg').write_text(svg)
    (directory / 'velocity.html').write_text('<!doctype html><meta charset="utf-8"><title>collision 速度曲线</title>' + svg)


# 连接已有仿真的 collision 注入与 1.5 m/s Action 例子；不直接发布底盘速度或合成定位。
# 先解析参数与检查运行前提，再按本文件流程采集／处理并写结果；异常和 finally 收口分支不应被注释描述为正常通过。
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--a', nargs=2, type=float, default=[9.16, -4.37])
    parser.add_argument('--b', nargs=2, type=float, default=[13.2, -.47])
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-topic', default='/odometry')
    parser.add_argument('--max-legs', type=int, default=0, help='0 表示持续运行；4 可用于一轮回归')
    parser.add_argument('--schedule', choices=['cruise', 'periodic'], default='cruise')
    parser.add_argument('--return-direction', choices=['forward', 'backward'], default='forward')
    parser.add_argument('--state-durations', nargs=5, type=float, default=[4., 3., 3., 3., 3.], metavar=('S4','S3','S2','S1','S0'))
    args = parser.parse_args(argv)
    if not all(math.isfinite(v) for v in args.a + args.b) or math.dist(args.a, args.b) < 1 or args.max_legs < 0:
        parser.error('端点须有限且距离至少 1 米；max-legs 不得为负')
    try:
        schedule = PeriodicSchedule(args.state_durations)
    except ValueError as error:
        parser.error(str(error))

    import rclpy
    import uuid
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.clock import Clock, ClockType
    from rclpy.parameter import Parameter
    from unique_identifier_msgs.msg import UUID
    from std_msgs.msg import UInt8
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from action_msgs.msg import GoalStatusArray
    from action_msgs.srv import CancelGoal
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import GetState
    from tf2_ros import Buffer, TransformListener
    from byd_custom_msgs.action import NavigationService
    from byd_custom_msgs.msg import NaviSegment, ControlRes

    args.output_dir.mkdir(parents=True, exist_ok=False)
    trace = (args.output_dir / 'trace.jsonl').open('w')
    csvfile = (args.output_dir / 'velocity.csv').open('w', newline='')
    writer = csv.writer(csvfile)
    writer.writerow(['elapsed', 'topic', 'v', 'w', 'state', 'leg', 'direction', 'cycle', 'goal_uuid', 'active_goal'])
    report = {'status': 'PREFLIGHT', 'physical_robot': False, 'speed': 1.5, 'a': args.a, 'b': args.b,
              'schedule': args.schedule, 'return_direction': args.return_direction,
              'state_durations_s': args.state_durations, 'legs': [], 'intervals': []}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('collision_action_test')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    started = time.monotonic()
    latest, statuses = {}, {}
    state = 0
    emitting = False
    interrupted = False
    handle = accept = result = None
    goal_uuid = None
    last_print = 0.
    leg = 0
    direction = 1
    active_goal = False
    confirmed_terminal = False
    cycle_epoch = None
    cycle_key = (-1, -1)
    schedule_running = False
    evidence = None
    publish_stats = PublishStats()
    publisher = node.create_publisher(UInt8, '/collision_state', 1)
    # 把当前阶段事件追加到 JSONL 并刷新，时间为相对启动的单调秒。
    # 事件日志用于还原发布／任务时序，不是 DDS 接收确认。
    def event(kind, **values):
        trace.write(json.dumps(dict(elapsed=time.monotonic()-started, kind=kind, **values))+'\n')
        trace.flush()
    # 发布一帧 UInt8 并记录本机调用时间、状态、任务与周期。
    # 由此统计的是发布调用频率，不是独立订阅端的实收频率。
    def publish_state():
        now = time.monotonic()
        publisher.publish(UInt8(data=state))
        publish_stats.add(now)
        event('publish', monotonic=now, state=state, cycle=cycle_key[0], index=cycle_key[1],
              leg=leg, direction=direction, goal_uuid=bytes(goal_uuid.uuid).hex() if goal_uuid else None,
              active_goal=active_goal)
    # 按 steady timer 触发；periodic 根据首个已接受任务的 epoch 选择状态。
    # 延迟回调按当前经过时间跳到正确区间，不把整个周期重新从 4 开始。
    def publish_tick():
        nonlocal state, cycle_key
        if not emitting:
            return
        if schedule_running and cycle_epoch is not None:
            cycle, index, selected = schedule.at(time.monotonic() - cycle_epoch)
            if (cycle, index) != cycle_key:
                old = cycle_key
                cycle_key = (cycle, index)
                state = selected
                event('state', state=state, leg=leg, cycle=cycle, index=index,
                      skipped_intervals=max(0, cycle*5+index-(old[0]*5+old[1])-1) if old[0]>=0 else 0)
        publish_state()
    timer = node.create_timer(.02, publish_tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
    # 保存话题的本机接收时刻和速度，再写入带当前任务／状态标签的 CSV。
    # 不同话题的回调时间不同，不构成严格同步控制帧。
    def receive(topic, v, w):
        latest[topic] = (time.monotonic(), v, w)
        writer.writerow([time.monotonic()-started, topic, v, w, state, leg, direction, cycle_key[0],
                         bytes(goal_uuid.uuid).hex() if goal_uuid else '', active_goal])
    subscriptions = []
    for topic in ['/cmd_vel_nav', '/cmd_vel']:
        subscriptions.append(node.create_subscription(Twist, topic, lambda m,t=topic: receive(t,m.linear.x,m.angular.z), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(ControlRes, '/control_to_uart', lambda m: receive('/control_to_uart',m.v,m.w), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Odometry, args.odom_topic, lambda m: receive(args.odom_topic,m.twist.twist.linear.x,m.twist.twist.angular.z), qos_profile_sensor_data))
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for topic in ['/navigation_service', '/follow_path']:
        subscriptions.append(node.create_subscription(GoalStatusArray, topic+'/_action/status', lambda m,t=topic: statuses.__setitem__(t,m.status_list), qos))
    client = ActionClient(node, NavigationService, '/navigation_service')
    old_handlers = {}
    # 仅设置中断标志，使主循环走自身取消与停稳收口；信号回调不直接销毁 ROS 节点。
    def interrupt(*_):
        nonlocal interrupted
        interrupted = True
    for sig in [signal.SIGINT, signal.SIGTERM]:
        old_handlers[sig] = signal.signal(sig, interrupt)
    # 推进一次 ROS 回调并执行观察逻辑，必要时根据中断标志退出正常流程。
    # cleanup 路径仍允许处理取消结果，避免收到信号后立刻失去通信能力。
    def pump(cleanup=False):
        nonlocal last_print
        rclpy.spin_once(node, timeout_sec=.01)
        observe_periodic()
        if interrupted and not cleanup:
            raise KeyboardInterrupt
        now = time.monotonic()
        if now-last_print >= 1:
            uart = latest.get('/control_to_uart')
            node.get_logger().info(f'leg={leg} collision={state} cap={direction*1.5*{4:1.,3:.65,2:.35,1:0.,0:0.}[state]:.3f} uart_v={uart[1] if uart else "missing"}')
            last_print = now
    # 用单调截止时间循环处理回调，条件满足才返回，超时抛异常进入收口。
    # seconds 是墙钟秒，不是仿真 /clock 秒。
    def wait(predicate, seconds, cleanup=False):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            pump(cleanup)
            if predicate():
                return
        raise TimeoutError('等待条件超时')
    # 短期创建服务客户端，等发现与响应后返回结果，在 finally 销毁客户端。
    # 服务完成只表示调用有返回，不证明相应机器人动作已执行。
    def call(service, service_type, request):
        c = node.create_client(service_type, service)
        try:
            wait(c.service_is_ready, 5)
            f = c.call_async(request)
            wait(f.done, 5)
            return f.result()
        finally:
            node.destroy_client(c)
    # 读取 map→base_frame 最新 TF，检查节点时钟下的时间戳年龄并返回 XY（m）。
    # 这是当前观测采样，不把原始 Odometry 位姿直接当地图位置。
    def pose():
        tf = buffer.lookup_transform('map', args.base_frame, Time())
        age = (node.get_clock().now().nanoseconds - Time.from_msg(tf.header.stamp).nanoseconds)/1e9
        if not -.1 <= age <= .25:
            raise RuntimeError(f'TF 年龄异常 {age:.3f}')
        return tf.transform.translation.x, tf.transform.translation.y
    # 要求话题曾接收、接收年龄≤0.25 s 且速度有限；不单凭发布者存在判断数据有效。
    def fresh(topic):
        return topic in latest and time.monotonic()-latest[topic][0] <= .25 and all(math.isfinite(v) for v in latest[topic][1:])
    # 要求新鲜 UART 和轮速同时满足线速度≤0.01 m/s、角速度≤0.05 rad/s。
    # 它仅评价本次采样，连续保持由外部 StageWindow 累积。
    def stopped():
        return all(fresh(t) and abs(latest[t][1]) <= .01 and abs(latest[t][2]) <= .05 for t in ['/control_to_uart', args.odom_topic])
    # 将已确认自身 Action 终态的标志交给退出停稳判据，不为仍活动的任务放宽 UART 新鲜度。
    def terminal_stopped():
        return stationary_after_terminal(time.monotonic(),latest.get('/control_to_uart'),
                                         latest.get(args.odom_topic),confirmed_terminal)
    # 只更新下一次发布所用状态并记事件，不在此函数额外发送一帧。
    def set_state(value):
        nonlocal state
        state = value
        event('state', state=state, leg=leg)
    # 结束当前“状态区间＋任务”证据，写入报告后释放窗口，避免跨区间拼接样本。
    def finish_evidence(reason):
        nonlocal evidence
        if evidence is not None:
            summary=evidence.finish(time.monotonic()-started,reason)
            report['intervals'].append(summary)
            event('interval', **summary)
            evidence=None
    # 按周期、状态、腿、UUID、方向和活动状态切分证据，再以新鲜遥测推进判据。
    # 观察结果不控制 periodic 的状态切换时间，短平台可记 UNAVAILABLE 而不是阻塞周期。
    def observe_periodic():
        nonlocal evidence
        if args.schedule != 'periodic' or not emitting or cycle_epoch is None or not schedule_running:
            return
        now=time.monotonic()-started
        key=(*cycle_key,state,leg,bytes(goal_uuid.uuid).hex() if goal_uuid else None,direction,active_goal)
        if evidence is None or evidence.key!=key:
            finish_evidence('state_or_task_boundary')
            initial=latest.get('/control_to_uart',(0,0.,0.))
            evidence=IntervalEvidence(key,now,initial[1],initial[2])
        if not fresh('/control_to_uart'):
            return
        try:
            xy=pose()
            remaining=math.dist(xy,args.b if leg%2 else args.a)
        except Exception:
            remaining=float('nan')
        uart=latest['/control_to_uart']
        raw=latest.get('/cmd_vel_nav',(0,float('nan'),0.))
        wheel=latest.get(args.odom_topic,(0,float('nan'),float('nan')))
        evidence.sample(uart[0]-started,uart[1:],raw[1],wheel[1:],remaining,
                        fresh('/cmd_vel_nav') and fresh(args.odom_topic) and schedule_running)
    # 保存腿、状态区间与发布时序摘要，刷新 CSV 并生成预览。
    # 分方向聚合必须保留 UNAVAILABLE，不能只凭 Action 成功给整项通过。
    def save():
        report['publish_timing']=publish_stats.result()
        summary={}
        for interval in report['intervals']:
            key=f"state_{interval['state']}_direction_{interval['direction']}"
            group=summary.setdefault(key,{'PASS':0,'FAIL':0,'UNAVAILABLE':0})
            group[interval['status']]+=1
        report['response_summary']=summary
        (args.output_dir/'results.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    exit_code = 1
    try:
        wait(client.server_is_ready, 15)
        discovery_end=time.monotonic()+1
        wait(lambda:time.monotonic()>=discovery_end,2)
        if any(s.status in (1,2,3) for group in statuses.values() for s in group):
            raise RuntimeError('已有导航任务，拒绝发送新任务')
        if node.count_publishers('/collision_state') != 1:
            raise RuntimeError('存在其他 collision 发布者')
        common = []
        for name in ['controller_server', 'velocity_smoother', 'regulated_navigator']:
            request = GetParameters.Request()
            request.names = ['collision_state_enabled','collision_state_topic','collision_state_timeout','use_sim_time']
            values = call('/'+name+'/get_parameters', GetParameters, request).values
            if not values[0].bool_value or values[1].string_value != '/collision_state':
                raise RuntimeError(f'{name} 未正确启用 collision 门控')
            common.append((values[2].double_value,values[3].bool_value))
        if len(set(common)) != 1 or common[0][0] < .2:
            raise RuntimeError('门控超时或时钟配置不一致／超时过短')
        node.set_parameters([Parameter('use_sim_time',value=common[0][1])])
        for name in ['controller_server','velocity_smoother']:
            if call('/'+name+'/get_state', GetState, GetState.Request()).current_state.id != 3:
                raise RuntimeError(name+' 非 active')
        for name,param in [('regulated_navigator','fixed_path_max_speed'),('velocity_smoother','max_velocity'),('controller_server','FixedPathController.desired_linear_vel')]:
            req = GetParameters.Request(); req.names=[param]
            v = call('/'+name+'/get_parameters',GetParameters,req).values[0]
            cap = v.double_array_value[0] if v.double_array_value else v.double_value
            if cap < 1.5-1e-6:
                raise RuntimeError(f'{name}.{param}={cap}，低于 1.5')
        wait(lambda: fresh(args.odom_topic), 5)
        wait(lambda: buffer.can_transform('map', args.base_frame, Time()), 5)
        if math.dist(pose(),args.a) > .2:
            raise RuntimeError('机器人不在 A 点附近 0.2 米内；请手动就位')
        # The controlpub publisher is lazy: initial missing UART is allowed only before motion.
        if any(abs(v) > limit for v,limit in zip(latest[args.odom_topic][1:], [.01,.05])):
            raise RuntimeError('起始轮速非零')
        # Advertised publishers may remain silent at rest; require measured rest,
        # and reject any received nonzero command without manufacturing a zero.
        initial_rest=StageWindow(1)
        # 起步前拒绝已收到的非零／非法指令，并要求新鲜轮速连续零速。
        # 尚无 UART 消息本身不构成失败，因为 controlpub 空闲时可能尚未创建或不周期发布。
        def ready_at_rest():
            for topic in ['/control_to_uart','/cmd_vel']:
                if topic in latest and (not all(math.isfinite(v) for v in latest[topic][1:]) or
                                        abs(latest[topic][1])>.01 or abs(latest[topic][2])>.05):
                    raise RuntimeError('起始底盘指令非零或无效')
            return initial_rest.update(time.monotonic(),fresh(args.odom_topic) and
                                      abs(latest[args.odom_topic][1])<=.01 and
                                      abs(latest[args.odom_topic][2])<=.05)
        wait(ready_at_rest,5)
        emitting = True
        report['status'] = 'RUNNING'
        while not args.max_legs or leg < args.max_legs:
            stage = [3,2,1,0][leg % 4]
            a,b = (args.a,args.b) if leg % 2 == 0 else (args.b,args.a)
            leg += 1
            direction = -1 if leg%2==0 and args.return_direction=='backward' else 1
            if args.schedule=='cruise' or cycle_epoch is None:
                set_state(4)
            segment = NaviSegment()
            segment.segment_type = 1; segment.motion_direction = 1 if direction==1 else 2
            segment.segment_name = 'collision_straight'; segment.segment_id = f'leg_{leg}'
            segment.node1.x,segment.node1.y = a
            segment.node2.x,segment.node2.y = b
            segment.max_speed = 1.5
            goal = NavigationService.Goal(); goal.task_id=f'collision_{time.time_ns()}'
            goal.navi_segment=[segment]
            record = dict(leg=leg,state=stage if args.schedule=='cruise' else None,
                          direction=direction,status='UNAVAILABLE',feedback_stopped=False)
            report['legs'].append(record)
            goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
            confirmed_terminal=False
            accept = client.send_goal_async(goal, goal_uuid=goal_uuid, feedback_callback=lambda f: event('feedback',progress=f.feedback.progress))
            wait(accept.done, 10)
            handle = accept.result()
            if not handle.accepted:
                raise RuntimeError('Action 拒绝')
            record['goal_uuid'] = bytes(handle.goal_id.uuid).hex()
            result = handle.get_result_async()
            active_goal = True
            if args.schedule=='periodic' and cycle_epoch is None:
                cycle_epoch=time.monotonic()
                cycle_key=(0,0)
                schedule_running=True
                event('schedule_start',state=4,cycle=0,index=0,period_s=schedule.period)
            baseline = StageWindow(.2); window = StageWindow(1 if stage <= 1 else .3)
            wheel_window = StageWindow(1)
            phase = 'BASELINE' if args.schedule=='cruise' else 'PERIODIC'
            phase_start = time.monotonic(); goal_start=phase_start
            bad_since=None; seen_uart=False; recovered=StageWindow(.2)
            while not result.done():
                pump()
                now=time.monotonic()
                if node.count_publishers('/collision_state') != 1:
                    raise RuntimeError('出现其他 collision 发布者')
                try:
                    xy=pose()
                    event('pose',x=xy[0],y=xy[1],leg=leg)
                    if not fresh(args.odom_topic): raise RuntimeError('轮速数据过期')
                    bad_since=None
                except Exception:
                    bad_since=bad_since or now
                    if now-bad_since>=1: raise RuntimeError('定位／轮速连续失效 1 秒')
                    continue
                if fresh('/control_to_uart'): seen_uart=True
                if seen_uart and not fresh('/control_to_uart'):
                    raise RuntimeError('底盘输出数据缺失超过 0.25 秒')
                remaining=math.dist(xy,b)
                uart=latest.get('/control_to_uart',(0,float('nan'),float('nan')))[1:]
                raw=latest.get('/cmd_vel_nav',(0,float('nan'),0))[1] if fresh('/cmd_vel_nav') else float('nan')
                if args.schedule=='periodic':
                    # Evidence only: state publication is owned by the steady timer.
                    pass
                elif phase=='BASELINE':
                    if baseline.update(now,fresh('/control_to_uart') and stage_condition(4,uart,raw,remaining,direction)):
                        set_state(stage); phase='OBSERVE';phase_start=now
                    elif now-phase_start>30: raise RuntimeError('UNAVAILABLE：30 秒未建立 1.5 m/s 巡航基准')
                elif phase=='OBSERVE':
                    record['feedback_stopped'] = wheel_window.update(now,stopped()) if stage<=1 else False
                    command_stable = window.update(now,fresh('/control_to_uart') and stage_condition(stage,uart,raw,remaining,direction))
                    if command_stable:
                        record['status']='PASS'
                    elif stage<=1:
                        record['status']='UNAVAILABLE'
                    if (now-phase_start>=3 if stage<=1 else record['status']=='PASS' or now-phase_start>=3):
                        if record['status']!='PASS':
                            record['status']='UNAVAILABLE' if remaining<.5 or not math.isfinite(raw) else 'FAIL'
                            raise RuntimeError('collision 阶段未通过：'+record['status'])
                        if stage<=1 and not record['feedback_stopped']:
                            record['status']='FAIL';raise RuntimeError('指令停止但轮速未停稳')
                        set_state(4);phase='FINISH';phase_start=now
                else:
                    if recovered.update(now,fresh('/control_to_uart') and direction*uart[0]>.1 and direction*raw>.1): record['recovered']=True
                if now-goal_start>120: raise RuntimeError('单程 Action 超时')
            response=result.result()
            confirmed_terminal=response.status in (4,5,6)
            record['action_status']=response.status
            record['finish']=response.result.finish
            active_goal=False
            observe_periodic()
            if response.status!=4 or not response.result.finish:
                record['status']='FAIL'
                raise RuntimeError('Action 未成功结束')
            handle=None;accept=None;goal_uuid=None
            if args.schedule=='cruise' and (phase!='FINISH' or not record.get('recovered')):
                record['status']='UNAVAILABLE';raise RuntimeError('短路线提前结束或未观察到恢复运动')
            if args.schedule=='periodic':
                record['status']='ACTION_SUCCEEDED'
            settle=StageWindow(1)
            wait(lambda:settle.update(time.monotonic(),terminal_stopped()),10)
            save();csvfile.flush()
            if args.schedule=='cruise':
                write_plot(args.output_dir)
        report['status']='COMPLETED';exit_code=0
    except KeyboardInterrupt:
        if handle is not None and report['legs'] and report['legs'][-1]['status']=='PASS':
            report['legs'][-1].update(collision_response='PASS',status='UNAVAILABLE')
        report['status']='INTERRUPTED';exit_code=0
    except Exception as error:
        if handle is not None and report['legs'] and report['legs'][-1]['status']=='PASS':
            report['legs'][-1].update(collision_response='PASS',status='UNAVAILABLE')
        report['status']='ERROR';report['error']=str(error)
        node.get_logger().error(str(error))
    finally:
        # Keep publishing STOP while resolving a late acceptance and cancelling our own UUID.
        if emitting:
            schedule_running=False
            active_goal=False
            finish_evidence('shutdown')
            set_state(1)
            publish_state()
            try:
                if handle is None and accept is not None:
                    try:
                        wait(accept.done,3,cleanup=True)
                        handle=accept.result()
                    except TimeoutError:
                        # Exact UUID cancellation also covers a delayed goal response.
                        c=node.create_client(CancelGoal,'/navigation_service/_action/cancel_goal')
                        try:
                            wait(c.service_is_ready,3,cleanup=True)
                            request=CancelGoal.Request();request.goal_info.goal_id=goal_uuid
                            f=c.call_async(request);wait(f.done,3,cleanup=True)
                            event('cancel_pending',goal_uuid=bytes(goal_uuid.uuid).hex(),return_code=f.result().return_code)
                        finally:
                            node.destroy_client(c)
                        raise RuntimeError('Goal 接受结果超时，已按本工具 UUID 请求取消；执行结果未确认')
                if handle is not None and handle.accepted:
                    future=handle.cancel_goal_async();wait(future.done,3,cleanup=True)
                    event('cancel',goal_uuid=bytes(handle.goal_id.uuid).hex(),accepted=bool(future.result().goals_canceling))
                    terminal=result if result is not None else handle.get_result_async()
                    wait(terminal.done,3,cleanup=True)
                    confirmed_terminal=terminal.result().status in (4,5,6)
                    event('terminal',status=terminal.result().status)
                elif accept is not None and handle is None:
                    raise RuntimeError('未能解析本工具 Goal 的接受结果')
                settle=StageWindow(1)
                wait(lambda:settle.update(time.monotonic(),terminal_stopped()),10,cleanup=True)
                report['shutdown_stopped']=True
                report['shutdown_stop_evidence']='fresh_output_and_wheel' if fresh('/control_to_uart') else 'terminal_goal_last_zero_command_and_fresh_wheel'
            except Exception as error:
                report['shutdown_stopped']=False;report['shutdown_error']=str(error);exit_code=1
        save();csvfile.close();trace.close();write_plot(args.output_dir)
        for sig,old in old_handlers.items():signal.signal(sig,old)
        node.destroy_node();rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
