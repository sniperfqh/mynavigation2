# collision 客户端的纯证据窗口与调度测试：合格平台、短窗口、缺帧、方向、退出静默和绘图。
# 构造的样本只验证判据与输出格式，不能据此宣布当前地图或实车减速通过。
"""Pure evidence-window tests; no motion or ROS initialization."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace
from contextlib import redirect_stderr
from io import StringIO

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('collision_action_tool', ROOT / 'tools/collision_action_test.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


# 组织本文件的回归场景；各 test_ 方法构造输入并断言预期，不把模拟样本当作现场证据。
class EvidenceTest(unittest.TestCase):
    def test_runtime_defaults_and_explicit_overrides(self):
        sim = tool.parse_args(['--output-dir', '/tmp/not-created'])
        self.assertEqual((sim.runtime, sim.feedback_topic, sim.schedule, sim.return_direction, sim.max_legs),
                         ('sim', '/odometry', 'cruise', 'forward', 0))
        self.assertEqual(sim.state_durations, [4, 3, 3, 3, 3])
        real_args = ['--runtime', 'real', '--a', '0', '0', '--b', '30', '0',
                     '--output-dir', '/tmp/not-created']
        real = tool.parse_args(real_args)
        self.assertEqual((real.feedback_topic, real.feedback_type, real.schedule, real.return_direction, real.max_legs),
                         ('/motion_state', 'byd_custom_msgs/msg/MotionState', 'periodic', 'backward', 4))
        self.assertEqual(real.state_durations, [12, 4, 4, 3, 3])
        explicit = tool.parse_args(real_args + ['--feedback-topic', '/vehicle/motion_state',
                                   '--max-legs', '0', '--schedule', 'cruise', '--return-direction', 'forward',
                                   '--state-durations', '5', '4', '3', '2', '1'])
        self.assertEqual((explicit.feedback_topic, explicit.max_legs, explicit.schedule, explicit.return_direction),
                         ('/vehicle/motion_state', 0, 'cruise', 'forward'))
        self.assertEqual(explicit.state_durations, [5, 4, 3, 2, 1])
        self.assertEqual(tool.parse_args(['--output-dir', '/tmp/not-created', '--odom-topic', '/odom']).feedback_topic,
                         '/odom')

    def test_invalid_real_route_or_feedback_is_rejected_before_ros(self):
        base = ['--runtime', 'real', '--output-dir', '/tmp/not-created']
        for extra in [[], ['--a', '0', '0'], ['--a', 'nan', '0', '--b', '10', '0'],
                      ['--a', '0', '0', '--b', '.5', '0'],
                      ['--a', '0', '0', '--b', '10', '0', '--odom-topic', '/odometry'],
                      ['--a', '0', '0', '--b', '10', '0', '--feedback-topic', ''],
                      ['--a', '0', '0', '--b', '10', '0', '--max-legs', '-1'],
                      ['--a', '0', '0', '--b', '10', '0', '--state-durations', '1', '1', '0', '1', '1']]:
            with self.subTest(extra=extra), redirect_stderr(StringIO()), self.assertRaises(SystemExit) as error:
                tool.parse_args(base + extra)
            self.assertEqual(error.exception.code, 2)

    def test_feedback_conversion_preserves_reverse_and_angular_speed(self):
        self.assertEqual(tool.feedback_velocity(SimpleNamespace(v_car=-1.2, w_car=.04), 'real'), (-1.2, .04))
        odom = SimpleNamespace(twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=-.9), angular=SimpleNamespace(z=-.03)) ))
        self.assertEqual(tool.feedback_velocity(odom, 'sim'), (-.9, -.03))

    def test_summary_requires_all_states_both_directions_and_recovery(self):
        records = [dict(direction=d, state=s, status='PASS') for d in (1, -1) for s in range(5)]
        records[0].update(recovery_required=True, recovery_status='PASS')
        records[5].update(recovery_required=True, recovery_status='PASS')
        summary, status = tool.response_summary(records, (1, -1))
        self.assertEqual(status, 'PASS')
        self.assertEqual(summary['state_2_direction_-1']['PASS'], 1)
        self.assertEqual(tool.response_summary(records[:-1], (1, -1))[1], 'UNAVAILABLE')
        self.assertEqual(tool.response_summary(records[:5], (1, -1))[1], 'UNAVAILABLE')
        records.append(dict(direction=-1, state=2, status='FAIL'))
        self.assertEqual(tool.response_summary(records, (1, -1))[1], 'FAIL')
        self.assertEqual(tool.response_summary([], (1, -1))[1], 'UNAVAILABLE')

    def test_recovery_window_is_distinct_from_full_speed_platform(self):
        evidence = tool.IntervalEvidence((1, 0, 4, 1, 'own', 1, True), 0., 0., recovery_required=True)
        for i in range(6):
            evidence.sample(i*.1, (.3, 0.), .3, (.3, 0.), 10., True)
        result = evidence.finish(.6, 'state')
        self.assertEqual(result['status'], 'UNAVAILABLE')
        self.assertEqual(result['recovery_status'], 'PASS')
        evidence = tool.IntervalEvidence((1, 0, 4, 2, 'own', -1, True), 0., 0., recovery_required=True)
        for i in range(31):
            evidence.sample(i*.1, (0., 0.), -1.5, (0., 0.), 10., True)
        self.assertEqual(evidence.finish(3.1, 'state')['recovery_status'], 'FAIL')

    # 证明 UART 静默例外只能用于自身任务终态，且必须有新鲜、有限的实测零轮速。
    def test_silent_terminal_requires_own_result_and_fresh_measured_zero(self):
        self.assertTrue(tool.stationary_after_terminal(10.,(1.,0.,0.),(10.,0.,0.),True))
        self.assertFalse(tool.stationary_after_terminal(10.,(1.,0.,0.),(10.,0.,0.),False))
        self.assertFalse(tool.stationary_after_terminal(10.,(1.,.1,0.),(10.,0.,0.),True))
        self.assertFalse(tool.stationary_after_terminal(10.,(1.,0.,0.),(9.,0.,0.),True))
        self.assertFalse(tool.stationary_after_terminal(10.,None,(10.,0.,0.),True))
    # 验证五段周期边界、跨周期延迟回调及非正／非有限时长拒绝。
    def test_periodic_boundaries_and_delayed_callback(self):
        schedule=tool.PeriodicSchedule([4,3,3,3,3])
        for elapsed, expected in [(0,(0,0,4)),(3.99,(0,0,4)),(4,(0,1,3)),
                                  (7,(0,2,2)),(10,(0,3,1)),(13,(0,4,0)),
                                  (16,(1,0,4)),(39,(2,2,2))]:
            self.assertEqual(schedule.at(elapsed),expected)
        with self.assertRaises(ValueError):tool.PeriodicSchedule([4,3,0,3,3])
        with self.assertRaises(ValueError):tool.PeriodicSchedule([4,3,float('nan'),3,3])

    # 用理想 20 ms 调用间隔验证实测频率与直方图次数守恒，不以配置值冒充测量。
    def test_frequency_statistics(self):
        stats=tool.PublishStats()
        for i in range(101):stats.add(i*.02)
        result=stats.result()
        self.assertAlmostEqual(result['measured_hz'],50.)
        self.assertAlmostEqual(result['max_interval_s'],.02)
        self.assertEqual(sum(result['interval_histogram'].values()),100)

    # 验证倒车负速度减速平台可通过，而原始需求不足时保持 UNAVAILABLE。
    def test_reverse_interval_and_evidence_shortage(self):
        key=(0,1,3,2,'own-goal',-1,True)
        evidence=tool.IntervalEvidence(key,0.,-1.5)
        for i in range(6):evidence.sample(i*.1,(-.975,0.),-1.5,(-.975,0.),2.,True)
        result=evidence.finish(.6,'state')
        self.assertEqual(result['status'],'PASS')
        self.assertEqual(result['response'],'减速')
        evidence=tool.IntervalEvidence(key,0.,-.4)
        for i in range(31):evidence.sample(i*.1,(-.4,0.),-.4,(-.4,0.),2.,True)
        self.assertEqual(evidence.finish(3.1,'state')['status'],'UNAVAILABLE')

    # 验证切入时已为零的停车证据标注保持停稳，不声称发生新的制动。
    def test_hold_zero_is_not_new_braking(self):
        evidence=tool.IntervalEvidence((0,4,0,1,'own',1,True),0.,0.)
        for i in range(21):evidence.sample(i*.1,(0.,0.),0.,(0.,0.),2.,True)
        result=evidence.finish(2.1,'state')
        self.assertEqual(result['response'],'保持停稳')
        self.assertEqual(result['status'],'PASS')

    # 验证无活动任务或距终点太近的样本不能构成碰撞减速证据。
    def test_idle_and_approach_never_pass(self):
        for active,remaining in [(False,2.),(True,.1)]:
            evidence=tool.IntervalEvidence((0,1,3,1,'own',1,active),0.,1.5)
            for i in range(31):evidence.sample(i*.1,(.975,0.),1.5,(.975,0.),remaining,True)
            self.assertEqual(evidence.finish(3.1,'state')['status'],'UNAVAILABLE')

    # 正常状态只达到短时爬升速度时应不可评价，不能误报 collision 失败。
    def test_normal_ramp_shortage_is_not_collision_failure(self):
        evidence=tool.IntervalEvidence((0,0,4,1,'own',1,True),0.,0.)
        for i in range(31):evidence.sample(i*.1,(.8,0.),1.5,(.8,0.),2.,True)
        self.assertEqual(evidence.finish(3.1,'state')['status'],'UNAVAILABLE')

    # 核对各状态比例，并排除终点制动和无更高原始请求的假平台。
    def test_ratios_and_terminal_braking(self):
        for state, v in [(4,1.5),(3,.975),(2,.525),(1,0.),(0,0.)]:
            self.assertTrue(tool.stage_condition(state,(v,0.),1.5,2.))
            self.assertFalse(tool.stage_condition(state,(v,0.),1.5,.1))
        self.assertFalse(tool.stage_condition(3,(.975,0.),.975,2.))
        self.assertFalse(tool.stage_condition(2,(1.5,0.),1.5,2.))

    # 线速度为零但仍在旋转或数值非法时不能判停稳。
    def test_stop_requires_angular_zero(self):
        self.assertFalse(tool.stage_condition(1,(0.,.2),0.,2.))
        self.assertFalse(tool.stage_condition(1,(float('nan'),0.),0.,2.))

    # 观察缺口必须打断连续窗口，不能把缺失期间当作保持。
    def test_missing_data_does_not_count_as_hold(self):
        window=tool.StageWindow(.3)
        self.assertFalse(window.update(0.,True))
        self.assertFalse(window.update(.2,True))
        self.assertFalse(window.update(1.,True))
        self.assertFalse(window.update(1.2,True))
        self.assertTrue(window.update(1.4,True))
        self.assertFalse(window.update(1.5,False))

    # 用临时 CSV 验证 SVG／HTML 生成，不依赖外部绘图库或 ROS。
    def test_plot_without_dependency(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'velocity.csv').write_text('elapsed,topic,v,w,state,leg\n0,/control_to_uart,1.5,0,4,1\n1,/control_to_uart,.975,0,3,1\n')
            tool.write_plot(p)
            self.assertIn('<svg', (p/'velocity.html').read_text())

    # 静态解析两个仿真包装文件，确认开关默认值与转发路径都存在。
    def test_sim_wrappers_forward_switch(self):
        import ast
        for name in ['entry.launch.py','navigation_launch.py']:
            path=ROOT.parent/'myworld_bringup/launch'/name
            tree=ast.parse(path.read_text())
            declarations=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='DeclareLaunchArgument' and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value=='enable_collision_state']
            self.assertEqual(len(declarations),1)
            self.assertTrue(any(k.arg=='default_value' and k.value.value=='true' for k in declarations[0].keywords))
            values=[v for n in ast.walk(tree) if isinstance(n,ast.Dict) for k,v in zip(n.keys,n.values) if isinstance(k,ast.Constant) and k.value=='enable_collision_state']
            self.assertEqual(len(values),1)
            self.assertIsInstance(values[0],ast.Call)
            self.assertEqual(values[0].func.id,'LaunchConfiguration')
            self.assertEqual(values[0].args[0].value,'enable_collision_state')


if __name__ == '__main__':
    unittest.main()
