#!/usr/bin/env python3
# 隔离 ROS mock 协议测试：模拟 Action、生命周期、参数、TF 与速度，驱动真实测试工具。
# 模拟速度按状态直接合成，能验证客户端流程，不能验证真实控制器制动；必须使用非零隔离域和 localhost。
"""ROS mock-server protocol checks only; no real controller or physical speed evidence."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from tf2_ros import TransformBroadcaster
from byd_custom_msgs.action import NavigationService
from byd_custom_msgs.msg import ControlRes, MotionState


# 提供隔离 Action／参数／生命周期替身及合成遥测，驱动真实客户端的协议回归。
class Mock(Node):
    # 建立隔离 mock 服务与合成观测定时器，所有平台均由测试逻辑直接生成。
    def __init__(self):
        super().__init__('collision_action_mock')
        self.state=0; self.xy=[9.16,-4.37]; self.target=None; self.active=None
        self.reject=False; self.cancelled=[]
        self.case='';self.goal_started=0.
        self.direction=1;self.directions=[]
        self.group=ReentrantCallbackGroup()
        self.tf=TransformBroadcaster(self)
        self.uart=self.create_publisher(ControlRes,'/control_to_uart',10)
        self.raw=self.create_publisher(Twist,'/cmd_vel_nav',10)
        self.cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.odom=self.create_publisher(Odometry,'/odometry',10)
        self.motion=self.create_publisher(MotionState,'/motion_state',10)
        self.custom_motion=self.create_publisher(MotionState,'/vehicle/motion_state',10)
        self.extra_collision=None
        self.speeds=[]
        self.create_subscription(UInt8,'/collision_state',self.collision,1)
        self.parameter_services=[]
        for name in ['controller_server','velocity_smoother','regulated_navigator']:
            self.parameter_services.append(self.create_service(GetParameters,'/'+name+'/get_parameters',self.parameters,callback_group=self.group))
        for name in ['controller_server','velocity_smoother']:
            self.parameter_services.append(self.create_service(GetState,'/'+name+'/get_state',self.lifecycle,callback_group=self.group))
        self.server=ActionServer(self,NavigationService,'/navigation_service',execute_callback=self.execute,
                                 goal_callback=lambda _: GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT,
                                 cancel_callback=self.cancel,callback_group=self.group)
        self.create_timer(.02,self.tick)

    # 缓存测试工具发布的状态值，供 mock 定时器按固定比例合成速度；不调用生产 collision 算法。
    def collision(self,m):
        self.state=m.data

    # 模拟预检所需参数及对应 ROS ParameterValue 类型，使真实工具执行参数一致性检查。
    def parameters(self,req,res):
        for key in req.names:
            p=ParameterValue()
            if key in ('collision_state_enabled','use_sim_time'):
                p.type=ParameterType.PARAMETER_BOOL
                p.bool_value=key=='collision_state_enabled' or (key=='use_sim_time' and self.case=='real_sim_time')
            elif key=='collision_state_topic':
                p.type=ParameterType.PARAMETER_STRING;p.string_value='/collision_state'
            elif key=='max_velocity':
                p.type=ParameterType.PARAMETER_DOUBLE_ARRAY;p.double_array_value=[1.5,0.,2.]
            else:
                p.type=ParameterType.PARAMETER_DOUBLE;p.double_value=.5 if key=='collision_state_timeout' else 1.5
            res.values.append(p)
        return res

    # 构造或请求生命周期状态；该测试中的 active 不能证明实际生产节点已经激活。
    def lifecycle(self,req,res):
        res.current_state.id=3;res.current_state.label='active';return res

    # 记录或检查本测试自己持有的 Goal UUID，验证取消只作用于自身任务。
    def cancel(self,goal):
        self.cancelled.append(bytes(goal.goal_id.uuid).hex())
        return CancelResponse.ACCEPT

    # mock 接受任务后保存方向、目标和起始时间，循环响应取消或完成条件。
    # 结果来自模拟服务器，不代表真实规划和制动通过。
    def execute(self,goal):
        self.active=goal
        self.goal_started=time.monotonic()
        segment=goal.request.navi_segment[0]
        self.direction=1 if segment.motion_direction==1 else -1
        self.directions.append(segment.motion_direction)
        self.speeds.append(segment.max_speed)
        self.target=[segment.node2.x,segment.node2.y]
        while rclpy.ok() and self.target is not None:
            if goal.is_cancel_requested:
                self.target=None;self.active=None;goal.canceled()
                return NavigationService.Result(finish=False)
            time.sleep(.01)
        self.active=None
        goal.succeed()
        return NavigationService.Result(finish=True)

    # 推进测试用状态并发布合成 TF／轮速／速度；保持消息时间戳与节点时钟一致。
    # 合成反馈只验证隔离链路，不接入真实底盘。
    def tick(self):
        v=0.
        if self.target is not None:
            speed=1. if self.case=='periodic' else 1.5
            v=self.direction*min(speed,1.5*{4:1.,3:.65,2:.35}.get(self.state,0.))
            delta=[b-a for a,b in zip(self.xy,self.target)]
            distance=math.hypot(*delta)
            if distance <= max(.02,abs(v)*.02):
                self.xy=self.target;self.target=None;v=0.
            elif v:
                self.xy=[a+d/distance*abs(v)*.02 for a,d in zip(self.xy,delta)]
            if self.case=='early_finish' and time.monotonic()-self.goal_started>.15:
                self.target=None;v=0.
        if not ((self.case=='telemetry_gap' and self.goal_started and time.monotonic()-self.goal_started>.5) or
                (self.case=='silent_initial' and not self.goal_started) or
                (self.case=='quiet_terminal' and self.goal_started and self.active is None)):
            self.uart.publish(ControlRes(v=v,w=0.))
        self.cmd.publish(Twist()) if not v else self.cmd.publish(self.twist(v))
        self.raw.publish(self.twist(self.direction*(1. if self.case=='periodic' else 1.5) if self.active else 0.))
        odom=Odometry();odom.header.stamp=self.get_clock().now().to_msg()
        odom.twist.twist.linear.x=v;self.odom.publish(odom)
        if not (self.case=='real_feedback_gap' and self.active is not None and
                time.monotonic()-self.goal_started>.5):
            motion=MotionState();motion.header.stamp=odom.header.stamp
            motion.v_car=v;motion.w_car=0.
            self.motion.publish(motion);self.custom_motion.publish(motion)
        tf=TransformStamped();tf.header.stamp=odom.header.stamp
        tf.header.frame_id='map';tf.child_frame_id='base_link'
        tf.transform.translation.x,tf.transform.translation.y=self.xy
        tf.transform.rotation.w=1.;self.tf.sendTransform(tf)

    # 生成仅 linear.x 非零的测试 Twist；它是 mock 的观测数据，不代表物理控制响应。
    @staticmethod
    def twist(v):
        msg=Twist();msg.linear.x=v;return msg


# 隔离 ROS mock 协议测试：模拟 Action、生命周期、参数、TF 与速度，驱动真实测试工具。
# 先解析参数与检查运行前提，再按本文件流程采集／处理并写结果；异常和 finally 收口分支不应被注释描述为正常通过。
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or os.environ['ROS_DOMAIN_ID']=='0' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('Require explicit nonzero isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    rclpy.init();mock=Mock();executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(mock);thread=threading.Thread(target=executor.spin);thread.start()
    tool=Path(__file__).resolve().parents[1]/'tools/collision_action_test.py'
    reports={};process=None
    try:
        for case in ['four_legs','rejected','interrupt','early_finish','telemetry_gap','silent_initial',
                     'periodic','quiet_terminal','periodic_interrupt','real_periodic','real_feedback_gap',
                     'real_duplicate','real_rejected','real_interrupt','real_sim_time']:
            mock.case=case;mock.goal_started=0.
            mock.directions=[];mock.speeds=[]
            mock.xy=[9.16,-4.37];mock.state=0;mock.reject=case in ('rejected','real_rejected')
            if mock.extra_collision is not None:
                mock.destroy_publisher(mock.extra_collision);mock.extra_collision=None
            if case=='real_duplicate':
                mock.extra_collision=mock.create_publisher(UInt8,'/collision_state',1)
            out=args.output_dir/case
            command=['python3',str(tool),'--output-dir',str(out)]
            if case.startswith('real_'):
                command+=['--runtime','real','--a','9.16','-4.37','--b','13.2','-.47']
            if case=='four_legs':command+=['--max-legs','4']
            if case=='silent_initial':command+=['--max-legs','1']
            if case=='quiet_terminal':command+=['--max-legs','1']
            if case in ('periodic','real_periodic'):
                command+=['--state-durations','2','1','1','1.5','1.5','--max-legs','2']
                if case=='periodic':command+=['--schedule','periodic','--return-direction','backward']
                else:command+=['--feedback-topic','/vehicle/motion_state']
            if case=='periodic_interrupt':command+=['--schedule','periodic','--return-direction','backward']
            with (args.output_dir/(case+'.log')).open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=log)
                if case in ('interrupt','periodic_interrupt','real_interrupt'):
                    deadline=time.monotonic()+15
                    while mock.active is None and process.poll() is None and time.monotonic()<deadline:time.sleep(.05)
                    assert mock.active is not None,'No accepted goal to interrupt'
                    expected=bytes(mock.active.goal_id.uuid).hex()
                    if case=='periodic_interrupt':time.sleep(.4)
                    process.send_signal(signal.SIGINT)
                process.wait(timeout=100)
            report=json.loads((out/'results.json').read_text())
            reports[case]=report
            if case in ('four_legs','silent_initial','quiet_terminal'):
                assert process.returncode==0,report
                assert report['status']=='COMPLETED',report
                assert [r['state'] for r in report['legs']]==([3,2,1,0] if case=='four_legs' else [3]),report
                assert all(r['status']=='PASS' for r in report['legs']),report
                assert report['shutdown_stopped'],report
            elif case in ('rejected','real_rejected'):
                assert process.returncode!=0 and '拒绝' in report['error'],report
            elif case=='real_duplicate':
                assert process.returncode!=0 and '其他 collision 发布者' in report['error'],report
                assert mock.directions==[],mock.directions
            elif case=='real_sim_time':
                assert process.returncode!=0 and 'use_sim_time=true' in report['error'],report
                assert mock.directions==[],mock.directions
            elif case=='real_feedback_gap':
                assert process.returncode!=0 and '定位／轮速连续失效' in report['error'],report
                assert report['shutdown_stopped'],report
                assert report['collision_status']!='PASS',report
            elif case in ('interrupt','periodic_interrupt','real_interrupt'):
                assert process.returncode==0 and report['status']=='INTERRUPTED',report
                assert report['shutdown_stopped'] and expected in mock.cancelled,report
                if case=='periodic_interrupt':
                    events=[json.loads(line) for line in (out/'trace.jsonl').read_text().splitlines()]
                    stop=[e for e in events if e['kind']=='state' and e['state']==1][-1]
                    after=[e for e in events if e['kind']=='publish' and e['elapsed']>=stop['elapsed']]
                    assert len(after)>=40 and all(e['state']==1 for e in after),after
            elif case=='early_finish':
                assert process.returncode!=0 and report['legs'][0]['status']=='UNAVAILABLE',report
            elif case=='telemetry_gap':
                assert process.returncode!=0 and '数据缺失' in report['error'],report
                assert report['shutdown_stopped'] is False,report
                assert report['legs'][0]['status']=='UNAVAILABLE',report
            else:
                assert process.returncode==0 and report['status']=='COMPLETED',report
                assert mock.directions==[1,2],mock.directions
                assert report['shutdown_stopped'],report
                trace=[json.loads(line) for line in (out/'trace.jsonl').read_text().splitlines()]
                starts=[r for r in trace if r['kind']=='schedule_start']
                assert len(starts)==1,starts
                states=[r for r in trace if r['kind']=='state' and 'index' in r]
                assert {r['state'] for r in states}=={0,1,2,3,4},states
                assert any(r['leg']==2 and r['cycle']>0 for r in states),states
                publications=[r for r in trace if r['kind']=='publish']
                assert any(r['direction']==-1 for r in publications),publications[-5:]
                assert 45 < report['publish_timing']['measured_hz'] < 55,report['publish_timing']
                negative=[r for r in __import__('csv').DictReader((out/'velocity.csv').open())
                          if r['topic']=='/control_to_uart' and float(r['v'])<-.05]
                assert negative,'No reverse velocity recorded'
                assert all(abs(v-1.5)<1e-6 for v in mock.speeds),mock.speeds
                assert len(report['response_summary'])==12,report
                if case=='real_periodic':
                    assert report['runtime']=='real' and report['physical_robot'],report
                    assert report['feedback_topic']=='/vehicle/motion_state',report
                    assert report['feedback_type']=='byd_custom_msgs/msg/MotionState',report
                    assert any(r['topic']=='/vehicle/motion_state' for r in
                               __import__('csv').DictReader((out/'velocity.csv').open())),report
                    assert any(r.get('recovery_required') and r['recovery_status']=='PASS'
                               for r in report['intervals']),report
            process=None
        (args.output_dir/'protocol_results.json').write_text(json.dumps(reports,ensure_ascii=False,indent=2))
        print('PASS: 15 simulation / MotionState protocol scenarios; synthetic feedback, no vehicle proof')
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        mock.target=None
        executor.shutdown();thread.join();mock.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    main()
