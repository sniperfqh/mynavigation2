#!/usr/bin/env python3
import json,time,math,re,subprocess,sys,os,threading,queue,datetime,uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer,TransformListener
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from byd_custom_msgs.msg import MotionState
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from rcl_interfaces.srv import GetParameters,ListParameters
from rclpy.action import ActionClient
from unique_identifier_msgs.msg import UUID
from byd_custom_msgs.action import NavigationService
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
import yaml
from stationary_acceptance import StationaryAcceptance
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
os.environ['ROS_LOG_DIR']=str(out/'ros_logs')
(out/'runner_snapshot.py').write_bytes(Path(__file__).read_bytes())
rclpy.init();n=Node('action_only_speed_observer');b=Buffer();tl=TransformListener(b,n)
latest={};velocity_history=[];statuses=[];subs=[];raw=(out/'trace.jsonl').open('w');report={'acceptance_protocol':'separated_velocity_pose_v2','pose_min_coverage':.8,'pose_max_gap_s':.2,'cases':[],'planned':28,'status':'PREFLIGHT'};current=None;handle=None;result_future=None;accept_future=None;last_error_log=0
class QueuedWriter:
 def __init__(self,file):
  self.file=file;self.q=queue.Queue();self.thread=threading.Thread(target=self.run);self.thread.start()
 def run(self):
  while True:
   value=self.q.get()
   if value is None:break
   self.file.write(value)
  self.file.close()
 def write(self,value):self.q.put(value)
 def flush(self):pass
 def close(self):self.q.put(None);self.thread.join()
raw=QueuedWriter(raw)
def spin():
 try:rclpy.spin(n)
 except Exception:
  if rclpy.ok():raise
spin_thread=threading.Thread(target=spin,daemon=True);spin_thread.start()
bad_since=None
def save(): (out/'results.json').write_text(json.dumps(report,indent=2))
def recv(t,m):
 latest[t]=(time.monotonic(),m)
 v,w=(m.twist.twist.linear.x,m.twist.twist.angular.z) if t=='/odometry' else ((m.v_car,m.w_car) if t=='/motion_state' else (m.linear.x,m.angular.z))
 stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9 if hasattr(m,'header') else None
 velocity_history.append((time.monotonic(),t,v,w))
 raw.write(json.dumps([time.monotonic(),time.time(),t,v,w,stamp])+'\n')
 if t=='/odometry':
  p=m.pose.pose.position;raw.write(json.dumps([time.monotonic(),time.time(),'/odom_position',p.x,p.y,stamp])+'\n')
for t,ty in [('/odometry',Odometry),('/motion_state',MotionState),('/cmd_vel_nav',Twist),('/cmd_vel',Twist)]:subs.append(n.create_subscription(ty,t,lambda m,t=t:recv(t,m),qos_profile_sensor_data))
def status(m):
 global statuses
 statuses=m.status_list
status_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
subs.append(n.create_subscription(GoalStatusArray,'/navigation_service/_action/status',status,status_qos))
follow_status=[]
def follow_cb(m):
 global follow_status
 follow_status=m.status_list
subs.append(n.create_subscription(GoalStatusArray,'/follow_path/_action/status',follow_cb,status_qos))
def pump():time.sleep(.01)
def sample():
 global bad_since,last_error_log
 now=time.monotonic();reason=None;age=None
 try:
  for t in ['/odometry','/motion_state']:
   if t not in latest or now-latest[t][0]>.25:raise RuntimeError('stale '+t)
  tf=b.lookup_transform('map','base_link',Time());age=time.time()-tf.header.stamp.sec-tf.header.stamp.nanosec/1e9
  if age>.25 or age<-.1:raise RuntimeError('invalid TF age '+str(age))
  p=tf.transform.translation;m=latest['/motion_state'][1];c=latest.get('/cmd_vel',(0,Twist()))[1]
  q=tf.transform.rotation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  s=[now,time.time(),p.x,p.y,m.v_car,m.w_car,c.linear.x,c.angular.z,age,yaw]
  raw.write(json.dumps(s)+'\n');bad_since=None
  if current and now-last_error_log>=.1:
   g=current['goal']['navi_segment'][0];a=g['node1'];z=g['node2'];dx=z['x']-a['x'];dy=z['y']-a['y'];length=math.hypot(dx,dy);ex=p.x-z['x'];ey=p.y-z['y'];heading=math.atan2(dy,dx)+(math.pi if g['motion_direction']==2 else 0);yaw_error=math.atan2(math.sin(yaw-heading),math.cos(yaw-heading))
   raw.write(json.dumps({'wall':now,'epoch':s[1],'goal_uuid':current['uuid'],'distance':math.hypot(ex,ey),'longitudinal':(ex*dx+ey*dy)/length,'lateral':(-ex*dy+ey*dx)/length,'yaw_error':yaw_error})+'\n');last_error_log=now
  return s
 except Exception as e:reason=str(e)
 raw.write(json.dumps({'wall':now,'epoch':time.time(),'invalid':reason,'tf_age':age})+'\n')
 bad_since=bad_since or now
 if now-bad_since>=1:raise RuntimeError('no valid telemetry for 1 s: '+reason)
 return None
def stop(s):return abs(s[4])<=.01 and abs(s[5])<=.05 and abs(s[6])<=1e-6 and abs(s[7])<=1e-6
last_settle_metrics = {}
def velocity_stopped():
 now=time.monotonic();epoch=time.time()
 for topic in ['/odometry','/motion_state']:
  if topic not in latest or now-latest[topic][0]>.25:return False
  msg=latest[topic][1];age=epoch-msg.header.stamp.sec-msg.header.stamp.nanosec/1e9
  if not 0<=age<=.25:return False
 m=latest['/motion_state'][1];o=latest['/odometry'][1].twist.twist
 values=[m.v_car,m.w_car,o.linear.x,o.linear.y,o.angular.z]
 if not all(math.isfinite(v) for v in values):return False
 if abs(m.v_car)>.01 or abs(m.w_car)>.05 or math.hypot(o.linear.x,o.linear.y)>.01 or abs(o.angular.z)>.05:return False
 if '/cmd_vel' not in latest:return current is None and not any(x.status in [1,2,3] for x in statuses+follow_status)
 cmd=latest['/cmd_vel'][1]
 return abs(cmd.linear.x)<=1e-6 and abs(cmd.angular.z)<=1e-6
def settle(seconds=12):
 global last_settle_metrics,bad_since
 end=time.monotonic()+seconds;window=StationaryAcceptance();best=None
 while time.monotonic()<end:
  pump()
  try:s=sample()
  except RuntimeError:
   # During stationary acceptance, missing TF is a localization-quality result.
   # Moving-task telemetry guards in run() remain unchanged.
   s=None;bad_since=None
  stopped=velocity_stopped();now=time.monotonic()
  pose=s if s is not None and 0<=s[8]<=.25 else None
  window.observe(now,stopped,pose);metrics=window.evaluate(now)
  if metrics['stationary']:
   if best is None or metrics['pose_coverage']>best['pose_coverage']:best=metrics
   if metrics['pose_evaluable']:
    last_settle_metrics=metrics
    return metrics['poses']
 if best is not None and velocity_stopped():
  last_settle_metrics=best
  return best['poses']
 raise RuntimeError('no independent stationary velocity window')
def await_future(f,seconds=5):
 end=time.monotonic()+seconds
 while not f.done() and time.monotonic()<end:pump()
 if not f.done():raise RuntimeError('service or future timeout')
 return f.result()
def get_params(name):
 c=n.create_client(ListParameters,name+'/list_parameters');req=ListParameters.Request();req.depth=0
 if not c.wait_for_service(timeout_sec=3):raise RuntimeError('parameters unavailable '+name)
 names=await_future(c.call_async(req)).result.names;c=n.create_client(GetParameters,name+'/get_parameters');req=GetParameters.Request();req.names=names;vals=await_future(c.call_async(req)).values
 attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
 values={k:(list(getattr(v,attrs[v.type])) if v.type>=5 else getattr(v,attrs[v.type])) for k,v in zip(names,vals) if v.type in attrs}
 (out/(name.strip('/')+'_runtime.yaml')).write_text(yaml.safe_dump({name:{'ros__parameters':values}},allow_unicode=True));return values
client=ActionClient(n,NavigationService,'/navigation_service')
cancel_client=n.create_client(CancelGoal,'/navigation_service/_action/cancel_goal')
def cancel_own(reason):
 if not current:raise RuntimeError('no own goal')
 req=CancelGoal.Request();req.goal_info.goal_id.uuid=list(bytes.fromhex(current['uuid']))
 if not cancel_client.wait_for_service(timeout_sec=3):raise RuntimeError('own cancel service unavailable')
 res=await_future(cancel_client.call_async(req));current['cancel_reason']=reason;current['cancel_response']=res.return_code;save()
 return res

def run(start,target,speed,direction,return_leg=False):
 global current,handle,result_future,accept_future
 win=settle()
 if not last_settle_metrics['pose_evaluable']:raise RuntimeError('start localization coverage unavailable; no goal sent')
 if math.dist(win[-1][2:4],start)>.1:raise RuntimeError('next start farther than 0.1 m')
 if any(x.status in [1,2,3] for x in statuses+follow_status):raise RuntimeError('another active navigation task')
 seg={'segment_type':1,'segment_name':'diagonal_corridor','segment_id':'segment_001','node1':dict(x=start[0],y=start[1],z=0.),'node2':dict(x=target[0],y=target[1],z=0.),'control_pos1':dict(x=10.,y=-4.37,z=0.),'control_pos2':dict(x=11.,y=-4.37,z=0.),'max_load_speed':0.,'max_speed':speed,'motion_direction':direction,'dwell_time':0}
 if return_leg:seg['control_pos1']=dict(x=13.2,y=-.47,z=0.)
 goal=NavigationService.Goal();payload={'task_id':'myworld_fixed_path_001','navi_segment':[seg]};set_message_fields(goal,json.loads(json.dumps(payload)))
 own_uuid=uuid.uuid4().hex;begin=time.monotonic();label=f'{speed:.1f}_{2 if return_leg else 1}'
 current={'speed':speed,'direction':direction,'return_leg':return_leg,'label':label,'goal':payload,'uuid':own_uuid,'begin_wall':begin,'begin_epoch':time.time(),'status':'WAITING_ACCEPTANCE'};report['cases'].append(current);report['status']='RUNNING';handle=None;result_future=None;save()
 raw.write(json.dumps({'event':'submit','wall':begin,'uuid':own_uuid,'goal':payload})+'\n')
 def feedback(m):raw.write(json.dumps({'event':'feedback','wall':time.monotonic(),'uuid':own_uuid,'feedback':message_to_ordereddict(m.feedback)})+'\n')
 accept_future=client.send_goal_async(goal,feedback_callback=feedback,goal_uuid=UUID(uuid=list(bytes.fromhex(own_uuid))))
 print('START',len(report['cases']),label,own_uuid,flush=True)
 while not accept_future.done():
  pump();sample()
  if time.monotonic()-begin>20:
   current['status']='ACCEPTANCE_TIMEOUT';save();cancel_own('no acceptance within 20 seconds')
   raise RuntimeError('acceptance timeout; disposition must be checked before another task')
 handle=accept_future.result()
 if not handle or not handle.accepted:current['status']='REJECTED';save();raise RuntimeError('goal rejected')
 current['accepted_wall']=time.monotonic();current['status']='EXECUTING';result_future=handle.get_result_async();save()
 trace=[];park=None;cancel_time=None
 while not result_future.done():
  pump();s=sample()
  if s is None:park=None;continue
  trace.append(s)
  observed='/cmd_vel' in latest and latest['/cmd_vel'][0]>=begin
  if observed and stop(s) and math.dist(s[2:4],target)<=.5:park=park or s[0]
  else:park=None
  if cancel_time is None and (s[0]-begin>180 or (park and s[0]-park>=10)):
   cancel_own('terminal parked without result' if park else 'timeout 180 seconds');cancel_time=s[0]
  if cancel_time and s[0]-cancel_time>15:raise RuntimeError('own result missing after cancel')
 res=result_future.result();current['action_status']=res.status;current['result']=message_to_ordereddict(res.result);current['status']={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(res.status,str(res.status));current['result_wall']=time.monotonic();save()
 win=settle();trace+=win;current['end_wall']=time.monotonic();current['stop_window']=win;current['stationary_acceptance']=last_settle_metrics;current['final_pose']=win[-1][2:4] if win else None;current['observed_error_mm']=1000*max(math.dist(x[2:4],target) for x in win) if win else None;current['error_mm']=current['observed_error_mm'] if last_settle_metrics['pose_evaluable'] else None;current['precision_status']='UNAVAILABLE' if current['error_mm'] is None else ('PASS' if current['error_mm']<=10 else 'FAIL')
 first=None;stability=StationaryAcceptance();recent={}
 terminal_entry=next((x[0] for x in trace if math.dist(x[2:4],target)<=.5),None)
 for t,topic,v,w in list(velocity_history):
  recent[topic]=(t,v,w)
  if terminal_entry is None or t<terminal_entry:continue
  fresh=all(k in recent and 0<=t-recent[k][0]<=.25 for k in ['/odometry','/motion_state'])
  stopped=fresh and all(abs(recent[k][1])<=.01 and abs(recent[k][2])<=.05 for k in ['/odometry','/motion_state'])
  stopped=stopped and '/cmd_vel' in recent and abs(recent['/cmd_vel'][1])<=1e-6 and abs(recent['/cmd_vel'][2])<=1e-6
  stability.observe(t,stopped)
  result=stability.evaluate(t)
  if result['stationary']:first=result['stationary_since'];break
 current['stop_wall']=first;current['physical_stop_s']=first-begin if first else None;current['result_elapsed_s']=current['result_wall']-begin;current['peak_mps']=max((abs(x[4]) for x in trace),default=0.0);current['peak_reached_95pct']=current['peak_mps']>=.95*speed
 current['sampled_creep_command_s']=0.;current['sampled_creep_measured_s']=0.
 for a,z in zip(trace,trace[1:]):
  dt=z[0]-a[0]
  if first and z[0]<=first and dt<=.1 and math.dist(a[2:4],target)<=.5:
   for k,i in [('sampled_creep_command_s',6),('sampled_creep_measured_s',4)]:
    if .005<=abs(a[i])<=.0200001:current[k]+=dt
 save();
 if current['status']!='SUCCEEDED':raise RuntimeError('case did not succeed; stop subsequent goals')
 print('DONE',json.dumps({k:v for k,v in current.items() if k not in ['goal','stop_window','stationary_acceptance']}),flush=True);unavailable=current['precision_status']=='UNAVAILABLE';current=None;handle=None;result_future=None;accept_future=None
 if unavailable:raise RuntimeError('precision unavailable; completed action and velocity data preserved; no next goal sent')
try:
 end=time.monotonic()+20;s=None
 while time.monotonic()<end:
  pump()
  try:
   s=sample()
   if s is not None:break
  except Exception:bad_since=None
 if s is None:raise RuntimeError('no fresh pose/feedback for preflight')
 end=time.monotonic()+10
 while time.monotonic()<end:pump();sample()
 if any(x.status in [1,2,3] for x in statuses+follow_status):raise RuntimeError('another active navigation task')
 params=get_params('/controller_server');get_params('/velocity_smoother')
 report['enabled_flags']={k:params.get(k) for k in ['FixedPathController.terminal_convergence_enabled','FixedPathController.dynamic_goal_braking_margin_enabled']};save()
 if not all(x is True for x in report['enabled_flags'].values()):raise RuntimeError('both flags must be enabled')
 if params.get('FixedPathController.goal_stop_entry_tolerance')!=.008 or params.get('FixedPathController.goal_braking_min_distance_margin')!=.03:raise RuntimeError('stage1 requires loaded entry 8mm and low margin 0.03m')
 win=settle()
 if not last_settle_metrics['pose_evaluable']:raise RuntimeError('preflight localization coverage unavailable')
 report['initial_pose']=win[-1][2:4];report['idle_cmd_received']='/cmd_vel' in latest;save()
 A=(9.16,-4.37);B=(13.2,-.47)
 if math.dist(win[-1][2:4],A)>.1:raise RuntimeError('not at A; no extra navigation authorized')
 if not client.wait_for_server(timeout_sec=5):raise RuntimeError('navigation action server unavailable')
 for speed in [round(.2+i*.1,1) for i in range(14)]:
  for leg in [1,2]:run(A if leg==1 else B,B if leg==1 else A,speed,1,leg==2)
 win=settle();report['status']='COMPLETE';report['final_stationary_verified']=True;report['final_pose']=win[-1][2:4] if win else None;save()
except Exception as e:
 report['status']='BLOCKED';report['blocker']=str(e)
 if current:
  try:
   if not result_future or not result_future.done():cancel_own(str(e))
   if result_future and not result_future.done():await_future(result_future,15)
   if result_future and result_future.done():
    res=result_future.result();current['action_status']=res.status;current['status']={4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}.get(res.status,str(res.status))
  except Exception as ce:report['cancel_error']=str(ce)
 try:
  bad_since=None;win=settle();report['final_stationary_verified']=True;report['final_pose']=win[-1][2:4] if win else None
 except Exception as se:report['stop_check_error']=str(se)
 save();print('BLOCKED',str(e),flush=True)
finally:
 rclpy.shutdown();spin_thread.join(timeout=3);raw.close();n.destroy_node()
