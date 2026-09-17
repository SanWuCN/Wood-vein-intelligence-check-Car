"""Real Nav2 BT with fake planner/controller, localhost-only domain 88. No chassis."""
import os
os.environ['ROS_DOMAIN_ID']='88';os.environ['ROS_LOCALHOST_ONLY']='1'
import time,threading,subprocess,signal
from pathlib import Path
import rclpy
from rclpy.action import ActionServer,ActionClient,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateThroughPoses,ComputePathThroughPoses,FollowPath,Wait,BackUp
from nav2_msgs.srv import ClearEntireCostmap
from lifecycle_msgs.srv import ChangeState
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

def main():
 rclpy.init();n=rclpy.create_node('continuous_bt_fixture');group=ReentrantCallbackGroup();plans=[];feedback=[];location=[0.,0.];closing=threading.Event()
 def plan(h):
  plans.append([(p.pose.position.x,p.pose.position.y) for p in h.request.goals]);r=ComputePathThroughPoses.Result();r.path.header.frame_id='map';r.path.header.stamp=n.get_clock().now().to_msg();r.path.poses=list(h.request.goals);h.succeed();return r
 def follow(h):
  while not closing.is_set() and not h.is_cancel_requested:time.sleep(.03)
  if h.is_cancel_requested:h.canceled()
  else:h.abort()
  return FollowPath.Result()
 def wait(h):h.succeed();return Wait.Result()
 def backup(h):h.succeed();return BackUp.Result()
 servers=[ActionServer(n,t,name,execute_callback=fn,callback_group=group,cancel_callback=lambda h:CancelResponse.ACCEPT) for t,name,fn in [(ComputePathThroughPoses,'compute_path_through_poses',plan),(FollowPath,'follow_path',follow),(Wait,'wait',wait),(BackUp,'backup',backup)]]
 services=[n.create_service(ClearEntireCostmap,name,lambda q,r:r,callback_group=group) for name in ['global_costmap/clear_entirely_global_costmap','local_costmap/clear_entirely_local_costmap']]
 tf=TransformBroadcaster(n);odom=n.create_publisher(Odometry,'odom',10)
 def publish():
  t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint';t.header.stamp=n.get_clock().now().to_msg();t.transform.translation.x=location[0];t.transform.translation.y=location[1];t.transform.rotation.w=1.;tf.sendTransform(t)
  m=Odometry();m.header=t.header;m.child_frame_id='base_footprint';m.pose.pose.position.x=location[0];m.pose.pose.position.y=location[1];m.pose.pose.orientation.w=1.;odom.publish(m)
 timer=n.create_timer(.03,publish,callback_group=group);executor=MultiThreadedExecutor(num_threads=12);executor.add_node(n);thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
 xml=str(Path(__file__).resolve().parents[1]/'backend/behavior_trees/continuous_navigation.xml');log=open('/tmp/continuous-bt-check.log','w')
 process=subprocess.Popen(['/home/wheeltec/wheeltec_ros2/install/nav2_bt_navigator/lib/nav2_bt_navigator/bt_navigator','--ros-args','-p','global_frame:=map','-p','robot_base_frame:=base_footprint','-p','default_nav_through_poses_bt_xml:='+xml,'-p','default_nav_to_pose_bt_xml:='+xml],stdout=log,stderr=subprocess.STDOUT)
 def await_future(f,seconds=12):
  end=time.monotonic()+seconds
  while not f.done() and time.monotonic()<end:time.sleep(.03)
  assert f.done(),'ROS future timeout';return f.result()
 def until(fn):
  end=time.monotonic()+12
  while not fn() and time.monotonic()<end:time.sleep(.05)
  assert fn(),'condition timed out; inspect /tmp/continuous-bt-check.log'
 try:
  client=n.create_client(ChangeState,'/bt_navigator/change_state',callback_group=group);assert client.wait_for_service(timeout_sec=15)
  for transition in [1,3]:
   req=ChangeState.Request();req.transition.id=transition;assert await_future(client.call_async(req)).success
  nav=ActionClient(n,NavigateThroughPoses,'/navigate_through_poses',callback_group=group);assert nav.wait_for_server(timeout_sec=10)
  goal=NavigateThroughPoses.Goal();goal.behavior_tree=xml
  for x,y in [(1.,0.),(2.,1.),(3.,1.)]:
   p=PoseStamped();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;goal.poses.append(p)
  handle=await_future(nav.send_goal_async(goal,feedback_callback=lambda m:feedback.append(m.feedback.number_of_poses_remaining)));assert handle.accepted
  until(lambda:any(len(p)==3 for p in plans))
  location[:]=[.80,0.] # Inside the configured 25 cm pass radius.
  until(lambda:2 in feedback and any(len(p)==2 for p in plans))
  location[:]=[1.6,.6] # Outside the previous radius; it must remain removed.
  before=len(plans);until(lambda:len(plans)>before)
  assert all((1.,0.) not in p for p in plans[before:]),'passed point reappeared'
  await_future(handle.cancel_goal_async());await_future(handle.get_result_async())
  print('PASS: real Nav2 tree loads; planner receives all 3 points; 20 cm proximity removes first; later replanning never restores it; cancellation works. Fake controller only; no cmd_vel or chassis.')
 finally:
  closing.set()
  if process.poll() is None:
   process.send_signal(signal.SIGINT)
   try:process.wait(timeout=8)
   except subprocess.TimeoutExpired:process.kill();process.wait()
  executor.shutdown(timeout_sec=3);n.destroy_node();rclpy.shutdown();log.close()
if __name__=='__main__':main()
