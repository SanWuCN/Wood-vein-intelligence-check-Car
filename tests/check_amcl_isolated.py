"""Standalone integration check. Domain 87 + localhost; only AMCL and a test map.
Run explicitly after building the readiness patch. Never starts robot launch files.
"""
import os
os.environ['ROS_DOMAIN_ID']='87'
os.environ['ROS_LOCALHOST_ONLY']='1'
import subprocess,time,signal
from pathlib import Path
import rclpy
from lifecycle_msgs.srv import ChangeState,GetState
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy

def main():
 log=Path('/tmp/mumai-amcl-isolated.log')
 with log.open('w') as output:
  process=subprocess.Popen(['/home/wheeltec/wheeltec_ros2/install/nav2_amcl/lib/nav2_amcl/amcl','--ros-args','-p','set_initial_pose:=false'],stdout=output,stderr=subprocess.STDOUT)
  rclpy.init();node=rclpy.create_node('amcl_readiness_check')
  try:
   change=node.create_client(ChangeState,'/amcl/change_state');glob=node.create_client(Empty,'/reinitialize_global_localization')
   def call(client,req):
    assert client.wait_for_service(timeout_sec=15),'service unavailable'
    f=client.call_async(req);rclpy.spin_until_future_complete(node,f,timeout_sec=8)
    assert f.done(),'service timed out';return f.result()
   req=ChangeState.Request();req.transition.id=1;assert call(change,req).success
   call(glob,Empty.Request());assert process.poll() is None
   assert 'deferred: AMCL or map is not ready' in log.read_text()
   req.transition.id=3;assert call(change,req).success
   call(glob,Empty.Request());assert process.poll() is None
   pub=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
   msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=.05;msg.info.width=40;msg.info.height=40;msg.info.origin.orientation.w=1.;msg.data=[0]*1600
   for y in range(40):
    for x in range(40):
     if x in (0,39) or y in (0,39):msg.data[y*40+x]=100
   pub.publish(msg)
   end=time.monotonic()+10
   while 'Received a 40 X 40 map' not in log.read_text() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
   assert 'Received a 40 X 40 map' in log.read_text()
   call(glob,Empty.Request());assert process.poll() is None
   assert 'Global initialisation done!' in log.read_text()
   print('PASS: early requests before activation/map do not crash; after map global localization succeeds; isolated domain 87, AMCL only, no chassis or navigation actions')
  finally:
   node.destroy_node();rclpy.shutdown()
   if process.poll() is None:
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=8)
    except subprocess.TimeoutExpired:process.kill();process.wait()
if __name__=='__main__':main()
