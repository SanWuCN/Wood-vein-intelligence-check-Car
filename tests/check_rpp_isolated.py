"""Real installed Nav2 controller, fake odometry, isolated domain, no chassis."""
import os
os.environ['ROS_DOMAIN_ID']='89'
os.environ['ROS_LOCALHOST_ONLY']='1'
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import ChangeState
from tf2_ros import TransformBroadcaster
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'backend'))
from navigation_profile import apply_forward_profile


def main():
    root=Path(__file__).resolve().parents[1]
    source=Path('/home/wheeltec/mumai-console/runtime/backups/cruise-20260917/navigation.yaml')
    cfg=apply_forward_profile(yaml.safe_load(source.read_text()),root,avoidance=False)
    local=cfg['local_costmap']['local_costmap']['ros__parameters']
    local.update(plugins=['static_layer','inflation_layer'],width=4,height=4,
                 static_layer={'plugin':'nav2_costmap_2d::StaticLayer',
                               'map_topic':'/fixture_map','map_subscribe_transient_local':True})
    cs=cfg['controller_server']['ros__parameters']
    cs.update(goal_checker_plugins=['goal_checker'],bond_heartbeat_period=0.)
    rclpy.init();node=rclpy.create_node('rpp_fixture')
    pose=[0.,0.,0.];command=[0.,0.];last_command=[0.];samples=[];enabled=[False]
    tf=TransformBroadcaster(node);odom=node.create_publisher(Odometry,'odom_combined',10)
    map_pub=node.create_publisher(OccupancyGrid,'fixture_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def publish_map(blocked=False):
        msg=OccupancyGrid();msg.header.frame_id='odom_combined';msg.header.stamp=node.get_clock().now().to_msg()
        msg.info.resolution=.05;msg.info.width=200;msg.info.height=200
        msg.info.origin.position.x=-5.;msg.info.origin.position.y=-5.;msg.info.origin.orientation.w=1.
        msg.data=[0]*40000
        if blocked:
            for y in range(95,106):
                for x in range(107,109):msg.data[y*200+x]=100
        map_pub.publish(msg)
    def receive(msg):
        command[:]=[msg.linear.x,msg.angular.z];last_command[0]=time.monotonic()
        if enabled[0]:samples.append((time.monotonic(),*pose,*command))
    sub=node.create_subscription(Twist,'fixture_cmd_vel',receive,10)
    previous=[time.monotonic()]
    def tick():
        now=time.monotonic();dt=min(.05,now-previous[0]);previous[0]=now
        v,w=command if enabled[0] and now-last_command[0]<.3 else (0.,0.)
        # Ackermann plant cannot exceed the configured steering curvature.
        w=max(-abs(v)/.30,min(abs(v)/.30,w))
        pose[0]+=v*math.cos(pose[2])*dt;pose[1]+=v*math.sin(pose[2])*dt;pose[2]+=w*dt
        stamp=node.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='odom_combined';t.child_frame_id='base_footprint'
        t.transform.translation.x=pose[0];t.transform.translation.y=pose[1]
        t.transform.rotation.z=math.sin(pose[2]/2);t.transform.rotation.w=math.cos(pose[2]/2);tf.sendTransform(t)
        msg=Odometry();msg.header=t.header;msg.child_frame_id=t.child_frame_id
        msg.pose.pose.position.x=pose[0];msg.pose.pose.position.y=pose[1];msg.pose.pose.orientation=t.transform.rotation
        msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w;odom.publish(msg)
    timer=node.create_timer(.02,tick)
    executor=MultiThreadedExecutor(num_threads=3);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def wait(f,timeout=15):
        deadline=time.monotonic()+timeout
        while not f.done() and time.monotonic()<deadline:time.sleep(.02)
        assert f.done(),'ROS future timed out'
        return f.result()
    reports=[]
    with tempfile.TemporaryDirectory(prefix='rpp-isolated-') as directory:
        params=Path(directory)/'params.yaml';params.write_text(yaml.safe_dump(cfg))
        log=Path('/tmp/rpp-isolated.log').open('w')
        process=subprocess.Popen(['ros2','run','nav2_controller','controller_server','--ros-args',
                                  '--params-file',str(params),'-r','cmd_vel:=fixture_cmd_vel'],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            publish_map();client=node.create_client(ChangeState,'controller_server/change_state')
            assert client.wait_for_service(timeout_sec=20)
            for transition in (1,3):
                req=ChangeState.Request();req.transition.id=transition
                assert wait(client.call_async(req)).success,'Controller lifecycle failed'
            action=ActionClient(node,FollowPath,'follow_path');assert action.wait_for_server(timeout_sec=10)
            def run_case(name,points,start,blocked=False,cancel=False):
                enabled[0]=False;command[:]=[0.,0.];pose[:]=start;samples.clear();publish_map(blocked)
                time.sleep(1.0)
                req=FollowPath.Goal();req.controller_id='FollowPath';req.goal_checker_id='goal_checker'
                req.path.header.frame_id='odom_combined';req.path.header.stamp=node.get_clock().now().to_msg()
                for x,y,a in points:
                    p=PoseStamped();p.header=req.path.header;p.pose.position.x=float(x);p.pose.position.y=float(y)
                    p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2);req.path.poses.append(p)
                enabled[0]=True;handle=wait(action.send_goal_async(req));assert handle.accepted
                future=handle.get_result_async()
                if cancel:
                    time.sleep(1.0);wait(handle.cancel_goal_async())
                result=wait(future,55);enabled[0]=False
                values=list(samples);assert values,'No controller output'
                error=max(min(math.hypot(v[1]-x,v[2]-y) for x,y,_ in points) for v in values)
                turns=[1 if v[5]>.015 else -1 for v in values if abs(v[5])>.015]
                reversals=sum(a!=b for a,b in zip(turns,turns[1:]))
                gaps=[b[0]-a[0] for a,b in zip(values,values[1:])]
                report={'case':name,'status':result.status,'samples':len(values),'max_cross_track_m':round(error,4),
                        'steering_reversals_above_0015':reversals,'max_command_gap_s':round(max(gaps,default=0),3),
                        'max_angular_rps':round(max(abs(v[5]) for v in values),4),
                        'end_pose':[round(x,4) for x in pose]}
                print(json.dumps(report),flush=True);reports.append(report)
                assert result.status==(6 if blocked else 5 if cancel else 4),report
                assert abs(command[0])+abs(command[1])<1e-6,'Terminal action did not stop'
                assert all(-1e-6<=v[4]<=.10001 for v in values),'Velocity limit violated'
                if not blocked and not cancel:
                    assert error<.12,report
                    assert max(gaps,default=0)<.5,report
                if name=='straight':assert report['max_angular_rps']<.001,report
                if name=='offset_straight':assert reversals<=2 and abs(pose[1])<.025,report
            straight=[(i*.025,0.,0.) for i in range(49)]
            run_case('straight',straight,[0.,0.,0.])
            run_case('offset_straight',straight,[0.,.06,.06])
            bend=[(i*.025,0.,0.) for i in range(33)]
            bend +=[(.8+.4*math.sin(i*math.pi/80),.4-.4*math.cos(i*math.pi/80),i*math.pi/80) for i in range(1,41)]
            bend +=[(1.2,.4+i*.025,math.pi/2) for i in range(1,25)]
            run_case('quarter_turn',bend,[0.,0.,0.])
            run_case('cancel',straight,[0.,0.,0.],cancel=True)
            run_case('collision',straight,[0.,0.,0.],blocked=True)
            Path('/tmp/rpp-isolated-results.json').write_text(json.dumps(reports,indent=2))
            print('PASS: real RPP tracking, low speed, curvature, cancellation, collision stop; isolated domain 89.',flush=True)
        finally:
            enabled[0]=False;os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            executor.shutdown(timeout_sec=3);node.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
