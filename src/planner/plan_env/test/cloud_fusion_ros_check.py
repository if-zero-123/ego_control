#!/usr/bin/env python3
"""Isolated ROS map/planner checks and optional recorded raw-cloud replay.

Only localhost:11432 is allowed. No MAVROS/bridge/payload process is launched.
"""
import argparse
import copy
import json
import os
import signal
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
from pathlib import Path

os.environ['ROS_MASTER_URI']='http://127.0.0.1:11432'
os.environ['ROS_IP']='127.0.0.1'
os.environ.pop('ROS_HOSTNAME',None)
os.environ['ROS_LOG_DIR']=tempfile.mkdtemp(prefix='ego_cloud_fusion_ros_logs_')

import numpy as np
import rospy
import rosbag
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header,String
from traj_utils.msg import Bspline

class Check:
    def __init__(self,params):
        self.params=params;self.children=[];self.logs=[];self.status={};self.maps={};self.status_history={};self.trajs=[]
        self.passed=[];self.lock=threading.Lock()
    def spawn(self,args,name):
        log=open(Path(os.environ['ROS_LOG_DIR'])/(name+'.log'),'w');self.logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);self.children.append(p);return p
    def stop(self,p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    def cleanup(self):
        for p in reversed(self.children):self.stop(p)
        for f in self.logs:f.close()
    def wait(self,fn,seconds=4):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if fn():return True
            if any(p.poll() is not None for p in self.children[:1]):raise RuntimeError('isolated master exited')
            time.sleep(.02)
        return False
    def check(self,name,ok):
        if not ok:raise AssertionError(name)
        self.passed.append(name);print('PASS',name,flush=True)
    def on_status(self,m,name):
        value=json.loads(m.data)
        with self.lock:
            self.status[name]=value;self.status_history.setdefault(name,[]).append(value)
    def on_map(self,m,name):
        # PCL XYZ has 16-byte point stride and unorganized output.
        if not m.width:
            with self.lock:self.maps[name]=np.empty((0,3))
            return
        dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,
                        'offsets':[next(f.offset for f in m.fields if f.name==k) for k in ['x','y','z']],
                        'itemsize':m.point_step})
        a=np.frombuffer(m.data,dtype=dtype,count=m.width*m.height)
        xyz=np.column_stack((a['x'],a['y'],a['z']))
        with self.lock:self.maps[name]=xyz
    def occupied(self,p,name='cloud_probe'):
        with self.lock:a=self.maps.get(name)
        return a is not None and bool(np.any(np.max(np.abs(a-np.asarray(p)),axis=1)<.024))
    def node(self,name,planner=False,synthetic=True):
        original='/drone_0_ego_planner_node'
        cfg={k.replace(original,'/'+name,1):v for k,v in self.params.items() if k.startswith(original+'/')}
        cfg.update({'/'+name+'/grid_map/cloud_pose_topic':'/Odometry',
                    '/'+name+'/grid_map/cloud_fusion':True,
                    '/'+name+'/fsm/realworld_experiment':False})
        if synthetic:
            cfg.update({'/'+name+'/grid_map/map_size_x':8.,'/'+name+'/grid_map/map_size_y':8.,
                        '/'+name+'/grid_map/local_update_range_x':2.5,'/'+name+'/grid_map/local_update_range_y':2.5,
                        '/'+name+'/grid_map/cloud_sensor_translation':[0.,0.,0.]})
        for k,v in cfg.items():rospy.set_param(k,v)
        rospy.Subscriber('/'+name+'/grid_map/cloud_fusion_status',String,self.on_status,callback_args=name,queue_size=100)
        rospy.Subscriber('/'+name+'/grid_map/occupancy_inflate',PointCloud2,self.on_map,callback_args=name,queue_size=1)
        pkg,exe=('ego_planner','ego_planner_node') if planner else ('plan_env','cloud_fusion_probe')
        return self.spawn(['rosrun',pkg,exe,'__name:='+name,'~grid_map/cloud:=/cloud_registered',
                           '~grid_map/odom:=/mavros/local_position/odom','~odom_world:=/mavros/local_position/odom',
                           '~planning/bspline:=/test/bspline'],name)
    def setup(self):
        try:
            xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/fusion_check')
            raise RuntimeError('port 11432 already has a master; refusing to reuse it')
        except OSError:pass
        self.spawn(['roscore','-p','11432'],'master')
        def ready():
            try:return xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getUri('/fusion_check')[0]==1
            except OSError:return False
        assert self.wait(ready,10)
        rospy.init_node('cloud_fusion_isolated_check',anonymous=True,disable_signals=True)
        assert rospy.get_master().getUri()[2]=='http://127.0.0.1:11432/' or '11432' in rospy.get_master().getUri()[2]
        self.cloud_pub=rospy.Publisher('/cloud_registered',PointCloud2,queue_size=1)
        self.pose_pub=rospy.Publisher('/Odometry',Odometry,queue_size=10)
        self.odom_pub=rospy.Publisher('/mavros/local_position/odom',Odometry,queue_size=10)
        self.goal_pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
        rospy.Subscriber('/test/bspline',Bspline,lambda m:self.trajs.append(m),queue_size=10)
    def send(self,points,x=.025,y=.025,z=1.025,cloud_first=False,frame='camera_init',velocity=None,stamp_offset=0.):
        stamp=rospy.Time.now()+rospy.Duration(stamp_offset);p=Odometry();p.header=Header(stamp=stamp,frame_id='camera_init')
        p.pose.pose.position.x=x;p.pose.pose.position.y=y;p.pose.pose.position.z=z;p.pose.pose.orientation.w=1.
        if velocity is not None:
            p.twist.twist.linear.x,p.twist.twist.linear.y,p.twist.twist.linear.z=velocity
        m=point_cloud2.create_cloud_xyz32(Header(stamp=stamp,frame_id=frame),points)
        od=copy.deepcopy(p);od.header.frame_id='map';self.odom_pub.publish(od)
        if cloud_first:self.cloud_pub.publish(m);time.sleep(.005);self.pose_pub.publish(p)
        else:self.pose_pub.publish(p);self.cloud_pub.publish(m)
        time.sleep(.11)
    def synthetic(self):
        p=self.node('cloud_probe');assert self.wait(lambda:self.cloud_pub.get_num_connections()>0,10)
        wall=[1.025,.025,1.025];far=[2.025,.025,1.025]
        for i in range(5):self.send([wall],cloud_first=i%2==0)
        self.check('matched pose works in either topic delivery order',self.status['cloud_probe']['ready'])
        self.check('current obstacle is present in published inflated map',self.wait(lambda:self.occupied(wall)))
        for i in range(3):self.send([wall],stamp_offset=.04)
        self.check('small allowed timestamp lead does not falsely trigger timeout',self.status['cloud_probe']['ready'])
        self.send([])
        self.check('empty scan retains remembered obstacle',self.occupied(wall))
        for x in [.225,.425,.625,.825]:self.send([far],x=x)
        for i in range(4):self.send([far],x=.825)
        self.check('blind-zone obstacle survives valid longer returns',self.occupied(wall))
        for x in [.625,.425,.225,.025]:self.send([far],x=x)
        self.send([far]);self.check('single reliable free frame does not immediately erase obstacle',self.occupied(wall))
        for i in range(7):self.send([far])
        self.check('repeated reliable free frames restore an open passage',self.wait(lambda:not self.occupied(wall)))
        self.send([wall]);self.check('new obstacle is protected without confirmation delay',self.wait(lambda:self.occupied(wall)))
        before=self.status['cloud_probe']['stamp']
        self.send([far],frame='wrong_world')
        self.check('wrong-frame cloud cannot refresh map freshness',self.status['cloud_probe']['stamp']==before)
        time.sleep(.6)
        self.check('missing valid cloud activates existing map timeout',self.wait(lambda:not self.status['cloud_probe']['ready']))
        for i in range(5):self.send([wall])
        self.check('fresh synchronized scans restore map readiness',self.status['cloud_probe']['ready'])
        bad=Odometry();bad.header=Header(stamp=rospy.Time.now()+rospy.Duration(2.),frame_id='camera_init')
        bad.pose.pose.orientation.w=1.;bad.pose.pose.position.x=10.
        self.pose_pub.publish(bad)
        for i in range(3):self.send([wall])
        self.check('future-dated bad pose cannot poison cache or erase map',self.status['cloud_probe']['ready'] and self.occupied(wall))
        new_wall=[3.025,.025,1.025]
        self.send([new_wall],x=2.025,cloud_first=True)
        self.check('localization jump requires map warmup again',not self.status['cloud_probe']['ready'])
        self.check('localization jump replaces old ghosts with its matched first frame',
                   self.wait(lambda:not self.occupied(wall) and self.occupied(new_wall)))
        for i in range(3):self.send([new_wall],x=2.025)
        self.check('new-coordinate synchronized map becomes ready after warmup',self.status['cloud_probe']['ready'] and self.occupied(new_wall))
        self.stop(p)
    def moving_clearing(self):
        name='moving_probe';p=self.node(name)
        assert self.wait(lambda:self.cloud_pub.get_num_connections()>0,10)
        wall=[1.025,.025,1.025];far=[2.025,.025,1.025]
        for i in range(5):self.send([wall])
        self.check('moving test starts with remembered obstacle',self.wait(lambda:self.occupied(wall,name)))
        for i in range(12):self.send([far],x=.025+(.028 if i%2 else -.028))
        self.check('bounded motion permits reliable ray clearing',self.wait(lambda:not self.occupied(wall,name)))
        history=self.status_history[name]
        self.check('motion uses explicit conservative origin envelope',any(s.get('clear_reason')=='bounded_scan_origins' and s.get('origin_uncertainty',0)>.02 for s in history))
        self.check('moving current endpoints stay occupied',self.occupied(far,name))
        for i in range(3):self.send([wall])
        self.wait(lambda:self.occupied(wall,name))
        time.sleep(.3);self.send([far])
        self.check('pose coverage gap disables clearing but preserves hits',self.status[name].get('clear_reason') in ('scan_pose_gap','insufficient_scan_pose_coverage') and self.occupied(wall,name))
        self.stop(p)

    def planning(self):
        p=self.node('fusion_planner',planner=True);assert self.wait(lambda:self.cloud_pub.get_num_connections()>0,10)
        # Wall at x=1.025 leaves horizontal room on both sides; no extra height floor.
        wall=[[1.025,y,z] for y in np.arange(-.525,.576,.05) for z in np.arange(.025,1.976,.05)]
        floor=[[x,y,-.275] for x in np.arange(-1.975,3.026,.1) for y in np.arange(-1.975,2.026,.1)]
        scene=wall+floor
        for i in range(6):self.send(scene)
        goal=PoseStamped();goal.header.frame_id='world';goal.pose.position.x=2.525;goal.pose.position.y=.025
        goal.pose.position.z=1.025;goal.pose.orientation.w=1.
        goal.header.stamp=rospy.Time.now();self.goal_pub.publish(goal)
        for i in range(50):
            self.send(scene)
            if self.trajs:break
        self.check('actual EGO generates a route around a blocked direct line',bool(self.trajs))
        # De Boor evaluation uses the same knots/control points as EGO.
        traj=self.trajs[-1]
        cp=np.array([[v.x,v.y,v.z] for v in traj.pos_pts])
        times=np.linspace(traj.knots[traj.order],traj.knots[len(cp)],1001)
        knots=np.array(traj.knots);degree=traj.order
        def evaluate(u):
            k=min(len(cp)-1,max(degree,int(np.searchsorted(knots,u,side='right'))-1))
            d=cp[k-degree:k+1].copy()
            for r in range(1,degree+1):
                for j in range(degree,r-1,-1):
                    lo=j+k-degree;hi=j+1+k-r;denom=knots[hi]-knots[lo]
                    alpha=(u-knots[lo])/denom if denom else 0.
                    d[j]=(1-alpha)*d[j-1]+alpha*d[j]
            return d[degree]
        path=np.array([evaluate(u) for u in times])
        with self.lock:occupied=self.maps.get('fusion_planner')
        collided=False
        if occupied is not None:
            occ_idx=set(map(tuple,np.floor((occupied-np.array([-4.,-4.,-.5]))/.05).astype(int)))
            for idx in np.floor((path-np.array([-4.,-4.,-.5]))/.05).astype(int):
                if tuple(idx) in occ_idx:collided=True;break
        self.check('generated trajectory avoids published inflated obstacles',occupied is not None and not collided)
        # Feed planned motion back as synthetic odometry. This exercises map
        # window movement and re-planning without a flight controller.
        def position(message,elapsed):
            controls=np.array([[v.x,v.y,v.z] for v in message.pos_pts]);ks=np.array(message.knots);deg=message.order
            u=min(ks[len(controls)],max(ks[deg],ks[deg]+elapsed))
            k=min(len(controls)-1,max(deg,int(np.searchsorted(ks,u,side='right'))-1))
            d=controls[k-deg:k+1].copy()
            for r in range(1,deg+1):
                for j in range(deg,r-1,-1):
                    lo=j+k-deg;hi=j+1+k-r;denom=ks[hi]-ks[lo]
                    a=(u-ks[lo])/denom if denom else 0.;d[j]=(1-a)*d[j-1]+a*d[j]
            return d[deg]
        reached=False;collision=False;deadline=time.monotonic()+40
        while time.monotonic()<deadline:
            latest=self.trajs[-1];elapsed=rospy.Time.now().to_sec()-latest.start_time.to_sec()
            pos=position(latest,elapsed);vel=(position(latest,elapsed+.001)-pos)/.001
            if self.occupied(pos,'fusion_planner'):collision=True;break
            self.send(scene,x=float(pos[0]),y=float(pos[1]),z=float(pos[2]),velocity=vel)
            if np.linalg.norm(pos-np.array([2.525,.025,1.025]))<.16:reached=True;break
        self.check('closed-loop synthetic motion reaches goal without getting stuck',reached)
        self.check('closed-loop motion never enters published occupied voxels',not collision)
        self.check('map remains ready while moving and replanning',self.status['fusion_planner']['ready'])
        self.stop(p)
    def replay(self,bag_path):
        name='raw_replay_probe';p=self.node(name,synthetic=False)
        assert self.wait(lambda:self.cloud_pub.get_num_connections()>0,10)
        self.status_history[name]=[]
        with rosbag.Bag(bag_path) as bag:
            start=bag.get_start_time();wall_start=time.monotonic()+.1;shift=rospy.Time.now().to_sec()+.1-start
            cloud_count=0
            for topic,msg,t in bag.read_messages(topics=['/cloud_registered','/Odometry','/mavros/local_position/odom']):
                pause=wall_start+(t.to_sec()-start)-time.monotonic()
                if pause>0:time.sleep(pause)
                msg.header.stamp=rospy.Time.from_sec(msg.header.stamp.to_sec()+shift)
                if topic=='/cloud_registered':self.cloud_pub.publish(msg);cloud_count+=1
                elif topic=='/Odometry':self.pose_pub.publish(msg)
                else:self.odom_pub.publish(msg)
            time.sleep(.15)
        with self.lock:history=list(self.status_history[name])
        updates=[s for s in history if s.get('update_ms',0)>0]
        ms=np.array([s['update_ms'] for s in updates]);ready=sum(s['ready'] for s in updates)
        self.check('recorded raw LiDAR data fuses with matching LIO poses',len(updates)>=cloud_count*.85)
        self.check('recorded replay retains ready state on most valid frames',ready>=len(updates)*.9)
        rss=int(next(line.split()[1] for line in Path('/proc/%d/status'%p.pid).read_text().splitlines() if line.startswith('VmRSS:')))/1024.
        result={'source_cloud_frames':cloud_count,'fused_frames':len(updates),'ready_frames':ready,
                'median_ms':float(np.median(ms)),'p95_ms':float(np.percentile(ms,95)),
                'max_ms':float(np.max(ms)),'map_process_rss_MiB':rss,
                'active_cells':updates[-1]['active_cells'],'work_capacity_bytes':updates[-1]['work_capacity_bytes'],
                'clearing_enabled_frames':sum(s['clearing_enabled'] for s in updates)}
        self.stop(p);return result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--params',required=True);parser.add_argument('--bag')
    parser.add_argument('--mode',choices=['all','synthetic','replay'],default='all')
    parser.add_argument('--output',required=True);args=parser.parse_args()
    with open(args.params) as f:params=yaml.safe_load(f)
    c=Check(params);result={}
    try:
        c.setup()
        if args.mode!='replay':c.synthetic();c.moving_clearing()
        if args.mode=='all':c.planning()
        if args.bag and args.mode in ('all','replay'):result['replay']=c.replay(args.bag)
        result.update(passed=c.passed,master=os.environ['ROS_MASTER_URI'],log_dir=os.environ['ROS_LOG_DIR'])
        Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,indent=2))
    finally:c.cleanup()
if __name__=='__main__':main()
