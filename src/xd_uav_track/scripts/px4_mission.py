#!/usr/bin/env python3
"""Execute the UE waypoint mission using a local PX4/Gazebo SITL feedback loop."""
import argparse
import json
import math
from pathlib import Path
import time
from xd_uav_track.mission_io import write_json
from xd_uav_track.px4_pose import map_to_enu,enu_to_map


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene',type=Path,required=True)
    p.add_argument('--namespace',default='/uav1');p.add_argument('--out',type=Path,required=True)
    p.add_argument('--status-namespace',default='ue_test')
    p.add_argument('--stage-timeout',type=float,default=100.)
    a=p.parse_args()
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State,ExtendedState
    from mavros_msgs.srv import CommandBool,SetMode,ParamSet
    from mavros_msgs.msg import ParamValue
    from gazebo_msgs.srv import GetModelState
    from std_msgs.msg import String
    rospy.init_node('airground_px4_mission')
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('This supervised run requires wall time')
    if not rospy.get_param('/airground_px4_simulation',False):raise RuntimeError('Launch the owned local SITL stack first')
    rospy.wait_for_service('/gazebo/get_model_state',timeout=120)
    model_state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
    spawn_deadline=time.monotonic()+120
    while not model_state('airground_iris','world').success:
        if rospy.is_shutdown() or time.monotonic()>spawn_deadline:
            raise RuntimeError('Expected local simulation vehicle did not spawn')
        time.sleep(.2)
    cfg=json.loads(a.scene.read_text())['mission'];home=cfg['home_ned_m']
    latest={'state':None,'pose':None,'received':0.,'landed':None}
    def pose(msg):latest.update(pose=msg,received=time.monotonic())
    def state(msg):latest['state']=msg
    def landed(msg):latest['landed']=msg.landed_state
    subs=[rospy.Subscriber(a.namespace+'/mavros/local_position/pose',PoseStamped,pose,queue_size=1),
          rospy.Subscriber(a.namespace+'/mavros/state',State,state,queue_size=1),
          rospy.Subscriber(a.namespace+'/mavros/extended_state',ExtendedState,landed,queue_size=1)]
    publisher=rospy.Publisher(a.namespace+'/mavros/setpoint_position/local',PoseStamped,queue_size=1)
    status_pub=rospy.Publisher('/'+a.status_namespace.strip('/')+'/px4_mission_status',String,queue_size=1,latch=True)
    arm=rospy.ServiceProxy(a.namespace+'/mavros/cmd/arming',CommandBool)
    mode=rospy.ServiceProxy(a.namespace+'/mavros/set_mode',SetMode)
    deadline=time.monotonic()+90
    while not rospy.is_shutdown():
        if latest['pose'] and latest['state'] and latest['state'].connected:break
        if time.monotonic()>deadline:raise RuntimeError('PX4 connection/pose timeout')
        time.sleep(.1)
    initial=latest['pose'].pose.position;origin=[initial.x,initial.y,initial.z]
    # This exception is scoped to OFFBOARD in our owned simulator, which has no RC.
    result=rospy.ServiceProxy(a.namespace+'/mavros/param/set',ParamSet)(
        param_id='COM_RCL_EXCEPT',value=ParamValue(integer=4,real=0.))
    if not result.success:raise RuntimeError('Cannot configure SITL OFFBOARD RC exception')
    stages=[('takeoff',[home[0],home[1],-cfg['cruise_altitude_m']],0.)]
    hover_by_waypoint=cfg.get('hover_seconds_by_waypoint',[cfg['hover_sec']]*len(cfg['waypoints_ned_m']))
    if len(hover_by_waypoint)!=len(cfg['waypoints_ned_m']) or any(not math.isfinite(v) or v<0 for v in hover_by_waypoint):
        raise ValueError('hover_seconds_by_waypoint must have one non-negative duration per waypoint')
    for i,point in enumerate(cfg['waypoints_ned_m']):
        stages.extend([('transit_%02d'%(i+1),point,0.),('inspect_%02d'%(i+1),point,hover_by_waypoint[i])])
    stages.append(('return',[home[0],home[1],stages[-1][1][2]],0.))
    stages.append(('approach_land',[home[0],home[1],-3.0],0.))
    a.out.mkdir(parents=True,exist_ok=False);log=(a.out/'telemetry.jsonl').open('w')
    started=time.monotonic();status={};armed_by_us=False;success=False
    def update(stage,index,target):
        now=time.monotonic();wait_started=now
        while now-latest['received']>.8:
            if rospy.is_shutdown() or now-wait_started>5.:raise RuntimeError('PX4 pose stream stopped beyond recovery deadline')
            time.sleep(.02);now=time.monotonic()
        if now-wait_started>.02:
            status['pose_wait_events']=status.get('pose_wait_events',0)+1
            status['pose_wait_sec']=status.get('pose_wait_sec',0.)+now-wait_started
        msg=latest['pose']
        actual=[msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        status.update(stage=stage,elapsed_sec=now-started,progress=index/(len(stages)+1),complete=False,
            local_origin_enu_m=origin,target_ned_m=target,observer_ned_m=enu_to_map(actual,origin,home),
            mode=latest['state'].mode,armed=latest['state'].armed,
            position_error_m=math.dist(actual,map_to_enu(target,origin,home)),
            pose_source='PX4_MAVROS_actual_feedback')
        status_pub.publish(String(data=json.dumps(status)));write_json(a.out/'status.json',status)
        log.write(json.dumps(status)+'\n');log.flush()
        return actual
    try:
        # Stream setpoints before OFFBOARD; PX4 rejects mode switching without this.
        target=home[:];desired=origin[:];heading=0.
        def stream(_event):
            msg=PoseStamped();msg.header.stamp=rospy.Time.now()
            msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=desired
            msg.pose.orientation.z=math.sin(heading/2);msg.pose.orientation.w=math.cos(heading/2)
            publisher.publish(msg)
        timer=rospy.Timer(rospy.Duration(.05),stream)
        for _ in range(60):
            msg=PoseStamped();msg.header.stamp=rospy.Time.now();msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=desired
            msg.pose.orientation.w=1.;publisher.publish(msg);update('prestream',0,target);time.sleep(.05)
        last_request=0.;stage_index=0;stage_started=time.monotonic();reached_since=None
        while not rospy.is_shutdown() and stage_index<len(stages):
            tick=time.monotonic();stage,target,hold=stages[stage_index]
            actual=update(stage,stage_index,target)
            if tick-last_request>2:
                if latest['state'].mode!='OFFBOARD':mode(custom_mode='OFFBOARD')
                elif not latest['state'].armed:
                    if stage_index>0:raise RuntimeError('PX4 unexpectedly disarmed')
                    armed_by_us=arm(True).success
                last_request=tick
            if tick-stage_started>a.stage_timeout:raise RuntimeError('PX4 stage timeout: '+stage)
            target_enu=map_to_enu(target,origin,home)
            # Limit setpoint velocity independently of slower UE capture calls.
            delta=[b-c for b,c in zip(target_enu,desired)]
            horizontal=math.hypot(delta[0],delta[1]);horizontal_step=cfg['speed_mps']*.05
            if horizontal>0:
                ratio=min(1.,horizontal_step/horizontal);desired[0]+=delta[0]*ratio;desired[1]+=delta[1]*ratio
                heading=math.atan2(delta[1],delta[0])
            desired[2]+=max(-cfg['vertical_speed_mps']*.05,min(cfg['vertical_speed_mps']*.05,delta[2]))
            msg=PoseStamped();msg.header.stamp=rospy.Time.now();msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=desired
            msg.pose.orientation.z=math.sin(heading/2);msg.pose.orientation.w=math.cos(heading/2);publisher.publish(msg)
            if latest['state'].armed and latest['state'].mode=='OFFBOARD' and math.dist(actual,target_enu)<1.0:
                reached_since=reached_since or tick
                if tick-reached_since>=max(.7,hold):
                    stage_index+=1;stage_started=tick;reached_since=None
            else:reached_since=None
            time.sleep(max(0.,.05-(time.monotonic()-tick)))
        if rospy.is_shutdown():raise RuntimeError('Mission interrupted')
        timer.shutdown()
        mode(custom_mode='AUTO.LAND');deadline=time.monotonic()+90;last_request=0.
        while not rospy.is_shutdown():
            update('land',len(stages),home)
            if latest['landed']==ExtendedState.LANDED_STATE_ON_GROUND and not latest['state'].armed:
                success=True;break
            if time.monotonic()>deadline:raise RuntimeError('Landing/disarm timeout')
            if time.monotonic()-last_request>3 and latest['state'].mode!='AUTO.LAND':mode(custom_mode='AUTO.LAND');last_request=time.monotonic()
            time.sleep(.1)
        if not success:raise RuntimeError('Landing interrupted before touchdown')
        status.update(stage='complete',progress=1.,complete=True)
        status_pub.publish(String(data=json.dumps(status)));write_json(a.out/'status.json',status)
        time.sleep(2)
    finally:
        if not success and armed_by_us and latest['state'] and latest['state'].armed:
            try:mode(custom_mode='AUTO.LAND')
            except Exception:pass
        if 'timer' in locals():timer.shutdown()
        log.close();write_json(a.out/'result.json',{'passed':success,'final_status':status,'physics':'PX4 v1.13.2 + Gazebo Classic ODE','ue_collision_meshes_shared':False})
    if not success:raise RuntimeError('Mission did not complete')


if __name__=='__main__':main()
