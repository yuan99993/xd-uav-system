"""MAVROS ENU/FLU feedback to the UE map's NED coordinates."""
import math
import threading
import time


def quaternion_angles(q):
    x,y,z,w=q
    norm=math.sqrt(sum(v*v for v in q))
    if norm<1e-8 or not all(math.isfinite(v) for v in q):raise ValueError('Invalid attitude')
    x,y,z,w=(v/norm for v in q)
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def ue_rotation_deg(q):
    roll,pitch,yaw=quaternion_angles(q)
    # UE FRotationTranslationMatrix: +pitch is nose-up, +roll is right-wing-down.
    return {'pitch':-math.degrees(pitch),'roll':math.degrees(roll),'yaw':90.-math.degrees(yaw)}


def enu_to_map(position, origin, home):
    return [home[0]+position[1]-origin[1],home[1]+position[0]-origin[0],home[2]-position[2]+origin[2]]


def map_to_enu(position, origin, home):
    return [origin[0]+position[1]-home[1],origin[1]+position[0]-home[0],origin[2]-position[2]+home[2]]


class PoseFeedback:
    def __init__(self, namespace, home, rospy, status_namespace="ue_test"):
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        self.home=home;self.rospy=rospy;self.lock=threading.Lock();self.latest=None;self.status={}
        self.sub=rospy.Subscriber(namespace+'/mavros/local_position/pose',PoseStamped,self.callback,queue_size=1)
        self.mission_sub=rospy.Subscriber('/'+status_namespace.strip('/')+'/px4_mission_status',String,self.mission_callback,queue_size=1)

    def mission_callback(self,msg):
        import json
        try:status=json.loads(msg.data)
        except ValueError:return
        with self.lock:self.status=status

    def callback(self,msg):
        p=msg.pose.position;q=msg.pose.orientation
        with self.lock:self.latest=(time.monotonic(),msg.header.stamp.to_sec(),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w])

    def wait_sample(self,timeout=5.):
        deadline=time.monotonic()+timeout
        while True:
            if self.rospy.is_shutdown():raise RuntimeError('PX4 capture interrupted')
            try:return self.sample()
            except RuntimeError as error:
                if time.monotonic()>=deadline:raise RuntimeError('No fresh PX4 pose within recovery deadline') from error
                time.sleep(.02)  # Pause rendering; never reuse stale feedback.

    def sample(self):
        with self.lock:latest=self.latest;status=dict(self.status)
        if latest is None:raise RuntimeError('PX4 pose feedback unavailable')
        received,stamp,p,q=latest
        age=time.monotonic()-received
        source_age=self.rospy.Time.now().to_sec()-stamp
        if age>1. or source_age>1. or source_age<-.2:raise RuntimeError('PX4 pose feedback stale; rendering stopped')
        origin=status.get('local_origin_enu_m')
        if origin is None:raise RuntimeError('PX4 mission has not supplied its local map origin')
        position=enu_to_map(p,origin,self.home)
        rotation=ue_rotation_deg(q)
        heading=math.radians(rotation['yaw'])
        status.update(observer_ned_m=position,yaw_deg=math.degrees(heading),pose_age_sec=age,
                      pose_source_stamp=stamp,pose_source='PX4_MAVROS_actual_feedback',
                      rotation_deg=rotation)
        return position,heading,status
