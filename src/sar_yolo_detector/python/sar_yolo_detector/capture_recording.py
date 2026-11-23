"""Bounded exact-capture bag writer with acknowledgements after image writes."""
from collections import OrderedDict
import io
import json
import struct
import threading
import time


def capture_header(data):
    if len(data)<16:raise ValueError('Truncated capture header')
    _,secs,nsecs,length=struct.unpack_from('<IIII',data)
    if nsecs>=1000000000 or length>len(data)-16:raise ValueError('Invalid capture header')
    return secs*1000000000+nsecs,data[16:16+length].decode('utf8')


class AcknowledgedBag:
    def __init__(self,bag,source,ack,maximum_images=64,maximum_bytes=64*1024*1024):
        self.bag=bag;self.source=source;self.ack=ack
        self.maximum_images=maximum_images;self.maximum_bytes=maximum_bytes
        self.images=OrderedDict();self.infos=OrderedDict();self.poses=OrderedDict()
        self.written=OrderedDict();self.evidence=OrderedDict()
        self.last_capture=0;self.bytes=0;self.accept_images=True;self.closed=False
        self.lock=threading.RLock();self.count=0

    def add(self,topic,data,cls,received,connection_header=None):
        with self.lock:
            if self.closed:return
            raw=(cls._type,data,cls._md5sum,cls)
            if topic==self.source['image_topic']:
                if not self.accept_images:return
                stamp,frame=capture_header(data)
                if frame!=self.source['frame_id']:raise ValueError('Image optical frame differs')
                if stamp<=self.last_capture or stamp in self.images:raise ValueError('Non-increasing image capture')
                self.images[stamp]=(raw,received,connection_header,time.monotonic())
                self.bytes+=len(data)
                if len(self.images)>self.maximum_images or self.bytes>self.maximum_bytes:
                    raise RuntimeError('Exact-capture recording buffer overflow')
            else:
                self.bag.write(topic,raw,received,raw=True,connection_header=connection_header)
                if topic==self.source['camera_info_topic']:
                    stamp,frame=capture_header(data)
                    if frame!=self.source['frame_id']:raise ValueError('Calibration optical frame differs')
                    self.infos[stamp]=True
                elif topic.endswith('/fusion_evidence'):
                    value=json.loads(cls().deserialize(data).data)
                    header=value.get('capture_header',{});stamp=header.get('stamp',{})
                    if header.get('frame_id')==self.source['frame_id']:
                        self.evidence[stamp.get('secs',0)*1000000000+stamp.get('nsecs',0)]=True
                elif topic=='/tf':
                    message=cls().deserialize(data)
                    for transform in message.transforms:
                        if transform.child_frame_id==self.source['frame_id']:
                            self.poses[transform.header.stamp.to_nsec()]=True
            self._commit()
            self._ack_ready()
            for cache in (self.infos,self.poses,self.written,self.evidence):
                while len(cache)>512:cache.popitem(last=False)

    def _commit(self):
        # Preserve order. Missing metadata is a recording error, never a reason
        # to drop an internal capture silently and continue scoring.
        for stamp in list(self.images):
            if stamp not in self.infos or stamp not in self.poses:break
            raw,received,header,_=self.images.pop(stamp);self.bytes-=len(raw[1])
            self.bag.write(self.source['image_topic'],raw,received,raw=True,connection_header=header)
            self.bag.flush()
            self.last_capture=stamp;self.count+=1
            self.written[stamp]=True

    def _ack_ready(self):
        for stamp in list(self.written):
            if stamp in self.evidence:
                self.bag.flush()
                self.ack(stamp,self.source['frame_id'])
                del self.written[stamp]

    def check(self):
        with self.lock:
            if self.images and time.monotonic()-next(iter(self.images.values()))[3]>2.:
                raise RuntimeError('Recorded image lacks exact-capture metadata for over two seconds')

    def typed(self,topic,message,received):
        data=io.BytesIO();message.serialize(data)
        self.add(topic,data.getvalue(),type(message),received)


def record_acknowledged(path,source,topics,seconds,ready_file,namespace):
    import signal
    import rosbag
    import roslib.message
    import rospy
    from std_msgs.msg import Header,String
    from sensor_msgs.msg import Image
    stopped=threading.Event();errors=[];types={}
    ack_topic=namespace+'/recording/capture_written'
    publisher=rospy.Publisher(ack_topic,Header,queue_size=128,latch=True)
    status_pub=rospy.Publisher(namespace+'/recording/status',String,queue_size=1,latch=True)
    import uuid
    record_id=uuid.uuid4().hex
    def announce(active):
        status_pub.publish(String(data=json.dumps(dict(active=active,record_id=record_id,
            caller_id=rospy.get_name(),session=source.get('session'),frame_id=source['frame_id'],
            protocol='exact_capture_bag_write_v1',wall=time.time()))))
    announce(True)
    heartbeat=rospy.Timer(rospy.Duration(.5),lambda _:announce(True))
    def acknowledge(stamp,frame):
        h=Header(stamp=rospy.Time(stamp//1000000000,stamp%1000000000),frame_id=frame)
        publisher.publish(h)
        if ready_file and not ready_file.exists():
            ready_file.write_text(json.dumps(dict(ready=True,after_capture_nsec=stamp,
                session=source.get('session'),ack_topic=ack_topic,protocol='exact_capture_bag_write_v1'))+'\n')
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    for sig in previous:signal.signal(sig,lambda *_:stopped.set())
    subscriptions=[]
    bag=None;writer=None
    try:
        bag=rosbag.Bag(str(path),'w',compression='lz4')
        writer=AcknowledgedBag(bag,source,acknowledge)
        def callback(message,topic):
            try:
                header=getattr(message,'_connection_header',None)
                if hasattr(message,'_buff'):
                    kind=header['type'];cls=types.get(kind)
                    if cls is None:
                        cls=roslib.message.get_message_class(kind)
                        if cls is None:raise ValueError('Unknown recorded message type '+kind)
                        types[kind]=cls
                    data=bytes(message._buff)
                else:
                    cls=type(message);buffer=io.BytesIO();message.serialize(buffer);data=buffer.getvalue()
                writer.add(topic,data,cls,rospy.Time.now(),header)
            except Exception as error:
                errors.append(str(error));stopped.set()
        # Register metadata and evidence first; image subscription last.
        for topic in [t for t in topics if t!=source['image_topic']]:
            subscriptions.append(rospy.Subscriber(topic,rospy.AnyMsg,callback,callback_args=topic,
                queue_size=256,buff_size=16*1024*1024,tcp_nodelay=True))
        rospy.wait_for_message(source['camera_info_topic'],rospy.AnyMsg,timeout=30)
        rospy.wait_for_message('/tf',rospy.AnyMsg,timeout=30)
        subscriptions.append(rospy.Subscriber(source['image_topic'],rospy.AnyMsg,callback,
            callback_args=source['image_topic'],queue_size=64,buff_size=16*1024*1024,tcp_nodelay=True))
        deadline=time.monotonic()+seconds
        while not stopped.is_set() and not rospy.is_shutdown() and time.monotonic()<deadline:
            writer.check();stopped.wait(.02)
        with writer.lock:writer.accept_images=False
        # Stop accepting new pixels, allow already captured metadata to drain.
        drain=time.monotonic()+1.
        while writer.images and time.monotonic()<drain and not errors:time.sleep(.01)
        for subscriber in subscriptions:subscriber.unregister()
        subscriptions=[]
        with writer.lock:
            if errors:raise RuntimeError(errors[0])
            if writer.images:raise RuntimeError('Incomplete exact-capture recording at shutdown')
            if not writer.count:raise RuntimeError('No complete captures recorded')
            writer.closed=True
    finally:
        heartbeat.shutdown();announce(False)
        if writer is not None:
            with writer.lock:writer.closed=True
        for subscriber in subscriptions:subscriber.unregister()
        if bag is not None:
            if writer is not None:
                with writer.lock:bag.close()
            else:bag.close()
        for sig,handler in previous.items():signal.signal(sig,handler)
