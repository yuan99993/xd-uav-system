"""Four bounded POSIX mmap slots. Reader leases prevent producer overwrite."""
import ctypes
import fcntl
import json
import mmap
import os
from pathlib import Path
import re
import struct
import uuid
import numpy as np

MAGIC=b'EOIRSHM1'
HEADER=struct.Struct('<8sQQIII')
OFFSET=64
NAME=re.compile(r'^eo_ir_[0-9a-f]{32}_[0-3]$')

class StaleSharedImage(ValueError):pass

class _Lease:
    def __init__(self,fd):self.fd=fd
    def __del__(self):
        try:os.close(self.fd)
        except (OSError,TypeError):pass

class SharedImagePublisher:
    def __init__(self,height,width,directory='/dev/shm'):
        self.height=int(height);self.width=int(width);self.size=self.height*self.width*3
        if min(self.height,self.width)<=0 or self.size>33554432:raise ValueError('Invalid shared image size')
        self.directory=Path(directory);self.slots=[];self.sequence=0;self.next_slot=0;self.dropped=0
        token=uuid.uuid4().hex
        try:
            for slot in range(4):
                name='eo_ir_'+token+'_'+str(slot);path=self.directory/name
                fd=os.open(str(path),os.O_CREAT|os.O_EXCL|os.O_RDWR|os.O_CLOEXEC,0o600)
                try:
                    os.ftruncate(fd,OFFSET+self.size)
                    mapping=mmap.mmap(fd,OFFSET+self.size)
                except Exception:
                    os.close(fd);path.unlink()
                    raise
                self.slots.append((name,fd,mapping))
        except Exception:self.close();raise
    def publish(self,image,capture_ns,frame_id,header_seq=0):
        if image.shape!=(self.height,self.width,3) or image.dtype!=np.uint8 or not image.flags.c_contiguous:
            raise ValueError('Shared image must be contiguous BGR8 with fixed dimensions')
        for delta in range(4):
            slot=(self.next_slot+delta)%4;name,fd,mapping=self.slots[slot]
            try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:continue
            try:
                self.sequence+=1
                mapping[OFFSET:]=memoryview(image).cast('B')
                mapping[:HEADER.size]=HEADER.pack(MAGIC,self.sequence,int(capture_ns),self.height,self.width,self.width*3)
                self.next_slot=(slot+1)%4
                return dict(protocol=1,name=name,sequence=self.sequence,capture_ns=int(capture_ns),
                    height=self.height,width=self.width,step=self.width*3,encoding='bgr8',
                    frame_id=str(frame_id),header_seq=int(header_seq))
            finally:fcntl.flock(fd,fcntl.LOCK_UN)
        self.dropped+=1
        return None
    def slot_status(self):
        status=[]
        for index,(_,fd,_) in enumerate(self.slots):
            try:
                fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:status.append(dict(slot=index,busy=True))
            else:
                fcntl.flock(fd,fcntl.LOCK_UN);status.append(dict(slot=index,busy=False))
        return status

    def close(self):
        for name,fd,mapping in self.slots:
            mapping.close();os.close(fd)
            try:(self.directory/name).unlink()
            except FileNotFoundError:pass
        self.slots=[]


def validate_descriptor(descriptor,maximum_bytes=33554432):
    if descriptor.get('protocol')!=1 or not NAME.fullmatch(str(descriptor.get('name',''))):
        raise ValueError('Invalid shared image protocol/name')
    for key in ('sequence','capture_ns','height','width','step','header_seq'):
        if type(descriptor.get(key)) is not int or descriptor[key]<0:raise ValueError('Invalid shared image '+key)
    h,w,step=descriptor['height'],descriptor['width'],descriptor['step']
    if min(h,w)<=0 or step!=w*3 or h*step>maximum_bytes or descriptor.get('encoding')!='bgr8':
        raise ValueError('Invalid shared image dimensions')
    if not isinstance(descriptor.get('frame_id'),str) or not 0<len(descriptor['frame_id'])<=256:
        raise ValueError('Invalid optical frame')
    return descriptor


def read_shared_image(descriptor,directory='/dev/shm',maximum_bytes=33554432):
    d=validate_descriptor(descriptor,maximum_bytes)
    fd=os.open(str(Path(directory)/d['name']),os.O_RDONLY|os.O_CLOEXEC|os.O_NOFOLLOW)
    mapping=None
    try:
        fcntl.flock(fd,fcntl.LOCK_SH|fcntl.LOCK_NB)
        size=OFFSET+d['height']*d['step']
        if os.fstat(fd).st_size!=size:raise ValueError('Shared image allocation differs')
        # ACCESS_COPY shares physical pages for reads and protects the publisher
        # if a native image consumer accidentally writes despite read-only flags.
        mapping=mmap.mmap(fd,size,access=mmap.ACCESS_COPY)
        header=HEADER.unpack(mapping[:HEADER.size])
        if header!=(MAGIC,d['sequence'],d['capture_ns'],d['height'],d['width'],d['step']):
            raise StaleSharedImage('Slot was superseded before consumption')
        region=(ctypes.c_ubyte*(d['height']*d['step'])).from_buffer(mapping,OFFSET)
        region._lease=_Lease(fd)
        # ctypes owns the lease, so numpy.asarray/slices retain it through base
        # buffers even after dropping the original array object.
        image=np.ndarray((d['height'],d['width'],3),np.uint8,buffer=region)
        image.setflags(write=False)
        return image
    except Exception:
        if mapping is not None:mapping.close()
        os.close(fd)
        raise


def metadata_image(message):
    from types import SimpleNamespace
    from std_msgs.msg import Header
    import rospy
    if len(message.data)>8192:raise ValueError('Oversized shared image descriptor')
    d=validate_descriptor(json.loads(message.data))
    ns=d['capture_ns']
    h=Header(seq=d['header_seq']%(2**32),stamp=rospy.Time(ns//1000000000,ns%1000000000),frame_id=d['frame_id'])
    return SimpleNamespace(header=h,height=d['height'],width=d['width'],step=d['step'],encoding='bgr8',
        is_bigendian=0,data=message.data.encode('utf8'),_shared_descriptor=d,_latency_timestamps=d.get('latency_timestamps',{}))
