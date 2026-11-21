"""Bounded SRF1 framing and newest-value mailbox for camera/control streams."""
import json
import math
import socket
import struct
import threading
import time

HEADER=struct.Struct('!4sII')
MAX_HEADER=65536
MAX_PAYLOAD=1920*1080*4


def encode_packet(metadata,payload=b''):
    data=json.dumps(metadata,separators=(',',':'),allow_nan=False).encode('utf-8')
    if not 0<len(data)<=MAX_HEADER or len(payload)>MAX_PAYLOAD:
        raise ValueError('Stream packet exceeds bounds')
    return HEADER.pack(b'SRF1',len(data),len(payload))+data+payload


def receive_packet(read):
    def exact(length):
        parts=[];remaining=length
        while remaining:
            value=read(remaining)
            if not value:raise EOFError('Camera stream disconnected mid-packet')
            if len(value)>remaining:raise ValueError('Invalid stream read size')
            parts.append(value);remaining-=len(value)
        return b''.join(parts)
    magic,head,length=HEADER.unpack(exact(HEADER.size))
    if magic!=b'SRF1' or not 0<head<=MAX_HEADER or length>MAX_PAYLOAD:
        raise ValueError('Invalid camera stream header')
    metadata=json.loads(exact(head))
    if not isinstance(metadata,dict):raise ValueError('Invalid camera metadata')
    payload=exact(length)
    return metadata,payload


class LatestValue:
    def __init__(self):
        self.condition=threading.Condition();self.value=None;self.sequence=-1
        self.replaced=0;self.out_of_order=0;self.closed=False

    def put(self,sequence,value):
        with self.condition:
            if self.closed:return False
            if sequence<=self.sequence:self.out_of_order+=1;return False
            if self.value is not None:self.replaced+=1
            self.sequence=sequence;self.value=value;self.condition.notify();return True

    def get(self,timeout=.1):
        with self.condition:
            self.condition.wait_for(lambda:self.value is not None or self.closed,timeout)
            value,self.value=self.value,None
            return value

    def close(self):
        with self.condition:self.closed=True;self.value=None;self.condition.notify_all()


class ClockAlignment:
    """NTP-style UE wall minus ROS-host wall estimate, with RTT uncertainty."""
    def __init__(self):self.samples=[]

    def add(self,sent,received_ue,sent_ue,received):
        if not all(math.isfinite(v) for v in (sent,received_ue,sent_ue,received)):
            raise ValueError('Nonfinite stream clock sample')
        network=(received-sent)-(sent_ue-received_ue)
        if network<-.002 or received<sent or sent_ue<received_ue:
            raise ValueError('Invalid stream clock ordering')
        offset=((received_ue-sent)+(sent_ue-received))/2.
        self.samples.append((max(0.,network)/2.,offset));self.samples=self.samples[-32:]

    def result(self):
        if not self.samples:raise ValueError('Stream clock not aligned')
        uncertainty,offset=min(self.samples)
        return dict(ue_minus_host_sec=offset,uncertainty_sec=uncertainty,samples=len(self.samples))


class SocketWire:
    def __init__(self,host,port):
        self.socket=socket.create_connection((host,port),timeout=3.)
        self.socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        self.socket.settimeout(5.)
        self.lock=threading.Lock()

    def send(self,metadata):
        with self.lock:self.socket.sendall(encode_packet(metadata))

    def receive(self):return receive_packet(self.socket.recv)

    def close(self):
        try:self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:pass
        self.socket.close()


class StreamWatchdog:
    """Feedback/pings cannot renew the camera-frame liveness deadline."""
    def __init__(self,now,timeout=5.):
        self.last_frame=self.last_packet=now;self.timeout=timeout
    def received(self,kind,now):
        self.last_packet=now
        if kind=='frame':self.last_frame=now
    def reason(self,now):
        if now-self.last_packet>self.timeout:return 'UE stream/feedback stalled for five seconds'
        if now-self.last_frame>self.timeout:return 'UE camera frames stalled for five seconds'
        return None
