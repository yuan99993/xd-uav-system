"""Per-peer TCPROS buffering for bounded latest-image publishers (ROS Noetic)."""

def bound_ros_send_buffers(publisher,maximum_bytes,configured):
    """Bound in-flight TCPROS backlog in addition to rospy's one-item queue.

    Noetic's QueuedConnection forwards its underlying socket. Configure each
    new peer once; full messages still use blocking sendall and standard ROS
    framing. A slow peer drops pending messages at the publisher's queue.
    """
    if not maximum_bytes:return
    import socket
    for connection in list(publisher.impl.connections):
        wire=getattr(connection,'socket',None)
        if wire is None or wire in configured:continue
        try:
            wire.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,maximum_bytes)
            wire.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        except OSError:
            # A subscriber can disconnect concurrently with this snapshot.
            continue
        configured.add(wire)
    configured.intersection_update(getattr(c,'socket',None) for c in publisher.impl.connections)

