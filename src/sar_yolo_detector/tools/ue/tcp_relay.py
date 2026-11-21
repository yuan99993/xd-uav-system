"""Windows localhost TCP to binary stdio relay for WSL installations without loopback access."""
import argparse
import os
import socket
import sys
import threading


def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,required=True);a=p.parse_args()
    if os.name=='nt':
        import msvcrt
        msvcrt.setmode(sys.stdin.fileno(),os.O_BINARY);msvcrt.setmode(sys.stdout.fileno(),os.O_BINARY)
    channel=socket.create_connection(('127.0.0.1',a.port),timeout=3.);channel.settimeout(None)
    channel.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    def upstream():
        try:
            while True:
                value=sys.stdin.buffer.read1(65536)
                if not value:break
                channel.sendall(value)
        finally:
            try:channel.shutdown(socket.SHUT_WR)
            except OSError:pass
    worker=threading.Thread(target=upstream,daemon=True);worker.start()
    try:
        while True:
            value=channel.recv(65536)
            if not value:break
            sys.stdout.buffer.write(value);sys.stdout.buffer.flush()
    finally:channel.close()

if __name__=='__main__':main()
