"""PC-only moving colour-bar sender, using the same wire format as the board."""
import argparse
import json
import math
import socket
import time
from video_protocol import packetize

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7000)
    parser.add_argument('--video-port', type=int, default=7001)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=5)
    parser.add_argument('--seconds', type=float, default=0)
    args=parser.parse_args()
    if args.fps<=0: parser.error('--fps must be positive')
    sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start=time.monotonic(); seq=0; session=time.time_ns() & 0xffffffff
    colours=((255,0,0),(0,255,0),(0,0,255),(255,255,255))
    try:
        while not args.seconds or time.monotonic()-start<args.seconds:
            frame_start=time.monotonic()
            def pack(colour):
                r,g,b=colour
                return (((r&248)<<8)|((g&252)<<3)|(b>>3)).to_bytes(2,'big')
            row=b''.join(pack(colours[((x+seq*4)*4//args.width)%4]) for x in range(args.width))
            pixels=row*args.height
            for packet in packetize(pixels,args.width,args.height,seq,session):
                sock.sendto(packet,(args.host,args.video_port))
                time.sleep(0.0001)
            value={'pressure':50,'angle':int(45*math.sin(seq/10)),
                   'cnn_valid':True,'cnn_age_ms':0,'pressure_source':'test'}
            sock.sendto(json.dumps(value).encode(),(args.host,args.port))
            seq+=1
            time.sleep(max(0,1/args.fps-(time.monotonic()-frame_start)))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

if __name__=='__main__': main()
