"""Receive board JSON without Qt. Close the HUD first: both use UDP 7000."""
import argparse
import json
import socket
import time

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',type=int,default=7000)
    p.add_argument('--seconds',type=float,default=10)
    args=p.parse_args();count=0;end=time.monotonic()+args.seconds
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.bind(('0.0.0.0',args.port));s.settimeout(.5)
        print(f'Listening UDP {args.port}; expected board 192.168.10.2',flush=True)
        while time.monotonic()<end:
            try:payload,peer=s.recvfrom(4096)
            except socket.timeout:continue
            try:data=json.loads(payload)
            except (ValueError,UnicodeError):continue
            if not isinstance(data,dict) or not {'pressure','angle'}<=data.keys():continue
            count+=1
            if count<=5:print(peer[0],data,flush=True)
    print(f'{"PASS" if count else "FAIL"}: {count} JSON packets. Video is a separate test.')
    return 0 if count else 1
if __name__=='__main__':raise SystemExit(main())
