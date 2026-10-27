#!/usr/bin/env python3
"""Fake rosserial MCU on a pty, to test RosserialSystem without hardware.

Usage: fake_board.py  -> prints the pty path; point the URDF 'port' at it.
Publishes led_state (Bool) at 50 Hz; the LED follows the led (Bool) command.
"""
import os, pty, select, struct, time, tty

def cdr_str(s):
    b = s.encode() + b"\0"
    return struct.pack("<I", len(b)) + b

def pad(b, a=4):
    return b + b"\0" * (-len(b) % a)

def topic_info(tid, name, typ, md5):
    body = struct.pack("<H", tid)
    body = pad(body) + cdr_str(name)
    body = pad(body) + cdr_str(typ)
    body = pad(body) + cdr_str(md5)
    body = pad(body) + struct.pack("<i", 512)
    return b"\x00\x01\x00\x00" + body  # CDR header + payload (offsets relative to payload start)

def float_array(vals):
    # layout.dim = [], data_offset = 0, data = vals
    body = struct.pack("<I", 0) + struct.pack("<I", 0) + struct.pack("<I", len(vals)) + struct.pack(f"<{len(vals)}f", *vals)
    return b"\x00\x01\x00\x00" + body

def frame(tid, body):
    n = len(body)
    lc = 255 - ((n >> 8) + n) % 256
    chk = 255 - ((tid >> 8) + tid + sum(body)) % 256
    return bytes([0xff, 0xfe, n & 255, n >> 8, lc, tid & 255, tid >> 8]) + body + bytes([chk])

m, s = pty.openpty()
tty.setraw(m)
print(os.ttyname(s), flush=True)
buf = b""; pos = 0.0; vel = 0.0; cmd = 0.0
pub_id, sub_id = 101, 102
prev = None
last = time.time(); last_time_req = 0
os.write(m, b"")
while True:
    r, _, _ = select.select([m], [], [], 0.02)
    if r:
        buf += os.read(m, 1024)
        while True:
            i = buf.find(b"\xff\xfe")
            if i < 0 or len(buf) - i < 8: break
            buf = buf[i:]
            n = buf[2] | buf[3] << 8
            if len(buf) < n + 8: break
            tid = buf[5] | buf[6] << 8; body = buf[7:7+n]; buf = buf[n+8:]
            if tid == 0:  # request topics
                os.write(m, frame(0, topic_info(pub_id, "led_state", "std_msgs/msg/Bool", "x")))
                os.write(m, frame(1, topic_info(sub_id, "led", "std_msgs/msg/Bool", "x")))
            elif tid == sub_id:
                cmd = body[4]
    now = time.time()
    if now - last_time_req > 1.0:
        os.write(m, frame(10, b"")); last_time_req = now
    dt = now - last; last = now
    os.write(m, frame(pub_id, b"\x00\x01\x00\x00" + bytes([1 if cmd else 0])))
    if cmd != prev:
        prev = cmd
        print('LED', 'ON' if cmd else 'OFF', flush=True)

