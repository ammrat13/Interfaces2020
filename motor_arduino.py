#!/usr/bin/env python
# Mostly copied from `arduino-python3`

import serial
import time


sr = None
recent_rcmd = 0
recent_lcmd = 0
recent_aggr = 0


def setup(self, port, baud=115200, tout=.05):
    global sr
    global recent_rcmd
    global recent_lcmd
    global recent_aggr
    # Make sure to wait for the Arduino to boot
    sr = serial.Serial(port, baud, timeout=tout)
    time.sleep(3)
    # Also store the most recent commanded velocities
    recent_rcmd = 0
    recent_lcmd = 0
    recent_aggr = 0

def close(self):
    global sr
    if sr.isOpen():
        sr.flush()
        sr.close()

def set_target_vels(self, w_r, w_l, aggr):
    global sr
    global recent_rcmd
    global recent_lcmd
    global recent_aggr
    # Store these if needed for get_vels
    recent_rcmd = w_r
    recent_lcmd = w_l
    recent_aggr = aggr
    # Can generate an exception
    # If we fail for any reason, return None
    try:
        # Write to the Serial
        sr.write(str.encode(f"<{w_l},{w_r},{int(aggr)}>\n"))
        sr.flush()
        # Parse the result
        return tuple(map(int, sr.readline().decode("utf-8")[1:-2].split(",")))
    except:
        return None


def get_vels(self):
    global sr
    global recent_rcmd
    global recent_lcmd
    global recent_aggr
    # Just set the target velocities again
    # Board provides no way of just reading
    set_target_vels(recent_rcmd, recent_lcmd, recent_aggr)
