#!/usr/bin/env python
# Mostly copied from `arduino-python3`

import serial
import time


sr = None
recent_cmd = 0


def setup(self, port, baud=115200, tout=.05):
    global sr
    global recent_cmd
    # Make sure to wait for the Arduino to boot
    sr = serial.Serial(port, baud, timeout=tout)
    time.sleep(3)
    # Also store the most recent commanded velocities
    recent_cmd = 0

def close(self):
    global sr
    if sr.isOpen():
        sr.flush()
        sr.close()

def set_stepper_target(self, pos):
    global sr
    global recent_cmd
    # Store this if needed for the other methods
    recent_cmd = pos
    # Can generate an exception
    # If we fail for any reason, return None
    try:
        # Write to the Serial
        sr.write(str.encode(f"{cmd}\n"))
        sr.flush()
        # Parse the result
        ret = list(sr.readline().decode("utf-8")[1:-2].split(","))
        ret[0] = int(ret[0])
        ret[1] = bool(int(ret[1]))
        return tuple(ret)
    except:
        return None


def get_stepper_dtg(self):
    global sr
    global recent_cmd
    # Just recommand and return the right one
    set_stepper_target(recent_cmd)[0]


def get_enabled(self):
    global sr
    global recent_cmd
    # Just recommand and return the right one
    set_stepper_target(recent_cmd)[1]
