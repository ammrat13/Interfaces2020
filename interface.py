#!/usr/bin/env python3
"""
File:          interface.py
Author:        Binit Shah
Last Modified: Binit on 2/20
"""

import time
import sim

system_type = None

def set_system(stype, sim_config=None):
    global system_type
    system_type = stype
    if system_type == "sim":
        sim.start(sim_config)

def get_time():
    if system_type == "sim":
        return sim.get_time()
    elif system_type == "raspi":
        return time.time()
    elif system_type == "jetson":
        return time.time()
    else:
        raise ValueError('system type has not been set')

def is_enabled():
    if system_type == "sim":
        return sim.get_enabled()
    elif system_type == "raspi":
        pass
    elif system_type == "jetson":
        pass
    else:
        raise ValueError('system type has not been set')

def read_image():
    if system_type == "sim":
        return sim.read_robot_cam()
    elif system_type == "raspi":
        pass
    elif system_type == "jetson":
        pass
    else:
        raise ValueError('system type has not been set')

def command_wheel_velocities(wheel_vels):
    if system_type == "sim":
        return sim.command_robot_vels(*wheel_vels)
    elif system_type == "raspi":
        pass
    elif system_type == "jetson":
        pass
    else:
        raise ValueError('system type has not been set')

def read_wheel_velocities():
    if system_type == "sim":
        return sim.read_robot_vels()
    elif system_type == "raspi":
        pass
    elif system_type == "jetson":
        pass
    else:
        raise ValueError('system type has not been set')
