#!/usr/bin/env python3
"""
File:          interface.py
Author:        Ammar Ratnani
Last Modified: Ammar on 2/22
"""

from simple_pid import PID
from math import sin, cos, pi, sqrt
from time import time

from iface_lib.arduino import Arduino
import sim



system_type = None
board = None


def set_system(stype, sim_config=None):
    
    global system_type
    global board
    
    system_type = stype

    if system_type == "sim":
        sim.start(sim_config)

    elif system_type == "raspi":
        pass

    elif system_type == "jetson":
        board = Arduino()

    else:
        raise ValueError(f'Invalid system type: {stype}')


def get_time():

    if system_type == "sim":
        return sim.get_time()

    elif system_type in ["raspi", "jetson"]:
        return time()

    else:
        raise ValueError('System type has not been set')


def is_enabled():

    if system_type == "sim":
        return sim.get_enabled()

    elif system_type == "raspi":
        return None

    elif system_type == "jetson":
        return board.enabled()

    else:
        raise ValueError('System type has not been set')


def read_image():

    if system_type == "sim":
        return sim.read_robot_cam()

    elif system_type in ["raspi", "jetson"]:
        return None

    else:
        raise ValueError('System type has not been set')


def command_wheel_velocities(omega_r, omega_l):

    if system_type == "sim":
        return sim.command_robot_vels(omega_r, omega_l)

    elif system_type == "raspi":
        pass

    elif system_type == "jetson":
        board.setTargetVels(omega_r, omega_l)

    else:
        raise ValueError('System type has not been set')

def read_wheel_velocities():

    # Return format is a tuple of right and left

    if system_type == "sim":
        return sim.read_robot_vels()

    elif system_type == "raspi":
        return None

    elif system_type == "jetson":
        return board.getVels()

    else:
        raise ValueError('System type has not been set')
