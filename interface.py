#!/usr/bin/env python3
"""
File:          interface.py
Author:        Ammar Ratnani
Last Modified: Ammar on 2/22
"""

from Arduino import Arduino
from simple_pid import PID
from math import sin, cos, pi, sqrt
from time import time

import sim



PIN_EN = 13

PIN_IN1 = 9
PIN_IN2 = 8
PIN_IN3 = 3
PIN_IN4 = 2

PIN_ENC1 = 10
PIN_ENC3 = 5


KP = .035
KI = .0003
KD = .0005



system_type = None
board = None


pid_r = None
pid_l = None

pwm_r = None
pwm_l = None



def set_system(stype, sim_config=None):
    
    global system_type
    global board

    global pid_r
    global pid_l
    global pwm_r
    global pwm_l

    system_type = stype

    if system_type == "sim":
        sim.start(sim_config)

    elif system_type in ["raspi", "jetson"]:
        board = Arduino()
        
        pid_r = PID(KP, KI, KD)
        pid_l = PID(KP, KI, KD)

        pwm_r = 0
        pwm_l = 0

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

    elif system_type in ["raspi", "jetson"]:
        pass

    else:
        raise ValueError('System type has not been set')


def read_image():

    if system_type == "sim":
        return sim.read_robot_cam()

    elif system_type in ["raspi", "jetson"]:
        pass

    else:
        raise ValueError('System type has not been set')


def command_wheel_velocities(wheel_vels):

    if system_type == "sim":
        return sim.command_robot_vels(*wheel_vels)


    elif system_type == "raspi":
        pass


    elif system_type == "jetson":
        pass


    else:
        raise ValueError('System type has not been set')

def read_wheel_velocities():

    if system_type == "sim":
        return sim.read_robot_vels()


    elif system_type == "raspi":
        pass


    elif system_type == "jetson":
        pass


    else:
        raise ValueError('System type has not been set')
