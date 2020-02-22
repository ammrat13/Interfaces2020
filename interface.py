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

    elif system_type == "raspi":
        pid_r = PID(KP, KI, KD)
        pid_r.sample_time = .01

        pid_l = PID(KP, KI, KD)
        pid_l.sample_time = .01

        pwm_r = 0
        pwm_l = 0

    elif system_type == "jetson":
        # Arduino setup
        # Set up which pins are I/O and their defaults
        board = Arduino()
        board.pinMode(PIN_EN, "OUTPUT")
        board.pinMode(PIN_IN1, "OUTPUT")
        board.pinMode(PIN_IN2, "OUTPUT")
        board.digitalWrite(PIN_EN, "HIGH")
        board.digitalWrite(PIN_IN2, "LOW")
        board.digitalWrite(PIN_IN4, "LOW")

        # Setup the PID controllers and PWM
        pid_r = PID(KP, KI, KD)
        pid_r.sample_time = .01
        pid_l = PID(KP, KI, KD)
        pid_l.sample_time = .01

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
        return None

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
        # Set the setpoints of the PID controllers
        pidRight.setpoint, pidLeft.setpoint = omegaRight, omegaLeft

        # Get the output of the PID loops
        # Make sure to scale the output from rad/s to PWM -- thus the 
        #   arbitrary constant of 300
        pwm_r += 300 * pid_r(omega_r)
        pwm_l += 300 * pid_l(omega_l)

        # Normalize them to the range of ints between 0 and 255
        pwm_r = int(max(0, min(255, pwm_r)))
        pwm_l = int(max(0, min(255, pwm_l)))

        # Actually do the write
        board.analogWrite(PIN_IN1, pwm_r)
        board.analogWrite(PIN_IN3, pwm_l)


    else:
        raise ValueError('System type has not been set')

def read_wheel_velocities():

    # Return format is a tuple of right and left

    if system_type == "sim":
        return sim.read_robot_vels()


    elif system_type == "raspi":
        return None


    elif system_type == "jetson":
        # Get the pulse widths from the board
        # Set the timeout in the arduino code
        pulseRight = board.pulseIn(PIN_ENC1, "HIGH")
        pulseLeft = board.pulseIn(PIN_ENC3, "HIGH")

        # Do the calculation for the right wheel
        # From the old arduino code -- look there for magic number explanation
        omegaRight = float('inf')
        if pulseRight != 0:
            omegaRight = 2*pi / (pulseRight * 2.24886e-3)

        # Same for the left wheel
        omegaLeft = float('inf')
        if pulseLeft != 0:
            omegaLeft = 2*pi / (pulseLeft * 2.24886e-3)

        return (omegaRight, omegaLeft)

    else:
        raise ValueError('System type has not been set')
