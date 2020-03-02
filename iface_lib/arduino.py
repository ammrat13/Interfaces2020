#!/usr/bin/env python
# Mostly copied from `arduino-python3`

import serial
import glob



def build_cmd_str(cmd, args=None):
    if args:
        args = '%'.join(map(str, args))
    else:
        args = ''
    return "@{cmd}%{args}$!".format(cmd=cmd, args=args)


class Arduino(object):


    def __init__(self, port, baud=115200, sr=None):
        if not sr:
            # I should probably explain this bit somewhat
            # For some reason, on bootup, we must wait a certain amount of 
            #   time before the rest of the code will work as expected
            # We must then call readline()
            # Don't ask me why
            sr = serial.Serial(port, baud, timeout=2)
            sr.readline()
            sr.timeout = None

        sr.flush()
        self.sr = sr


    def close(self):
        if self.sr.isOpen():
            self.sr.flush()
            self.sr.close()


    def enabled(self):
        cmd_str = build_cmd_str("en")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return rd
        except:
            return None


    def setTargetVels(self, w_r, w_l, pwm_cr=255, pwm_cl=255):
        cmd_str = build_cmd_str("tv", (w_r, w_l, pwm_cr, pwm_cl))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def getVels(self):
        cmd_str = build_cmd_str("gv")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return tuple(map(int, rd.split(',')))
        except:
            return None


    def stMove(self, relPos):
        cmd_str = build_cmd_str("smv", (relPos,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def stDistanceToGo(self):
        cmd_str = build_cmd_str("sdg")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return int(rd)
        except:
            pass
