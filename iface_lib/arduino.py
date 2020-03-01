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


def find_port(baud):
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")

    for p in ports:
        try:
            sr = serial.Serial(p, baud)
        except (serial.serialutil.SerialException, OSError) as e:
            continue

        sr.readline()
        if sr:
            return sr

    return None



class Arduino(object):


    def __init__(self, baud=115200, port=None, sr=None):
        if not sr:
            if not port:
                sr = find_port(baud)
                if not sr:
                    raise ValueError("Could not find port.")
            else:
                sr = serial.Serial(port, baud, timeout=timeout)
                sr.readline()

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


    def stRunToNewPosition(self, pos):
        cmd_str = build_cmd_str("srt", (pos,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")


    def stStop(self):
        cmd_str = build_cmd_str("sst")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass
