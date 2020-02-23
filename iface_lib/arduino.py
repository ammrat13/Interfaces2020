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


def find_port(baud, timeout):
    ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")

    for p in ports:
        try:
            sr = serial.Serial(p, baud, timeout=timeout)
        except (serial.serialutil.SerialException, OSError) as e:
            continue

        sr.readline()
        if sr:
            return sr

    return None



class Arduino(object):


    def __init__(self, baud=115200, port=None, timeout=2, sr=None):
        if not sr:
            if not port:
                sr = find_port(baud, timeout)
                if not sr:
                    raise ValueError("Could not find port.")
            else:
                sr = serial.Serial(port, baud, timeout=timeout)
                sr.readline()

        sr.flush()
        self.sr = sr


    def digitalWrite(self, pin, val):
        if val.upper() == "LOW":
            pin_ = -pin
        else:
            pin_ = pin

        cmd_str = build_cmd_str("dw", (pin_,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def analogWrite(self, pin, val):
        if val > 255:
            val = 255
        elif val < 0:
            val = 0

        cmd_str = build_cmd_str("aw", (pin, val))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def analogRead(self, pin):
        cmd_str = build_cmd_str("ar", (pin,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return int(rd)
        except:
            return None


    def pinMode(self, pin, val):
        if val == "INPUT":
            pin_ = -pin
        else:
            pin_ = pin

        cmd_str = build_cmd_str("pm", (pin_,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def pulseIn(self, pin, val):
        if val.upper() == "LOW":
            pin_ = -pin
        else:
            pin_ = pin

        cmd_str = build_cmd_str("pi", (pin_,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return float(rd)
        except:
            return None


    def close(self):
        if self.sr.isOpen():
            self.sr.flush()
            self.sr.close()


    def digitalRead(self, pin):
        cmd_str = build_cmd_str("dr", (pin,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return int(rd)
        except:
            return None


    def stRun(self):
        cmd_str = build_cmd_str("sru")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def stSetMaxSpeed(self, speed):
        cmd_str = build_cmd_str("sms", (speed,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def stSetSpeed(self, speed):
        cmd_str = build_cmd_str("sss", (speed,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def stMove(self, pos):
        cmd_str = build_cmd_str("sss", (pos,))
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass


    def stCurrentPosition(self):
        cmd_str = build_cmd_str("scp")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

        rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
        try:
            return int(rd)
        except:
            return None


    def stStop(self):
        cmd_str = build_cmd_str("sst")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass
