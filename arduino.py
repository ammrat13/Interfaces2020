#!/usr/bin/env python
import logging
import itertools
import platform
import serial
import time
from serial.tools import list_ports

import sys
if sys.platform.startswith('win'):
    import winreg
else:
    import glob

libraryVersion = 'V0.6'

log = logging.getLogger(__name__)


def enumerate_serial_ports():
    """
    Uses the Win32 registry to return a iterator of serial
        (COM) ports existing on this computer.
    """
    path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
    except OSError:
        raise Exception

    for i in itertools.count():
        try:
            val = winreg.EnumValue(key, i)
            yield (str(val[1]))  # , str(val[0]))
        except EnvironmentError:
            break


def build_cmd_str(cmd, args=None):
    """
    Build a command string that can be sent to the arduino.

    Input:
        cmd (str): the command to send to the arduino, must not
            contain a % character
        args (iterable): the arguments to send to the command

    @TODO: a strategy is needed to escape % characters in the args
    """
    if args:
        args = '%'.join(map(str, args))
    else:
        args = ''
    return "@{cmd}%{args}$!".format(cmd=cmd, args=args)


def find_port(baud, timeout):
    """
    Find the first port that is connected to an arduino with a compatible
    sketch installed.
    """
    if platform.system() == 'Windows':
        ports = enumerate_serial_ports()
    elif platform.system() == 'Darwin':
        ports = [i[0] for i in list_ports.comports()]
        ports = ports[::-1]
    else:
        ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
    for p in ports:
        log.debug('Found {0}, testing...'.format(p))
        try:
            sr = serial.Serial(p, baud, timeout=timeout)
        except (serial.serialutil.SerialException, OSError) as e:
            log.debug(str(e))
            continue

        sr.readline() # wait for board to start up again

        log.info('Using port {0}.'.format(p))
        if sr:
            return sr
    return None

class Arduino(object):


    def __init__(self, baud=115200, port=None, timeout=2, sr=None):
        """
        Initializes serial communication with Arduino if no connection is
        given. Attempts to self-select COM port, if not specified.
        """
        if not sr:
            if not port:
                sr = find_port(baud, timeout)
                if not sr:
                    raise ValueError("Could not find port.")
            else:
                sr = serial.Serial(port, baud, timeout=timeout)
                sr.readline() # wait til board has rebooted and is connected

        sr.flush()
        self.sr = sr

    def digitalWrite(self, pin, val):
        """
        Sends digitalWrite command
        to digital pin on Arduino
        -------------
        inputs:
           pin : digital pin number
           val : either "HIGH" or "LOW"
        """
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
        """
        Sends analogWrite pwm command
        to pin on Arduino
        -------------
        inputs:
           pin : pin number
           val : integer 0 (off) to 255 (always on)
        """
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
        """
        Returns the value of a specified
        analog pin.
        inputs:
           pin : analog pin number for measurement
        returns:
           value: integer from 1 to 1023
        """
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
            return 0

    def pinMode(self, pin, val):
        """
        Sets I/O mode of pin
        inputs:
           pin: pin number to toggle
           val: "INPUT" or "OUTPUT"
        """
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
        """
        Reads a pulse from a pin

        inputs:
           pin: pin number for pulse measurement
        returns:
           duration : pulse length measurement
        """
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
            return -1

    def pulseIn_set(self, pin, val, numTrials=5):
        """
        Sets a digital pin value, then reads the response
        as a pulse width.
        Useful for some ultrasonic rangefinders, etc.

        inputs:
           pin: pin number for pulse measurement
           val: "HIGH" or "LOW". Pulse is measured
                when this state is detected
           numTrials: number of trials (for an average)
        returns:
           duration : an average of pulse length measurements

        This method will automatically toggle
        I/O modes on the pin and precondition the
        measurment with a clean LOW/HIGH pulse.
        Arduino.pulseIn_set(pin,"HIGH") is
        equivalent to the Arduino sketch code:

        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
        delayMicroseconds(2);
        digitalWrite(pin, HIGH);
        delayMicroseconds(5);
        digitalWrite(pin, LOW);
        pinMode(pin, INPUT);
        long duration = pulseIn(pin, HIGH);
        """
        if val.upper() == "LOW":
            pin_ = -pin
        else:
            pin_ = pin
        cmd_str = build_cmd_str("ps", (pin_,))
        durations = []
        for s in range(numTrials):
            try:
                self.sr.write(str.encode(cmd_str))
                self.sr.flush()
            except:
                pass
            rd = self.sr.readline().decode("utf-8").replace("\r\n", "")
            if rd.isdigit():
                if (int(rd) > 1):
                    durations.append(int(rd))
        if len(durations) > 0:
            duration = int(sum(durations)) / int(len(durations))
        else:
            duration = None

        try:
            return float(duration)
        except:
            return -1

    def close(self):
        if self.sr.isOpen():
            self.sr.flush()
            self.sr.close()

    def digitalRead(self, pin):
        """
        Returns the value of a specified
        digital pin.
        inputs:
           pin : digital pin number for measurement
        returns:
           value: 0 for "LOW", 1 for "HIGH"
        """
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
            return 0

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
            return -1

    def stStop(self):
        cmd_str = build_cmd_str("sst")
        try:
            self.sr.write(str.encode(cmd_str))
            self.sr.flush()
        except:
            pass

class Shrimp(Arduino):

    def __init__(self):
        Arduino.__init__(self)
