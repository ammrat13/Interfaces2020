import serial
import time

class Arduinos:

    def __init__(self, portA='/dev/ttyACM0', portB='/dev/ttyACM1', baud=115200, tout=.5):
        # Set default values
        self.recent_rcmd = 0
        self.recent_lcmd = 0
        self.recent_aggr = 0
        self.recent_stcmd = 0

        # Read from both of the ports
        sr1 = serial.Serial(portA, baud, timeout=tout)
        time.sleep(3)
        sr1res = sr1.readline().decode("utf-8")[0:-2]
        sr2 = serial.Serial(portA, baud, timeout=tout)
        time.sleep(3)
        sr2res = sr2.readline().decode("utf-8")[0:-2]

        # Figure out which one is which
        if sr1res not in ["stepper","motor"] \
        or sr2res not in ["stepper","motor"] \
        or sr1res != sr2res:
            raise RuntimeError("Invalid arduinos")
        elif sr1res == "stepper":
            self.sr_stepper = sr1
            self.sr_motor = sr2
        elif sr2res == "stepper":
            self.sr_stepper = sr2
            self.sr_motor = sr1
        else:
            raise RuntimeError("Invalid arduinos")

    def close(self):
        if self.sr_motor.isOpen():
            self.sr_motor.flush()
            self.sr_motor.close()
        if self.sr_stepper.isOpen():
            self.sr_stepper.flush()
            self.sr_stepper.close()

    def set_target_vels(self, w_r, w_l, aggr):
        # Store these if needed for get_vels
        self.recent_rcmd = w_r
        self.recent_lcmd = w_l
        self.recent_aggr = aggr
        # Can generate an exception
        # If we fail for any reason, return None
        try:
            # Write to the Serial
            self.sr_motor.write(str.encode(f"<{w_l},{w_r},{int(aggr)}>\n"))
            self.sr_motor.flush()
            # Parse the result
            return tuple(map(int,
                self.sr_motor.readline().decode("utf-8")[1:-3].split(",")))
        except:
            return None

    def get_vels(self):
        # Just set the target velocities again
        # Board provides no way of just reading
        self.set_target_vels(
            self.recent_rcmd, self.recent_lcmd, self.recent_aggr)

    def set_stepper_target(self, pos):
        # Store this if needed for the other methods
        self.recent_stcmd = pos
        # Can generate an exception
        # If we fail for any reason, return None
        try:
            # Write to the Serial
            self.sr_stepper.write(str.encode(f"{pos}\n"))
            self.sr_stepper.flush()
            # Parse the result
            ret = list(
                self.sr_stepper.readline().decode("utf-8")[1:-3].split(","))
            ret[0] = int(ret[0])
            ret[1] = bool(int(ret[1]))
            return tuple(ret)
        except:
            return None

    def get_stepper_dtg(self):
        # Just recommand and return the right one
        try:
            self.set_stepper_target(self.recent_stcmd)[0]
        except:
            return None

    def get_enabled(self):
        # Just recommand and return the right one
        try:
            self.set_stepper_target(self.recent_stcmd)[1]
        except:
            return None
