import serial
import time

class Arduinos:

    def __init__(self, portA='/dev/ttyACM0', portB='/dev/ttyACM1', baud=115200, tout=.5):
        # Set default values
        self.recent_rcmd = 0
        self.recent_lcmd = 0
        self.recent_aggr = 0

        # Read from both of the ports
        sr1 = serial.Serial(portA, baud, timeout=tout)
        sr2 = serial.Serial(portA, baud, timeout=tout)
        # Wait for the arduinos to boot, then read the result
        time.sleep(3)
        sr1res = sr1.readline().decode("utf-8")[0:-2]
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
        return self.set_target_vels(
            self.recent_rcmd, self.recent_lcmd, self.recent_aggr)

    def prepare_for_block(self):
        try:
            # Write to the Serial
            self.sr_stepper.write(str.encode(f"P\n"))
            self.sr_stepper.flush()
            # No result for this one
        except:
            pass

    def query_stepper_state(self):
        # The stepper board handles enabled as well
        try:
            # Write the command
            self.sr_stepper.write(str.encode("Q\n"))
            self.sr_stepper.flush()
            # Parse the result, stripping the <, >, and \r\n
            return tuple(map(lambda x: bool(int(x)),
                self.sr_stepper.readline()[1:-3].split(",")))
        except:
            return (None, None)
