#!/usr/bin/env python3
"""
File:          test_sim.py
Author:        Binit Shah
Last Modified: Binit on 2/21
"""

import interface
from sim import SimConfig

if __name__ == "__main__":
    c = SimConfig("")
    interface.set_system("sim", sim_config=c)
    while True:
        print(interface.get_time())
