#!/usr/bin/env python3
"""
File:          test_raspi.py
Author:        Binit Shah
Last Modified: Binit on 2/21
"""

import interface

if __name__ == "__main__":
    interface.set_system("raspi")
    while True:
        print(interface.get_time())
