#!/usr/bin/env python

import serial
import datetime

tty = '/dev/cu.usbserial-AJV9MSUG'
timeout_s = 5*60

ser = serial.Serial(tty, 115200, timeout = timeout_s)

try:
    while True:
        line = ser.readline()
        now = datetime.datetime.now()
        if line:
            print("%-28s %s" % (now, line.rstrip()))
        else:
            print("%-28s No data in %d seconds" % (now, timeout_s))
except KeyboardInterrupt:
    print("ctrl-c")