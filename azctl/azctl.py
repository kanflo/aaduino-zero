#!/usr/bin/env python

"""
The MIT License (MIT)

Copyright (c) 2018 Johan Kanflo (github.com/kanflo)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

import argparse
import sys
import os
import socket
try:
    import serial
except:
    print("Missing dependency pyserial:")
    print(" sudo pip%s install pyserial" % ("3" if sys.version_info.major == 3 else ""))
    raise SystemExit()
import threading
import time
from protocol import *
import uframe
import binascii
try:
    from PyCRC.CRCCCITT import CRCCCITT
except:
    print("Missing dependency pycrc:")
    print(" sudo pip%s install pycrc" % ("3" if sys.version_info.major == 3 else ""))
    raise SystemExit()

parameters = []

"""
An abstract class that describes a communication interface
"""
class comm_interface(object):

    _if_name = None

    def __init__(self, if_name):
        self._if_name = if_name

    def open(self):
        return False

    def close(self):
        return False

    def write(self, bytes):
        return False

    def read(self):
        return bytearray()

    def name(self):
        return self._if_name

"""
A class that describes a serial interface
"""
class tty_interface(comm_interface):

    _port_handle = None

    def __init__(self, if_name):
        self._if_name = if_name

    def open(self):
        if not self._port_handle:
            self._port_handle = serial.Serial(baudrate = 115200, timeout = 10.0)
            self._port_handle.port = self._if_name
            self._port_handle.open()
        return True

    def close(self):
        self._port_handle.port.close()
        self._port_handle = None
        return True

    def write(self, bytes):
        self._port_handle.write(bytes)
        return True

    def read(self):
        bytes = bytearray()
        sof = False
        while True:
            b = self._port_handle.read(1)
            if not b: # timeout
                break
            b = ord(b)
            # The device may send debug prints inbetween frames if these lines
            # begin with # and end with \n
            if not sof and b == ord('#'):
                sys.stdout.write("#")
                while True:
                    b = self._port_handle.read(1)
                    sys.stdout.write(b)
                    sys.stdout.flush()
                    if b == '\n':
                        break;
            if b == uframe._SOF:
                bytes = bytearray()
                sof = True
            if sof:
                bytes.append(b)
            if b == uframe._EOF:
                break
        return bytes

"""
Print error message and exit with error
"""
def fail(message):
        print("Error: %s." % (message))
        sys.exit(1)


"""
Handle a response frame from the device.
Return a dictionaty of interesting information.
"""
def handle_response(command, frame, args):
    ret_dict = {}
    resp_command = frame.get_frame()[0]
    if resp_command & cmd_response:
        resp_command ^= cmd_response
        success = frame.get_frame()[1]
        if resp_command != command:
            print("Warning: sent command %02x, response was %02x." % (command, resp_command))
        if resp_command !=  cmd_upgrade_start and resp_command != cmd_upgrade_data and not success:
            fail("command failed according to device")

    if resp_command == cmd_ping:
        print("Got pong from device")
    elif resp_command == cmd_upgrade_start:
    #  *  DPS BL: [cmd_response | cmd_upgrade_start] [<upgrade_status_t>] [<chunk_size:16>]
        cmd = frame.unpack8()
        status = frame.unpack8()
        chunk_size = frame.unpack16()
        ret_dict["status"] = status
        ret_dict["chunk_size"] = chunk_size
    elif resp_command == cmd_upgrade_data:
        cmd = frame.unpack8()
        status = frame.unpack8()
        ret_dict["status"] = status
    elif resp_command == cmd_fwu_download_start or resp_command == cmd_fwu_data:
        cmd = frame.unpack8()
        status = frame.unpack8()
        ret_dict["status"] = status
    elif resp_command == cmd_fwu_upgrade or resp_command == cmd_fwu_downgrade:
        cmd = frame.unpack8()
        status = frame.unpack8()
        ret_dict["status"] = status
    else:
        print("Unknown response %d from device." % (resp_command))

    return ret_dict

"""
Communicate with the AAduino Zero according to the user's whishes
"""
def communicate(comms, frame, args):
    bytes = frame.get_frame()

    if not comms:
        fail("no communication interface specified")
    if not comms.open():
        fail("could not open %s" % (comms.name()))
    if args.verbose:
        print("Communicating with %s" % (comms.name()))
        print("TX %2d bytes [%s]" % (len(bytes), " ".join("%02x" % b for b in bytes)))
    if not comms.write(bytes):
        fail("write failed on %s" % (comms.name()))
    resp = comms.read()
    if len(resp) == 0:
        fail("timeout talking to device %s" % (comms._if_name))
    elif args.verbose:
        print("RX %2d bytes [%s]\n" % (len(resp), " ".join("%02x" % b for b in resp)))
    if not comms.close:
        print("Warning: could not close %s" % (comms.name()))

    f = uFrame()
    res = f.set_frame(resp)
    if res < 0:
        fail("protocol error (%d)" % (res))
    else:
        return handle_response(frame.get_frame()[1], f, args)

"""
Handle commands
"""
def handle_commands(args):
    comms = create_comms(args)
    if args.ping:
        communicate(comms, create_cmd(cmd_ping), args)
    if args.firmware:
        run_upgrade(comms, args.firmware, args)
    if args.fwu:
        download_fwu(comms, args.fwu, args)
    if args.runfwu:
        run_fwu(comms, args)

# Darn beautiful, from SO: https://stackoverflow.com/a/1035456
def chunk_from_file(filename, chunk_size):
    with open(filename, "rb") as f:
        while True:
            chunk = f.read(chunk_size)
            if chunk:
                yield bytearray(chunk)
            else:
                break

"""
Run AAduino Zero firmware upgrade
"""
def run_upgrade(comms, fw_file_name, args):
    with open(fw_file_name, mode='rb') as file:
        #crc = binascii.crc32(file.read()) % (1<<32)
        content = file.read()
        if content.encode('hex')[6:8] != "20" and not args.force:
            fail("The firmware file does not seem valid, use --force to force upgrade")
        crc = CRCCCITT().calculate(content)
#    chunk_size = 16
    chunk_size = 1024
    ret_dict = communicate(comms, create_upgrade_start(chunk_size, crc), args)
    if ret_dict["status"] == upgrade_continue:
        if chunk_size != ret_dict["chunk_size"]:
            chunk_size = ret_dict["chunk_size"]
        counter = 0
        for chunk in chunk_from_file(fw_file_name, chunk_size):
            counter += len(chunk)
            sys.stdout.write("\rDownload progress: %d%% " % (counter*1.0/len(content)*100.0) )
            sys.stdout.flush()
#            print(" %d bytes" % (counter))

            ret_dict = communicate(comms, create_upgrade_data(chunk), args)
            status = ret_dict["status"]
            if status == upgrade_continue:
                pass
            elif status == upgrade_crc_error:
                print("")
                fail("device reported CRC error")
            elif status == upgrade_erase_error:
                print("")
                fail("device reported erasing error")
            elif status == upgrade_flash_error:
                print("")
                fail("device reported flashing error")
            elif status == upgrade_overflow_error:
                print("")
                fail("device reported firmware overflow error")
            elif status == upgrade_success:
                print("")
            else:
                print("")
                fail("device reported an unknown error (%d)" % status)
    else:
        fail("Device rejected firmware upgrade")
    sys.exit(os.EX_OK)

"""
Download FWU image
"""
def download_fwu(comms, fw_file_name, args):
    with open(fw_file_name, mode='rb') as file:
        #crc = binascii.crc32(file.read()) % (1<<32)
        content = file.read()
        if content.encode('hex')[6:8] != "20" and not args.force:
            fail("The firmware file does not seem valid, use --force to force upgrade")
        crc = CRCCCITT().calculate(content)
        print("Crc: %x" % crc)
    ret_dict = communicate(comms, create_fwu_download_start(len(content), crc), args)
    if ret_dict["status"] == 1:
        chunk_size = 16
        counter = 0
        for chunk in chunk_from_file(fw_file_name, chunk_size):
            counter += len(chunk)
            sys.stdout.write("\rDownload progress: %d%% " % (counter*1.0/len(content)*100.0) )
            sys.stdout.flush()
#            print(" %d bytes" % (counter))

            ret_dict = communicate(comms, create_fwu_download_data(chunk), args)
            status = ret_dict["status"]
            if status == 1:
                pass
            else:
                print("")
                fail("device reported an unknown error (%d)" % status)
    else:
        fail("Device rejected FWU")
    sys.exit(os.EX_OK)

"""
Run FWU
"""
def run_fwu(comms, args):
    ret_dict = communicate(comms, create_run_fwu(), args)
    if ret_dict["status"] == 1:
        print("Success!")
    else:
        fail("Device rejected FWU")
    sys.exit(os.EX_OK)

"""
Create and return a comminications interface object or None if no comms if
was specified.
"""
def create_comms(args):
    if_name = None
    comms = None
    if args.device:
        if_name = args.device
    elif 'AZPORT' in os.environ and len(os.environ['AZPORT']) > 0:
        if_name = os.environ['AZPORT']

    if if_name != None:
        comms = tty_interface(if_name)
    else:
        fail("no comms interface specified")
    return comms


"""
Ye olde main
"""
def main():
    global args
    testing = '--testing' in sys.argv
    parser = argparse.ArgumentParser(description='Flash an AAduino Zero device')
    parser.add_argument('-d', '--device', help="/dev/tty device to connect to. If omitted, azctl will try the environment variable AZPORT", default='')
    parser.add_argument('-p', '--ping', action='store_true', help="Ping device")
    parser.add_argument('-v', '--verbose', action='store_true', help="Verbose communications")
    parser.add_argument('-u', '--upgrade', type=str, dest="firmware", help="Flash an application")
    parser.add_argument('-f', '--force', action='store_true', help="Force upgrade even if azctl complains about the firmware")
    parser.add_argument('-F', '--fwu', type=str, dest="fwu", help="Download FWU image, if zeroboot is build with support for it")
    parser.add_argument('-U', '--runfwu', action='store_true', help="Run FWU")
    args, unknown = parser.parse_known_args()
    try:
        handle_commands(args)
    except KeyboardInterrupt:
        print("")

if __name__ == "__main__":
    main()
