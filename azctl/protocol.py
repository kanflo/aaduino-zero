"""
The MIT License (MIT)

Copyright (c) 2017 Johan Kanflo (github.com/kanflo)

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

from uframe import *
import struct

# command_t
cmd_ping = 1
cmd_upgrade_start = 9 # For historical reasons, these are 9 and 10
cmd_upgrade_data = 10

cmd_fwu_download_start = 11
cmd_fwu_data = 12
cmd_fwu_upgrade = 13
cmd_fwu_downgrade = 14


cmd_response = 0x80

# upgrade_status_t
upgrade_continue = 0
upgrade_bootcom_error = 1
upgrade_crc_error = 2
upgrade_erase_error = 3
upgrade_flash_error = 4
upgrade_overflow_error = 5
upgrade_success = 16


"""
 Helpers for creating frames.
 Each function returns a complete frame ready for transmission.

"""
def create_response(command, success):
    f = uFrame()
    f.pack8(cmd_response | command)
    f.pack8(success)
    f.end()
    return f

def create_cmd(cmd):
    f = uFrame()
    f.pack8(cmd)
    f.end()
    return f

def create_upgrade_start(window_size, crc):
    f = uFrame()
    f.pack8(cmd_upgrade_start)
    f.pack16(window_size)
    f.pack16(crc)
    f.end()
    return f

def create_upgrade_data(data):
    f = uFrame()
    f.pack8(cmd_upgrade_data)
    for d in data:
        f.pack8(d)
    f.end()
    return f

def create_fwu_download_start(size, crc):
    f = uFrame()
    f.pack8(cmd_fwu_download_start)
    f.pack16(size)
    f.pack16(crc)
    f.end()
    return f

def create_fwu_download_data(data):
    f = uFrame()
    f.pack8(cmd_fwu_data)
    for d in data:
        f.pack8(d)
    f.end()
    return f

def create_run_fwu():
    f = uFrame()
    f.pack8(cmd_fwu_upgrade)
    f.end()
    return f


"""
Helpers for unpacking frames.

These functions will unpack the content of the unframed payload and return
true. If the command byte of the frame does not match the expectation or the
frame is too short to unpack the expected payload, false will be returned.
"""

# Returns success
def unpack_response():
    return uframe.unpack8()
