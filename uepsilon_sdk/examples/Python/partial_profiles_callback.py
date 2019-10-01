#
# scanCONTROL Linux SDK - example code
#
# MIT License
#
# Copyright (c) 2017-2018 Micro-Epsilon Messtechnik GmbH & Co. KG
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Daniel Rauch <daniel.rauch@micro-epsilon.de>
#

import time
import ctypes as ct
import numpy as np
import threading
import pylinllt as llt

from matplotlib import pyplot as plt
import matplotlib.animation as animation

def profile_callback(data, size, user_data):
    user_data_cast = ct.cast(user_data, ct.POINTER(ct.c_uint))
    if user_data_cast.contents.value == 1:
        global profile_buffer
        ct.memmove(profile_buffer, data, size)
        event.set()

# Parametrize transmission --- Important: make sure this is compliant to sensor
start_data = 4
data_width = 4
resolution = 1280
scanner_type = ct.c_int(0)

# Init profile buffer and timestamp info
profile_buffer = (ct.c_ubyte * (resolution * data_width))()
timestamp = (ct.c_ubyte * 16)()
available_resolutions = (ct.c_uint * 4)()

available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))

lost_profiles = ct.c_int()
shutter_opened = ct.c_double(0.0)
shutter_closed = ct.c_double(0.0)
profile_count = ct.c_uint(0)
cb_user_data = ct.c_uint(1)  # Sensor 1

get_profile_cb = llt.buffer_cb_func(profile_callback)
event = threading.Event()

# Partial profile struct
partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)
me_device_data = llt.MEDeviceData()

# Declare measuring data array
x = np.empty(resolution, dtype=float)  # (ct.c_double * resolution)()
z = np.empty(resolution, dtype=float)  # (ct.c_double * resolution)()
x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))

# Null pointer if data not necessary
null_ptr_short = ct.POINTER(ct.c_ushort)()
null_ptr_int = ct.POINTER(ct.c_uint)()

hLLT = llt.create_llt_device()

ret = llt.get_device_interfaces(available_interfaces_p, len(available_interfaces))
if ret < 1:
    raise ValueError("Error getting interfaces : " + str(ret))

ret = llt.set_device_interface(hLLT, available_interfaces[0])
if ret < 1:
    raise ValueError("Error setting device interface: " + str(ret))

# Connect
ret = llt.connect(hLLT)
if ret < 1:
    raise ConnectionError("Error connect: " + str(ret))

# Get available resolutions
ret = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
if ret < 1:
    raise ValueError("Error getting resolutions : " + str(ret))

if resolution not in available_resolutions:
    raise AttributeError("Wrong resolution")

# Scanner type
ret = llt.get_llt_type(hLLT, ct.byref(scanner_type))
if ret < 1:
    raise ValueError("Error scanner type: " + str(ret))

# Scanner type
ret = llt.set_resolution(hLLT, resolution)
if ret < 1:
    raise ValueError("Error setting resolution: " + str(ret))

# Set partial profile
ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PARTIAL_PROFILE)
if ret < 1:
    raise ValueError("Error setting profile config: " + str(ret))

ret = llt.set_partial_profile(hLLT, ct.byref(partial_profile_struct))
if ret < 1:
    raise ValueError("Error setting partial profile: " + str(ret))

ret = llt.register_buffer_callback(hLLT, get_profile_cb, ct.byref(cb_user_data))
if ret < 1:
    raise ValueError("Error setting callback: " + str(ret))

# Start transfer
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
if ret < 1:
    raise ValueError("Error starting transfer profiles: " + str(ret))

# Warm-up time
time.sleep(0.1)

fig, ax = plt.subplots()
line, = ax.plot([], [], ".b", lw=2)
ax.grid()
ax.set_xlim(-60, 60)
ax.set_ylim(25, 350)
line.set_data(x, z)


def data_gen(*args):
    event.wait()
    fret = llt.convert_part_profile_2_values(profile_buffer, len(profile_buffer), ct.byref(partial_profile_struct), scanner_type, 0,
                                         null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
    if fret & llt.CONVERT_X is 0 or fret & llt.CONVERT_Z is 0:
        raise ValueError("Error converting data: " + str(ret))

    for i in range(16):
        timestamp[i] = profile_buffer[resolution * data_width - 16 + i]

    llt.timestamp_2_time_and_count(timestamp, ct.byref(shutter_opened), ct.byref(shutter_closed), ct.byref(profile_count), null_ptr_short)
    event.clear()

    yield x, z


def update(data):
    ux, uz = data
    line.set_data(ux, uz)
    return line,

ani = animation.FuncAnimation(fig, update, frames=data_gen, interval=40)
plt.show()

ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
if ret < 1:
    raise ValueError("Error stopping transfer profiles: " + str(ret))

# Disconnect
ret = llt.disconnect(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while disconnect: " + str(ret))

# Delete
ret = llt.del_device(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while delete: " + str(ret))