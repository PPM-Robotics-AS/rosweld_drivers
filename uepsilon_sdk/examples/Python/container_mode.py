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
from matplotlib import pyplot as plt
import math as m
import pylinllt as llt

# Parametrize transmission --- Important: make sure this is compliant to sensor
resolution = 1280
container_size = 100
scanner_type = ct.c_int(0)

# Init profile buffer and timestamp info
profile_buffer = (ct.c_ubyte * (resolution * 2 * container_size))()
timestamp = (ct.c_ubyte * 16)()
available_resolutions = (ct.c_uint * 4)()

available_interfaces = [ct.create_string_buffer(8) for i in range(6)]
available_interfaces_p = (ct.c_char_p * 6)(*map(ct.addressof, available_interfaces))

lost_profiles = ct.c_uint()

# Equidistant ranges
x = np.linspace(0, resolution, resolution)
y = np.linspace(0, container_size, container_size)
X, Y = np.meshgrid(x, y)

# Create instance and set IP address
hLLT = llt.create_llt_device()

# Get available interfaces
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
ret = llt.set_profile_config(hLLT, llt.TProfileConfig.CONTAINER)
if ret < 1:
    raise ValueError("Error setting profile config: " + str(ret))

ret = llt.set_packet_size(hLLT, 320)
if ret < 1:
    raise ValueError("Error setting packet size: " + str(ret))

# Set shutter time
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_SHUTTERTIME, 100)
if ret < 1:
    raise ValueError("Error setting shutter time: " + str(ret))

# Set idle time
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLETIME, 900)
if ret < 1:
    raise ValueError("Error idle time: " + str(ret))

# Set rearrangement (z only)
rec_log2 = 1.0 / m.log(2.0)
container_resolution = m.floor((m.log(resolution) * rec_log2) + 0.5)
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_REARRANGEMENT_PROFILE, llt.CONTAINER_DATA_Z | llt.CONTAINER_STRIPE_1 | (container_resolution << 12))
if ret < 1:
    raise ValueError("Error setting rearrangement: " + str(ret))

# Set container size
ret = llt.set_profile_container_size(hLLT, resolution, container_size)
if ret < 1:
    raise ValueError("Error setting profile container size: " + str(ret))

# Start transfer
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_CONTAINER_MODE, 1)
if ret < 1:
    raise ValueError("Error starting transfer profiles: " + str(ret))

# Warm-up time
time.sleep(2)

ret = llt.get_actual_profile(hLLT, profile_buffer, len(profile_buffer), llt.TProfileConfig.CONTAINER,
                           ct.byref(lost_profiles))
if ret != len(profile_buffer):
    raise ValueError("Error get profile buffer data: " + str(ret))

# Stop transmission
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_CONTAINER_MODE, 0)
if ret < 1:
    raise ValueError("Error stopping transfer profiles: " + str(ret))

# Disconnect
ret = llt.disconnect(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while disconnect: " + str(ret))

ret = llt.del_device(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while delete: " + str(ret))

# Convert buffer to big-endian ushort values and reshape them to 2D array
Z = np.frombuffer(profile_buffer, dtype='>H').reshape((container_size, resolution))

fig = plt.figure()
fig.subplots_adjust(wspace=0.3)
plt.pcolormesh(X, Y, Z)
plt.colorbar()
plt.show()
