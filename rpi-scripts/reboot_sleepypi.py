#!/usr/bin/python3


import serial

serialHandle = serial.Serial('/dev/ttyS0', 9600)
d = (int(65535)).to_bytes(2, byteorder='little', signed=False)
d1 = (int(1)).to_bytes(2, byteorder='little', signed=False)
d2 = (int(0)).to_bytes(2, byteorder='little', signed=False)
data = d + d1 + d2
serialHandle.write(data)

