# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 21:41:18 2022

@author: jonathan
"""

import serial
import time
ser = serial.Serial(port='COM4',
                    baudrate=115200,
                    timeout=0)

print("connected to: " + ser.portstr)

while True:                             # runs this loop forever
    time.sleep(.001)                    # delay of 1ms
    val = ser.readline()                # read complete line from serial output
    while not '\\n'in str(val):         # check if full data is received. 
        # This loop is entered only if serial read value doesn't contain \n
        # which indicates end of a sentence. 
        # str(val) - val is byte where string operation to check `\\n` 
        # can't be performed
        time.sleep(.001)                # delay of 1ms 
        temp = ser.readline()           # check for serial output.
        if not not temp.decode():       # if temp is not empty.
            val = (val.decode()+temp.decode()).encode()
            # requrired to decode, sum, then encode because
            # long values might require multiple passes
    val = val.decode()                  # decoding from bytes
    val = val.strip()                   # stripping leading and trailing spaces.
    print(val)