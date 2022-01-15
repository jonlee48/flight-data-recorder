# -*- coding: utf-8 -*-
"""
Created on Wed Jan 12 21:41:18 2022

@author: jonathan

scripts:
groundstation.ino
fdrv7.ino
"""

import serial
import time
ser = serial.Serial(port='COM7',
                    baudrate=115200,
                    timeout=0)

# Stack overflow solution:
# https://stackoverflow.com/questions/61166544/readline-in-pyserial-sometimes-captures-incomplete-values-being-streamed-from
def checkPort():
    time.sleep(.001)                    # delay of 1ms
    val = ser.readline()                # read complete line from serial output
    while not '\\n' in str(val):         # check if full data is received.
        # This loop is entered only if serial read value doesn't contain \n
        time.sleep(.001)
        temp = ser.readline()           # check for serial output.
        if not not temp.decode():       # if temp is not empty.
            val = (val.decode()+temp.decode()).encode()
            # required to decode, sum, then encode because
            # long values might require multiple passes
    val = val.decode()                  # decoding from bytes
    val = val.strip()                   # stripping leading and trailing spaces.
    print(val)

if __name__ == '__main__':
    while True:                             # runs this loop forever
        time.sleep(0.1)
        checkPort()

