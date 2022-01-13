# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 16:25:48 2021

@author: jonathan
"""
import time as t
import serial
import pandas as pd

df = pd.DataFrame(columns=["cm","degrees","time"])
print(df)

ser = serial.Serial('COM3',9600)

    
def readlines():
    while(line := ser.readline().decode('UTF-8')):
        line = line.strip()
        print(line)
        try:
            sep = line.split(',')
            dist, deg, time = line.strip().split(',')
            df.loc[len(df)] = [float(dist),float(deg),float(time)]
        except Exception as e:
            #print(sep)
            print(e)

while(True):
    t.sleep(1)
    readlines()