# -*- coding: utf-8 -*-
"""
Created on Sat Oct  2 13:29:57 2021

@author: jonathan
"""

import serial
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation

input_file_path = 'data.txt'
#data = pd.read_csv(input_file_path, sep=",", header=None)
# We no longer have a file to read from
# Create an empty dataframe with columns
data = pd.DataFrame(columns = ['cm', 'degrees'])

ser = serial.Serial('COM3',9600) # change to serial port Arduino is connected to

fig = plt.figure()
ax = fig.add_subplot(projection="polar")

# passed number of interations
def redraw(i):
    print(i)
    global data
    line = ser.readline().decode('UTF-8') # Arduino encodes UTF-8 in serial data
    print(line)
    cm, deg = line.strip().split(",")
    data.loc[len(data)] = [cm,deg]
        
    
    #print(data)
    ax.clear()
    print(data['cm'])
    ax.scatter(data["degrees"], data["cm"], s=8,c="blue",alpha=1)
    ax.set_rmax(200)
    ax.set_thetamin(0)
    ax.set_thetamax(180)
    ax.set_title("Radar data", va='bottom')
    fig.set_figwidth(10)
    fig.set_figheight(10)
    

# Need to save animation to an object, or it will be garbage collected
animation_object = animation.FuncAnimation(fig, redraw, interval=100)
plt.show()
'''
fig = plt.figure()
ax = fig.add_subplot(projection="polar")

ser = serial.Serial('COM3',9600) # change to serial port Arduino is connected to

while(True):
    line = ser.readline().decode('UTF-8') # Arduino encodes UTF-8 in serial data
    print(line)

c = ax.scatter(data["degrees"], data["cm"], s=4,c="blue",alpha=0.75)
#.plot(data["degrees"], data["cm"])
ax.set_rmax(20)
ax.set_thetamin(0)
ax.set_thetamax(180)
ax.set_title("Radar data", va='bottom')
fig.set_figwidth(10)
fig.set_figheight(10)
plt.show()
# function is called every 100 ms to get update from serial port

def animate(i,time,heading,pitch,roll,temp,pres,alt,ser):
    graph_data = open('TEST.txt','r').read()
    line = ser.readline().decode('UTF-8') # Arduino encodes UTF-8 in serial data
    data = [float(val) for val in line.split(' ')]
    if len(data) == 7: # check if file is in right format
        #a,b,c,d,e,f,g  = data.split() # split line into each sensor data
        time.append(data[0]) # add each sensor reading to respective array
        heading.append(data[1])
        pitch.append(data[2])
        roll.append(data[3])
        temp.append(data[4])
        pres.append(data[5])
        alt.append(data[6])
        time = time[-100:] #limit size of array to 50 elems, so graph x range is always 50
        heading = heading[-100:]
        pitch = pitch[-100:]
        roll = roll[-100:]
        temp = temp[-100:]
        pres = pres[-100:]
        alt = alt[-100:]
        
    ax1.clear()

    # Plot orientation:
    ax1.plot(time,heading,'b',time,pitch,'r',time,roll,'g') 
    # Plot altitude:
    #ax1.plot(time,alt,'b')
    
    # set bounds of plot:
    x1,x2,y1,y2 = plt.axis() # get Xmin, Xmax, Ymin, Ymax
    plt.axis((x1,x1 + 10,-370,370)) #plot X range 5 sec, Y range -360 to 360

ani = animation.FuncAnimation(fig, animate,fargs=(time,heading,pitch,roll,temp,pres,alt,ser), interval = 100) # update every 50 ms
plt.show()
'''