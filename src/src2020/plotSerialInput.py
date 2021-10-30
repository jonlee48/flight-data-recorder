import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
"""
EACH LINE CONTAINS 6 FLOATS TAKEN AT A SINGLE INSTANCE
THERE ARE 10 LINES PER SECOND
DATA IS SPACE DELIMITED: time, heading, pitch, roll, temp, pressure, altitude
                  UNITS:    s, degrees, degrees, degrees, F, hPa, meters  
"""
# create arrays to store each sensor reading
time = []
heading = []
pitch = []
roll = []
temp = []
pres = []
alt = []

ser = serial.Serial('COM3',115200) # change to serial port Arduino is connected to

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
