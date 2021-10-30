import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import pandas as pd

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

num = 0

def data_to_csv(flight_num):
    df_flight_data =  pd.read_csv('TEST.txt', sep=" ", header=None)
    df_flight_data.columns = ['time', 'heading', 'pitch', 'roll', 'xaccel', 'yaccel', 'zaccel', 'temp', 'pressure', 'altitude']
    df_flight_data.to_csv(r'/Users/Jonathan Lee/MyFiles/FlightRecorder2/flight_data_test_{}.csv'.format(flight_num), index=False)

def animate(i,time,heading,pitch,roll,temp,pres,alt):
    graph_data = open('TEST.txt','r').read()
    lines = graph_data.split('\n') # lines is array consisting of each line of sensor data
    global num    
    if len(lines) > num and len(lines[num]) > 1: # check if file is in right format
        a,b,c,d,e,f,g,h,i,j  = lines[num].split() # split line into each sensor data
        time.append(float(a)) # add each sensor reading to respective array
        heading.append(float(b))
        pitch.append(float(c))
        roll.append(float(d))
        temp.append(float(e))
        pres.append(float(f))
        alt.append(float(g))
        time = time[-50:] #limit size of array to 50 elems, so graph x range is always 50
        heading = heading[-50:]
        pitch = pitch[-50:]
        roll = roll[-50:]
        temp = temp[-50:]
        pres = pres[-50:]
        alt = alt[-50:]
        num += 1

    ax1.clear()

    # Plot orientation:
#    ax1.plot(time,heading,'b',time,pitch,'r',time,roll,'g') 
    # Plot altitude:
    ax1.plot(time,alt,'b')
    
    # set bounds of plot:
    x1,x2,y1,y2 = plt.axis() # get Xmin, Xmax, Ymin, Ymax
    plt.axis((x2 - 5,x2,-50,50)) #plot X range 5 sec, Y range -360 to 360


#need to assign a flight number to save the file
flight_num = 0

#this puts the data into a csv file
data_to_csv(flight_num)

ani = animation.FuncAnimation(fig, animate,fargs=(time,heading,pitch,roll,temp,pres,alt), interval = 50) # update every 50 ms
plt.show()
