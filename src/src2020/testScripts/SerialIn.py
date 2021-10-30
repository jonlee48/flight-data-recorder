import serial # serial library for reading serial port
from collections import deque # used to do quick list operations (linked list)
import matplotlib.pyplot as plt # for plotting with standard alias
import matplotlib.animation as animation #

port = 'COM3' # change to port Arduino is on
stream = serial.Serial(port ,115200) # baud rate
stream.flushInput() # clear queue so data in queue doesn't interfere

#graphBuffer = deque([]) # float data is stored in buffer array and graphed
maxLen = 50 # max of 50 points in graphBuffer

def update(frameNum,plot,stream,graphBuffer,maxLen):
    try:
        line = stream.readline().decode('UTF-8') # read line and decode Arduino default decoding
        data = [int(val//1) for val in line.split()] # takes space delimited floats and stores in array of floats
        #print(data)
        #print('Heading: ',data[0],' Pitch: ',data[1],' Roll: ',data[2])

        if (len(graphBuffer) == maxLen):
            graphBuffer.pop()
        graphBuffer.appendleft(data[0])
        #heading,pitch,roll = graphBuffer.split()
        plot.set_data(range(maxLen),graphBuffer)

    except KeyboardInterrupt:
        print('Exiting')
    except serial.SerialException:
        print("Error reading from serial port")
        stream.flush()
        stream.close()
    except ValueError:
        print("Value error")

    return plot,


dim = ((0,50),(0,365))
buf = deque([0.0]*dim[0][1])

fig = plt.figure()
ax = plt.axes(xlim=dim[0], ylim=dim[1])
a0, = ax.plot([],[])

anim = animation.FuncAnimation(fig, update, fargs=(a0,stream,buf,dim[0][1]), interval = 50) # calls update every 50 ms
plt.show() # show plot
