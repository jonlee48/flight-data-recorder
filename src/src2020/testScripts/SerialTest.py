"""
Listens to serial COM3 and graphs every third float
If sensor values are (A,B,C), it will graph A
Arduino runs AltitudeSerial.ino
"""

import serial
from collections import deque # faster list operations O(1) instead O(n)
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

# Stores count
num = -1

def update(frameNum, plot, ser, buffer, maxLen):
  try:
    line = ser.readline().decode('UTF-8')
    data = [int(val) for val in line.split()]

    global num
    num += 1

    if num%3==0:
      print("graph ", data[0])
      if len(buffer) < maxLen: 
        buffer.append(data[0])
      else:
        buffer.pop()
        buffer.appendleft(data[0])
    else:
      print(data[0])
    plot.set_data(range(maxLen), buffer)
  except KeyboardInterrupt:
    print('Exiting')
  except serial.SerialException:
    print("Error reading from serial port")
    ser.flush()
    ser.close()    
  except ValueError:
    print("Value error, dropping packet.")
  return plot,

strPort = 'COM3' # change to serial port Arduino is connected to
extens=((0,100),(0,200))  # (x_lo, x_hi),(y_lo, y_hi)

ser = serial.Serial(strPort, 9600) #9600 baud rate
buf = deque([0.0]*extens[0][1])
print(buf)

fig = plt.figure()
ax = plt.axes(xlim=extens[0], ylim=extens[1])
ax.set_title("Measured Distance")
ax.set_xlabel("Buffered Sample")
ax.set_ylabel("Distance (cm)")
a0, = ax.plot([], [])
anim = animation.FuncAnimation(fig,
                               update,
                               fargs=(a0, ser, buf, extens[0][1]),
                               interval=50)
plt.show()

ser.flush()
ser.close()    

print('Exiting')
