import serial
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
num = 0
xs = []
ys = []
zs = []
ws = []
demo = True

# uncomment below for serial input
#stream = serial.Serial('COM3',115200)
#stream.flushInput()

maxLen = 50

def animate(i,xs,ys,zs,ws,demo,stream):
    if (demo == True):
        graph_data = open('TEST.txt','r').read()
        lines = graph_data.split('\n')
        global num
        if len(lines) > num and len(lines[num]) > 1:
            count,heading,pitch,roll,temp,pres,alt  = lines[num].split()
            xs.append(float(count))
            ys.append(float(heading))
            zs.append(float(pitch))
            ws.append(float(roll))
            xs = xs[-50:]
            ys = ys[-50:]
            zs = zs[-50:]
            ws = ws[-50:]
        num += 1
    else:
        try:
            line = stream.readline().decode('UTF-8')
            count,heading,pitch,roll,temp,pres,alt = line.split()
            if (len(buf) == maxLen):
                buf.pop()
            xs.append(float(count))
            ys.append(float(heading))
            zs.append(float(pitch))
            ws.append(float(roll))
            xs = xs[-50:]
            ys = ys[-50:]
            zs = zs[-50:]
            ws = ws[-50:]
        except KeyboardInterrupt:
            print('Exiting')
        except ValueError:
            print("Value Error")
    ax1.clear()
    ax1.plot(xs, ys,'b',xs,zs,'r',xs,ws,'g')

    x1,x2,y1,y2 = plt.axis()
    plt.axis((x2-5,x2,-360,360))


ani = animation.FuncAnimation(fig, animate,fargs=(xs,ys,zs,ws,demo,stream), interval = 50) # update every 50 ms
plt.show()
