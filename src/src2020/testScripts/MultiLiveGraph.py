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


def animate(i,xs,ys,zs,ws):
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
    ax1.clear()
    #ax1 = plt.axes(xlim=dim[0], ylim=dim[1])
    ax1.plot(xs, ys,'b',xs,zs,'r',xs,ws,'g')

    x1,x2,y1,y2 = plt.axis()
    plt.axis((x2 - 5,x2,-360,360))

ani = animation.FuncAnimation(fig, animate,fargs=(xs,ys,zs,ws), interval = 100) # update every 50 ms
plt.show()
