import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import serial
import threading
import signal
import sys

#Setup serial ports
ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM8'
ser.open()
if(ser.is_open):
    print("SUCCESS COM port opened")
else:
    print("ERROR COM port failed to open")

s = ""
arduino_node0x = 0
arduino_node0y = 0
arduino_node1x = 0
arduino_node1y = 0

def pol2cart(rho,phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

# FORMAT: (radius0, angle0, radius1, angle1) angles in radians
def read_COM():
    global s
    global arduino_node0x
    global arduino_node0y
    global arduino_node1x
    global arduino_node1y
    while True:
        s = ser.readline()
        s = str(s)
        coords = s.split(",")
        if(len(coords) == 4):
            coords[0] = coords[0].split("'")[1]
            coords[3] = coords[3].split("\\")[0]
            coords[0] = float(coords[0])
            coords[1] = float(coords[1])
            coords[2] = float(coords[2])
            coords[3] = float(coords[3])
            (coords[0],coords[1]) = pol2cart(coords[0],coords[1])
            (coords[2],coords[3]) = pol2cart(coords[2],coords[3])
            arduino_node0x = coords[0]
            arduino_node0y = coords[1]
            arduino_node1x = coords[2]
            arduino_node1y = coords[3]
            print(coords)

#Graph
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

ax = plt.axes(xlim=(-7, 7), ylim=(-7, 7))
center = plt.Circle((0,0), 0.75, fc='y')
node0 = plt.Circle((0,0), 0.75, fc='y')
compass0 = plt.Circle((0,0),0.15, fc='r')
node1 = plt.Circle((0,0), 0.75, fc='y')
compass1 = plt.Circle((0,0),0.15, fc='r')

def init():
    ax.add_patch(center)
    ax.add_patch(node0)
    ax.add_patch(compass0)
    ax.add_patch(node1)
    ax.add_patch(compass1)
    return center,node0,compass0,node1,compass1

def animate(i):
    global arduino_node0x
    global arduino_node0y
    global arduino_node1x
    global arduino_node1y
    #Center Node
    center.center = (0,0)
    #Node 0
    node0x, node0y = node0.center
    node0x = arduino_node0x # Read from com port
    node0y = arduino_node0y # Read from com port
    node0.center = (node0x, node0y)
    #Node 0 Compass Dot
    compx, compy = compass0.center
    xtrans = 0.5*np.sin(np.radians(i))
    ytrans = 0.5*np.cos(np.radians(i))
    compx = node0x + xtrans
    compy = node0y + ytrans
    compass0.center = (compx,compy)
    #Node 1
    node1x, node1y = node1.center
    node1x = arduino_node1x # Read from com port
    node1y = arduino_node1y # Read from com port
    node1.center = (node1x, node1y)
    #Node 1 Compass Dot
    compx, compy = compass1.center
    xtrans = 0.5*np.sin(np.radians(i))
    ytrans = 0.5*np.cos(np.radians(i))
    compx = node1x + xtrans
    compy = node1y - ytrans
    compass1.center = (compx,compy)
    return center,node0,compass0,node1,compass1

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=360, 
                               interval=20,
                               blit=True)

t = threading.Thread(target=read_COM,daemon = True)
t.start()
plt.show()