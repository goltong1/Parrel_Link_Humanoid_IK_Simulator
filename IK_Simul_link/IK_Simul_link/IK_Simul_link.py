import math
"""
============
3D animation
============

A simple example of an animated plot... In 3D!
"""
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.animation import FuncAnimation

pattern=[[0,0,0],[0.5,0.5,0.4],[1,1,0.8],[1.5,1.5,0.4],[1.6,1.6,0],[1.5,1.5,-0.4],[1,1,-0.8],[0.5,0.5,-0.4],[0,0,0],[0,0,0]]

def DegreeToRadian(Degree):
    return Degree/180*3.141592

def RadianToDegree(Radian):
    return Radian/3.141592*180

L1=1
link=1
def func_animate(i):
    print("frame",i)
    x=pattern[i][0]
    y=pattern[i][1]
    z=pattern[i][2]
    if z!=0:
        theta3=math.atan2(z,(-y+2*L1+link))
        
        n=(-y+2*L1+link)/(math.cos(theta3)*L1)-link/L1
        o=math.acos((n**2+(x**2)/L1-2)/2)
        p=math.asin((2*n*x)/(L1*(n**2)+x**2))
    else:
        n=(-y+2*L1)/L1
        o=math.acos((n**2+(x**2)/L1-2)/2)
        p=math.asin((2*n*x)/(L1*(n**2)+x**2))
        theta3=0
     

    theta1=(o+p)/2
    theta2=(o-p)/2
    line = np.linspace(0,1, 1000)

    plt.cla()
    ax.set_xlim3d([0.0, 3.0])

    ax.set_xlabel('X')

    ax.set_ylim3d([0.0, 3.0])
    ax.set_ylabel('Y')

    ax.set_zlim3d([0.0, 3.0])
    ax.set_zlabel('Z')
    ax.plot3D(line*math.sin(theta1)*L1,L1*2+link-line*math.cos(theta1)*L1*math.cos(theta3),line*math.sin(theta3)*L1*math.cos(theta1), 'green')
    ax.plot3D(math.sin(theta1)*L1+0*line,L1*2+link-(link*line+math.cos(theta1)*L1)*math.cos(theta3),(L1*math.cos(theta1)+link*line)*math.sin(theta3), 'red')
    ax.plot3D(math.sin(theta1)*L1-math.sin(theta2)*line*L1,L1*2+link-(link+math.cos(theta1)*L1+math.cos(theta2)*L1*line)*math.cos(theta3),(L1*math.cos(theta1)+link+L1*math.cos(theta2)*line)*math.sin(theta3), 'blue')
    print(theta1,theta2,theta3)
    print(math.sin(theta1)*L1-math.sin(theta2)*L1,L1*2+link-(link+math.cos(theta1)*L1+math.cos(theta2)*L1)*math.cos(theta3),(L1*math.cos(theta1)+link+L1*math.cos(theta2))*math.sin(theta3))
#x=sin1*L1-sin2*L2
#y=(L1+L2+link)-(cos1*L1+link+cos2*L2)
#z=(cos1*L1+link+cos2*L2)*sin3
fig = plt.figure()
ax = p3.Axes3D(fig)

#theta1=DegreeToRadian(60)
#theta2=DegreeToRadian(60)
#theta3=0
# Data for a three-dimensional line

plt.axis([0, 4*np.pi, -1, 1])

ani = FuncAnimation(fig,
                    func_animate,
                    frames=10,
                    interval=1000)
plt.show()












