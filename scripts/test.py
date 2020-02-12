#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from teb_local_planner.msg import OptimizationCostArray

# plt.ion()
fig = plt.figure()
ax = fig.add_subplot(1,1,1)

global data
global last

data = []

def callback(msg):
    global last
    global data
    last = rospy.Time.now()
    print(data)
    for i in range(0,len(msg.costs)):
        if (float(msg.costs[i].type)==17):
            data.append(msg.costs[i].cost)
    ax.clear()
    ax.plot(data)
    plt.draw()
    plt.pause(0.00000000001)

def timerCB(event):
    global last
    global data
    if (rospy.Time.now()-last).secs>1.0:
        data = []

def listner():
    rospy.init_node('Live_plot')
    rospy.Subscriber("move_base_node/TebLocalPlannerROS/optimization_costs", OptimizationCostArray, callback)
    global last
    last = rospy.Time.now()
    rospy.Timer(rospy.Duration(0.1), timerCB)
    plt.ion()
    plt.show()
    rospy.spin()

listner()
# style.use('fivethirtyeight')


# def animate(ax,xs,ys):
#     ax.plot(xs,ys,'r-')

# i=0
# x = []
# y = []

# while(1):
#     y.append(float(input("Enter y \n")))
#     x.append(i+1)
#     print(x)
#     print(y)
#     animate(ax1,x,y)
#     i=i+1
