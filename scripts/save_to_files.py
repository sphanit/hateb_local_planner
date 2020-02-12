#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from hanp_msgs.msg  import TimeToGoal
from hanp_msgs.msg  import HumanTimeToGoalArray
from hanp_msgs.msg  import HumanPathArray
from hanp_msgs.msg  import HumanTrajectoryArray
from hanp_msgs.msg  import Trajectory
from nav_msgs.msg import Path

def traj_ttg(msg):
    data["ttg_traj"].append(msg.time_to_goal)

def plot_traj_ttg():
    n = len(data["ttg_traj"])
    x = np.arange(0,n,1)
    y =[]
    for i in range(0,n):
        y.append(data["ttg_traj"][i].to_sec())


    plt.figure()
    plt.plot(x,y)
    plt.ion()
    plt.show()
    plt.title('TTG Traj')


def path_ttg(msg):
    data["ttg_path"].append(msg.time_to_goal)

def plot_path_ttg():
    n = len(data["ttg_path"])
    x = np.arange(0,n,1)

    y =[]
    for i in range(0,n):
        y.append(data["ttg_path"][i].to_sec())

    plt.figure()
    plt.plot(x,y)
    plt.ion()
    plt.show()
    plt.title('TTG Path')


def h_traj_ttg(msg):
    data["h_ttg_traj"].append(msg.times_to_goal[0].time_to_goal)

def plot_h_traj_ttg():
    n = len(data["h_ttg_traj"])
    x = np.arange(0,n,1)

    y =[]
    for i in range(0,n):
        y.append(data["h_ttg_traj"][i].to_sec())

    plt.figure()
    plt.plot(x,y)
    plt.ion()
    plt.show()
    plt.title('TTG H_Traj')

def h_path_ttg(msg):
    data["h_ttg_path"].append(msg.times_to_goal[0].time_to_goal)

def plot_h_path_ttg():
    n = len(data["h_ttg_path"])
    x = np.arange(0,n,1)

    y =[]
    for i in range(0,n):
        y.append(data["h_ttg_path"][i].to_sec())

    plt.figure()
    plt.plot(x,y)
    plt.ion()
    plt.show()
    plt.title('TTG H_path')

def global_plan(msg):
    global last_time
    global one_save_gr

    last_time = rospy.Time.now()
    if one_save_gr:
        data["g_plan"].append(msg)
        one_save_gr = False

# def plot_global_plan():
#     n = len(data["g_plan"])
#     x = np.arange(0,n,1)
#
#     y =[]
#     for i in range(0,n):
#         y.append(data["g_plan"][i].to_sec())
#
#     plt.figure()
#     plt.plot(x,y)
#     plt.ion()
#     plt.show()
#     plt.title('Global plan')

def h_global_plan(msg):
    global last_time
    global one_save_gh

    last_time = rospy.Time.now()
    if one_save_gh:
        data["h_g_plan"].append(msg.paths[0])
        one_save_hr = False

def local_plan(msg):
    data["l_plan"].append(msg)

def h_local_plan(msg):
    data["h_l_plan"].append(msg.paths[0])

def local_traj(msg):
    data["l_traj"].append(msg)

def h_local_traj(msg):
    data["h_l_traj"].append(msg.trajectories[0])

def timerCB(event):
    now = rospy.Time.now()
    global last_time
    if (last_time-now).secs > 2:
        one_save_gr = True
        one_save_hr = True

def clear():
    data = { "ttg_traj":[], "ttg_path":[], "h_ttg_traj":[], "h_ttg_path":[], "g_plan":[],"l_plan":[], "l_traj":[], "h_g_plan":[], "h_l_plan":[], "h_l_traj":[]}

def listener():
    global last_time
    global one_save_gr
    global one_save_gh

    one_save_gr = True
    one_save_gh = True
    rospy.init_node('data_saving_teb')
    last_time = rospy.Time.now()
    # root = Tk()
    # my_gui = GuessingGame(root)
    # root.mainloop()
    # Subscribe to all topics
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/global_plan",Path,global_plan)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_plan",Path,local_plan)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_traj",Trajectory,local_traj)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/plan_time",TimeToGoal,path_ttg)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/traj_time",TimeToGoal,traj_ttg)

    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_global_plans",HumanPathArray,h_global_plan)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_local_plans",HumanPathArray,h_local_plan)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_local_trajs",HumanTrajectoryArray,h_local_traj)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_plans_time",HumanTimeToGoalArray,h_path_ttg)
    rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_trajs_time",HumanTimeToGoalArray,h_traj_ttg)

    rospy.Timer(rospy.Duration(0.1), timerCB)
    print("Started")


    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
