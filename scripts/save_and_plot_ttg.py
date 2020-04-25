#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
# implement the default mpl key bindings
from matplotlib.backend_bases import key_press_handler

from hanp_msgs.msg  import TimeToGoal
from hanp_msgs.msg  import HumanTimeToGoalArray
from hanp_msgs.msg  import HumanPathArray
from hanp_msgs.msg  import HumanTrajectoryArray
from hanp_msgs.msg  import Trajectory
from nav_msgs.msg import Path
from tkinter import filedialog
from tkinter import *
from tkinter import Tk, Label, Button, Entry, StringVar, DISABLED, NORMAL, END, W, E,N,S, RIGHT, LEFT

global one_save_gr
global one_save_gh
global last_time


class PlotTTG_GUI:
    def __init__(self, master):
        self.master = master
        master.title("Plotter HATEB")
        self.data = { "ttg_traj":[], "ttg_path":[], "h_ttg_traj":[], "h_ttg_path":[], "g_plan":[],"l_plan":[], "l_traj":[], "h_g_plan":[], "h_l_plan":[], "h_l_traj":[]}
        self.PlotWindows = []
        self.num = 5

        global last_time
        global one_save_gr
        global one_save_gh

        one_save_gr = True
        one_save_gh = True

        rospy.init_node('data_saving_teb')
        last_time = rospy.Time.now()

        # Subscribe to all topics
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/global_plan",Path,self.global_plan)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_plan",Path,self.local_plan)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/local_traj",Trajectory,self.local_traj)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/plan_time",TimeToGoal,self.path_ttg)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/traj_time",TimeToGoal,self.traj_ttg)

        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_global_plans",HumanPathArray,self.h_global_plan)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_local_plans",HumanPathArray,self.h_local_plan)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_local_trajs",HumanTrajectoryArray,self.h_local_traj)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_plans_time",HumanTimeToGoalArray,self.h_path_ttg)
        rospy.Subscriber("/move_base_node/TebLocalPlannerROS/human_trajs_time",HumanTimeToGoalArray,self.h_traj_ttg)

        rospy.Timer(rospy.Duration(0.1), self.timerCB)
        print("Started")

        self.changed=False

        self.message = "Status: .... "
        self.label_text = StringVar()
        self.label_text.set(self.message)
        self.label = Label(master, textvariable=self.label_text)
        self.label.grid(row=0, column=0,sticky=W)


        self.mode_var = StringVar()
        self.mode = 0
        self.modes = {'TTG_Robot_Traj','TTG_Robot_Path','TTG_Human_Traj','TTG_Human_Path'}
        self.mode_var.set('TTG_Robot_Traj')

        popupMenu = OptionMenu(master, self.mode_var, *self.modes)
        Label(master, text="Choose the attribute to plot").grid(row = 1, column = 0,sticky=W)
        popupMenu.grid(row = 1, column =1,sticky=W)
        self.mode_var.trace('w',self.change_dropdown)


        self.plot_button = Button(master, text="Plot", command=self.plot_graph, state=DISABLED,borderwidth=3)
        self.plot_button.grid(row=4, column=1,sticky=W+E)

        vcmd = master.register(self.avg_num) # we have to wrap the command
        self.entry = Entry(master, validate="key", validatecommand=(vcmd, '%P'),bd=3,width=5)
        Label(master, text="Samples for smoothing (default : 5) ").grid(row = 3, column = 0,sticky=W)
        self.entry.grid(row=3, column=1,sticky=W)

        self.clear_button = Button(master, text="Clear", command=self.clear, state=DISABLED, borderwidth=3)
        self.clear_button.grid(row=4, column=0,sticky=W)

        self.exit_button = Button(master, text="Close", command=self._exit, state=NORMAL, borderwidth=3)
        self.exit_button.grid(row=5, column=0,sticky=W)
        plt.figure()
        # plt.figure(num=None, figsize=(20, 4), dpi=80, facecolor='w', edgecolor='k')

    def update_status(self):
        global one_save_gr

        if(self.changed):
            if(not one_save_gr):
                self.message = "Simulation is running.."
                self.label_text.set(self.message)
                self.plot_button.config(state="disabled")
                self.clear_button.config(state="disabled")
            else:
                self.message = "Ready..you can plot the graphs now"
                self.label_text.set(self.message)
                self.plot_button.config(state="normal")
                self.clear_button.config(state="normal")
        else:
            self.message = "Data is not loaded"
            self.label_text.set(self.message)
            self.plot_button.config(state="disabled")
            self.clear_button.config(state="disabled")

    def plot_graph(self):
        if(self.mode==0):
            self.plot_traj_ttg()
        elif(self.mode==1):
            self.plot_path_ttg()
        elif(self.mode==2):
            self.plot_h_traj_ttg()
        elif(self.mode==3):
            self.plot_h_path_ttg()
        else:
            print("Please select an attribute to plot")

    def avg_num(self, new_text):
        if not new_text: # the field is being cleared
            self.num = None
            return True
        else:
            self.num = int(new_text)
            return True

    def change_dropdown(self,*args):
        'TTG_Robot_Traj','TTG_Robot_Path','TTG_Human_Traj','TTG_Human_Path'
        if(self.mode_var.get() == 'TTG_Robot_Traj'):
            self.mode=0
        elif(self.mode_var.get() == 'TTG_Robot_Path'):
            self.mode=1
        elif(self.mode_var.get() == 'TTG_Human_Traj'):
            self.mode=2
        elif(self.mode_var.get() == 'TTG_Human_Path'):
            self.mode=3
        else:
            self.mode = 0
        # print(self.mode)

    def open_new_plot_window(self, f):
        plt_window = Toplevel(self.master)
        self.PlotWindows.append(plt_window)
        PlotNumber = len(self.PlotWindows)
        # reminder.geometry("100x120+{}+200".format(str(150*windowNumber)))

        self.canvas = FigureCanvasTkAgg(f, master=plt_window)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=TOP, fill=BOTH, expand=1)

        self.toolbar = NavigationToolbar2Tk(self.canvas, plt_window)
        self.toolbar.update()
        self.canvas._tkcanvas.pack(side=TOP, fill=BOTH, expand=1)

        self.canvas.mpl_connect('key_press_event', self.on_key_event)

        def _quit():
            # plt_window.quit()     # stops mainloop
            plt_window.destroy()  # this is necessary on Windows to prevent
                                # Fatal Python Error: PyEval_RestoreThread: NULL tstate
            plt_window.update()

        button = Button(master=plt_window, text='Quit', command=_quit)
        button.pack(side=BOTTOM)



    def on_key_event(self,event):
        print('you pressed %s' % event.key)
        key_press_handler(event, self.canvas, self.toolbar)

    def _exit(self):
        self.master.quit()
        self.master.destroy()

    def smooth(self,y, box_pts):
        box = np.ones(box_pts)/box_pts
        y_smooth = np.convolve(y, box, mode='same')
        return y_smooth

    def traj_ttg(self, msg):
        self.changed=True
        self.data["ttg_traj"].append(msg.time_to_goal)

    def plot_traj_ttg(self):
        n = len(self.data["ttg_traj"])
        x = np.arange(0,n,1)
        y =[]
        for i in range(0,n):
            y.append(self.data["ttg_traj"][i].to_sec())

        yhat = self.smooth(y,self.num)

        f = plt.figure(figsize=(5, 4), dpi=100)
        a = f.add_subplot(111)
        a.plot(x,yhat)
        plt.title('TTG Traj')
        self.open_new_plot_window(f)


    def path_ttg(self, msg):
        self.data["ttg_path"].append(msg.time_to_goal)

    def plot_path_ttg(self):
        n = len(self.data["ttg_path"])
        x = np.arange(0,n,1)

        y =[]
        for i in range(0,n):
            y.append(self.data["ttg_path"][i].to_sec())

        yhat = self.smooth(y,self.num)

        f = plt.figure(figsize=(5, 4), dpi=100)
        a = f.add_subplot(111)
        a.plot(x,yhat)
        plt.title('TTG Path')
        self.open_new_plot_window(f)


    def h_traj_ttg(self, msg):
        self.data["h_ttg_traj"].append(msg.times_to_goal[0].time_to_goal)

    def plot_h_traj_ttg(self):
        n = len(self.data["h_ttg_traj"])
        x = np.arange(0,n,1)

        y =[]
        for i in range(0,n):
            y.append(self.data["h_ttg_traj"][i].to_sec())

        yhat = self.smooth(y,self.num)

        f = plt.figure(figsize=(5, 4), dpi=100)
        a = f.add_subplot(111)
        a.plot(x,yhat)
        plt.title('TTG H_Traj')
        self.open_new_plot_window(f)

    def h_path_ttg(self, msg):
        self.data["h_ttg_path"].append(msg.times_to_goal[0].time_to_goal)

    def plot_h_path_ttg(self):
        n = len(self.data["h_ttg_path"])
        x = np.arange(0,n,1)

        y =[]
        for i in range(0,n):
            y.append(self.data["h_ttg_path"][i].to_sec())

        yhat = self.smooth(y,self.num)

        f = plt.figure(figsize=(5, 4), dpi=100)
        a = f.add_subplot(111)
        a.plot(x,yhat)
        plt.title('TTG H_path')
        self.open_new_plot_window(f)

    def global_plan(self, msg):
        global last_time
        global one_save_gr

        last_time = rospy.Time.now()
        if one_save_gr:
            self.data["g_plan"].append(msg)
            one_save_gr = False

    def h_global_plan(self, msg):
        global last_time
        global one_save_gh

        last_time = rospy.Time.now()
        if one_save_gh:
            self.data["h_g_plan"].append(msg.paths[0])
            one_save_gh = False

    def local_plan(self, msg):
        self.data["l_plan"].append(msg)

    def h_local_plan(self, msg):
        self.data["h_l_plan"].append(msg.paths[0])

    def local_traj(self, msg):
        self.data["l_traj"].append(msg)

    def h_local_traj(self, msg):
        self.data["h_l_traj"].append(msg.trajectories[0])

    def timerCB(self, event):
        self.update_status()
        now = rospy.Time.now()
        global last_time
        global one_save_gh
        global one_save_gr
        if (now-last_time).secs > 2:
            one_save_gr = True
            one_save_gh = True

    def clear(self):
        self.data = { "ttg_traj":[], "ttg_path":[], "h_ttg_traj":[], "h_ttg_path":[], "g_plan":[],"l_plan":[], "l_traj":[], "h_g_plan":[], "h_l_plan":[], "h_l_traj":[]}
        self.changed = False

# def listener():


if __name__=='__main__':
    try:
        root = Tk()
        my_gui = PlotTTG_GUI(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
