#!/usr/bin/env python 

# TESTED RUN
# python3 /home/crslab/catkin_ws/src/real_time_plot/scripts/aces_connect.py /dev/ttyUSB1

import serial
import numpy as np
import time
import scipy.io as sio
import rospy
from std_msgs.msg import String
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class AcesReader():
    def __init__(self, argv):

        self.b_order = sys.byteorder
        self.ser = serial.Serial(port = argv[1],baudrate=115200,bytesize=8,timeout=0.01,stopbits=serial.STOPBITS_ONE)
        print('Connected to: '+self.ser.portstr)
        self.plotting = False
        self.fn_sub = rospy.Subscriber("filename", String, self.callback)
        self.pub = rospy.Publisher('aces_real_time', String, queue_size=10)
        self.t_window = time.time()
        # init plot stuff
        self.plot_tools_init()
        #self.update_plot()

    def load_FRANKA_fingers(self, left_finger_path, right_finger_path):
        LEFT= np.asarray(sio.loadmat(left_finger_path)['ACES_ROBOTIQ_LEFT_LUT'])
        RIGHT = np.asarray(sio.loadmat(right_finger_path)['ACES_ROBOTIQ_RIGHT_LUT'])
        return LEFT, RIGHT


    def plot_tools_init(self):
        self.LEFT_FINGER_MAP, self.RIGHT_FINGER_MAP = self.load_FRANKA_fingers('/home/crslab/panda_sim/aces_finger_not_catkin_pkg/ACES_ROBOTIQ_LEFT_LUT.mat','/home/crslab/panda_sim/aces_finger_not_catkin_pkg/ACES_ROBOTIQ_RIGHT_LUT.mat')
        self.stacked_finger_map = np.vstack((self.LEFT_FINGER_MAP,self.RIGHT_FINGER_MAP))
        self.stacked_finger_map = np.squeeze(self.stacked_finger_map, axis=1)
        self.recorded_nodes = list()
        self.base_LUT_2 = np.asarray(sio.loadmat('/home/crslab/aces_finger_not_catkin_pkg/ROBOTIQ_BASE_LUT_rev2.mat')['L2'])




    def produce_plots(self):
        ######
        #print('Right finger:', len(self.recorded_nodes))
        nodes_2, count_nodes_2 = np.unique(self.recorded_nodes,return_counts=True)

        #left_C_2 = np.zeros((39,1))
        right_C_2 = np.zeros((39,1))
        #print(len(nodes_2))

        # can be optimized further only for one finger
        for n in range(len(nodes_2)):
            # if nodes_2[n] in self.LEFT_FINGER_MAP:
            #     ind = np.where(self.LEFT_FINGER_MAP==nodes_2[n])
            #     left_C_2[ind] = count_nodes_2[n]
            if nodes_2[n] in self.RIGHT_FINGER_MAP:
                ind = np.where(self.RIGHT_FINGER_MAP==nodes_2[n])
                right_C_2[ind] = count_nodes_2[n]


        str_right_C_2 = ''
        for a in right_C_2:
            str_right_C_2 += str(a[0]) + ','

        self.pub.publish(str_right_C_2)



        self.recorded_nodes = list()

    def callback(self, data):
        if self.plotting and data.data == 'stop':
            print("Stop recording")
            self.plotting=False
        else:
            print('Start plotting')
            self.plotting=True

    def bytereader(self, byteobject):
        x = bin(int.from_bytes(byteobject,byteorder=self.b_order)) #python3
        #print('x', x)
        if len(x) == 10:   # negative to have up to 10 values
            x = x[0:2]+x[3:]
            x = 80 - int(x,2)
            isneg = 1
        elif len(x) == 3 and x[-1]=='0':   # For no value zeros
            x = x[:]
            x = int(x,2)
            isneg = 0
        else:
            x = x[0:2]+'0'*(10-len(x))+x[2:]
            x = 80 - int(x,2)
            isneg = 0
        return np.int8(x), isneg

    def update_plot(self):
        self.produce_plots()
        
rospy.init_node('real_time_plotting', anonymous=True)
aces = AcesReader(sys.argv)



while True:

    # read aces
    b = aces.ser.read()
    #print(len(a))\
    #print(str(b))

    add = False
    if rospy.is_shutdown():
        break

    if len(b) > 0:
        val, polarity = aces.bytereader(b)
        add = True

    # plot within specified time
    if time.time() - aces.t_window >= 0.05:
        #print(time.time() - aces.t_window)
        aces.t_window = time.time()# + 0.25
        #print('here')
        #print(add)
        #print(len(aces.recorded_nodes))
        aces.update_plot()
    else:
        if add:
            #print('here2', count)
            aces.recorded_nodes.append(val)

aces.ser.close()   # <- PLEASE RUN THIS AFTER YOU CTRL-C OR STOP. IF NOT YOU NEED TO RESTART YOUR CONSOLE.