#!/usr/bin/env python 
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
        self.ser = serial.Serial(port = argv[1],baudrate=115200,bytesize=8,timeout=0.5,stopbits=serial.STOPBITS_ONE)
        print('Connected to: '+self.ser.portstr)
        self.plotting = False
        self.fn_sub = rospy.Subscriber("filename", String, self.callback)
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
        self.fig, self.ax = plt.subplots(1,1)
        self.im = self.ax.imshow(np.zeros((876,529)))
        plt.ion()
        #plt.show(block=True)


    def produce_plots(self):
        ######
        print('Right finger:', len(self.recorded_nodes))
        nodes_2, count_nodes_2 = np.unique(self.recorded_nodes,return_counts=True)

        left_C_2 = np.zeros((39,1))
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

        right_L_2 = self.base_LUT_2.copy()
        self.to_plot_right_2 = np.zeros((876,529))

        for m in range(len(right_C_2)):
            self.to_plot_right_2[right_L_2==m+1] = right_C_2[m][0]
        self.t_window = time.time() + 0.5
        #plt.imshow(to_plot_right_2, cmap = 'rainbow')
        #plt.colorbar()
        #plt.savefig(self.current_filename+"_right.png")
        #plt.clf()

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
        #self.im.set_array(self.to_plot_right_2)
        #self.fig.canvas.draw()
        self.ax.imshow(self.to_plot_right_2)
        #plt.draw()
        plt.pause(0.00001)

rospy.init_node('real_time_plotting', anonymous=True)
aces = AcesReader(sys.argv)


# def plot_x(msg):
#     global counter
#     if counter % 10 == 0:
#         stamp = msg.header.stamp
#         time = stamp.secs + stamp.nsecs * 1e-9
#         plt.plot(msg.position.y, msg.position.x, '*')
#         plt.axis("equal")
#         plt.draw()
#         plt.pause(0.00000000001)

#     counter += 1
count = 0
while True:

    # read aces
    b = aces.ser.read()

    add = False
    if rospy.is_shutdown():
        break
    if str(b) == 'b\'\'':
        continue

    if len(b) > 0:
        val, polarity = aces.bytereader(b)
        add = True


    # plot within specified time
    if time.time() >= aces.t_window:
        print('here')
        aces.update_plot()
    else:
        if add:
            #print('here2', count)
            count += 1
            aces.recorded_nodes.append(val)

    # else:
    #     print('here2')
    #     aces.recorded_nodes.append(val)

    #plt.ion()
    #plt.show()
    #rospy.spin()
aces.ser.close()   # <- PLEASE RUN THIS AFTER YOU CTRL-C OR STOP. IF NOT YOU NEED TO RESTART YOUR CONSOLE.