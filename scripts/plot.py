#!/usr/bin/env python 

# PROPER TESTED RUN:
# rosrun real_time_plot plot.py

import numpy as np
from matplotlib import pyplot as plt
import rospy
import scipy.io as sio
from std_msgs.msg import String
from matplotlib import animation

base_LUT_2 = np.asarray(sio.loadmat('/home/crslab/aces_finger_not_catkin_pkg/ROBOTIQ_BASE_LUT_rev2.mat')['L2'])
to_plot_right_2 = np.zeros((876,529))

fig = plt.figure()
fig.canvas.set_window_title('Tactile Spikes')
fig.patch.set_facecolor('white')
ax = plt.axes()
im = ax.imshow(np.zeros((876,529)), vmin=0, vmax=20)
#plt.colorbar(im, ax=ax)
ax.axis('off')
# initialization function: plot the background of each frame
def init():
    global im
    im.set_array(np.zeros((876,529)))
    return im,

# animation function.  This is called sequentially
def animate(i):
    print(i)
    global im, to_plot_right_2
    #print(np.unique(to_plot_right_2, return_counts=True))
    im.set_array(to_plot_right_2)
    return im,

def get_vector(str_x):
    x = str_x.split(',')

    res = np.zeros((39,1))
    for i in range(39):
        res[i,0] = float(x[i])
    return res

def plot_x(msg):
    global base_LUT_2, to_plot_right_2

    right_L_2 = base_LUT_2.copy()
    right_C_2 = get_vector(msg.data)
    
    for m in range(len(right_C_2)):
        to_plot_right_2[right_L_2==m+1] = right_C_2[m][0]

    #plt.imshow(to_plot_right_2)
    #plt.axis("equal")
    #plt.draw()
    #plt.pause(0.001)


if __name__ == '__main__':

    rospy.init_node("plotter")
    rospy.Subscriber("aces_real_time", String, plot_x)

    anim = animation.FuncAnimation(fig, animate, init_func=init, interval=10, blit=True)
    plt.show()
    rospy.spin()