#!/usr/bin/env python

# rosrun rolling fc_t_plot.py _filepath:=PATH/TO/DATA/FROM/CURRENT/DIRECTORY

import rospy
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

rospy.init_node("fc_t_plot")

filepath = rospy.get_param("~filepath", "")

f = open(filepath)
line = f.read().split('\n')

tau_x_list = []
tau_y_list = []
tau_z_list = []

for i in range(len(line) - 2): # modify offset according to input data
    line_i  = line[i+1].split(" ")
    tau_x_list.append(float(line_i[0]))
    tau_y_list.append(float(line_i[1]))
    tau_z_list.append(float(line_i[2]))

print(len(tau_x_list))

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title("Feasible Control Torque", size=20)
ax.set_xlabel("tau_x", size=14)
ax.set_ylabel("tau_y", size=14)
ax.set_zlabel("tau_z", size=14)
ax.set_xlim(-3.0, 3.0)
ax.set_ylim(-3.0, 3.0)
ax.set_zlim(-3.0, 3.0)
ax.scatter(tau_x_list, tau_y_list, tau_z_list, s=0.1)
plt.show()


