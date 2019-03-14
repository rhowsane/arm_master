import numpy as np
import sys
import os
import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file = os.path.expanduser('~/Dropbox/loc.txt')
f  = open(file, "w")
goals = []



b_width = 0.086
b_length = 0.192
b_depth = 0.62
b_space = 0.192 - (b_width * 2)
tol = 0.03

brick1 = [0.5, 0.25, 0.116, 0, 0, 1.57]
# brick2 = [0, 0, b_length+b_width, 0, 0, 0]
# brick3 = [0, 0, 0, 0, 0, 0]

#---------------------------- DEFINE OFFSETS HERE

layer_num = 5
for i in range(layer_num): #FirstLayer
    c_brick = copy.deepcopy(brick1)
    c_brick[1] -= i * (b_width + b_space)
    goals.append(c_brick)

layer_num -= 1
brick1[2] += b_width + tol #'shift up brick'
brick1[1] -= b_length/2
for i in range(layer_num): #FirstLayer
    c_brick = copy.deepcopy(brick1)
    c_brick[1] -= i * (b_width + b_space)
    goals.append(c_brick)

layer_num -= 1
brick1[2] += b_width + tol #'shift up brick'
brick1[1] -= b_length/2
for i in range(layer_num): #FirstLayer
    c_brick = copy.deepcopy(brick1)
    c_brick[1] -= i * (b_width + b_space)
    goals.append(c_brick)


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for goal in goals:
    ax.scatter(goal[0],goal[1],goal[2])
plt.show()
print(goals)

for g in goals:
    for p in g:
        f.write(str(p))
        f.write(",")
