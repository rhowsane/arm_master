
import numpy as np
import matplotlib.pyplot as plt


def get_LR_ind(i):
    left = i-1
    if (left < 0):
        left = 19

    right = i+1
    if right > 19:
        right = 0

    return right,left

def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def get_round_points():
    round_path = dict()
    res = float(20)
    diameter = 1.25
    r = diameter/2 #diameter of the circle
    height = 0.5 #height of the circle

    x_c = 0
    y_c = 0
    num = 20
    for i in np.arange(num):
        theta = (2 * np.pi) * ((i + 1 )/res)
        right,left = get_LR_ind(i)
        neighbour = [right,left]
        x = x_c + r * np.cos(theta)
        y = y_c + r * np.sin(theta)
        pos = [x_c + r * np.cos(theta), y_c + r * np.sin(theta), height]
        round_path[i] = [pos,neighbour]

    x_thresh = -0.2 #x threshold behind the arm

    #remove illegal points
    to_remove = []
    for key, value in round_path.items():
        if value[0][0] < x_thresh:
            r_i,l_i = get_LR_ind(key)

            #go to thoose values and delete your self
            right_neighbour_list = round_path[r_i][1]
            left_neighbour_list = round_path[l_i][1]
            # print(l_i)
            # print(key)
            # print(r_i)
            #
            # print(left_neighbour_list)
            # print(right_neighbour_list)

            right_neighbour_list.remove(key)
            left_neighbour_list.remove(key)
            to_remove.append(key)

    for i in to_remove:
        # print(i)
        del round_path[i]

    return round_path


def left_or_right(start_ind, end_ind, graph):
    #count number if you go left and count number if you go right, if list length < 2 then no more neighrs you hit wall, select the other

    num_left = 0
    num_right = 0

    #count left first
    curr_ind = start_ind
    while curr_ind != end_ind:
        curr_neigh = graph[curr_ind][1]
        if len(curr_neigh) != 2:
            num_left += 100 #random big number
            break
        curr_ind = curr_neigh[0]
        num_left += 1

    curr_ind = start_ind
    while curr_ind != end_ind:
        curr_neigh = graph[curr_ind][1]
        if len(curr_neigh) != 2:
            num_right += 100 #random big number
            break
        curr_ind = curr_neigh[1]
        num_right += 1

    if num_left < num_right:
        return 0
    else:
        return 1


# round_way_points = get_round_points()
# for key, value in round_way_points.items():
#     plt.plot(value[0][0],value[0][1],'+')
#     print((value[0][0],value[0][1]))
#
# print(left_or_right(1, 16, round_way_points))
# plt.axis('equal')
# plt.show()
