import numpy as np
import matplotlib.pyplot as plt

def get_round_points():
    round_path = dict()
    res = float(15)
    diameter = 0.75
    r = diameter/2
    height = 0.5

    x_c = 0
    y_c = 0

    for i in np.arange(res):
        theta = (2 * np.pi) * ((i + 1 )/res)
        print(theta)
        left = i-1
        if (left < 0):
            left = res-1

        right = i+1
        if right > res-1:
            right = 0

        neighbour = (right,left)
        x = x_c + r * np.cos(theta)
        y = y_c + r * np.sin(theta)
        plt.plot(x,y,'+')
        pos = [x_c + r * np.cos(theta), y_c + r * np.sin(theta), height]
        round_path[i] = (pos,neighbour)
    print(round_path)

    return round_path
# plt.show()
# print(round_path)

round_way_points = get_round_points()
def distance(p1, p2):
    return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
def move_towards(start, end):
    #find nearest point to pick
    min_start_dist = 10000
    min_start_ind = 0
    min_end_dist = 10000
    min_end_ind = 0

    for key, value in round_way_points.items():
        # print(key, value)
        p = value[0]
        dist_start = distance(start,p)
        dist_end = distance(end,p)

        if dist_start < min_start_dist:
            min_start_dist = dist_start
            min_start_ind = key

        if dist_end < min_end_dist:
            min_end_dist = dist_end
            min_end_ind = key

        print(p)

    curr_ind = min_start_ind
    curr_node = round_way_points[min_start_ind] #start with curre node
    while curr_ind != min_end_ind:
        break;
        #move arm to the curr node positon
    print(min_start_ind)
    print(min_end_ind)

    #move toward location in a controlled maner without running into
    pass

pick = [0.5, 0.5, 0.05 + 0.1, 3.14, 0, 3.14/4]
place = [-0.5, 0.5, 0.2,  3.14, 0, 0]
move_towards(pick, place)
