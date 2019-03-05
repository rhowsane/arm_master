from samrowan import *
import time
GoalManager = SamRowan(5,4)

#MANUAL TESTING :
for i in list(range(10)):
    new = False
    print("placed: ", i)
    while not new:
        loc, new = GoalManager.get_next_goal_loc(i)
        print("waiting")
        time.sleep(0.1)
# dropped_brick = True
#
# def check_dropped():
#     if dropped_brick:
#         holding_brick = False
#         # dropped_brick = False
#         return True
#     return False
#
#
#
# succ = check_dropped()
# print(succ)
