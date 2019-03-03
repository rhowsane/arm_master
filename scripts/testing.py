# from samrowan import *
# GoalManager = SamRowan(5,4)
#
# #MANUAL TESTING :
# for i in list(range(10)):
#     print(i)
#     print(GoalManager.get_next_goal_loc())

dropped_brick = True

def check_dropped():
    if dropped_brick:
        holding_brick = False
        # dropped_brick = False
        return True
    return False



succ = check_dropped()
print(succ)
