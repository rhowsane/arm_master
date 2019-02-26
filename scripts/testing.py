from samrowan import *
GoalManager = SamRowan(20,4)

#MANUAL TESTING :
for i in list(range(10)):
    print(i)
    print(GoalManager.get_next_goal_loc())
