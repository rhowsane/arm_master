class SamRowan():
    def __init__(self, num_brick, height):
        print("Instantiating")
        self.locations = self.generate_simple_wall(num_brick, height)
        print("Generated Wall")
        self.pointer = 0
        self.h = height
        self.n = num_brick

    def get_next_goal_loc(self): #TODO DEAL WITH OVERFLOW CASE
        loc = self.locations[self.pointer]
        self.pointer += 1
        return loc

    def generate_simple_wall(self, num_brick=20, height=4): #COPY PASTE SAMS FUNCTINO IN HERE AND RETURN THE DESIRED VALUE

        xpickup=0.5
        ypickup=0.5
        xpicktheta=0
        ypicktheta=0
        zpicktheta=0

        if ypickup >= 0:
            xstart = -0.5
            ystart = -0.5
        else:
            xstart = -0.5
            ystart = 0.5

        zstart=0.2 #add a small offset
        xtheta= 3.14 #THIS MEANS THE GRIPPER WILL BE FACING downward
        ytheta=0
        ztheta= 3.14/4 #AT THIS VALUE THE GRIPPER IS straight with respect to base

        blength = 0.2+0.005                                                       #geometries of the brick
        bwidth = 0.09+0.005
        bheight = 0.062

        bstart=[xstart,ystart,zstart,xtheta,ytheta,ztheta]     #first brick position mirrors position of where we place the bricks
        pos_list = []

        # input_nos = input()
        input_nos = num_brick #Pass in from class variable
        brick_number = int(input_nos)                                                 #number of bricks being placed into the wall
        n = 1

        xnos = 0
        znos = 0

        # input_height = input()
        input_height = height #preset for now
        wall_height = int(input_height)

        if wall_height == int(brick_number/wall_height):
            removal = 0
        else:
            removal = round(wall_height-(brick_number/wall_height - int(brick_number/wall_height))*wall_height)

        round_up = brick_number+removal
        width = (round_up/wall_height)-1

        for i in range(1, int(round_up)):

            pos_list.append([round(bstart[0]+xnos*blength, 3),bstart[1],round(bstart[2]+znos*bheight, 3),bstart[3],bstart[4],bstart[5]]) #edit this so the alignment is always correct

            if xnos < width:
                xnos+=1
            else:
                xnos=0
                znos+=1

        pos_final = pos_list[:brick_number]

        print(pos_final)

        return pos_final
