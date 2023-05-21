#!/usr/bin/env python3
import sys
from robot_messages.msg import SSL_DetectionFrame,SSL_DetectionRobot
#from geometry import Vector2D
#from ctypes import *
from utils_updated.geometry_functions.geometry_functions import Vector2D
import rospy
import math
from robot_messages.msg import gr_Robot_Command, gr_Commands
from std_msgs.msg import String
#from geometry_functions import Vector2D
#from utils import *
import time
import math
import numpy as np
ball_pos=[0,0]
full_data=[0,0]
yellow_bots_Vision_data=[0,0,0,0,0]
blue_bots_Vision_data=[0,0,0,0,0]

class Goto:
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    print(yellow_bots_Vision_data)

    def __init__(self,bot_id,isteamyellow):
        self.bot_id=bot_id
        self.isteamyellow=bool(isteamyellow)
        #self.Dx=Dx
        #self.Dy=Dy
        self.my_x=0
        self.my_y=0
        self.my_orientation=0

        self.prevTimeStamp=0
        self.prevError=0
        self.output=0
        self.followBall=False
        print(self.bot_id)


        #ball_vector= Vector2D(ball_x, ball_y)

    def Bot_pos(self,Dx,Dy):
        global full_data
        global yellow_bots_Vision_data
        global blue_bots_Vision_data
        self.Dx=Dx
        self.Dy=Dy
        print("bot poss fn")

        #print(data)
        #print("*********************************************************************")
        print("bot_pos methord")
        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        
        if self.isteamyellow ==True:
            #print(yellow_bots_Vision_data[1])
            #yellow_bots_Vision_data[1]

            if (yellow_bots_Vision_data[self.bot_id]) != 0:
                print("data presnt")

                self.my_x = yellow_bots_Vision_data[self.bot_id]['x']
                self.my_y = yellow_bots_Vision_data[self.bot_id]['y']
                self.my_orientation=yellow_bots_Vision_data[self.bot_id]['orientation']
                point_to_reach = Vector2D(self.Dx, self.Dy) 

                self.bot_pos = Vector2D(int(self.my_x), int(self.my_y))

                distan = self.bot_pos.dist(point_to_reach)/1000
                print("Distance to goal point:= ",distan)
                # maxDisToTurn = distan 
                UPPER_THRESHOLD = 1 # m/s
                LOWER_THRESHOLD = 0.65 # m/s
                bbt=0
                if(self.followBall==True):
                    bbt=0.25
                if(distan>0.1+bbt):
                #############################----PID TEMPLATE-----###############################################
                # Calculate the distance to go i.e. the error and then issue the velocity commands based on the PID gains
                    Kp = 2
                    Kd = 0.04
                    Ki = 0
                    
                    
                    # # Controller run time.
                    # if t - self.prevTimeStamp < 0.05:
                    #     return self.output
                    # else:
                    currTime = rospy.Time.now()
                    currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
                    dt = currTime - self.prevTimeStamp

                    print("deltaTime:= ",dt)
                    self.prevTimeStamp = currTime
                    # INSERT CODE BELOW

                    # Calculate error.
                    error  = distan
                    # Calculate proportional control output.
                    P_out = error*Kp

                    # Calculate derivative control output.
                    # HINT: Use self.prev_error to store old
                    # error values and dt for time difference.
                    if self.prevError != None:
                        D_out = (error-self.prevError)/dt *Kd
                    else:
                        D_out = 0
                        # Set this to error.
                        self.prevError = error

                    # Calculate final output.
                    self.output = P_out + D_out

                    #print("PID_OUTPUT:=  ",self.output)

                    if(self.output>UPPER_THRESHOLD):
                        self.output=UPPER_THRESHOLD
                    elif (self.output<LOWER_THRESHOLD):
                        self.output=LOWER_THRESHOLD
                    print("PID_OUTPUT:=  ",self.output)
                    #############################----PID TEMPLATE-----###############################################

                    velocity_of_bot = self.output
                    angle_of_motion = self.bot_pos.angle(point_to_reach)
                    angle_of_motion = angle_of_motion - self.my_orientation 
                    angle_of_motion += (math.pi*0.5)

                    vx = velocity_of_bot*math.cos(angle_of_motion)
                    vy = velocity_of_bot*math.sin(angle_of_motion)

                    command = gr_Robot_Command()
                    command.id=self.bot_id
                    command.isteamyellow=1
                    command.kickspeedx=0
                    command.kickspeedz=0
                    command.velangular=0
                    command.velnormal=vy
                    command.veltangent=vx
                    command.spinner=0
                    command.wheelsspeed=0
                    command.orientation=0

                    command1 = gr_Commands()
                    command1.timestamp=0
                    command1.isteamyellow = True
                    command1.robot_commands = command
                    pub.publish(command1)

                else:

                    command = gr_Robot_Command()
                    command.id=self.bot_id
                    command.isteamyellow=1
                    command.kickspeedx=0
                    command.kickspeedz=0
                    command.velangular=0
                    command.velnormal=0
                    command.veltangent=0
                    command.spinner=0
                    command.wheelsspeed=0
                    command.orientation=0

                    command1 = gr_Commands()
                    command1.timestamp=0
                    command1.isteamyellow = True
                    command1.robot_commands = command
                    pub.publish(command1)
        elif self.isteamyellow ==False:
            #print(yellow_bots_Vision_data[1])
            #yellow_bots_Vision_data[1]

            if (blue_bots_Vision_data[self.bot_id]) != 0:
                print("data presnt")

                self.my_x = blue_bots_Vision_data[self.bot_id]['x']
                self.my_y = blue_bots_Vision_data[self.bot_id]['y']
                self.my_orientation=blue_bots_Vision_data[self.bot_id]['orientation']
                point_to_reach = Vector2D(self.Dx, self.Dy) 

                self.bot_pos = Vector2D(int(self.my_x), int(self.my_y))

                distan = self.bot_pos.dist(point_to_reach)/1000
                bbt=0
                if(self.followBall==True):
                    bbt=0.25
                print("Distance to goal point:= ",distan)

                distan=distan-bbt
                # maxDisToTurn = distan 
                UPPER_THRESHOLD = 1 # m/s
                LOWER_THRESHOLD = 0.65 # m/s

                if(distan>0.1):
                #############################----PID TEMPLATE-----###############################################
                # Calculate the distance to go i.e. the error and then issue the velocity commands based on the PID gains
                    Kp = 2
                    Kd = 0.04
                    Ki = 0
                    
                    
                    # # Controller run time.
                    # if t - self.prevTimeStamp < 0.05:
                    #     return self.output
                    # else:
                    currTime = rospy.Time.now()
                    currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
                    dt = currTime - self.prevTimeStamp

                    print("deltaTime:= ",dt)
                    self.prevTimeStamp = currTime
                    # INSERT CODE BELOW

                    # Calculate error.
                    error  = distan
                    # Calculate proportional control output.
                    P_out = error*Kp

                    # Calculate derivative control output.
                    # HINT: Use self.prev_error to store old
                    # error values and dt for time difference.
                    if self.prevError != None:
                        D_out = (error-self.prevError)/dt *Kd
                    else:
                        D_out = 0
                        # Set this to error.
                        self.prevError = error

                    # Calculate final output.
                    self.output = P_out + D_out

                    #print("PID_OUTPUT:=  ",self.output)

                    if(self.output>UPPER_THRESHOLD):
                        self.output=UPPER_THRESHOLD
                    elif (self.output<LOWER_THRESHOLD):
                        self.output=LOWER_THRESHOLD
                    print("PID_OUTPUT:=  ",self.output)
                    #############################----PID TEMPLATE-----###############################################

                    velocity_of_bot = self.output
                    # rospy.loginfo("Velocity of the bot (using error) :- "+str(velocity_of_bot))

                    #error_prev = error    
                    angle_of_motion = self.bot_pos.angle(point_to_reach)
                    angle_of_motion = angle_of_motion - self.my_orientation 
                    angle_of_motion += (math.pi*0.5)

                    #print (self.bot_pos, "bot pos", point_to_reach, "destination")
    
                    vx = velocity_of_bot*math.cos(angle_of_motion)
                    vy = velocity_of_bot*math.sin(angle_of_motion)

                    command = gr_Robot_Command()
                    command.id=self.bot_id
                    command.isteamyellow=0
                    command.kickspeedx=0
                    command.kickspeedz=0
                    command.velangular=0
                    command.velnormal=vy
                    command.veltangent=vx
                    command.spinner=0
                    command.wheelsspeed=0
                    command.orientation=0
                    
                    pub.publish(command)

                else:
                    command = gr_Robot_Command()
                    command.id=self.bot_id
                    command.isteamyellow=0
                    command.kickspeedx=0
                    command.kickspeedz=0
                    command.velangular=0
                    command.velnormal=0
                    command.veltangent=0
                    command.spinner=0
                    command.wheelsspeed=0
                    command.orientation=0  
                    pub.publish(command)
    def follw_ball(self):

        global ball_pos
        ball_x=ball_pos[0]
        ball_y=ball_pos[1]
        self.followBall=True
        # bot_to_ball_threshold=0.6
        self.Bot_pos(ball_x,ball_y) 

    def path_planning(self):

        '''if self.plan_data == None:
            return
        
        if not self.field_update:
            return'''

        blue_bots = self.plan_data.robots_blue
        yellow_bots = self.plan_data.robots_yellow

        obstacleList = []
        start = self.bot_pos.x, self.bot_pos.y

        for bot in blue_bots:
            if bot.confidence >= 0.01:
                if bot.robot_id == self.bot_id:
                    pass
                else:
                    obstacleList.append((bot.x, bot.y, 250))
        for bot in yellow_bots:
            if bot.confidence >= 0.01:
                if bot.robot_id == self.bot_id:
                    pass
                else:
                    obstacleList.append((bot.x, bot.y, 250))
                    
        goal = (self.plan_data.balls[0].x, self.plan_data.balls[0].y) 

        return rrt.main(obstacleList, start, goal)




    def blow_obstacle_with_inflation(self, _map, point, x_limits, y_limits):


        point_in_map = self.rw_to_map(point[0], point[1], x_limits, y_limits)
        rospy.loginfo(" Point in map :- " + str(point_in_map))

        for i in range(int(point_in_map[0]-300), int(point_in_map[0]+300)):
            for j in range(int(point_in_map[1]-300), int(point_in_map[1]+300)):
                if i < (x_limits[1]-x_limits[0]) and j < (y_limits[1]-y_limits[0]):
                    _map[i][j] = 1

        return _map

    def rw_to_map(self, x, y, x_limits, y_limits):
        x += -1*x_limits[1]
        y += -1*y_limits[0]
        return (x, y)


         #print(self.Dx)
    def rrt(self):
        global yellow_bots_Vision_data
        global blue_bots_Vision_data
        ballVec = Vector2D(ball_pos[0],ball_pos[1])
        botVec = Vector2D(self.my_x , self.my_y)
        print(botVec)
        angle_to_go = botVec.angle(ballVec)
        print(angle_to_go)
        blue_bots = blue_bots_Vision_data
        yellow_bots = yellow_bots_Vision_data

        x_limits = [-1800, 1800]
        y_limits = [-1256, 1256]
        import numpy as np
        field_map = np.zeros(((x_limits[1]-x_limits[0]) ,(y_limits[1]-y_limits[0])), dtype=np.uint8)
        print(field_map)
        #if self.isteamyellow is True:
        for i in range(0,7):
            if yellow_bots_Vision_data[i] !=0:
                start=yellow_bots_Vision_data[i]['pixel_x'],yellow_bots_Vision_data[i]['pixel_y']
                field_map = self.blow_obstacle_with_inflation(field_map, [yellow_bots_Vision_data[i]['x'],yellow_bots_Vision_data[i]['y']], x_limits, y_limits)
            if blue_bots_Vision_data[i] !=0:
                start=blue_bots_Vision_data[i]['pixel_x'],blue_bots_Vision_data[i]['pixel_y']
                field_map = self.blow_obstacle_with_inflation(field_map, [blue_bots_Vision_data[i]['x'],blue_bots_Vision_data[i]['y']], x_limits, y_limits)
        #goal = self.plan_data.balls[0]
        #self.path_planner(start, goal, field_map)

            
        '''
        for bot in yellow_bots:
            if self.team == "yellow" and bot.robot_id == self.bot_id:
                start = bot.pixel_x, bot.pixel_y
            else:
                field_map = self.blow_obstacle_with_inflation(field_map, [bot.x, bot.y], x_limits, y_limits) 
        #pass
        '''


def Bot_pos_callback(data):
    #print("dddd")
    global full_data
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    #print(data.balls[0].confidence)
    if data.balls[0].confidence > 0:
        ball_pos[0]=data.balls[0].x
        ball_pos[1]=data.balls[0].y

    #print("in callback  function") 
    for i in range(0,5):
        #change list initialization
        if data.robots_yellow[i].confidence >0.5:
            #yellow_bots_Vision_data[i]=data.robots_yellow[i]
            yellow_bots_Vision_data[i]={"x":data.robots_yellow[i].x,"y":data.robots_yellow[i].y,"orientation":data.robots_yellow[i].orientation,"pixel_x":data.robots_yellow[i].pixel_x,"pixel_y":data.robots_yellow[i].pixel_y}
        if data.robots_blue[i].confidence >0.5:
            blue_bots_Vision_data[i]={"x":data.robots_blue[i].x,"y":data.robots_blue[i].y,"orientation":data.robots_blue[i].orientation,"pixel_x":data.robots_blue[i].pixel_x,"pixel_y":data.robots_blue[i].pixel_y}
        full_data=[yellow_bots_Vision_data,blue_bots_Vision_data] #not using for  now 
    #b=Goto(3,1,400,-400)
    #b.Bot_pos()
    #c=Goto(2,0)
    #c.Bot_pos(-400,-700)otopoint_output
    #a=Goto(1,1)
    #a.Bot_pos(400,0)
    #a.rrt()
    if a is not None:
        a.follw_ball()
    #b=Goto(3,1)
    #b.Bot_pos(500,600)
    #a.follw_ball()

    #d=Goto(0,1,-400,400)
    #d.Bot_pos()
     #print("fulldata is",full_data)

    '''fulldata is [[{'y': 387.64764404296875, 'orientation': -3.0414834022521973, 'x': 387.56536865234375}, 
    {'y': -459.1063537597656, 'orientation': -0.872165322303772, 'x': -379.7416076660156}, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]
'''
    
        
    #print(yellow_bots_Vision_data[1])

    #yellow_data=data.robots_yellow
    

    #print(full_data)




#Goto()
#print(sys.argv[0], sys.argv[1])
#to take argumnts from key borad --botid isteamyellow dx dy  --seprated by space
#print(sys.argv[1],sys.argv[2],sys.argv[3],sys.argv[4])
a=None

#a=Goto(int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]))
#a.Bot_pos()
#while 1:
#b=Goto(1,1,400,-400)
#b.Bot_pos()
if __name__=='__main__':
    rospy.init_node('random_node')
    Subscriber=rospy.Subscriber('vision',SSL_DetectionFrame,Bot_pos_callback, queue_size=2)
    pub=rospy.Publisher('/autoOutput',gr_Commands, queue_size=2)

    
    a=Goto(1,1)
    rospy.spin()
