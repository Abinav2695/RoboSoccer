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
from rrt_planner import RRT

#from geometry_functions import Vector2D
#from utils import *
import time
import math
import numpy as np
ball_pos=[0,0]
full_data=[0,0]
yellow_bots_Vision_data=[None]*6
blue_bots_Vision_data=[None]*6
ROTATION_FACTOR = 0.1745/2 ##5 deg 
robotRadius=100
instanceList=[]
counter=0

class Goto:
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    print(yellow_bots_Vision_data)


    def __init__(self,Dx,Dy,bot_id,isteamyellow):
        self.bot_id=bot_id
        self.isteamyellow=bool(isteamyellow)
        #self.Dx=Dx
        #self.Dy=Dy
        self.my_x=0
        self.my_y=0
        self.my_orientation=0

        self.prevTimeStamp=0
        self.prevError=None
        self.prevErrorRotation=None
        self.output=0
        self.outputRotation=0
        self.integral=0
        self.windup=2
        self.nextWayPoint=0
        self.replan=False
        self.planned_path =None
        self.maxWayPoints=None
        self.reached=False
        self.reachedGoal=False
        self.orientFlag=False
        self.Dx=Dx
        self.Dy=Dy
        self.followBall=False
        self.botBallThreshold=0
        print(self.bot_id)


        #ball_vector= Vector2D(ball_x, ball_y)
    def CollisionCheck1(self,nextPoint, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - nextPoint.x
            dy = oy - nextPoint.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= 2.2*size:
                return False

        return True
    
    def LineCollisionCheck1(self,first, second, obstacleList):

        x1 = first.x
        y1 = first.y
        x2 = second.x
        y2 = second.y

        try:
            a = y2 - y1
            b = -(x2 - x1)
            c = y2 * (x2 - x1) - x2 * (y2 - y1)
        except ZeroDivisionError:
            return False

        for (ox, oy, size) in obstacleList:
            d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
            if d <= (2.3*size):
                return False

        return True

    def Bot_pos(self,OrientPoint,OrientFlag=False):
       
        #print("bot poss fn")
        global full_data
        global yellow_bots_Vision_data
        global blue_bots_Vision_data
        global ball_pos
        
        #print(data)
        #print("*********************************************************************")
        #print("bot_pos method")
        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        
        if self.isteamyellow ==True:
            #print(yellow_bots_Vision_data[1])
            #yellow_bots_Vision_data[1]

            if (yellow_bots_Vision_data[self.bot_id]) != 0:
                #print("data presnt")

                self.my_x = yellow_bots_Vision_data[self.bot_id]['x']
                self.my_y = yellow_bots_Vision_data[self.bot_id]['y']
                self.my_orientation=yellow_bots_Vision_data[self.bot_id]['orientation']
                self.my_team='Yellow'
                #point_to_reach = Vector2D(self.Dx, self.Dy) 
                self.bot_pos = Vector2D(int(self.my_x), int(self.my_y))
                final_point_to_reach = Vector2D(self.Dx,self.Dy)
                print(final_point_to_reach.x,final_point_to_reach.y)
                point_to_reach=0
                distan=0
                
                ##################Obstacle List############
                if(self.reachedGoal==False):
                    obstacleList = []
                        
                    iterator=0
                    for bot in blue_bots_Vision_data:           
                        if bot is not None:
                            if iterator == self.bot_id and self.my_team=='Blue':
                                pass
                            else:
                                obstacleList.append((bot['x'], bot['y'], robotRadius))
                            
                        iterator+=1

                    iterator=0
                    for bot in yellow_bots_Vision_data:           
                        if bot is not None:
                            if iterator == self.bot_id and self.my_team=='Yellow':
                                pass
                            else:
                                obstacleList.append((bot['x'], bot['y'], robotRadius))
                        iterator+=1
                    
                    if(self.planned_path!=None):
                        point_to_reach = Vector2D(self.planned_path[self.nextWayPoint][0],
                                                        self.planned_path[self.nextWayPoint][1])

                        if (self.CollisionCheck1(self.bot_pos,obstacleList)==False):
                            self.replan=True
                        
                        
                        # if (self.LineCollisionCheck1(self.bot_pos,point_to_reach,obstacleList)==False):
                        #     self.replan=True
                        

                    
                    ###path Planner addition#############
                    if(self.replan or self.planned_path==None):

                        start = self.my_x,self.my_y
                        goal = (final_point_to_reach.x,final_point_to_reach.y)

                        plan_ok=0
                        while(plan_ok<1):
                            self.planned_path = newPath.update(obstacleList=obstacleList,start=start,goal=goal)
                            if(len(self.planned_path)<10):
                                plan_ok+=1

                        self.planned_path = self.planned_path[::-1]
                        self.planned_path = self.planned_path[1:]
                        self.maxWayPoints =len(self.planned_path)
                        self.nextWayPoint=0

                        point_to_reach = Vector2D(self.planned_path[self.nextWayPoint][0],
                                                    self.planned_path[self.nextWayPoint][1])

                        print(goal)
                        print(self.planned_path)
                        print("Start Point: ",start)
                        print("Point to Reach: ",point_to_reach.x,point_to_reach.y)
                        print("Max Way Points: ",self.maxWayPoints)
                        self.replan=False
                    ##############################################
                    

                    distan = self.bot_pos.dist(point_to_reach)/1000
                #print("Distance to goal point:= ",distan)
                #print("Current waypoint:-",self.nextWayPoint)
                # maxDisToTurn = distan 
                UPPER_THRESHOLD = 1 # m/s
                LOWER_THRESHOLD = 0.65 # m/s

                UPPER_THRESHOLD_FOR_ROTATION = 2.5#rad/s
                LOWER_THRESHOLD_FOR_ROTATION = 1


                currTime = rospy.Time.now()
                currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)


                dt = currTime - self.prevTimeStamp

                if(dt>0.01):


                    #print("deltaTime:= ",dt)
                    self.prevTimeStamp = currTime
                    #print("Self Orientation: ",self.my_orientation)
                    vw=0
                    vx=0
                    vy=0
                    if(self.reachedGoal and self.orientFlag==False):
                        
                        angleOfOrientation = self.bot_pos.angle(OrientPoint)    # Vector2D(ball_pos[0],ball_pos[1])
                        # print("Orient1 : ",angleOfOrientation)              
                        angleOfOrientation = angleOfOrientation - self.my_orientation
                        # print("Orient2 : ",angleOfOrientation)   
                        
                        angleOfOrientation  = self.bot_pos.normalizeAngle(angleOfOrientation)
                        if(abs(angleOfOrientation)<ROTATION_FACTOR):
                            self.orientFlag=True
                            vw=0
                        else:
                        
                            ###Calculate vw based on PID output
                            Kpr = 0.4
                            Kir = 0.07
                            Kdr = 0.001

                            rot_error = angleOfOrientation
                            P_OUT_ROT = rot_error*Kpr

                            #print("P_OUT:=  ",P_OUT_ROT)

                            self.integral+= rot_error*dt
                            if(self.integral>self.windup):
                                self.integral=self.windup
                            elif(self.integral<-1*self.windup):
                                self.integral=self.windup*-1

                            I_OUT_ROT = self.integral*Kir

                            if self.prevErrorRotation != None:
                                D_OUT_ROT = (rot_error-self.prevErrorRotation)/dt *Kdr
                            else:
                                D_OUT_ROT = 0
                                # Set this to error.
                                self.prevErrorRotation =  rot_error

                            #print(rot_error-self.prevErrorRotation)
                            #print("P_OUT:= {} ;I_OUT:= {} ; D_OUT:= {} ;".format(P_OUT_ROT,I_OUT_ROT,D_OUT_ROT))

                            # Calculate final output.
                            self.outputRotation = P_OUT_ROT + D_OUT_ROT + I_OUT_ROT
                            
                            abs_value = abs(self.outputRotation)
                            
                            

                            if(abs_value>UPPER_THRESHOLD_FOR_ROTATION):
                                abs_value=UPPER_THRESHOLD_FOR_ROTATION
                            elif (abs_value<LOWER_THRESHOLD_FOR_ROTATION):
                                abs_value=LOWER_THRESHOLD_FOR_ROTATION
                            
                            if(self.outputRotation<0):
                                abs_value = abs_value*(-1)

                            self.outputRotation = abs_value
                            
                            vw=self.outputRotation
                            #vw=0
                            #print("Difference in Orientation : ",angleOfOrientation)
                            #print("VW:=  ",self.outputRotation)
                            



                    if(distan>0.1):
                        
                    #############################----PID TEMPLATE-----###############################################
                    # Calculate the distance to go i.e. the error and then issue the velocity commands based on the PID gains
                        Kp = 2
                        Kd = 0.05
                        Ki = 0
                        
                        error  = distan
                        # Calculate proportional control output.
                        P_out = error*Kp

                        # Calculate derivative control output.
                    
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
                        #print("PID_OUTPUT:=  ",self.output)
                        #############################----PID TEMPLATE-----###############################################

                        velocity_of_bot = self.output
                        angle_of_motion = self.bot_pos.angle(point_to_reach)
                        angle_of_motion = angle_of_motion - self.my_orientation 
                        angle_of_motion += (math.pi*0.5)

                        vx = velocity_of_bot*math.cos(angle_of_motion)
                        vy = velocity_of_bot*math.sin(angle_of_motion)

                    if(distan>0.1+self.botBallThreshold):
                        
                        command = gr_Robot_Command()
                        command.id=self.bot_id
                        command.isteamyellow=1
                        command.kickspeedx=0
                        command.kickspeedz=0
                        command.velangular=vw
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
                    
                    elif(self.orientFlag==False and self.reachedGoal==True):
                        command = gr_Robot_Command()
                        command.id=self.bot_id
                        command.isteamyellow=1
                        command.kickspeedx=0
                        command.kickspeedz=0
                        command.velangular=vw
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
                        if(self.reachedGoal==False):
                            self.nextWayPoint+=1
                            if(self.nextWayPoint==self.maxWayPoints-1 and self.followBall==True):
                                self.botBallThreshold=0.1
                            else:
                                self.botBallThreshold=0
                        #print('idhar aya')
                        
                        if(self.nextWayPoint>=self.maxWayPoints):
                            #self.planned_path=None
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
                            self.reachedGoal=True
                            print("bot id {} is reached {} {}pos".format(self.bot_id,self.Dx,self.Dy))
            
        elif self.isteamyellow ==False:
            #print(yellow_bots_Vision_data[1])
            #yellow_bots_Vision_data[1]

            if (blue_bots_Vision_data[self.bot_id]) != 0:
                #print("data presnt")

                self.my_x = blue_bots_Vision_data[self.bot_id]['x']
                self.my_y = blue_bots_Vision_data[self.bot_id]['y']
                self.my_orientation=blue_bots_Vision_data[self.bot_id]['orientation']
                point_to_reach = Vector2D(self.Dx, self.Dy) 

                self.bot_pos = Vector2D(int(self.my_x), int(self.my_y))

                distan = self.bot_pos.dist(point_to_reach)/1000
                print("Distance to goal point:= ",distan)
                # maxDisToTurn = distan 
                UPPER_THRESHOLD = 1 # m/s
                LOWER_THRESHOLD = 0.65 # m/s

                if(distan>0.1):
                    self.state=0
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
                    self.state=1
                    print("bot id {} is reached {} {}pos".format(self.bot_id,self.Dx,self.Dy))
                    return True
                    

    def follw_ball(self):

        global ball_pos
        ball_x=ball_pos[0]
        ball_y=ball_pos[1]
        bot_to_ball_threshold=0.3
        self.Bot_pos(ball_x,ball_y) 
        #print(self.Dx)

    def updateGoal(self,dx,dy):
        self.Dx=dx
        self.Dy=dy


def Bot_pos_callback(data):
#print("dddd")
    global a,b,c,d
    global full_data
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    
    global instanceList
    global counter
   
    counter+=1
    #print(data.balls[0].confidence)
    if data.balls[0].confidence > 0:
        ball_pos[0]=data.balls[0].x
        ball_pos[1]=data.balls[0].y
    #print(ball_pos)
    #print("in callback  function") 
    for i in range(0,6):
        #change list initialization
        if data.robots_yellow[i].confidence >0.8:
            #yellow_bots_Vision_data[i]=data.robots_yellow[i]
            yellow_bots_Vision_data[i]={"x":data.robots_yellow[i].x,"y":data.robots_yellow[i].y,"orientation":data.robots_yellow[i].orientation}
        if data.robots_blue[i].confidence >0.8:
            blue_bots_Vision_data[i]={"x":data.robots_blue[i].x,"y":data.robots_blue[i].y,"orientation":data.robots_blue[i].orientation}
        full_data=[yellow_bots_Vision_data,blue_bots_Vision_data] #not using for  now 

    
    if(counter>=100):
        
        if(instanceList==None):
            pass
        else:

            for ist in instanceList:
                
                if(ist.reachedGoal==False or ist.orientFlag==False):
                    ist.Bot_pos(Vector2D(0,0),True)
                    if(ist.followBall==True):
                        ist.updateGoal(ball_pos[0],ball_pos[1])
                
                # elif(ist.reachedGoal==True and ist.followBall==True):
                #     ist.reachedGoal=False
                #     ist.orientFlag=False
                #     ist.planned_path=None
                else:
                    instanceList=None
    #else:
        #print('Goal Reached')
        



a = Goto(-1000,600,1,1)
b = Goto(-1300,0,4,1)
c = Goto(500,500,3,1)

#instanceList =[a,b,c]
instanceList = None



newPath = RRT()




if __name__=="__main__":
    
    rospy.init_node('r_gotopoint_node_multiple',anonymous=False)
    rospy.Subscriber('vision',SSL_DetectionFrame,Bot_pos_callback)
    pub=rospy.Publisher('/grsim_data',gr_Commands, queue_size=10)
    
    while not rospy.is_shutdown():
        #in1 = int(input())

        
        # if(in1==1):
        print('single loop')
        time.sleep(1)
        a.followBall=False
        instanceList = [a]
        print(instanceList)

        time.sleep(8)
        # elif(in1==2):
        b.followBall=False
        instanceList = [b]
        
        time.sleep(5)
        # elif(in1==3):
        c.followBall=False
        instanceList = [c]

        time.sleep(5)
        print('one loop done')
        # # elif(in1==4):
        # a.followBall=True
        # instanceList= [a]
            
        
    
    #print(instanceList)

    r#ospy.spin()
