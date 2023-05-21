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


class Goto:

    def __init__(self,bot_id,isteamyellow,Dx,Dy):
        self.pub=rospy.Publisher('/grsim_data',gr_Commands, queue_size=100)
        self.Subscriber=rospy.Subscriber('vision',SSL_DetectionFrame,self.Bot_pos, queue_size=1)
        self.bot_id=bot_id
        self.isteamyellow=isteamyellow
        self.Dx=Dx
        self.Dy=Dy
        self.my_x=0
        self.my_y=0
        self.my_orientation=0

        self.prevTimeStamp=0
        self.prevError=None
        self.output=0

    def Bot_pos(self,data):
        
        #print(data)
        print("*********************************************************************")
        #print(data.robots_yellow)
        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("Bot ID:=",self.bot_id)
        if(data.robots_yellow[self.bot_id].confidence != 0):

            #get my current x and y coordiantes + orientation
            self.my_x = data.robots_yellow[self.bot_id].x
            self.my_y = data.robots_yellow[self.bot_id].y
            self.my_orientation=data.robots_yellow[self.bot_id].orientation

            #Dx,Dy are ref. to destination x,y coordinates
            #calling vector2d method --
            point_to_reach = Vector2D(self.Dx, self.Dy) 
            self.bot_pos = Vector2D(int(self.my_x), int(self.my_y))

            distan = self.bot_pos.dist(point_to_reach)/1000
            print("Distance to goal point:= ",distan)
            # maxDisToTurn = distan 
            UPPER_THRESHOLD = 1 # m/s
            LOWER_THRESHOLD = 0.66 # m/s

            if(distan>0.1):
            #############################----PID TEMPLATE-----###############################################
            # Calculate the distance to go i.e. the error and then issue the velocity commands based on the PID gains
                Kp = 0.4
                Kd = 0.04
                Ki = 0
                
                
                # # Controller run time.
                # if t - self.prevTimeStamp < 0.05:
                #     return self.output
                # else:
                currTime = rospy.Time.now()
                currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
                dt = currTime - self.prevTimeStamp

                print(dt,distan)
                #print("deltaTime:= ",dt)]
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
                
                self.pub.publish(command1)

            else:

                # velocity_of_bot = 0
                # # rospy.loginfo("Velocity of the bot (using error) :- "+str(velocity_of_bot))

                # #error_prev = error    
                # angle_of_motion = self.bot_pos.angle(point_to_reach)
                # angle_of_motion = angle_of_motion - self.my_orientation 
                # angle_of_motion += (math.pi*0.5)

                # #print (self.bot_pos, "bot pos", point_to_reach, "destination")
                

                # vx = velocity_of_bot*math.cos(angle_of_motion)
                # vy = velocity_of_bot*math.sin(angle_of_motion)

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
                
                self.pub.publish(command1)

            #print(self.Dx)
            #return data.robots_yellow[self.bot_id].y



rospy.init_node('r_gotopoint_node')
#Goto()
a=Goto(2,1,400,400)

#a=Goto(1,1,-1500,-1500)
#b = Goto(1,1,-400,400)


rospy.spin()


