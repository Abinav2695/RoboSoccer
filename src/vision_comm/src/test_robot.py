#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
import time
import socket
import math

from std_msgs.msg import String
from std_msgs.msg import Int16

from geometry import Vector2D

from robot_messages.msg import SSL_DetectionFrame
from robot_messages.msg import SSL_DetectionRobot
from robot_messages.msg import SSL_DetectionRobotList
from robot_messages.msg import SSL_DetectionBall
from robot_messages.msg import SSL_DetectionBluebotArray
from robot_messages.msg import SSL_DetectionYellowbotArray
from robot_messages.msg import gr_Commands
from robot_messages.msg import gr_Robot_Command

ball_pos=Vector2D(0,0)
bot_pos = Vector2D(0,0)
goal_pos = Vector2D(2000,0)
bot_orientation =0.0
PI = 3.14159265358979323

defender_bot = []
ballzz_data=[]
bot_state = 1
orient_check_flag =0
ball_grabbed=0
defend_line = -1100.0
bot_id=0
done_flag=0

defenderRobot = SSL_DetectionRobot()
ballData=SSL_DetectionBall()

msg_publish = rospy.Publisher('/grsim_data', gr_Robot_Command, queue_size=1000)


def callbackForData(data):
   
   # msg= SSL_DetectionFrame()
    defenderRobot = data.robots_blue[9]
    ballData = data.balls[0]

    ball_pos.x  = ballData.x
    ball_pos.y = ballData.y 

    bot_pos.x = defenderRobot.x
    bot_pos.y = defenderRobot.y
    bot_orientation = defenderRobot.orientation

    # print('------------------------------------------')
    # print(bot_pos.x,bot_pos.y)

def follow_ball():
    global bot_pos
    global bot_orientation
    global ball_pos
    global defend_line
    global defenderRobot
    
    defender_bot_command = gr_Robot_Command()
    final_msg = gr_Commands()
    #print("ball_pos :  bot_pos  :",ball_pos.y,bot_pos.y)
    
    #bot_pos.x,bot_pos.y = 600,600
    #ball_pos.x,ball_pos.y = -500,-500
    #bot_orientation=(PI/2)

    print(abs(ball_pos.y - bot_pos.y))
    if(abs(ball_pos.y - bot_pos.y)> 30):
        point_x = defend_line
        point_y = 0.0
        # if(ball_pos.y > 400):
        #     point_y=400
        # elif(ball_pos.y < -400):
        #     point_y= -400
        # else:
        point_y = ball_pos.y

        point_to_reach =  Vector2D(point_x,point_y)  

        
        dist = bot_pos.dist(point_to_reach)

        if(dist>100):
            velocity_of_bot = 0
            if(dist>1000):
                velocity_of_bot=1000
            elif(dist<700):
                velocity_of_bot=700
            else :
                velocity_of_bot=dist
        
            velocity_of_bot*=0.001
            angle_of_motion = bot_pos.angle(point_to_reach)
            angle_of_motion = angle_of_motion - bot_orientation

            
            angle_of_motion +=(math.pi*0.5)
            
            #print(dist)
            velocity_tangent = velocity_of_bot*math.cos(angle_of_motion)
            velocity_normal  = velocity_of_bot*math.sin(angle_of_motion)
            velocity_angular = 0.0
            
            defender_bot_command.id =9
            defender_bot_command.kickspeedx=0
            defender_bot_command.kickspeedz=0
            defender_bot_command.veltangent = velocity_tangent
            defender_bot_command.velnormal =velocity_normal
            defender_bot_command.velangular = velocity_angular
            defender_bot_command.spinner =0
            defender_bot_command.wheelsspeed=0
            defender_bot_command.orientation = 0

            

        else:
            defender_bot_command.id =9
            defender_bot_command.kickspeedx=0
            defender_bot_command.kickspeedz=0
            defender_bot_command.veltangent = 0
            defender_bot_command.velnormal =0
            defender_bot_command.velangular = 0
            defender_bot_command.spinner =0
            defender_bot_command.wheelsspeed=0
            defender_bot_command.orientation = 0
        
    else:
        defender_bot_command.id =9
        defender_bot_command.kickspeedx=0
        defender_bot_command.kickspeedz=0
        defender_bot_command.veltangent = 0
        defender_bot_command.velnormal =0
        defender_bot_command.velangular = 0
        defender_bot_command.spinner =0
        defender_bot_command.wheelsspeed=0
        defender_bot_command.orientation = 0
    
    ##Final message
    # final_msg.timestamp=0  ### no need for now  
    # final_msg.isteamyellow=0 #blueTeam
    # final_msg.robot_commands.append(defender_bot_command)

    msg_publish.publish(defender_bot_command)



def defender_fsm():
    
    rospy.Subscriber('vision',SSL_DetectionFrame,callbackForData)

    ##define publisher topic
    print('started')
    rospy.init_node('defender_new', anonymous=True)
    while not rospy.is_shutdown():   
               
        follow_ball()
        rospy.sleep(0.1)
        

        
        


if __name__ == '__main__':
    defender_fsm()
