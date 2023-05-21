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



import sys
#sys.path.append('/opt/ros/noetic/lib/python3/dist-packages/')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from robot_messages.msg import SSL_DetectionFrame
from robot_messages.msg import SSL_DetectionRobot
from robot_messages.msg import SSL_DetectionRobotList
from robot_messages.msg import SSL_DetectionBall
from robot_messages.msg import SSL_DetectionBluebotArray
from robot_messages.msg import SSL_DetectionYellowbotArray
from robot_messages.msg import point_2d
#from Robot_Messages.msg import 

import sslclient
import math
import time

import numpy as np

num_cam=2
grsim_port =10020
ssl_vision_port =10006
use_grsim_vision = 0
net_address = '224.5.23.2'
MAX_BOTS_PER_TEAM=6
TOTAL_NO_OF_CAMERAS=2  #4 for grSIm and 2 for ssl vision

def data_grabber():
    global num_cam
    global grsim_port
    global ssl_vision_port
    global net_address
    global use_grsim_vision


    arguments = rospy.myargv(sys.argv)

    if(len(arguments)>1):
        use_grsim_vision = int(arguments[1])
        
    #initialising ROS nodes
    rospy.init_node('vision_node', anonymous=False)
    rate = rospy.Rate(5) # 1hz

    ##define publisher topic
    vision_data_publisher = rospy.Publisher('/vision', SSL_DetectionFrame, queue_size=1)
    ball_data_publisher = rospy.Publisher('/ballposition',point_2d,queue_size=2)

    max_ball_confidence = 0
    #define null array
    max_blue_bot_confidence =[0]*MAX_BOTS_PER_TEAM
    max_yellow_bot_confidence =[0]*MAX_BOTS_PER_TEAM 
    camera_bool=[0]*num_cam


    #creating message and data structure variables
    
    ball = SSL_DetectionBall()
    blue= SSL_DetectionRobot()
    yellow = SSL_DetectionRobot()

    blue_bots = SSL_DetectionRobotList()
    yellow_bots = SSL_DetectionRobotList()
    

    is_blue_bot_detected = [0]*MAX_BOTS_PER_TEAM
    is_yellow_bot_detected = [0]*MAX_BOTS_PER_TEAM

    port = grsim_port if(use_grsim_vision) else ssl_vision_port
    c = sslclient.client(ip=net_address, port= port)
    c.connect()
    
    roll_counter=0
    cameraCount=0
    prevBallCount=0

    while not rospy.is_shutdown():  
    
        #for debugg
        #print('waiting for packet')

        #receive package over SSL Socket 
        try:
            packet = c.receive()  
        except:
            print("some error while reading data")

        
        

        if packet.HasField('detection'):

            balls_n = len(packet.detection.balls)
            robots_blue_n = len(packet.detection.robots_blue)
            robots_yellow_n = len(packet.detection.robots_yellow)

            msg= SSL_DetectionFrame()


            # print('blue robots detected : ',robots_blue_n)
            # print('yellow robots detected : ',robots_yellow_n)

            msg.frame_number = packet.detection.frame_number
            msg.t_capture = packet.detection.t_capture
            msg.camera_id = packet.detection.camera_id
           # print('Frame Number',msg.frame_number , 'Time Capture', msg.t_capture)

            
            if(msg.camera_id==0):
                cameraCount=0
                prevBallCount = balls_n
            else:
                cameraCount+=1
            #########################################################

            
            if(balls_n<1):
                if(prevBallCount==0):
                    ball.confidence = 0
            else:
            #print(cameraCount)
                for i in range(balls_n):
                    ball_new = SSL_DetectionBall()
                    ball_new = packet.detection.balls[i]
                    if(ball_new.confidence>max_ball_confidence):
                        #you are true one bitch
                        max_ball_confidence = ball_new.confidence
                        ball.confidence = 1
                        
                        ball.area = ball_new.area
                        ball.x = ball_new.x
                        ball.y = ball_new.y
                        ball.z = ball_new.z
           
            
            
            ##getting data for blue bots 
            for i in range (robots_blue_n):
                
                robot_new = SSL_DetectionRobot()
                robot_new = packet.detection.robots_blue[i]
                
                try:
                    blue_bots.robots_list[robot_new.robot_id].robot_id = robot_new.robot_id
                    if(robot_new.confidence > max_blue_bot_confidence[robot_new.robot_id]):

                        max_blue_bot_confidence[robot_new.robot_id] = robot_new.confidence

                        blue_bots.robots_list[robot_new.robot_id].confidence = robot_new.confidence
                        blue_bots.robots_list[robot_new.robot_id].x = robot_new.x
                        blue_bots.robots_list[robot_new.robot_id].y = robot_new.y
                        blue_bots.robots_list[robot_new.robot_id].pixel_x = robot_new.pixel_x
                        blue_bots.robots_list[robot_new.robot_id].pixel_y = robot_new.pixel_y
                        blue_bots.robots_list[robot_new.robot_id].orientation = robot_new.orientation

                except:
                    print('some error')

                
            ##getting data for yellow bots 
            for i in range (robots_yellow_n):
                robot_new = SSL_DetectionRobot()
                robot_new = packet.detection.robots_yellow[i]

                try:
                    yellow_bots.robots_list[robot_new.robot_id].robot_id = robot_new.robot_id
                    if(robot_new.confidence > max_yellow_bot_confidence[robot_new.robot_id]):

                        max_yellow_bot_confidence[robot_new.robot_id] = robot_new.confidence

                        yellow_bots.robots_list[robot_new.robot_id].confidence = robot_new.confidence
                        yellow_bots.robots_list[robot_new.robot_id].x = robot_new.x
                        yellow_bots.robots_list[robot_new.robot_id].y = robot_new.y

                        yellow_bots.robots_list[robot_new.robot_id].pixel_x = robot_new.pixel_x
                        yellow_bots.robots_list[robot_new.robot_id].pixel_y = robot_new.pixel_y
                        yellow_bots.robots_list[robot_new.robot_id].orientation = robot_new.orientation
                except:
                    print('some error')

                

            
            
            if(cameraCount>TOTAL_NO_OF_CAMERAS-2):
                for i in range (MAX_BOTS_PER_TEAM):
                    msg.robots_yellow.append(yellow_bots.robots_list[i])
                    msg.robots_blue.append(blue_bots.robots_list[i])

                msg.balls.append(ball)
                ###Publish Data only if both cameras have given data
                
                vision_data_publisher.publish(msg)
                
                ballData = point_2d()
                ballData.x= msg.balls[0].x
                ballData.y =msg.balls[0].y

                ball_data_publisher.publish(ballData)

                max_ball_confidence=0
                msg.robots_blue.clear()
                msg.robots_yellow.clear()
                msg.balls.clear()

                blue_bots = SSL_DetectionRobotList()
                yellow_bots = SSL_DetectionRobotList()
                dummyRobot = SSL_DetectionRobot()
                prevBallCount=0

                for i in range (MAX_BOTS_PER_TEAM):
                    
                    max_blue_bot_confidence[i]=0
                    is_blue_bot_detected[i] = 0

                    max_yellow_bot_confidence[i]=0
                    is_yellow_bot_detected[i] = 0

                    # blue_bots.robots_list.append(dummyRobot)
                    # yellow_bots.robots_list.append(dummyRobot)
            


if __name__ == '__main__':

    data_grabber()

