#!/usr/bin/env python3
import sys
from robot_messages.msg import SSL_DetectionFrame,SSL_DetectionRobot

from utils_updated.geometry_functions.geometry_functions import Vector2D
import rospy
import math

from robot_messages.msg import gr_Robot_Command, gr_Commands
from std_msgs.msg import String
import time

from argparse import ArgumentParser
import rrt_with_pathsmoothing as rrt



ball_pos=[0,0]
full_data=[0,0]
yellow_bots_Vision_data=[None]*6
blue_bots_Vision_data=[None]*6
robotRadius=110

requiredID = 3
requiredTeam='Yellow'
planPath=False
dataReady=False

def callbackForData(data):

    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    global dataReady
    global planPath

    # yellow_bots_Vision_data=[None]*6
    # blue_bots_Vision_data=[None]*6

    if data.balls[0].confidence > 0.8:
        ball_pos[0]=data.balls[0].x
        ball_pos[1]=data.balls[0].y

    #print("in callback  function") 
    for i in range(0,6):
        #change list initialization
        if data.robots_yellow[i].confidence >0.5 and data.robots_yellow[i].confidence!=0.0:     
            yellow_bots_Vision_data[i]={"x":data.robots_yellow[i].x,"y":data.robots_yellow[i].y,"orientation":data.robots_yellow[i].orientation}
        if data.robots_blue[i].confidence >0.5 and data.robots_blue[i].confidence!=0.0:
            blue_bots_Vision_data[i]={"x":data.robots_blue[i].x,"y":data.robots_blue[i].y,"orientation":data.robots_blue[i].orientation}

    #print("Blue Bots",blue_bots_Vision_data)
    # print("Yellow Bots",yellow_bots_Vision_data)
    dataReady=True
   
def path_planning():
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    global dataReady
    global planPath
    global robotRadius
    if planPath == False:
        return
    
    if dataReady==False:
        return

    
    # blue_bots = self.plan_data.robots_blue
    # yellow_bots = self.plan_data.robots_yellow

    obstacleList = []
    my_x=0
    my_y=0
    my_orientation=0

    if(requiredTeam=='Yellow'):
        my_x = yellow_bots_Vision_data[requiredID]['x']
        my_y = yellow_bots_Vision_data[requiredID]['y']
        my_orientation=yellow_bots_Vision_data[requiredID]['orientation']
    
    if(requiredTeam=='Blue'):
        my_x = blue_bots_Vision_data[requiredID]['x']
        my_y = blue_bots_Vision_data[requiredID]['y']
        my_orientation=blue_bots_Vision_data[requiredID]['orientation']
    
    
    start = my_x, my_y

    iterator=0

    for bot in blue_bots_Vision_data:
        
        if bot is not None:
            if iterator == requiredID and requiredTeam=='Blue':
                pass
            else:
                obstacleList.append((bot['x'], bot['y'], robotRadius))
            
        iterator+=1
    

    iterator=0
    for bot in yellow_bots_Vision_data:
        
        if bot is not None:
            if iterator == requiredID and requiredTeam=='Yellow':
                pass
            else:
                obstacleList.append((bot['x'], bot['y'], robotRadius))
        iterator+=1
    goal = (-1000,600) 

    dataReady=False
    planPath=False

    return rrt.main(obstacleList, start, goal)


if __name__=="__main__":


    rospy.Subscriber("/vision", SSL_DetectionFrame, callbackForData, queue_size=1)
    rospy.init_node('rrt_test')

    while not rospy.is_shutdown():
        userInput = int(input())
        if(userInput==1):
            planPath=True
            # print(path_planning())
            currTime = rospy.Time.now()
            currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
            sp = path_planning()
            currTime1 = rospy.Time.now()
            currTime1 = 1.0*currTime1.secs + 1.0*currTime1.nsecs/pow(10,9)
            
            dt = currTime1-currTime

            print("Smoothed path is :- ", sp)
            print("Time Required  :- ", dt)
