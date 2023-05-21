#!/usr/bin/env python3

import rospy
import sys
import math
import time

from service_hub.srv import mypos,myposResponse,obstacle_data,obstacle_dataResponse

from robot_messages.msg import SSL_DetectionFrame
from robot_messages.msg import my_pos,obstacle
from robot_messages.msg import gr_Robot_Command, gr_Commands,point_2d

from utils_updated.geometry_functions.geometry_functions import Line,Vector2D
from test import Minions


yellow_bots_Vision_data=[None]*6
blue_bots_Vision_data=[None]*6

BOT_RADIUS=100

id=0
team=False

blue1=Minions(id=id,team=team)

def vision_callback(data):

    global yellow_bots_Vision_data
    global blue_bots_Vision_data

    for i in range(0,6):
        #change list initialization
        if data.robots_yellow[i].confidence >0.8:
            #yellow_bots_Vision_data[i]=data.robots_yellow[i]
            yellow_bots_Vision_data[i]={"x":data.robots_yellow[i].x,"y":data.robots_yellow[i].y,"orientation":data.robots_yellow[i].orientation}
        if data.robots_blue[i].confidence >0.8:
            blue_bots_Vision_data[i]={"x":data.robots_blue[i].x,"y":data.robots_blue[i].y,"orientation":data.robots_blue[i].orientation}
        #full_data=[yellow_bots_Vision_data,blue_bots_Vision_data] #not using for  now 

    currentPos = my_pos()
        
    for i ,bots in enumerate(yellow_bots_Vision_data):
        obstacle_data=obstacle()
        if bots !=None:
            if team==True and id==i:
                currentPos.x=bots['x']
                currentPos.y=bots['y']
                currentPos.orientation=bots['orientation']
                
            else:
            
                obstacle_data.x=bots['x']
                obstacle_data.y=bots['y']
                obstacle_data.radius=BOT_RADIUS
                currentPos.obstacle_list.append(obstacle_data)



    for i ,bots in enumerate(blue_bots_Vision_data):
        obstacle_data=obstacle()
        if bots !=None:
            if team==False and id==i:
                currentPos.x=bots['x']
                currentPos.y=bots['y']
                currentPos.orientation=bots['orientation']
                    
            else:
                obstacle_data.x=bots['x']
                obstacle_data.y=bots['y']
                obstacle_data.radius=BOT_RADIUS
                currentPos.obstacle_list.append(obstacle_data)
        
    blue1.update_position(currentPos)


if __name__ == '__main__':
    
    
    rospy.init_node('dummy1')
    

    rospy.Subscriber('/vision',SSL_DetectionFrame,vision_callback)
	
    while not rospy.is_shutdown():
        blue1.mini_spin()
        time.sleep(0.01)