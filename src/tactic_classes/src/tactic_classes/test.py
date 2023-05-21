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



class Minions():
    def __init__(self,id,team):
        self.id=id
        self.team=team
        self.currentPosition=Vector2D(0,0)
        self.obs =  []

    def update_position(self,positionUpdate):
        self.currentPosition.x,self.currentPosition.y =   positionUpdate.x,positionUpdate.y
        self.obs = positionUpdate.obstacle_list
       

    def mini_spin(self):
        print(self.obs)


# if __name__ == '__main__':
    
#     rospy.init_node('test1')
    
#     while not rospy.is_shutdown():
#         input1=input()
#         cur_pos=my_pos()
#         cur_pos= blue1.get_position()
#         print(cur_pos.x)
	