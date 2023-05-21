#!/usr/bin/env python3
import rospy
from service_hub.srv import *
from utils_updated.geometry_functions.geometry_functions import Vector2D
#from robot_messages.msg import SSL_DetectionFrame

def get_ball_data():
    rospy.wait_for_service('ball_velocity')
    try:
        get_ball_data = rospy.ServiceProxy('ball_velocity', ball)
        resp1 = get_ball_data()
        print(resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



rospy.init_node("ball__pos_prediction_node")
get_ball_data()
