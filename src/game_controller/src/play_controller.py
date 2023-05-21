#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy


import time
from robot_messages.msg import gr_Commands
from robot_messages.msg import gr_Robot_Command

'''
joy format
Reports the state of a joysticks axes and buttons.
Header header           # timestamp in the header is the time the data is received from the joystick
float32[] axes          # the axes measurements from a joystick
int32[] buttons         # the buttons measurements from a joystick
'''

def map_fn(x,in_min,in_max,out_min,out_max):
    return float((x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min)


def callback_joy(joystickData):
    print(joystickData)


def auto_callback(autoModedata):
    print(autoModedata)
    mainPublisher.publish(autoModedata)



if __name__ == '__main__':

    rospy.init_node('play_controller', anonymous=False,log_level=rospy.INFO)
    

    mainPublisher=rospy.Publisher('/final_data',gr_Commands, queue_size=10)

    rospy.Subscriber("joy",Joy, callback_joy)
    rospy.Subscriber("/autoOutput",gr_Commands, auto_callback)
    

        

    rospy.spin()