#!/usr/bin/env python3

import time


import rospkg
import rospy
import sys
import serial

from robot_messages.msg import BeliefState,gr_Robot_Command, gr_Commands,point_2d,my_pos,obstacle,SSL_DetectionFrame
from std_msgs.msg import String





def referee_callback(calldata):


    if(calldata.data=='stop'):
        ESP_DATA.write(b'stop;')
    
    elif(calldata.data[0]=='g'):
        data = calldata.data+';'
        ESP_DATA.write(bytes(data,'utf-8'))


if __name__ == "__main__":
    
    rospy.init_node('socket_node', anonymous=False)
    rospy.Subscriber('/referee_topic',String,referee_callback)
    
    server_pub = rospy.Publisher('/server_events', String, queue_size=5)

    #The following line is for serial over GPIO
    try:
        port = '/dev/ttyUSB0'
    except:
        print('cannot open')


    ESP_DATA = serial.Serial(port,115200,timeout=5)
    #ESP_DATA.open()    

    spinRate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        newData = ESP_DATA.readline()
        newData = newData.decode("utf-8")
        newData=newData.split('\r')
        if(newData[0]=='start'):
            
            print('START')
            server_pub.publish("start")
        elif(newData[0]=='stop'):
            print('STOP')
            server_pub.publish("stop")
        
        spinRate.sleep()



