#!/usr/bin/env python3

import sys
#sys.path.append('/opt/ros/noetic/lib/python3/dist-packages/')
import rospy
import math
import time 
import socket 

from robot_messages.msg import gr_Commands
from robot_messages.msg import gr_Robot_Command
from std_msgs.msg import String
import time

try:
    ANY = "0.0.0.0"
    MCAST_ADDR = "237.252.249.227"
    MCAST_PORT = 12121
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.bind((ANY, 0))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 255)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except:
    print('Couldn\'t open Socket')
    print('Terminating')



#Team_ID = 127  # Team Blue
theta = [30.0,150.0,225.0,315.0]
bot_radius = 0.095 #in meter
bot_wheel_radius = 0.025  #in meter
max_vel_wheel = 500.0 # in rpm

FACTOR_T = 40
FACTOR_N = 40
FACTOR_W = 90

TIME_TO_STOP = 0.5    # in secs
T_INIT = time.time()
pubB = None
pubY = None
bot_id = None

#callback funtion for subscriber topic
def gr_Commands_CB(msg):
    global T_INIT , pubB, pubY, bList, yList, bot_id
    T_INIT = time.time()

    bot_id = int(msg.id)
    
    vel_xyw = [None]*3
    vel_xyw[0] = int(msg.veltangent * FACTOR_T)
    vel_xyw[1] = int(msg.velnormal * FACTOR_N) #*-1 negate in terms of opposite bot orientation
    vel_xyw[2] =int(msg.velangular * FACTOR_W)

    [v_4_wheel,dir_4_wheel] = vel_convert(vel_xyw)
    kick =0
    spinner=0
    if(msg.kickspeedx):
        kick=1
    if(msg.spinner):
        spinner=1
    
    rospy.loginfo(" Data is being sent to :- " + str(msg.id) + " Team :- " + rospy.get_namespace())

    data='{},{},{},{},{},{},{},{},{},{},{},{};'.format(msg.id,v_4_wheel[0],
                                            v_4_wheel[1],v_4_wheel[2],v_4_wheel[3],dir_4_wheel[0],
                                            dir_4_wheel[1],dir_4_wheel[2],dir_4_wheel[3],
                                            kick,spinner,msg.orientation)

    # pubB.publish(data)
    pubY.publish(data)
    
    ### not in use
    # data=str(msg.id) +','
    # for i in range 4:
    #     data=data+str(v_4_wheel[i])+','
    # for i in range 4:
    #     data=data+str(dir_4_wheel[i])+','
    # if(msg.kickspeedx):
    #     data=data+'1,'
    # else:d
    #     data=data+'0,'

    # if(msg.spinner):
    #     data=data+'1,'
    # else:
    #     data=data+'0,'
    
    # data=data+ str(msg.orientation)

    # sock_send(data)


def vel_convert(vel_3_wheel):
        global theta
        vx = vel_3_wheel[0]
        vy = vel_3_wheel[1]
        vw = -vel_3_wheel[2]

        dir_4_wheel = [1,1,1,1]        
        v_4_wheel = [0,0,0,0]

        #print("vx",vx,"vy",vy,"vw",vw)
        for i in range(4):
            v_4_wheel[i] = ((bot_radius*vw) - (vx*math.sin(theta[i]*math.pi/180.0)) + (vy*math.cos(theta[i]*math.pi/180.0)))/(bot_wheel_radius * math.pi)
        
        # use this in case of bldc motor
        # for i in range(4):
        #     if v_4_wheel[i] > 0 :
        #         v_4_wheel[i] = int(126 + ((v_4_wheel[i]-max_vel_wheel)*126) / max_vel_wheel)
        #     else :
        #         v_4_wheel[i] = int(256 - (v_4_wheel[i]*(129-256)) / max_vel_wheel)
        

        for i in range(4):
            if(v_4_wheel[i] <0):           
                dir_4_wheel[i]=0
                v_4_wheel[i]*=-1
                
            v_4_wheel[i]=int((v_4_wheel[i]*255)/max_vel_wheel)
            if(v_4_wheel[i]>255):
                v_4_wheel[i]=255
        
        return ([v_4_wheel,dir_4_wheel])

def sock_send(data):
    sock.sendto(data.encode(),(MCAST_ADDR, MCAST_PORT))
    print('--------------------')
    print(data.encode())

rospy.init_node('bot_comm_wifi',anonymous=False)
rospy.Subscriber('grsim_data',gr_Robot_Command,gr_Commands_CB, queue_size=1)
# pubB = rospy.Publisher('/velocityDataBlue', String, queue_size=1)
pubY = rospy.Publisher('/velocityDataYellow', String, queue_size=1)
# time.sleep(0.5)
# rospy.spin()

# bList = [0]
# yList = [0, 1, 2]

while not rospy.is_shutdown():

    if (time.time() - T_INIT) > TIME_TO_STOP:
        # for i in bList:
        if bot_id != None:
            data = '{},0,0,0,0,0,0,0,0,0,0,0;'.format(bot_id) # first element is bot id
            pubY.publish(data)
        # sock_send(data)

        # data = '0,0,0,0,0,0,0,0,0,0,0,0;' # first element is bot id
        # pubY.publish(data)
        # pubB.publish(data)
        # for i in yList:
        #     data = '{},0,0,0,0,0,0,0,0,0,0,0;'.format(i)
        #     # sock_send(data)