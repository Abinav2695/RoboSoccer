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



#callback funtion for subscriber topic
def gr_Commands_CB(msg1):
	  
    msg = msg1.robot_commands
    vel_xyw = [None]*3
    vel_xyw[0] = int(msg.veltangent * FACTOR_T)
    vel_xyw[1] = int(msg.velnormal * FACTOR_N) #*-1 negate in terms of opposite bot orientation
    vel_xyw[2] =int(msg.velangular * FACTOR_W)

    [v_4_wheel,dir_4_wheel] = vel_convert(vel_xyw)
    
    
    #rospy.loginfo(" Data is being sent to :- " + str(msg.id) + " Team :- " + rospy.get_namespace())

    data='{},{},{},{},{},{},{},{},{},{},{},{};'.format(msg.id,v_4_wheel[0],
                                            v_4_wheel[1],v_4_wheel[2],v_4_wheel[3],dir_4_wheel[0],
                                            dir_4_wheel[1],dir_4_wheel[2],dir_4_wheel[3],
                                            msg.kickspeedx,msg.spinner,msg.orientation)


    print(data)

    if(msg.isteamyellow):
        yellowTeamData.publish(data)
    else:
        blueTeamData.publish(data)


def vel_convert(vel_3_wheel):

	global theta
	vx = vel_3_wheel[0]
	vy = vel_3_wheel[1]
	vw = vel_3_wheel[2]

	dir_4_wheel = [1,1,1,1]        
	v_4_wheel = [0,0,0,0]

	for i in range(4):
	    v_4_wheel[i] = ((bot_radius*vw) - (vx*math.sin(theta[i]*math.pi/180.0)) + (vy*math.cos(theta[i]*math.pi/180.0)))/(bot_wheel_radius * math.pi)




	for i in range(4):
	    if(v_4_wheel[i] <0):           
	        dir_4_wheel[i]=0
	        v_4_wheel[i]*=-1
	        
	    v_4_wheel[i]=int((v_4_wheel[i]*255)/max_vel_wheel)
	    if(v_4_wheel[i]>255):
	        v_4_wheel[i]=255

	return ([v_4_wheel,dir_4_wheel])



if __name__ == '__main__':

 
    rospy.init_node('robot_data_publisher',anonymous=False)
    rospy.Subscriber('/grsim_data',gr_Commands,gr_Commands_CB)

    
    
   
    
    ##Publisher Topics
    yellowTeamData = rospy.Publisher('/velocityDataYellow', String, queue_size=1)
    blueTeamData = rospy.Publisher('/velocityDataBlue', String, queue_size=1)

    rospy.spin()
    
        

   # message=game()#for publishing joystick data --debug joystick
   # Bot_Command=gr_Robot_Command()#for controlling bot from joystick
    #listener_joy()
