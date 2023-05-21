#!/usr/bin/env python3

import rospy
import sys
import math
import time

################************Not Using any Services Now***********##########
# from service_hub.srv import pid,pidResponse
# from service_hub.srv import ball,ballResponse
# from service_hub.srv import mypos,myposResponse
# from service_hub.srv import obstacle_data,obstacle_dataResponse

# from service_hub.srv import rrtStar,rrtStarResponse
# from service_hub.srv import shouldReplanRRT,shouldReplanRRTResponse

from robot_messages.msg import gr_Robot_Command, gr_Commands,point_2d,my_pos,obstacle,SSL_DetectionFrame


from utils_updated.geometry_functions.geometry_functions import Vector2D



ORIENT_THRESHOLD = 0.0872665 #threshold in radians
DEFAULT_TIMEOUT = 5
DEFAULT_ROT_VELOCITY =1.2

yellow_bots_Vision_data=[None]*6
blue_bots_Vision_data=[None]*6

BOT_RADIUS=100
INV=9999

class orient_towards_point():


    def __init__(self,
                robotID,
                team,
                orientPoint=None,
                shouldOrientTowardsBall=False,
                pubTopic=None,
                timeout = DEFAULT_TIMEOUT
                ):

        self.robotID=robotID
        self.team = team
        self.currentPosition = Vector2D(INV,INV)
        self.ballPosition = Vector2D(INV,INV)
        self.currentOrientation=INV
        self.pubTopic =pubTopic
        self.timeout=timeout
        
        self.shouldOrientTowardsBall =shouldOrientTowardsBall

        self.orientPoint =None    
        if orientPoint is None:
            if self.shouldOrientTowardsBall:
                self.orientPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
            # else:
            #     return False
        else:
            self.orientPoint = orientPoint

        #parameters to be reset
        self.angleOfOrientation=0
        self.prevTimeStamp=0
        self.velocityOfBot=0
        self.orientDone=False
        self.orientStartTime=0
        self.abortOrient=False

        ##Kicker and Dribbler
        self.dribbler=0
        self.kicker=0


        ## wait for rospy services
        # rospy.wait_for_service('pos_service') 



    def update_robot_position(self,updatedPositions):

        global yellow_bots_Vision_data
        global blue_bots_Vision_data

        for i in range(0,6):
            #change list initialization
            if updatedPositions.robots_yellow[i].confidence >0.8:
                #yellow_bots_Vision_data[i]=data.robots_yellow[i]
                yellow_bots_Vision_data[i]={"x":updatedPositions.robots_yellow[i].x,
                        "y":updatedPositions.robots_yellow[i].y,"orientation":updatedPositions.robots_yellow[i].orientation}
            if updatedPositions.robots_blue[i].confidence >0.8:
                blue_bots_Vision_data[i]={"x":updatedPositions.robots_blue[i].x,
                        "y":updatedPositions.robots_blue[i].y,"orientation":updatedPositions.robots_blue[i].orientation}
            #full_data=[yellow_bots_Vision_data,blue_bots_Vision_data] #not using for  now 

        currentPos = my_pos()
            
        for i ,bots in enumerate(yellow_bots_Vision_data):
            obstacle_data=obstacle()
            if bots !=None:
                if self.team==True and self.robotID==i:
                    self.currentPosition.x=bots['x']
                    self.currentPosition.y=bots['y']
                    self.currentOrientation=bots['orientation']
                    
                else:
                
                    obstacle_data.x=bots['x']
                    obstacle_data.y=bots['y']
                    obstacle_data.radius=BOT_RADIUS
                    currentPos.obstacle_list.append(obstacle_data)



        for i ,bots in enumerate(blue_bots_Vision_data):
            obstacle_data=obstacle()
            if bots !=None:
                if self.team==False and self.robotID==i:
                    self.currentPosition.x=bots['x']
                    self.currentPosition.y=bots['y']
                    self.currentOrientation=bots['orientation']
                        
                else:
                    obstacle_data.x=bots['x']
                    obstacle_data.y=bots['y']
                    obstacle_data.radius=BOT_RADIUS
                    currentPos.obstacle_list.append(obstacle_data)
        
       
        self.obsList = currentPos.obstacle_list

    def update_ball_position(self,data):

        
        self.ballPosition.x = data.x
        self.ballPosition.y = data.y
        #print(self.ballPosition.x)

        if self.shouldOrientTowardsBall:
            self.orientPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
            
            
    def abort_orient_motion(self):
        self.abortOrient=True
    
    def reset_parameters(self):
        
        #self.firstCall=False
        self.orientDone=False
        self.prevTimeStamp=self.get_current_timestamp()
        self.velocityOfBot=0
        self.angleOfOrientation=0
        self.orientStartTime=self.get_current_timestamp()
        self.abortOrient=False
        
        

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    def check_if_reached_point(self,angleDifference):
        if(math.fabs(angleDifference)<ORIENT_THRESHOLD):
            return True
        else:
            return False

    

    def update_orient_point(self,
                orientPoint=None,
                shouldOrientTowardsBall=False,
                timeout = DEFAULT_TIMEOUT):
        
        self.reset_parameters()

        self.timeout=timeout
        self.orientPoint =None    

        self.shouldOrientTowardsBall=shouldOrientTowardsBall

        if orientPoint is None:
            if self.shouldOrientTowardsBall:
                self.orientPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
                
                
            # else:
            #     return False
        else:
            self.orientPoint = orientPoint

    def minion_execute_command(self,vx,vy,vw):
        command = gr_Robot_Command()
        command.id=self.robotID
        command.isteamyellow=self.team
        command.kickspeedx=self.kicker
        command.kickspeedz=0
        command.velangular=vw
        command.velnormal=vy
        command.veltangent=vx
        command.spinner=self.dribbler
        command.wheelsspeed=0
        command.orientation=0

        command1 = gr_Commands()
        command1.timestamp=0
        command1.isteamyellow=self.team     
        command1.robot_commands = command
        self.pubTopic.publish(command1)
        


    def otp_spin(self):
       

        if(self.orientPoint is None):
            print('No Goal Point defined')
            return False
        
        if(self.orientDone):
            return True
        
        self.orientStartTime = self.get_current_timestamp()
        while(self.orientDone==False and
                ((self.get_current_timestamp()-self.orientStartTime)<self.timeout) and self.abortOrient==False):

            if(self.currentOrientation==INV or self.orientPoint.x==INV):
                
                #print('Current Orientation:'+str(self.currentOrientation))
                #print('OrientPoint: '+str(self.orientPoint.x))
                continue
            
            deltaTime = self.get_current_timestamp() - self.prevTimeStamp
            if(deltaTime>0.005):
                self.angleOfOrientation = self.currentPosition.angle(self.orientPoint)    # Vector2D(ball_pos[0],ball_pos[1])
                self.angleOfOrientation = self.angleOfOrientation - self.currentOrientation
                self.angleOfOrientation  = self.currentPosition.normalizeAngle(self.angleOfOrientation)
                # print('angle of Orientation:',self.angleOfOrientation)
                # print('Robot Orientation:',self.currentOrientation)

                if(self.check_if_reached_point(self.angleOfOrientation)):
                    self.velocityOfBot=0
                    self.minion_execute_command(0,0,0)
                    self.minion_execute_command(0,0,0)
                    self.minion_execute_command(0,0,0)
                    self.minion_execute_command(0,0,0)
                    self.orientDone=True
                    
                
                else:
                    self.velocityOfBot= DEFAULT_ROT_VELOCITY
                    if(self.angleOfOrientation<0):
                        self.velocityOfBot=self.velocityOfBot*(-1)
                
                vw = self.velocityOfBot
                self.minion_execute_command(0,0,vw)

                self.prevTimeStamp=self.get_current_timestamp()
        
        if(self.orientDone==True):
            #print('Finished orientation to point:',(self.orientPoint.x,self.orientPoint.y))
            return True
        elif(self.abortOrient==True):
            #print('Aborted orientation to point:',(self.orientPoint.x,self.orientPoint.y))
            self.minion_execute_command(0,0,0)
            return False
        else:
            #print('Timeout')
            self.minion_execute_command(0,0,0)
            return False
    

    #####################Kicker and Dribbler Functions#################

    def dribble_in_motion(self,_enableFlag):
        if(_enableFlag):
            self.dribbler=1
        else:
            self.dribbler=0
    
    def kick_in_motion(self,_enableFlag):
        if(_enableFlag):
            self.kicker=1
        else:
            self.kicker=0


if __name__=='__main__':

    rospy.init_node('otp_class_test',anonymous=False)
    

    pub =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
    #pub =rospy.Publisher ("/grsim_data",gr_Commands, queue_size=2)
    
    
    minion1 = orient_towards_point(robotID= 3,team=False,pubTopic=pub)
    minion1.update_orient_point(shouldOrientTowardsBall=True,timeout=10)
    
    rospy.Subscriber('/ballposition',point_2d,minion1.update_ball_position)
    
    rospy.Subscriber('/vision',SSL_DetectionFrame,minion1.update_robot_position)

    print('Executing OTP')
    while not rospy.is_shutdown():
        if(minion1.otp_spin()):
            print('Goal Reached')
            exit()
        else:
            print('Goal Not Reached')
            exit()