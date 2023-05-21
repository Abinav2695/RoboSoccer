#!/usr/bin/env python3

import rospy
import sys
import math
import time

from service_hub.srv import pid,pidResponse
from service_hub.srv import ball,ballResponse
from service_hub.srv import mypos,myposResponse
from service_hub.srv import obstacle_data,obstacle_dataResponse

from service_hub.srv import rrtStar,rrtStarResponse
from service_hub.srv import shouldReplanRRT,shouldReplanRRTResponse

from robot_messages.msg import gr_Robot_Command, gr_Commands,point_2d

from utils_updated.geometry_functions.geometry_functions import Vector2D


service_proxy_function1 = rospy.ServiceProxy('pos_service', mypos)
service_proxy_function2 = rospy.ServiceProxy('path_planner_service', rrtStar)
service_proxy_function3 = rospy.ServiceProxy('dynamic_obstacle_avoidance_service', shouldReplanRRT)


GO_TO_POINT_THRESHOLD=100 #distance in mm
ballPosition = point_2d()

class go_to_point():


    def __init__(self,
                robotID,
                team,
                goalPoint=None,
                avoidBallAsObstacle=False,
                useRRT=False,
                isGoalPointTheBall=False,
                pubTopic=None
                ):

        self.robotID=robotID
        self.team = team
        self.currentPosition = self.get_robot_position()
        self.currentOrientation = self.get_robot_orientation()
        self.pubTopic =pubTopic

        self.goalPoint =None    
        if goalPoint is None:
            if isGoalPointTheBall:
                #use ball as goal Point
                
                #self.goalPoint=Vector2D(ballPosition.x,ballPosition.y)
                pass
            # else:
            #     return False
        else:
            self.goalPoint = goalPoint
        
        self.avoidBallAsObstacle=avoidBallAsObstacle
        self.useRRT = useRRT

        ###Parameters to be reset on new position data
        self.pathIterator=0
        self.pathList=[]
        self.prevError=None
        self.firstCall=False
        self.prevTimeStamp=0
        self.velocityOfBot=0
        self.goalReached=False

        ##Kicker and Dribbler
        self.dribbler=False
        self.kicker=False

        ##PID Parameters
        self.Kp = 2
        self.Kd = 0.06
        self.Ki = 0

        self.UPPER_THRESHOLD = 1 # m/s
        self.LOWER_THRESHOLD = 0.65 # m/s

        ## wait for rospy services
        rospy.wait_for_service('pos_service') 
        rospy.wait_for_service('path_planner_service')
        rospy.wait_for_service('dynamic_obstacle_avoidance_service')
        
    def get_robot_position(self):
        
        try:
            
            response = service_proxy_function1(self.robotID,self.team,False)
            #print(response)
            return Vector2D(response.x,response.y)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
    
    def get_robot_orientation(self):
    
        try:
            
            response = service_proxy_function1(self.robotID,self.team,False)
            #print(response)
            return response.orientation
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None

    
    # def calculate_pid_output(self,error,dt):
    #     rospy.wait_for_service('pid_service')
    #     try:
    #         service_proxy_function = rospy.ServiceProxy('pid_service', pid)
    #         response = service_proxy_function(error,self.prevError,dt)
    #         return response.pid_output
    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)
    #         return None
    
    def get_new_path(self):
        
        try:
            
            response = service_proxy_function2(self.robotID,self.team,self.currentPosition.x,self.currentPosition.y,
                                self.goalPoint.x,self.goalPoint.y)

            #print(response)
            return response.node_list
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        

    def needToReplanRRT(self,nextWayPoint):

        try:
            
            response = service_proxy_function3(self.robotID,self.team,self.currentPosition.x,self.currentPosition.y,
                                nextWayPoint.x,nextWayPoint.y)

            return response.shouldReplanPath
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        pass
    

    def reset_parameters(self):
        self.pathIterator=0
        self.pathList=[]
        self.prevError=None
        self.firstCall=False
        self.goalReached=False
        self.prevTimeStamp=self.get_current_timestamp()
        self.velocityOfBot=0
        self.dribbler=False

        #print("Resetting Parametres")

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    
    def check_if_reached_point(self,distanceLeft):
        if(distanceLeft<GO_TO_POINT_THRESHOLD):
            return True
        else:
            return False

    def update_goal(self,
                goalPoint=None,
                avoidBallAsObstacle=False,
                useRRT=False,
                isGoalPointTheBall=False):
        
        self.reset_parameters()
        self.avoidBallAsObstacle=avoidBallAsObstacle
        self.useRRT = useRRT

        self.goalPoint =None
        if goalPoint is None:
            if isGoalPointTheBall:
                #use ball as goal Point
                #self.goalPoint=Vector2D(ballPosition.x,ballPosition.y)
                pass
            # else:
            #     return False
        else:
            self.goalPoint = goalPoint

        #self.reset_parameters()

    def minion_execute_command(self,vx,vy,vw):
        command = gr_Robot_Command()
        command.id=self.robotID
        command.isteamyellow=self.team
        command.kickspeedx=0
        command.kickspeedz=0
        command.velangular=vw
        command.velnormal=vy
        command.veltangent=vx
        command.spinner=0
        command.wheelsspeed=0
        command.orientation=0

        command1 = gr_Commands()
        command1.timestamp=0
        command1.isteamyellow=self.team     
        command1.robot_commands = command
        self.pubTopic.publish(command1)


    def calculate_pid_output(self,error,dt):

        # Calculate proportional control output.
        P_out = error*self.Kp

        # Calculate derivative control output.
        # error values and dt for time difference.
        if self.prevError != None:
            D_out = (error-self.prevError)/dt *self.Kd
        else:
            D_out = 0
            # Set this to error.
        # Calculate final output.
        output = P_out + D_out

        #print("PID_OUTPUT:=  ",output)

        if(output>self.UPPER_THRESHOLD):
            output=self.UPPER_THRESHOLD
        elif (output<self.LOWER_THRESHOLD):
            output=self.LOWER_THRESHOLD
        #print("PID_OUTPUT:=  ",output)
        
        return output

    def gtp_spin(self):
        
        self.currentPosition = self.get_robot_position()
        self.currentOrientation = self.get_robot_orientation()
        deltaTime = self.get_current_timestamp() - self.prevTimeStamp
        
        if(self.goalPoint is None):
            #print('No Goal Point defined')
            return False

        if(self.goalReached):
            return True

        if(deltaTime>0.01):
            
            
            if(self.useRRT):
                
                
                if not self.pathList or self.pathList is None:
                    self.pathList = self.get_new_path()
                    print(self.pathList)
                
                else:
                    if(self.needToReplanRRT(self.pathList[self.pathIterator])):
                        self.reset_parameters()
                        vx=0
                        vy=0
                        self.minion_execute_command(vx,vy,0)

                    else:
                        distanceToReach = self.currentPosition.dist(Vector2D(self.pathList[self.pathIterator].x,
                                                                              self.pathList[self.pathIterator].y))
                            
                        if(self.check_if_reached_point(distanceToReach)):

                            if(self.pathIterator<len(self.pathList)):
                                self.pathIterator+=1
                            
                            if(self.pathIterator==len(self.pathList)):
                                #print("Reached Goal")
                                self.goalReached=True
                                self.velocityOfBot=0


                        else:
                            self.velocityOfBot=self.calculate_pid_output(distanceToReach,deltaTime)

                        angle_of_motion=0

                        if(self.velocityOfBot!=0):
                            angle_of_motion = self.currentPosition.angle(self.pathList[self.pathIterator])
                            angle_of_motion = angle_of_motion - self.currentOrientation 
                            angle_of_motion += (math.pi*0.5)
                        
                        vx = self.velocityOfBot*math.cos(angle_of_motion)
                        vy = self.velocityOfBot*math.sin(angle_of_motion)
                    
                        self.minion_execute_command(vx,vy,0)
                        self.prevError = distanceToReach
                        #print(self.pathIterator,len(self.pathList))

            else:

                distanceToReach = self.currentPosition.dist(self.goalPoint)
                
                if(self.check_if_reached_point(distanceToReach)):
                    self.velocityOfBot=0
                    self.goalReached=True
                else:        
                    self.velocityOfBot=self.calculate_pid_output(distanceToReach,deltaTime)

                angle_of_motion = self.currentPosition.angle(self.goalPoint)
                angle_of_motion = angle_of_motion - self.currentOrientation 
                angle_of_motion += (math.pi*0.5)

                vx = self.velocityOfBot*math.cos(angle_of_motion)
                vy = self.velocityOfBot*math.sin(angle_of_motion)

                self.minion_execute_command(vx,vy,0)
                self.prevError = distanceToReach

            self.prevTimeStamp=self.get_current_timestamp()
            return False
        
        else:
            return False

    #####################Kicker and Dribbler Functions#################

    def enable_dribbler(self,_enableFlag):
        if(_enableFlag):
            self.dribbler=True
        else:
            self.dribbler=False
    
    def enable_kicker(self,_enableFlag):
        if(_enableFlag):
            self.kicker=True
        else:
            self.kicker=False

def update_ball_position_callback(data):
    ballPosition.x = data.x
    ballPosition.y = data.y


if __name__=='__main__':

    rospy.init_node('gtp_class_test',anonymous=False)
    

    pub =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
    
    #rospy.Subscriber('/ballposition',point_2d,update_ball_position_callback)
    
    minion1 = go_to_point(0,False,pubTopic=pub)
    minion1.update_goal(goalPoint=Vector2D(750,-500),useRRT=True)

    while not rospy.is_shutdown():
        if(minion1.gtp_spin()):
            print('Goal Reached')
