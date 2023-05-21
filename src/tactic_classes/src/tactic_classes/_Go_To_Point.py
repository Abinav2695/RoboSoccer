#!/usr/bin/env python3

import rospy
import sys
import math
import time
import random

################************Not Using any Services Now***********##########

# from service_hub.srv import pid,pidResponse
# from service_hub.srv import ball,ballResponse
# from service_hub.srv import mypos,myposResponse
# from service_hub.srv import obstacle_data,obstacle_dataResponse

# from service_hub.srv import rrtStar,rrtStarResponse
# from service_hub.srv import shouldReplanRRT,shouldReplanRRTResponse

# service_proxy_function1 = rospy.ServiceProxy('pos_service', mypos)
# service_proxy_function2 = rospy.ServiceProxy('path_planner_service', rrtStar)
# service_proxy_function3 = rospy.ServiceProxy('dynamic_obstacle_avoidance_service', shouldReplanRRT)


from robot_messages.msg import gr_Robot_Command, gr_Commands,point_2d,my_pos,obstacle,SSL_DetectionFrame

from utils_updated.geometry_functions.geometry_functions import Vector2D
from utils_updated.path_planner.rrt_planner import RRTStar

newPathInstance = RRTStar()

GO_TO_POINT_THRESHOLD=90 #distance in mm



yellow_bots_Vision_data=[None]*6
blue_bots_Vision_data=[None]*6

BOT_RADIUS=150
BALL_RADIUS = 50
INV=9999

GoalMaxY=300
GoalMinY=-300

GOALIE_X_POSITION_NEAR_GOAL_YELLOW=-1470
GOALIE_X_POSITION_FAR_GOAL_YELLOW=-1250

GOALIE_X_POSITION_NEAR_GOAL_BLUE= 1470
GOALIE_X_POSITION_FAR_GOAL_BLUE= 1250

UPPER_HALF_FIELD_Y = 450
LOWER_HALF_FIELD_Y = -450


HALF_FILED_X = 1650
HALF_OF_HALF_FIELD_X = HALF_FILED_X/2 



class go_to_point():


    def __init__(self,
                robotID,
                team,
                goalPoint=None,
                considerBallAsObstacle=False,
                useRRT=False,
                isGoalPointTheBall=False,
                pubTopic=None
                ):

        self.robotID=robotID
        self.team = team
        self.currentPosition = Vector2D(INV,INV)
        self.ballPosition = Vector2D(INV,INV)
        self.currentOrientation = 0
        self.pubTopic =pubTopic

        self.isGoalPointTheBall=isGoalPointTheBall
        self.goalPoint =None    
        if goalPoint is None:
            if self.isGoalPointTheBall:
                #use ball as goal Point
                self.goalPoint=Vector2D(self.ballPosition.x,self.ballPosition.y) 

        else:
            self.goalPoint = goalPoint
        
        self.considerBallAsObstacle=considerBallAsObstacle
        self.useRRT = useRRT

        ###Parameters to be reset on new position data
        self.pathIterator=0
        self.pathList=[]
        self.prevError=None
        self.firstCall=False
        self.prevTimeStamp=0
        self.prevTimeStamp1=0
        self.velocityOfBot=0
        self.goalReached=False
        self.obsList=[]
        self.tempGoalPoint=None
        self.tempMotion = False
        self.ballAsobstacle=obstacle()

        ##Kicker and Dribbler
        self.dribbler=0
        self.kicker=0

        ##PID Parameters
        self.Kp = 2
        self.Kd = 0.06
        self.Ki = 0

        self.MAX_UPPER_THRESHOLD = 1
        self.UPPER_THRESHOLD = 1 # m/s
        self.LOWER_THRESHOLD = 0.65 # m/s

        ## wait for rospy services
        # rospy.wait_for_service('pos_service') 
        # rospy.wait_for_service('path_planner_service')
        # rospy.wait_for_service('dynamic_obstacle_avoidance_service')
        
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


    def get_new_path(self):
        
        if(self.currentPosition.x==INV):
            return None

        
        return newPathInstance.update(start=(self.currentPosition.x,self.currentPosition.y),
                    goal=(self.goalPoint.x,self.goalPoint.y),obstacleList=self.obsList)
        

    def needToReplanRRT(self,nextWayPoint):

        isPathClear = newPathInstance.line_collision_check(first = (self.currentPosition.x,self.currentPosition.y),
                                second=(nextWayPoint.x,nextWayPoint.y),obstacleList= self.obsList)

        if isPathClear:
            return False
        else:
            return True
        
    

    def reset_parameters(self):
        self.pathIterator=0
        self.pathList=None
        self.prevError=None
        self.firstCall=False
        self.goalReached=False
        self.prevTimeStamp=self.get_current_timestamp()
        self.velocityOfBot=0
        # self.dribbler=0
        # self.kicker=0
        self.obsList=[]

        #print("Resetting Parametres")

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    def set_upper_threshold(self,upperValue):
        self.MAX_UPPER_THRESHOLD = upperValue
    
    def check_if_reached_point(self,distanceLeft):
        if(distanceLeft<GO_TO_POINT_THRESHOLD):
            return True
        else:
            return False

    def update_goal(self,
                goalPoint=None,
                considerBallAsObstacle=False,
                useRRT=False,
                isGoalPointTheBall=False):
        
        self.reset_parameters()
        self.considerBallAsObstacle=considerBallAsObstacle
        self.useRRT = useRRT
        self.isGoalPointTheBall=isGoalPointTheBall

        self.goalPoint =None
        if goalPoint is None:
            if self.isGoalPointTheBall:
                #use ball as goal Point
                self.goalPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
            else:
                return False
        else:
            self.goalPoint = goalPoint
        
        if( not self.pointInCollisionRange(self.goalPoint,self.obsList)):
            self.goalPoint = Vector2D(self.currentPosition.x,self.currentPosition.y)
            self.goalReached=True

        if(self.useRRT):
            self.UPPER_THRESHOLD = 0.9
        else:
            self.UPPER_THRESHOLD =1
        if(self.UPPER_THRESHOLD>self.MAX_UPPER_THRESHOLD):
            self.UPPER_THRESHOLD=self.MAX_UPPER_THRESHOLD
        
        if considerBallAsObstacle:
            self.obsList.append(self.ballAsObstacle)
        #self.reset_parameters()

    def pointInCollisionRange(self, node, obstacleList):

        for obs_data in obstacleList:
            dx = obs_data.x - node.x
            dy = obs_data.y - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= 2*obs_data.radius:
                return False

        return True


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
        
        #self.kicker=0


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

    def get_temp_clear_point_for_motion(self):

        maxIterations=200
        got_a_point=False
        clearPoint=Vector2D()
        for i in range (0,maxIterations):
            print('doing calculations')
            randomAngle = random.uniform(0,2*math.pi)
            r = 200

            x = r * math.cos(randomAngle) + self.currentPosition.x
            y = r * math.sin(randomAngle) + self.currentPosition.y

            if(newPathInstance.pointInCollisionRange(node=(x,y), obstacleList=self.obsList)):
                got_a_point=True
                clearPoint.x,clearPoint.y = x,y
                break
        if(got_a_point):
            print('Clear Point : ',(clearPoint.x,clearPoint.y))
            return (clearPoint)

        else:
            return None


    def gtp_spin(self):
        
        # self.currentPosition = self.get_robot_position()
        # self.currentOrientation = self.get_robot_orientation()
        deltaTime = self.get_current_timestamp() - self.prevTimeStamp
        
        if(self.goalPoint is None):
            #print('No Goal Point defined')
            return False

        if(self.goalReached):
            return True

        if(deltaTime>0.01):
            
            
            if(self.useRRT):         
                
                if(self.pathList is None):

                    for i in range(0,5):
                        newPathInstance.update_size_factor(2.3)
                        self.pathList = self.get_new_path()
                        if(self.pathList is not None):
                            self.pathIterator=0
                            break

                    if(self.pathList is None):
                        newPathInstance.update_size_factor(1.2)
                        for i in range(0,5):
                        
                            self.pathList = self.get_new_path()
                            if(self.pathList is not None):
                                self.pathIterator=0
                                break

                deltaTime1 = self.get_current_timestamp() - self.prevTimeStamp1
                if(deltaTime1>3):
                    self.pathList = self.get_new_path()
                    self.pathIterator=0
                    self.prevTimeStamp1 = self.get_current_timestamp()
                
                
                if(self.pathList is not None):
                    
                    # try:
                    distanceToReach = self.currentPosition.dist(Vector2D(self.pathList[self.pathIterator].x,
                                                                            self.pathList[self.pathIterator].y))
                    # except:
                    #     distanceToReach=0

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

                    #print(vx,vy)
                    self.minion_execute_command(vx,vy,0)
                    self.prevError = distanceToReach
                    #print(self.pathIterator,len(self.pathList))

            else:
                
               
                distanceToReach = self.currentPosition.dist(self.goalPoint)
                
                if(self.check_if_reached_point(distanceToReach)):
                    self.velocityOfBot=0

                    if(self.tempMotion):
                        self.tempMotion=False
                        self.update_goal(goalPoint=self.tempGoalPoint,useRRT=True)
                    else:       
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
    
    def execute_kick_immediately(self):
        command = gr_Robot_Command()
        command.id=self.robotID
        command.isteamyellow=self.team
        command.kickspeedx=1
        command.kickspeedz=0
        command.velangular=0
        command.velnormal=0
        command.veltangent=0
        command.spinner=self.dribbler
        command.wheelsspeed=0
        command.orientation=0

        command1 = gr_Commands()
        command1.timestamp=0
        command1.isteamyellow=self.team     
        command1.robot_commands = command
        self.pubTopic.publish(command1)

    def execute_dribble_immediately(self):
        command = gr_Robot_Command()
        command.id=self.robotID
        command.isteamyellow=self.team
        command.kickspeedx=0
        command.kickspeedz=0
        command.velangular=0
        command.velnormal=0
        command.veltangent=0
        command.spinner=1
        command.wheelsspeed=0
        command.orientation=0

        command1 = gr_Commands()
        command1.timestamp=0
        command1.isteamyellow=self.team     
        command1.robot_commands = command
        self.pubTopic.publish(command1)


    def update_ball_position(self,data):
        self.ballPosition.x = data.x
        self.ballPosition.y = data.y

        self.ballAsObstacle = obstacle()
        self.ballAsObstacle.x = data.x
        self.ballAsObstacle.y = data.y
        self.ballAsobstacle.radius=BALL_RADIUS

        if self.isGoalPointTheBall:
            self.goalPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)



if __name__=='__main__':

    rospy.init_node('gtp_class_test',anonymous=False)
    
   


    pub =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
    
    minion1 = go_to_point(2,True,pubTopic=pub)
    minion1.update_goal(goalPoint=Vector2D(-1000,0),useRRT=False)
    
    rospy.Subscriber('/ballposition',point_2d,minion1.update_ball_position)
    
    rospy.Subscriber('/vision',SSL_DetectionFrame,minion1.update_robot_position)
    
    

    while not rospy.is_shutdown():
        #time.sleep(3)

        if(minion1.gtp_spin()):
            print('Goal Reached')
