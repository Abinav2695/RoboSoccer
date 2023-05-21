#!/usr/bin/env python3
import sys
from robot_messages.msg import SSL_DetectionFrame,SSL_DetectionRobot
#from geometry import Vector2D
#from ctypes import *
from utils_updated.geometry_functions.geometry_functions import Vector2D
import rospy
import math
from robot_messages.msg import gr_Robot_Command, gr_Commands
from std_msgs.msg import String
from rrt_star_with_pathsmoothing import RRTStar
#from simple_rrt_with_pathsmoothing import RRT

#from geometry_functions import Vector2D
#from utils import *
import time
import math
import numpy as np

ROTATION_FACTOR = (0.174533) *0.8 ##5 deg 
ROBOT_RADIUS=100
BALL_RADIUS = 25
MAX_ROBOTS_PER_TEAM=6
BOT_ID=1
BOT_TEAM="Yellow"
UPPER_THRESHOLD_FOR_ROTATION = 0.9#rad/s
LOWER_THRESHOLD_FOR_ROTATION = 0.9


ball_pos=[0,0]
full_data=[0,0]
yellow_bots_Vision_data=[None]*MAX_ROBOTS_PER_TEAM
blue_bots_Vision_data=[None]*MAX_ROBOTS_PER_TEAM

previous_yellow_bots_Vision_data=[None]*MAX_ROBOTS_PER_TEAM
previous_blue_bots_Vision_data=[None]*MAX_ROBOTS_PER_TEAM

obstacleList=[]
newPath=None
updatePathFlag=False
goalNode = (1200,-700)
#goalNode = (0,0)
startNode=None
pathIterator=0
prevTimeStamp=0
prevError=None
my_orientation =0
botBallDistance=0
botPosition=0
prevbotPosition = Vector2D(0,0)
ballPosition=0
integral=0
prevErrorRotation=None
angleOfOrientation=0
velocityVector = Vector2D(0,0)
velocityCounter=0
pathInstance = RRTStar()
v_average=0

def vision_callback(visionData):
    global full_data
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    global obstacleList
    global BOT_ID
    global BOT_TEAM
    global newPath
    global updatePathFlag
    global goalNode
    global startNode
    global pathIterator
    global my_orientation 
    global botBallDistance
    global botPosition
    global ballPosition
    global angleOfOrientation
    global velocityVector
    global prevTimeStamp
    global prevbotPosition
    global velocityCounter
    global v_average

    if visionData.balls[0].confidence > 0:
        ball_pos[0]=visionData.balls[0].x
        ball_pos[1]=visionData.balls[0].y
    #goalNode = (ball_pos[0],ball_pos[1])
    
    if not pathInstance.pointInCollisionRange(goalNode,obstacleList):
        #print('Updating Goal as goal in obstacle')
        goalNode = (-1200,0)

    #print("in callback  function") 
    for i in range(0,MAX_ROBOTS_PER_TEAM):
        #change list initialization
        if visionData.robots_yellow[i].confidence >0.8:
            #yellow_bots_Vision_data[i]=data.robots_yellow[i]
            yellow_bots_Vision_data[i]={"x":visionData.robots_yellow[i].x,
                        "y":visionData.robots_yellow[i].y,"orientation":visionData.robots_yellow[i].orientation}
        if visionData.robots_blue[i].confidence >0.8:
            blue_bots_Vision_data[i]={"x":visionData.robots_blue[i].x,
                        "y":visionData.robots_blue[i].y,"orientation":visionData.robots_blue[i].orientation}
        full_data=[yellow_bots_Vision_data,blue_bots_Vision_data] #not using for  now 

   
    obstacleList = []
                        
    iterator=0
    for bot in blue_bots_Vision_data:           
        if bot is not None:
            if iterator == BOT_ID and BOT_TEAM=='Blue':
                startNode = (bot['x'], bot['y'])
                my_orientation = bot['orientation']
            else:
                obstacleList.append((bot['x'], bot['y'], ROBOT_RADIUS))
            
        iterator+=1

    iterator=0
    for bot in yellow_bots_Vision_data:           
        if bot is not None:
            if iterator == BOT_ID and BOT_TEAM=='Yellow':

                startNode = (bot['x'], bot['y'])
                my_orientation = bot['orientation']
            else:
                obstacleList.append((bot['x'], bot['y'], ROBOT_RADIUS))
        iterator+=1

    obstacleList.append((ball_pos[0], ball_pos[1], BALL_RADIUS))
    
    
    ballPosition = Vector2D(ball_pos[0],ball_pos[1])
    botPosition = Vector2D(startNode[0],startNode[1])

    botBallDistance = botPosition.dist(ballPosition)

    angleOfOrientation = botPosition.angle(ballPosition)    # Vector2D(ball_pos[0],ball_pos[1])
    # print("Orient1 : ",angleOfOrientation)              
    angleOfOrientation = angleOfOrientation - my_orientation
    # print("Orient2 : ",angleOfOrientation)   
    
    angleOfOrientation  = botPosition.normalizeAngle(angleOfOrientation)
    
    currTime = rospy.Time.now()
    currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
    deltaTime = currTime - prevTimeStamp

    if(deltaTime>0.001):

        prevTimeStamp = currTime
        velocityVector.x = ((ballPosition.x-prevbotPosition.x)/deltaTime) ## velocity in mm/s
        velocityVector.y = ((ballPosition.y-prevbotPosition.y)/deltaTime)
        v = velocityVector.magnitude()
        velocityCounter+=1
        v_average+=v
        if velocityCounter==5:
            #print("Velocity of Ball :",v)
            velocityCounter=0
            v_average=v_average/5
            distanceToCover = v_average*1
            print("Distance : ",distanceToCover)
            ballVector = ballPosition.__sub__(prevbotPosition)
            normalisedBallVector = ballVector.__norm__()
            nextPoint = ballVector.__add__(normalisedBallVector.__mul__(distanceToCover))
            print('Next Position :=',nextPoint.x,nextPoint.y)         
            v_average=0

    prevbotPosition.x = ballPosition.x
    prevbotPosition.y = ballPosition.y
    #print('Ball Position :=',(ballPosition.x,ballPosition.y))

    
    
       
def stop():
    command = gr_Robot_Command()
    command.id=BOT_ID
    if BOT_TEAM=='Blue':
        command.isteamyellow=0
    else:
        command.isteamyellow=1
    command.kickspeedx=0
    command.kickspeedz=0
    command.velangular=0
    command.velnormal=0
    command.veltangent=0
    command.spinner=0
    command.wheelsspeed=0
    command.orientation=0

    command1 = gr_Commands()
    command1.timestamp=0
    if BOT_TEAM=='Blue':
        command1.isteamyellow=0
    else:
        command1.isteamyellow=1
    command1.robot_commands = command
    pub.publish(command1)


def main():

    global full_data
    global yellow_bots_Vision_data
    global blue_bots_Vision_data
    global ball_pos
    global obstacleList
    global BOT_ID
    global BOT_TEAM
    global newPath
    global updatePathFlag
    global goalNode
    global startNode
    global pathIterator
    global prevTimeStamp
    global prevError
    global my_orientation 
    global prevTimeStamp
    global botBallDistance
    global botPosition
    global ballPosition
    global prevErrorRotation
    global integral
    global angleOfOrientation

    doneFlag=False
    orientDone=False
    prev_goalNode=Vector2D(goalNode[0],goalNode[1])

    while not rospy.is_shutdown():
        while True:
            pass
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        dt = currTime - prevTimeStamp

        
        
        g1=Vector2D(goalNode[0],goalNode[1])

        if(g1.dist(prev_goalNode)>100):
            newPath=None
            doneFlag=False
            orientDone=False
            prev_goalNode=Vector2D(goalNode[0],goalNode[1])
        else:
            pass

        if(doneFlag==False):
            if(newPath==None):
                #stop()
                if(startNode is not None):
                    
                    newPath = pathInstance.update(start=startNode,goal = goalNode,
                                            obstacleList= obstacleList)
                    pathIterator = 0
                prevTimeStamp = currTime
            
            elif(newPath is not None) :


                if (pathInstance.line_collision_check(startNode,newPath[pathIterator],obstacleList)==False):
                    print('Line Collision')
                    newPath=None 
                    prevTimeStamp = currTime

                elif(dt>0.01):
                    #print("deltaTime:= ",dt)
                    prevTimeStamp = currTime

                    point_to_reach = Vector2D(newPath[pathIterator][0], newPath[pathIterator][1]) 

                    bot_pos = Vector2D(startNode[0], startNode[1])

                    distan = bot_pos.dist(point_to_reach)/1000
                    #print("Distance to goal point:= ",distan)
                    # maxDisToTurn = distan 
                    UPPER_THRESHOLD = 1 # m/s
                    LOWER_THRESHOLD = 0.65 # m/s
                    bbt=0.3
                    # if(self.followBall==True):
                    #     bbt=0.25
                    if(distan>0.1+bbt):
                    #############################----PID TEMPLATE-----###############################################
                    # Calculate the distance to go i.e. the error and then issue the velocity commands based on the PID gains
                        Kp = 4
                        Kd = 0.02
                        Ki = 0
                        

                        # Calculate error.
                        error  = distan
                        # Calculate proportional control output.
                        P_out = error*Kp

                        # Calculate derivative control output.
                        # HINT: Use self.prev_error to store old
                        # error values and dt for time difference.
                        if prevError != None:
                            D_out = (error-prevError)/dt *Kd
                        else:
                            D_out = 0
                            # Set this to error.
                            prevError = error

                        # Calculate final output.
                        output = P_out + D_out

                        #print("PID_OUTPUT:=  ",self.output)

                        if(output>UPPER_THRESHOLD):
                            output=UPPER_THRESHOLD
                        elif (output<LOWER_THRESHOLD):
                            output=LOWER_THRESHOLD
                        #print("PID_OUTPUT:=  ",output)
                        #############################----PID TEMPLATE-----###############################################

                        velocity_of_bot = output
                        angle_of_motion = bot_pos.angle(point_to_reach)
                        angle_of_motion = angle_of_motion - my_orientation 
                        angle_of_motion += (math.pi*0.5)

                        vx = velocity_of_bot*math.cos(angle_of_motion)
                        vy = velocity_of_bot*math.sin(angle_of_motion)

                        command = gr_Robot_Command()
                        command.id=BOT_ID
                        if BOT_TEAM=='Blue':
                            command.isteamyellow=0
                        else:
                            command.isteamyellow=1
                        
                        command.kickspeedx=0
                        command.kickspeedz=0
                        command.velangular=0
                        command.velnormal=vy
                        command.veltangent=vx
                        command.spinner=0
                        command.wheelsspeed=0
                        command.orientation=0

                        command1 = gr_Commands()
                        command1.timestamp=0
                        if BOT_TEAM=='Blue':
                            command1.isteamyellow=0
                        else:
                            command1.isteamyellow=1
                        command1.robot_commands = command
                        pub.publish(command1)

                    else:
                        pathIterator+=1
                        
                        if(pathIterator>=len(newPath)):
                            pathIterator = 0
                            newPath=None
                            doneFlag=True
                            
                            command = gr_Robot_Command()
                            command.id=BOT_ID
                            if BOT_TEAM=='Blue':
                                command.isteamyellow=0
                            else:
                                command.isteamyellow=1
                            command.kickspeedx=0
                            command.kickspeedz=0
                            command.velangular=0
                            command.velnormal=0
                            command.veltangent=0
                            command.spinner=0
                            command.wheelsspeed=0
                            command.orientation=0

                            command1 = gr_Commands()
                            command1.timestamp=0
                            if BOT_TEAM=='Blue':
                                command1.isteamyellow=0
                            else:
                                command1.isteamyellow=1
                            command1.robot_commands = command
                            pub.publish(command1)
            
        # elif(abs(angleOfOrientation)>ROTATION_FACTOR and orientDone==False):
            
        #     while(abs(angleOfOrientation)>ROTATION_FACTOR):
        #         currTime = rospy.Time.now()
        #         currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        #         dt = currTime - prevTimeStamp
        #         if(dt>0.01):
        #             prevTimeStamp = currTime
                    
        #             ###Calculate vw based on PID output
        #             Kpr = 1
        #             Kir = 0
        #             Kdr = 0

        #             rot_error = angleOfOrientation
        #             P_OUT_ROT = rot_error*Kpr

        #             #print("P_OUT:=  ",P_OUT_ROT)

        #             integral+= rot_error*dt

        #             if(integral>2):
        #                 integral=2
        #             elif(integral<-2):
        #                 integral=-2

        #             I_OUT_ROT = integral*Kir

        #             if prevErrorRotation != None:
        #                 D_OUT_ROT = (rot_error-prevErrorRotation)/dt *Kdr
        #             else:
        #                 D_OUT_ROT = 0
        #                 # Set this to error.
        #                 prevErrorRotation =  rot_error

        #             #print(rot_error-self.prevErrorRotation)
        #             #print("P_OUT:= {} ;I_OUT:= {} ; D_OUT:= {} ;".format(P_OUT_ROT,I_OUT_ROT,D_OUT_ROT))

        #             # Calculate final output.
        #             outputRotation = P_OUT_ROT + D_OUT_ROT + I_OUT_ROT
                    
        #             abs_value = abs(outputRotation)
                    
                    

        #             if(abs_value>UPPER_THRESHOLD_FOR_ROTATION):
        #                 abs_value=UPPER_THRESHOLD_FOR_ROTATION
        #             elif (abs_value<LOWER_THRESHOLD_FOR_ROTATION):
        #                 abs_value=LOWER_THRESHOLD_FOR_ROTATION
                    
        #             if(outputRotation<0):
        #                 abs_value = abs_value*(-1)

        #             outputRotation = abs_value
                    
        #             vw=outputRotation
        #             print(vw)
        #             command = gr_Robot_Command()
        #             command.id=BOT_ID
        #             if BOT_TEAM=='Blue':
        #                 command.isteamyellow=0
        #             else:
        #                 command.isteamyellow=1
                    
        #             command.kickspeedx=0
        #             command.kickspeedz=0
        #             command.velangular=vw
        #             command.velnormal=0
        #             command.veltangent=0
        #             command.spinner=0
        #             command.wheelsspeed=0
        #             command.orientation=0

        #             command1 = gr_Commands()
        #             command1.timestamp=0
        #             if BOT_TEAM=='Blue':
        #                 command1.isteamyellow=0
        #             else:
        #                 command1.isteamyellow=1   
        #             command1.robot_commands = command
        #             pub.publish(command1)
        #     print('Stopping')                  
        #     stop()
        #     orientDone=True
        # elif(botBallDistance>140):
        #     velocity_of_bot = 0.40
        #     angle_of_motion = math.pi*0.5
        #     # angle_of_motion = angle_of_motion - my_orientation 
        #     # angle_of_motion += (math.pi*0.5)

        #     vx = velocity_of_bot*math.cos(angle_of_motion)
        #     vy = velocity_of_bot*math.sin(angle_of_motion)

        #     command = gr_Robot_Command()
        #     command.id=BOT_ID
        #     if BOT_TEAM=='Blue':
        #         command.isteamyellow=0
        #     else:
        #         command.isteamyellow=1
            
        #     command.kickspeedx=1
        #     command.kickspeedz=0
        #     command.velangular=0
        #     command.velnormal=vy
        #     command.veltangent=vx
        #     command.spinner=0
        #     command.wheelsspeed=0
        #     command.orientation=0

        #     command1 = gr_Commands()
        #     command1.timestamp=0
        #     if BOT_TEAM=='Blue':
        #         command1.isteamyellow=0
        #     else:
        #         command1.isteamyellow=1
        #     command1.robot_commands = command
        #     pub.publish(command1)    
        
        # else:
        #     #print("stopping")
        #     command = gr_Robot_Command()
        #     command.id=BOT_ID
        #     if BOT_TEAM=='Blue':
        #         command.isteamyellow=0
        #     else:
        #         command.isteamyellow=1
            
        #     command.kickspeedx=1
        #     command.kickspeedz=0
        #     command.velangular=0
        #     command.velnormal=0
        #     command.veltangent=0
        #     command.spinner=1
        #     command.wheelsspeed=0
        #     command.orientation=0

        #     command1 = gr_Commands()
        #     command1.timestamp=0
        #     if BOT_TEAM=='Blue':
        #         command1.isteamyellow=0
        #     else:
        #         command1.isteamyellow=1
        #     command1.robot_commands = command
        #     pub.publish(command1)    
            
        
if __name__=='__main__':

    rospy.init_node('sample_path_planner',anonymous=False)
    rospy.Subscriber('/vision',SSL_DetectionFrame,vision_callback)
    pub =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
    
   
    main()
   # rospy.spin()