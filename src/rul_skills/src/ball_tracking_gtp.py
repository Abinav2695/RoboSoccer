#!/usr/bin/env python3
import rospy
import sys

from service_hub.srv import pid,pidResponse
from service_hub.srv import ball,ballResponse
from service_hub.srv import mypos,myposResponse

from utils_updated.geometry_functions.geometry_functions import Vector2D
from rrt_star_with_pathsmoothing import RRTStar

checkOnce=False
goToNextPoint=False
nextPoint=Vector2D(0,0)
pathInstance = RRTStar()

prevTimeStamp=0
newPath=None
pathIterator=0

team =1
robotID = 3

def calculate_pid_output(error,prevError,dt):
    rospy.wait_for_service('pid_service')
    try:
        pid_output = rospy.ServiceProxy('pid_service', pid)
        response = pid_output(error,prevError,dt)
        return response.pid_output
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None
    
def get_nextPoint_data(distToCover):
    rospy.wait_for_service('ball_velocity_service')
    try:
        nextPointFunction = rospy.ServiceProxy('ball_velocity_service', ball)
        response = nextPointFunction(distToCover)
        return Vector2D(response.x,response.y)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None
    
def get_robot_position(robotID,team):
    rospy.wait_for_service('pos_service') 
    try:
        pos_data = rospy.ServiceProxy('pos_service', mypos)
        response = pos_data(robotID,team)
        #print(response)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None


def velocityCallback(velocityOfBall):
    global checkOnce
    global goToNextPoint
    global nextPoint

    if(velocityOfBall>0.3 and not checkOnce):

        nextPoint = get_nextPoint_data(1000)
        if(nextPoint is not None):
            goToNextPoint=True
            checkOnce=True
        


def main():

    global checkOnce
    global goToNextPoint
    global nextPoint
    global prevTimeStamp
    global newPath
    global pathIterator

    rospy.init_node('ball_gtp_tracking_node',anonymous=False)
    #rospy.Subscriber('/ballVelocity',velocityCallback)
    #robotPositionData = mypos()

    while not rospy.is_shutdown():
        # if(goToNextPoint):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        dt = currTime - prevTimeStamp
        
        if(dt>0.1):
            prevTimeStamp =currTime
            robotPositionData = get_robot_position(team=team,robotID=robotID)
            
            startNode = (robotPositionData.my_pos_data.x,robotPositionData.my_pos_data.y)
            #goalNode = (get_nextPoint_data.x,get_nextPoint_data.y)
            obstacleList = robotPositionData.my_pos_data.obstacle_list
            print(obstacleList)

            # if(newPath==None):
            #     #stop()
                
                    
            #     newPath = pathInstance.update(start=startNode,goal = goalNode,
            #                             obstacleList= obstacleList)
            #     pathIterator = 0
            
if __name__=="__main__":
    main()