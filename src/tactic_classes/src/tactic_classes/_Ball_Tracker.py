#!/usr/bin/env python3

import rospy
import sys
import math
import time


from robot_messages.msg import gr_Robot_Command, gr_Commands,point_2d,my_pos,obstacle,SSL_DetectionFrame

from utils_updated.geometry_functions.geometry_functions import Vector2D
from utils_updated.geometry_functions.geometry_functions import Line as Line2D
from utils_updated.geometry_functions.geometry_functions import Circle as Circle2D

BOT_RADIUS=110
INV=9999

YELLOW_GOAL_CENTER=Vector2D(-1650,0)
BLUE_GOAL_CENTER=Vector2D(1650,0)
YELLOW_GOAL_POST_MAX = Vector2D(-1650,300)
YELLOW_GOAL_POST_MIN = Vector2D(-1650,-300)

BLUE_GOAL_POST_MAX = Vector2D(1650,300)
BLUE_GOAL_POST_MIN = Vector2D(1650,-300)



class ball_tactics():

    def __init__(self,team):

        self.team =team
        self.ballPosition = Vector2D(INV,INV)
        self.prevBallPosition = Vector2D(INV,INV)
        self.prevTimeStamp=self.get_current_timestamp()

        self.ballVelocityVector = Vector2D(INV,INV)
        self.ballVelocityMag=0
    
    def update_ball_vector(self,ballData):
        self.ballPosition.x = ballData.x
        self.ballPosition.y = ballData.y

        
        if(self.get_current_timestamp()-self.prevTimeStamp>0.01):
            
            
            self.ballVelocityVector = self.ballPosition.__sub__(self.prevBallPosition)

            
            
            magVector = Vector2D(self.ballVelocityVector.x/0.01,self.ballVelocityVector.y/0.01)

            self.ballVelocityMag = magVector.magnitude()

            self.prevBallPosition.x=self.ballPosition.x
            self.prevBallPosition.y =self.ballPosition.y
            self.prevTimeStamp=self.get_current_timestamp()
            

    def find_next_point_in_line_of_motion(self,distance=None,time=0,lineOfIntersection=None):

        if (self.ballPosition.x==INV or self.prevBallPosition.x==INV):
            return None
        distanceToCover=0
        nextPoint=None
        if distance is not None:
            distanceToCover = distance
            normalVelocityVector = self.ballVelocityVector.__norm__()
            nextPoint =  self.ballPosition.__add__(normalVelocityVector.__mul__(distanceToCover))
            
            
        elif lineOfIntersection is not None:
            ballLineOfMotion = Line2D(point1 = self.ballPosition,point2=self.prevBallPosition)
            otherLine = lineOfIntersection
            nextPoint = ballLineOfMotion.intersection_with_line(otherLine)


        else:
            distanceToCover = time*self.ballVelocityMag  ##time in seconds
            normalVelocityVector = self.ballVelocityVector.__norm__()
            nextPoint =  self.ballPosition.__add__(normalVelocityVector.__mul__(distanceToCover))

        if nextPoint is None:
            return None
        elif(math.fabs(nextPoint.x)>1500 or math.fabs(nextPoint.y)>1200):
            return None
        else:
            return nextPoint
    
    def find_point_behind_ball_towards_goal(self,radius1=300):

        if (self.ballPosition.x==INV or self.prevBallPosition.x==INV):
            return None

        if(self.team):
            goalPoint = BLUE_GOAL_CENTER
        else:
            goalPoint = YELLOW_GOAL_CENTER

        ballToGoalLine = Line2D(point1 = self.ballPosition,point2=goalPoint)
        ballCircle = Circle2D(center=self.ballPosition, radius=radius1)

        (a,b) = ballCircle.intersection_with_line(ballToGoalLine)
        if(a.x<b.x):
            towardsYellowPoint = a
            towardsBluePoint=b
        else:
            towardsYellowPoint = b
            towardsBluePoint=a
        if(self.team):
           
            return towardsYellowPoint
        else:
            
            return towardsBluePoint
        
    def get_ball_velocity(self):
        return self.ballVelocityMag

    
    def is_ball_travelling_towards_goal(self):
        
        if (self.ballPosition.x==INV or self.prevBallPosition.x==INV):
            return False

        if(self.ballVelocityMag>200):
            
            if self.team:
                if(self.ballVelocityVector.x<0):
                    goalPoint = YELLOW_GOAL_CENTER
                    ballToGoalLine = Line2D(point1 = self.ballPosition,point2=goalPoint)
                    goalPostLine = Line2D(point1 = YELLOW_GOAL_POST_MAX,point2=YELLOW_GOAL_POST_MIN)

                    intersectionPoint = ballToGoalLine.intersection_with_line(goalPostLine)
                    if(YELLOW_GOAL_POST_MIN.y<= intersectionPoint.y <=YELLOW_GOAL_POST_MAX.y):
                        return True
            else:  
                if(self.ballVelocityVector.x>0):
                    goalPoint = BLUE_GOAL_CENTER
                    ballToGoalLine = Line2D(point1 = self.ballPosition,point2=goalPoint)
                    goalPostLine = Line2D(point1 = BLUE_GOAL_POST_MAX,point2=BLUE_GOAL_POST_MIN)

                    intersectionPoint = ballToGoalLine.intersection_with_line(goalPostLine)
                    if(BLUE_GOAL_POST_MIN.y<= intersectionPoint.y <=BLUE_GOAL_POST_MAX.y):
                        return True

        return False




    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)



if __name__=="__main__":
    
    rospy.init_node('ball_tactic_test',anonymous=False)
    newTactic = ball_tactics(team=True)

    rospy.Subscriber('/ballposition',point_2d,newTactic.update_ball_vector)

    while not rospy.is_shutdown():
        #time.sleep(3)

        i = int(input())

        if(i==1):
            destination = newTactic.find_next_point_in_line_of_motion(distance=500)
            print(destination.x,destination.y)

        elif(i==2):
            myLine = Line2D(point1 = Vector2D(900,200),point2=Vector2D(900,-200))
            destination = newTactic.find_next_point_in_line_of_motion(lineOfIntersection=myLine)
            
            print(destination.x,destination.y)
        
        elif(i==3):
            print(newTactic.is_ball_travelling_towards_goal())
        
        elif(i==4):
            destination = newTactic.find_point_behind_ball_towards_goal()
            print(destination.x,destination.y)

        elif(i==5):
            print(newTactic.get_ball_velocity())