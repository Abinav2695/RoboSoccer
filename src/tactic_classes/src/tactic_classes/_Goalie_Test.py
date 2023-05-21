#!/usr/bin/env python3
import rospy
import sys

from robot_messages.msg import gr_Robot_Command, gr_Commands,SSL_DetectionFrame,my_pos,BeliefState
from _Go_To_Point import go_to_point

from utils_updated.geometry_functions.geometry_functions import Vector2D

from utils_updated.geometry_functions.geometry_functions import Line as Line2D
from utils_updated.geometry_functions.geometry_functions import Circle as Circle2D

from robot_messages.msg import point_2d
from _Orient_Towards_Point import orient_towards_point

from _Ball_Tracker import ball_tactics
import math

from enum import Enum
from enum import IntEnum


GoalMaxY=300
GoalMinY=-300

GOALIE_X_POSITION_NEAR_GOAL_YELLOW=-1400
GOALIE_X_POSITION_FAR_GOAL_YELLOW=-1250

GOALIE_X_POSITION_NEAR_GOAL_BLUE= 1400
GOALIE_X_POSITION_FAR_GOAL_BLUE= 1250

GOALIE_ANTHEM_POSITION_X_BLUE=900
GOALIE_ANTHEM_POSITION_Y_BLUE=1000

GOALIE_ANTHEM_POSITION_X_YELLOW=-900
GOALIE_ANTHEM_POSITION_Y_YELLOW=-1000

UPPER_HALF_FIELD_Y = 450
LOWER_HALF_FIELD_Y = -450


HALF_FILED_X = 1650
HALF_OF_HALF_FIELD_X = HALF_FILED_X/2 

GOALIE_MAX_VELOCITY=1



INV=9999


class buffon_ka_baap():

    class State(Enum):
        ball_ko_track_kar = 1  #ball in oponent court
        ball_Apne_Court_Mai_Hai = 2 # ball in our court
        ball_Apne_Hadde_Mai_Hai =3 # ball in DBox

        ball_Mere_Gaand_Ke_Niche_Hai = 4 # ball close to me ##AAg lagi hai

        samaj_nahi_ara_kya_karnay =5

    class gameStates(IntEnum):
        idle=0
        minion_Anthem=1
        take_positions=2
        kick_off=3
        game_on=4
        goal_scored=5
        pause_game=6
        stop_game=7

    def __init__(self,team=False,robotId=99): ##default values are set to useless values
        self.goalieTeam=team
        self.goalieRobotID=robotId
        self.myState = buffon_ka_baap.State.ball_ko_track_kar
        self.prevState = buffon_ka_baap.State.samaj_nahi_ara_kya_karnay

        self.pubTopic =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
        
        # self.gtp_instance = go_to_point(team=self.goalieTeam,robotId=self.goalieRobotID,pubTopic=self.pubTopic)
        # self.otp_instance = go_to_point(team=self.goalieTeam,robotId=self.goalieRobotID,pubTopic=self.pubTopic)

        self.gtp_instance =None
        self.otp_instance = None
        self.ball_prediction=None

        
        self.firstCall=True
        self.ballPosition = Vector2D(0,0)
        self.prevBallPosition = Vector2D(0,0)
        self.moveToPoint=Vector2D(INV,INV)
        self.prevMoveToPoint = Vector2D(INV,INV)
        self.orientTowardsPoint = Vector2D(INV,INV)
        self.prevOrientTowardsPoint = Vector2D(INV,INV)
        self.kickReady = False
        self.lastKickTime = self.get_current_timestamp()
        self.newPredictedPointToMove=None

        if (self.goalieTeam):
            self.anthemPosition = Vector2D(GOALIE_ANTHEM_POSITION_X_YELLOW,GOALIE_ANTHEM_POSITION_Y_YELLOW)
            self.homePosition = Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,0)
        else:
            self.anthemPosition = Vector2D(GOALIE_ANTHEM_POSITION_X_BLUE,GOALIE_ANTHEM_POSITION_Y_BLUE)
            self.homePosition = Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,0)

       
    
    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    def vision_pos_callback(self,vision_data):
        
        if (self.gtp_instance is not None and self.otp_instance is not None):

            self.gtp_instance.update_robot_position(vision_data)
            self.otp_instance.update_robot_position(vision_data)

    def Ball_pos_callback(self,ball_data):
        
        if (self.gtp_instance is not None and self.otp_instance is not None):
            self.gtp_instance.update_ball_position(ball_data)
            self.otp_instance.update_ball_position(ball_data)

    def ball_prediction_callback(self,ball1_data):
        
        if (self.ball_prediction is not None):
            self.ball_prediction.update_ball_vector(ball1_data)

    def map_goalie_position(self):
        rowNumber=0
        columnNumber=0

        if(math.fabs(self.ballPosition.x)<HALF_OF_HALF_FIELD_X):
            rowNumber=1
        else:
            rowNumber=0

        if(self.ballPosition.y<LOWER_HALF_FIELD_Y):
            columnNumber=2
        elif(self.ballPosition.y<UPPER_HALF_FIELD_Y):
            columnNumber=1
        else:
            columnNumber=0

        point1 = self.goalieMappedPoints[rowNumber][columnNumber]
        if columnNumber==1:
            point1.y = self.ballPosition.y
        return point1
        
    def beliefdata_callback(self,beliefdata):

        
        #role definition based on belifedata
        self.goalieTeam = beliefdata.isteamyellow
        self.ballPosition.x,self.ballPosition.y = beliefdata.ballPos.x,beliefdata.ballPos.y
        

        


        if beliefdata.our_goalie != self.goalieRobotID:
            self.goalieRobotID = beliefdata.our_goalie
            self.gtp_instance = go_to_point(team=self.goalieTeam,robotID=self.goalieRobotID,pubTopic=self.pubTopic)
            self.gtp_instance.set_upper_threshold(GOALIE_MAX_VELOCITY)
            
            self.otp_instance = orient_towards_point(team=self.goalieTeam,robotID=self.goalieRobotID,pubTopic=self.pubTopic)
            self.ball_prediction = ball_tactics(team=self.goalieTeam)

            if(self.goalieTeam):
                self.homePosition = Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,0)
                self.anthemPosition = Vector2D(GOALIE_ANTHEM_POSITION_X_YELLOW,GOALIE_ANTHEM_POSITION_Y_YELLOW)

                self.goalieMappedPoints = [[ Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,0), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,GoalMinY+100)],
                                   [ Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,0), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,GoalMinY+100)]]
                
            else:
                self.homePosition = Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,0)

                self.anthemPosition = Vector2D(GOALIE_ANTHEM_POSITION_X_BLUE,GOALIE_ANTHEM_POSITION_Y_BLUE)

                self.goalieMappedPoints = [[ Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,0), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,GoalMinY+100)],
                                   [ Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,0), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,GoalMinY+100)]]

            self.firstCall=False

        if(self.firstCall==False):
           

            if (beliefdata.ref_command == buffon_ka_baap.gameStates.idle or
                beliefdata.ref_command == buffon_ka_baap.gameStates.stop_game or 
                beliefdata.ref_command == buffon_ka_baap.gameStates.minion_Anthem):

                self.gtp_instance.dribble_in_motion(False)
                self.otp_instance.dribble_in_motion(False)

                self.gtp_instance.kick_in_motion(False)
                self.otp_instance.kick_in_motion(False)

                
                self.moveToPoint = Vector2D(self.anthemPosition.x,self.anthemPosition.y)
                self.orientTowardsPoint = Vector2D(self.anthemPosition.x,0)

            elif(beliefdata.ref_command == buffon_ka_baap.gameStates.take_positions or 
                   beliefdata.ref_command == buffon_ka_baap.gameStates.goal_scored):

                self.gtp_instance.dribble_in_motion(False)
                self.otp_instance.dribble_in_motion(False)

                self.gtp_instance.kick_in_motion(False)
                self.otp_instance.kick_in_motion(False)

                self.gtp_instance.set_upper_threshold(0.7)
                self.moveToPoint = Vector2D(self.homePosition.x,self.homePosition.y)
                self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
            
            elif(beliefdata.ref_command == buffon_ka_baap.gameStates.game_on):

                self.gtp_instance.set_upper_threshold(1)
                if beliefdata.ball_in_our_possession and beliefdata.ball_in_whose_possession==self.goalieRobotID:
                    #golie should show some movements over y axis over fixed x
                    
                    self.myState = buffon_ka_baap.State.ball_Mere_Gaand_Ke_Niche_Hai
        

                elif beliefdata.ball_in_our_dbox:
                        
                    self.myState = buffon_ka_baap.State.ball_Apne_Hadde_Mai_Hai

                elif beliefdata.ball_in_our_half:
                
                    self.myState = buffon_ka_baap.State.ball_Apne_Court_Mai_Hai

                else:

                    self.myState = buffon_ka_baap.State.ball_ko_track_kar



                #### Track ball speed and movement direction to decide should the bot move or stay still

                if(self.ball_prediction.is_ball_travelling_towards_goal()): 

                    #print('Ball is travelling towards Goal!!')
                    
                    random_line = Line2D(point1 = Vector2D(self.homePosition.x,GoalMaxY),
                                                point2=Vector2D(self.homePosition.x,GoalMinY))
                    #self.newPredictedPointToMove = self.ball_prediction.find_next_point_in_line_of_motion(lineOfIntersection = random_line)

                    self.newPredictedPointToMove = self.ball_prediction.find_next_point_in_line_of_motion(time=0.5)

                    if(self.newPredictedPointToMove is not None):
                        if self.newPredictedPointToMove.y>GoalMaxY:
                            self.newPredictedPointToMove.y=GoalMaxY

                        elif self.newPredictedPointToMove.y<GoalMinY:
                            self.newPredictedPointToMove.y=GoalMinY
                        

                        print('Predicted Point = ',self.newPredictedPointToMove.x,self.newPredictedPointToMove.y)
                        
                    

                #########***********************############

                if(self.myState == buffon_ka_baap.State.ball_Apne_Court_Mai_Hai): ### change in Y will make robot move
                    
                    ##check for ball change in y direction in future
                    if(math.fabs(self.ballPosition.x)>HALF_OF_HALF_FIELD_X):
                        self.gtp_instance.dribble_in_motion(True)
                        self.otp_instance.dribble_in_motion(True)
                    else:
                        self.gtp_instance.dribble_in_motion(False)
                        self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                        
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    ####map ball region for positioning goalie at best point
                    newPoint = self.map_goalie_position()
                    self.moveToPoint = Vector2D(newPoint.x,newPoint.y)

                    if(self.moveToPoint.y>GoalMaxY-100):
                        self.moveToPoint.y = GoalMaxY-100
                    elif(self.moveToPoint.y<GoalMinY+100):
                        self.moveToPoint.y = GoalMinY+100
                    
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)


                    ### not checkinh previous state update instead chenge goalpoint and in the end 
                    ### if goalPoint has been updated then only upate gtp class

                
                elif(self.myState==buffon_ka_baap.State.ball_ko_track_kar):  ### ball out of half field
                    
                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    self.gtp_instance.kick_in_motion(False)
                    self.otp_instance.kick_in_motion(False)

                    
                    self.moveToPoint = Vector2D(self.homePosition.x,self.homePosition.y)
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                

                

                elif(self.myState==buffon_ka_baap.State.ball_Apne_Hadde_Mai_Hai):


                    self.gtp_instance.dribble_in_motion(True)
                    self.otp_instance.dribble_in_motion(True)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                        
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)
                    
                    
                    if self.newPredictedPointToMove is not None:
                        ballVelocity = self.ball_prediction.get_ball_velocity()
                        if(ballVelocity>400):
                            self.moveToPoint = Vector2D(self.newPredictedPointToMove.x,self.newPredictedPointToMove.y)
                        else:
                            self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    else:
                        self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    
                    if(math.fabs(self.moveToPoint.x)>1450):
                        self.moveToPoint.x = self.homePosition.x
                        
                    
                    
                elif(self.myState==buffon_ka_baap.State.ball_Mere_Gaand_Ke_Niche_Hai):
                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady and (self.get_current_timestamp()-self.lastKickTime)>2):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=False
                        self.lastKickTime=self.get_current_timestamp()

                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    #self.gtp_instance.execute_kick_immediately()
            



            
            if(self.moveToPoint.x==INV or self.moveToPoint.y==INV):
                pass
            elif(math.fabs(self.moveToPoint.x-self.prevMoveToPoint.x)>10 or 
                    math.fabs(self.moveToPoint.y-self.prevMoveToPoint.y)>10):
                self.gtp_instance.update_goal(goalPoint=self.moveToPoint)
                self.otp_instance.abort_orient_motion()
                self.prevMoveToPoint.x,self.prevMoveToPoint.y = self.moveToPoint.x,self.moveToPoint.y

                print('Next Point : ',self.moveToPoint.x,self.moveToPoint.y)
            
            if(self.orientTowardsPoint.x==INV or self.orientTowardsPoint.y==INV):
                pass
            elif(math.fabs(self.orientTowardsPoint.x-self.prevOrientTowardsPoint.x)>100 or 
                    math.fabs(self.orientTowardsPoint.y-self.prevOrientTowardsPoint.y)>100):
                self.otp_instance.update_orient_point(orientPoint=self.orientTowardsPoint)
                self.prevOrientTowardsPoint.x,self.prevOrientTowardsPoint.y = self.orientTowardsPoint.x,self.orientTowardsPoint.y

        
            


if __name__=='__main__':
    
    
    
    
    
    arguments = rospy.myargv(sys.argv)

    try:
        currentTeam = int(arguments[1])
        if currentTeam>0:
            currentTeam=True
        else:
            currentTeam=False
            
    except:
        currentTeam=False
    
    if(currentTeam):

        rospy.init_node("goalieYellow_updated",anonymous=False)
        meraGoalie = buffon_ka_baap()
        rospy.Subscriber("/belief_state_output_yellow",BeliefState,meraGoalie.beliefdata_callback,queue_size=2)
    else:
        rospy.init_node("goalieBlue_updated",anonymous=False)
        meraGoalie = buffon_ka_baap()

        rospy.Subscriber("/belief_state_output_blue",BeliefState,meraGoalie.beliefdata_callback,queue_size=2)
    
    rospy.Subscriber('/ballposition',point_2d,meraGoalie.Ball_pos_callback, queue_size=2)
    rospy.Subscriber('/vision',SSL_DetectionFrame,meraGoalie.vision_pos_callback, queue_size=2)
    rospy.Subscriber('/ballposition',point_2d,meraGoalie.ball_prediction_callback)

    spinRate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        if (meraGoalie.gtp_instance is not None and meraGoalie.otp_instance is not None):
            if(meraGoalie.gtp_instance.gtp_spin()):
                #print("=------------=")
                if(meraGoalie.otp_instance.otp_spin()):
                    pass
                    # print("=------------=")
                
        spinRate.sleep()