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

from enum import Enum,IntEnum



GoalMaxY=300
GoalMinY=-300

GOALIE_X_POSITION_NEAR_GOAL_YELLOW=-1470
GOALIE_X_POSITION_FAR_GOAL_YELLOW=-1250

GOALIE_X_POSITION_NEAR_GOAL_BLUE= 1470
GOALIE_X_POSITION_FAR_GOAL_BLUE= 1250

GOAL_POST_X_YELLOW = 1650
GOAL_POST_X_BLUE = -1650

UPPER_HALF_FIELD_Y = 450
LOWER_HALF_FIELD_Y = -450

ATTACKER_HOME_POSITION_X_YELLOW=500
ATTACKER_HOME_POSITION_Y_YELLOW=-600

ATTACKER_HOME_POSITION_X_BLUE=-500
ATTACKER_HOME_POSITION_Y_BLUE=600

ATTACKER_OFFENSIVE_X_LINE_YELLOW = 150
ATTACKER_OFFENSIVE_X_LINE_BLUE = -150

ATTACKER_ANTHEM_POSITION_X_BLUE=200
ATTACKER_ANTHEM_POSITION_Y_BLUE=1000

ATTACKER_ANTHEM_POSITION_X_YELLOW=-200
ATTACKER_ANTHEM_POSITION_Y_YELLOW=-1000

HALF_FILED_X = 1650
HALF_OF_HALF_FIELD_X = HALF_FILED_X/2 





INV=9999


class thoda_ronaldo_thoda_messi():

    class State(Enum):
        
        ball_apne_teammate_ke_pass_hai=1
        pass_ke_liye_ready_hoja = 2  #ball in oponent court
        ball_Apne_Court_Mai_Hai = 3 # ball in our court
        ball_opp_court_mai_hai =4 # ball in DBox

        ball_Mere_Gaand_Ke_Niche_Hai = 5 # ball close to me ##AAg lagi hai
        ball_ko_intercept_kar=6
        ball_ke_sabse_pass_mai_hai=7
        ball_pass_kar=8
        kuch_mat_kar_bas_game_dekh = 9
        ball_ko_corner_se_clear_kar=10
        ball_ko_clear_kar=11
        jaake_ball_leh_usse=12
        ball_ke_piche_aaja=13
        ball_pe_charge_kar =14 
        #samaj_nahi_ara_kya_karnay =6

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
        self.attackerTeam=team

        self.attackerRobotID=robotId
        self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh
        self.prevState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh

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
        self.considerBallAsObstacle=False
        self.behindBallMotion=False
        self.strikeBall=False
        self.shouldUseRRT=False
        self.forceOrient=False

        if(self.attackerTeam):
            self.anthemPosition = Vector2D(ATTACKER_ANTHEM_POSITION_X_YELLOW,ATTACKER_ANTHEM_POSITION_Y_YELLOW)
            self.homePosition = Vector2D(ATTACKER_HOME_POSITION_X_YELLOW,ATTACKER_HOME_POSITION_Y_YELLOW)
        else:
            self.homePosition = Vector2D(ATTACKER_HOME_POSITION_X_BLUE,ATTACKER_HOME_POSITION_Y_BLUE)
            self.anthemPosition = Vector2D(ATTACKER_ANTHEM_POSITION_X_BLUE,ATTACKER_ANTHEM_POSITION_Y_BLUE)

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

    def beliefdata_callback(self,beliefdata):

        #role definition based on belifedata
        self.attackerTeam = beliefdata.isteamyellow
        self.ballPosition.x,self.ballPosition.y = beliefdata.ballPos.x,beliefdata.ballPos.y
        
        if beliefdata.our_attacker != self.attackerRobotID:
            self.attackerRobotID = beliefdata.our_attacker
            self.gtp_instance = go_to_point(team=self.attackerTeam,robotID=self.attackerRobotID,pubTopic=self.pubTopic)
            self.otp_instance = orient_towards_point(team=self.attackerTeam,robotID=self.attackerRobotID,pubTopic=self.pubTopic)
            self.ball_prediction = ball_tactics(team=self.attackerTeam)

            if(self.attackerTeam):
                self.homePosition = Vector2D(ATTACKER_HOME_POSITION_X_YELLOW,ATTACKER_HOME_POSITION_Y_YELLOW)
                self.goalPost  =  Vector2D(GOAL_POST_X_YELLOW,0)
            else:
                self.homePosition = Vector2D(ATTACKER_HOME_POSITION_X_BLUE,ATTACKER_HOME_POSITION_Y_BLUE)
                self.goalPost  =  Vector2D(GOAL_POST_X_BLUE,0)

            if(self.attackerTeam):
                self.anthemPosition = Vector2D(ATTACKER_ANTHEM_POSITION_X_YELLOW,ATTACKER_ANTHEM_POSITION_Y_YELLOW)
            
            else:
                self.anthemPosition = Vector2D(ATTACKER_ANTHEM_POSITION_X_BLUE,ATTACKER_ANTHEM_POSITION_Y_BLUE)



                # self.goalieMappedPoints = [[ Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,0), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,GoalMinY+100)],
                #                    [ Vector2D(GOALIE_X_POSITION_FAR_GOAL_YELLOW,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_YELLOW,0), Vector2D(GOALIE_X_POSITION_FAR_GOAL_YELLOW,GoalMinY+100)]]
            

                # self.goalieMappedPoints = [[ Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,0), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,GoalMinY+100)],
                #                    [ Vector2D(GOALIE_X_POSITION_FAR_GOAL_BLUE,GoalMaxY-100), Vector2D(GOALIE_X_POSITION_NEAR_GOAL_BLUE,0), Vector2D(GOALIE_X_POSITION_FAR_GOAL_BLUE,GoalMinY+100)]]

            self.firstCall=False


        if(self.firstCall==False):


            if (beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.idle or
                beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.stop_game or 
                beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.minion_Anthem):

                self.gtp_instance.set_upper_threshold(0.7)
                self.gtp_instance.dribble_in_motion(False)
                self.otp_instance.dribble_in_motion(False)

                if(self.kickReady == False):
                    self.gtp_instance.kick_in_motion(True)
                    self.otp_instance.kick_in_motion(True)
                    self.kickReady=True
                else:
                    self.gtp_instance.kick_in_motion(False)
                    self.otp_instance.kick_in_motion(False)

                self.shouldUseRRT = True
                self.considerBallAsObstacle=False
                self.moveToPoint = Vector2D(self.anthemPosition.x,self.anthemPosition.y)
                self.orientTowardsPoint = Vector2D(self.anthemPosition.x,0)


               
            elif(beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.take_positions or 
                   beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.goal_scored):
                
                self.gtp_instance.set_upper_threshold(0.7)
                self.gtp_instance.dribble_in_motion(False)
                self.otp_instance.dribble_in_motion(False)

                if(self.kickReady == False):
                    self.gtp_instance.kick_in_motion(True)
                    self.otp_instance.kick_in_motion(True)
                    self.kickReady=True

                else:
                    self.gtp_instance.kick_in_motion(False)
                    self.otp_instance.kick_in_motion(False)

                
                self.shouldUseRRT = True
                self.considerBallAsObstacle=False
                if(self.attackerTeam):
                    self.moveToPoint = Vector2D(-400,0)
                else:
                    self.moveToPoint = Vector2D(300,0)
                
                self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

            elif(beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.kick_off):
                
                if(not self.attackerTeam):
                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    
                    elif(self.kickReady and (self.get_current_timestamp()-self.lastKickTime)>2):
                        if(self.gtp_instance.currentPosition.dist(self.ballPosition)<160):
                            self.gtp_instance.kick_in_motion(True)
                            self.otp_instance.kick_in_motion(True)
                            self.kickReady=False
                            self.lastKickTime=self.get_current_timestamp()

                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    self.considerBallAsObstacle=False
                    self.shouldUseRRT=False

                    angleDiff= self.gtp_instance.currentPosition.angle(self.ballPosition)    # Vector2D(ball_pos[0],ball_pos[1])
                    angleDiff = angleDiff - self.gtp_instance.currentOrientation
                    angleDiff  = self.gtp_instance.currentPosition.normalizeAngle(angleDiff)
                    

                    if(math.fabs(self.gtp_instance.currentPosition.y-self.ballPosition.y)>80):
                        self.moveToPoint = Vector2D(self.gtp_instance.currentPosition.x,self.ballPosition.y)
                    elif(math.fabs(angleDiff)<1.0472):
                        self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    self.forceOrient=True

            elif(beliefdata.ref_command == thoda_ronaldo_thoda_messi.gameStates.game_on):

                self.gtp_instance.set_upper_threshold(1)
                if not beliefdata.ballDetected:
                    self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh

                elif beliefdata.ball_in_our_possession:
                    
                    if beliefdata.ball_in_whose_possession==self.attackerRobotID:              
                        
                        if(self.attackerTeam):
                            if (self.gtp_instance.currentPosition.x<self.ballPosition.x):
                                angleDiff= self.gtp_instance.currentPosition.angle(self.ballPosition)    # Vector2D(ball_pos[0],ball_pos[1])
                                angleDiff = angleDiff - self.gtp_instance.currentOrientation
                                angleDiff  = self.gtp_instance.currentPosition.normalizeAngle(angleDiff)
                                if(math.fabs(angleDiff)<0.610865):
                                    self.myState = thoda_ronaldo_thoda_messi.State.ball_Mere_Gaand_Ke_Niche_Hai
                        
                        else:

                            if (self.gtp_instance.currentPosition.x>self.ballPosition.x):
                                angleDiff= self.gtp_instance.currentPosition.angle(self.ballPosition)    # Vector2D(ball_pos[0],ball_pos[1])
                                angleDiff = angleDiff - self.gtp_instance.currentOrientation
                                angleDiff  = self.gtp_instance.currentPosition.normalizeAngle(angleDiff)
                                if(math.fabs(angleDiff)<0.610865):
                                    self.myState = thoda_ronaldo_thoda_messi.State.ball_Mere_Gaand_Ke_Niche_Hai

                    else:                                
                        ##Go to Home position
                        self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh

                elif beliefdata.ball_in_opp_possession:
                    
                    if beliefdata.ball_in_our_half:
                        self.myState = thoda_ronaldo_thoda_messi.State.ball_ko_intercept_kar
                    
                    else:
                        if(beliefdata.ball_in_opp_dbox):
                            self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh
                        else:
                            self.myState = thoda_ronaldo_thoda_messi.State.jaake_ball_leh_usse


                elif beliefdata.our_bot_closest_to_ball == self.attackerRobotID:

                    if (beliefdata.ball_at_corners and not beliefdata.ball_in_our_dbox and not beliefdata.ball_in_opp_dbox
                            and beliefdata.closest_bot_to_ball==self.attackerRobotID):
                            ##try to get it out of the corner box
                            ##move to point
                            self.myState = thoda_ronaldo_thoda_messi.State.ball_ko_corner_se_clear_kar
                        
                    elif (beliefdata.ball_in_our_half and not beliefdata.ball_in_our_dbox):
                        
                        if(self.attackerTeam):
                            if self.gtp_instance.currentPosition.x>self.ballPosition.x:
                                self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh
                            else:
                                
                                ### clear kar ball
                                ##orient karke clear kar
                                self.myState  = thoda_ronaldo_thoda_messi.State.ball_ko_clear_kar
                        else:
                            if self.gtp_instance.currentPosition.x<self.ballPosition.x:
                                self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh
                            else:
                                
                                ### clear kar ball
                                ##orient karke clear kar
                                self.myState  = thoda_ronaldo_thoda_messi.State.ball_ko_clear_kar

                    elif(not beliefdata.ball_in_opp_dbox):
                        ### goal scoring option
                        #if(beliefdata.closest_bot_to_ball==self.attackerRobotID):
                        if(self.attackerTeam):
                            if self.gtp_instance.currentPosition.x>self.ballPosition.x-100:
                                ##ball ke piche aa
                                self.myState = thoda_ronaldo_thoda_messi.State.ball_ke_piche_aaja
                            else:
                                self.myState = thoda_ronaldo_thoda_messi.State.ball_pe_charge_kar


                        else:
                            
                            if self.gtp_instance.currentPosition.x<self.ballPosition.x+100:
                                ##ball ke piche aa
                                self.myState = thoda_ronaldo_thoda_messi.State.ball_ke_piche_aaja
                            else:
                                self.myState = thoda_ronaldo_thoda_messi.State.ball_pe_charge_kar

                        # else:
                        #     self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh
                    else:

                        self.myState = thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh
                    



                ################decision making based on state#################
                if(self.myState == thoda_ronaldo_thoda_messi.State.ball_Mere_Gaand_Ke_Niche_Hai):
                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady and (self.get_current_timestamp()-self.lastKickTime)>2):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=False
                        self.lastKickTime=self.get_current_timestamp()

                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    self.shouldUseRRT = False
                    self.considerBallAsObstacle=False
                    self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    self.orientTowardsPoint = Vector2D(self.goalPost.x,self.goalPost.y)
                
                elif(self.myState == thoda_ronaldo_thoda_messi.State.kuch_mat_kar_bas_game_dekh):

                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    self.shouldUseRRT = True
                    self.considerBallAsObstacle=False
                    self.moveToPoint = Vector2D(self.homePosition.x,self.homePosition.y)
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                elif(self.myState == thoda_ronaldo_thoda_messi.State.ball_ke_piche_aaja):

                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    nextPointToMove = self.ball_prediction.find_point_behind_ball_towards_goal(radius1=500)

                    if(nextPointToMove is not None):
                        self.moveToPoint = Vector2D(nextPointToMove.x,nextPointToMove.y)
                    else:
                        self.moveToPoint = Vector2D(self.gtp_instance.currentPosition.x,self.gtp_instance.currentPosition.y)

                    self.considerBallAsObstacle=False
                    self.shouldUseRRT=False
                    
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                
                elif(self.myState == thoda_ronaldo_thoda_messi.State.ball_ko_corner_se_clear_kar):

                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    clearBallPoint=Vector2D()
                    if(self.ballPosition.x>1500):
                        clearBallPoint.x = self.ballPosition.x-100
                    elif(self.ballPosition.x<-1500):
                        clearBallPoint.x = self.ballPosition.x+100
                    else:
                        clearBallPoint.x = self.ballPosition.x
                    
                    if(self.ballPosition.y>1200):
                        clearBallPoint.y = self.ballPosition.y-100
                    elif(self.ballPosition.y<-1200):
                        clearBallPoint.y = self.ballPosition.y+100
                    else:
                        clearBallPoint.y = self.ballPosition.y

                    self.considerBallAsObstacle=False
                    self.shouldUseRRT=False
                    self.moveToPoint = Vector2D(clearBallPoint.x,clearBallPoint.y)
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                elif(self.myState == thoda_ronaldo_thoda_messi.State.ball_ko_clear_kar):

                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    self.considerBallAsObstacle=False
                    self.shouldUseRRT=False
                    self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                elif(self.myState == thoda_ronaldo_thoda_messi.State.ball_pe_charge_kar):

                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)

                    self.considerBallAsObstacle=False
                    self.shouldUseRRT=False

                    angleDiff= self.gtp_instance.currentPosition.angle(self.ballPosition)    # Vector2D(ball_pos[0],ball_pos[1])
                    angleDiff = angleDiff - self.gtp_instance.currentOrientation
                    angleDiff  = self.gtp_instance.currentPosition.normalizeAngle(angleDiff)
                    

                    if(math.fabs(self.gtp_instance.currentPosition.y-self.ballPosition.y)>80):
                        self.moveToPoint = Vector2D(self.gtp_instance.currentPosition.x,self.ballPosition.y)
                    elif(math.fabs(angleDiff)<1.0472):
                        self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    self.forceOrient=True


                elif(self.myState == thoda_ronaldo_thoda_messi.State.jaake_ball_leh_usse):

                    self.gtp_instance.dribble_in_motion(False)
                    self.otp_instance.dribble_in_motion(False)

                    if(self.kickReady == False):
                        self.gtp_instance.kick_in_motion(True)
                        self.otp_instance.kick_in_motion(True)
                        self.kickReady=True
                    else:
                        self.gtp_instance.kick_in_motion(False)
                        self.otp_instance.kick_in_motion(False)
                    
                    self.considerBallAsObstacle=False
                    self.shouldUseRRT=False
                    if(self.attackerTeam):
                        self.moveToPoint = Vector2D(ATTACKER_HOME_POSITION_X_YELLOW,self.ballPosition.y)
                    else:
                        self.moveToPoint = Vector2D(ATTACKER_HOME_POSITION_X_BLUE,self.ballPosition.y)
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)


                
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

                    self.shouldUseRRT = True
                    self.considerBallAsObstacle=False
                    self.moveToPoint = Vector2D(self.homePosition.x,self.homePosition.y)
                    self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)

                print(self.myState)

            if(self.moveToPoint.x==INV or self.moveToPoint.y==INV):
                pass

            elif(math.fabs(self.moveToPoint.x-self.prevMoveToPoint.x)>10 or 
                    math.fabs(self.moveToPoint.y-self.prevMoveToPoint.y)>10 ):

                if(self.moveToPoint.x>1520):
                    self.moveToPoint.x = 1520
                elif(self.moveToPoint.x<-1520):
                    self.moveToPoint.x = -1520

                if(self.moveToPoint.y>1200):
                    self.moveToPoint.y = 1200
                elif(self.moveToPoint.y<-1200):
                    self.moveToPoint.y = -1200

                self.otp_instance.abort_orient_motion()
                self.gtp_instance.update_goal(goalPoint=self.moveToPoint,useRRT=self.shouldUseRRT,
                                                considerBallAsObstacle=self.considerBallAsObstacle)
                
                self.prevMoveToPoint.x,self.prevMoveToPoint.y = self.moveToPoint.x,self.moveToPoint.y

                print('Next Point : ',self.moveToPoint.x,self.moveToPoint.y)
                
            
            if(self.orientTowardsPoint.x==INV or self.orientTowardsPoint.y==INV):
                pass
            elif(math.fabs(self.orientTowardsPoint.x-self.prevOrientTowardsPoint.x)>100 or 
                    math.fabs(self.orientTowardsPoint.y-self.prevOrientTowardsPoint.y)>100 or self.forceOrient):
                
                self.otp_instance.update_orient_point(orientPoint=self.orientTowardsPoint)
                self.prevOrientTowardsPoint.x,self.prevOrientTowardsPoint.y = self.orientTowardsPoint.x,self.orientTowardsPoint.y
                self.forceOrient=False
            


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
        rospy.init_node("attackerYellow_updated",anonymous=False)
        edinsonCavani = thoda_ronaldo_thoda_messi()
        rospy.Subscriber("/belief_state_output_yellow",BeliefState,edinsonCavani.beliefdata_callback,queue_size=2)
    else:
        rospy.init_node("attackerBlue_updated",anonymous=False)
        edinsonCavani = thoda_ronaldo_thoda_messi()
        rospy.Subscriber("/belief_state_output_blue",BeliefState,edinsonCavani.beliefdata_callback,queue_size=2)
    
    
    rospy.Subscriber('/ballposition',point_2d,edinsonCavani.Ball_pos_callback, queue_size=2)
    rospy.Subscriber('/vision',SSL_DetectionFrame,edinsonCavani.vision_pos_callback, queue_size=2)
    rospy.Subscriber('/ballposition',point_2d,edinsonCavani.ball_prediction_callback)

    spinRate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        if (edinsonCavani.gtp_instance is not None and edinsonCavani.otp_instance is not None):
            if(edinsonCavani.gtp_instance.gtp_spin()):
                
                #print("************** Reached*********")
                edinsonCavani.otp_instance.otp_spin()
                    
                    
        spinRate.sleep()

