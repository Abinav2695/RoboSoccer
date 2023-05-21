#!/usr/bin/env python3
import rospy
from robot_messages.msg import gr_Robot_Command, gr_Commands,SSL_DetectionFrame,my_pos,BeliefState
from _Go_To_Point import go_to_point
from utils_updated.geometry_functions.geometry_functions import Vector2D
from robot_messages.msg import point_2d
from _Orient_Towards_Point import orient_towards_point
import math
from enum import Enum
from enum import IntEnum

INV=9999

class the_defender():
    class State(Enum):
        idle_state=0

        ball_with_me=1

        ball_with_goalie=2

        ball_is_in_my_front=3

        ball_is_in_my_back=4

        ball_comming_to_our_side=5

        ball_opposit_side = 6

        ball_with_my_attacker=7

    class gameStates(IntEnum):
        idle=0
        minion_Anthem=1
        take_positions=2
        kick_off=3
        game_on=4
        goal_scored=5
        pause_game=6
        stop_game=7

    def __init__(self,team=False,robotId=99):
        self.defenderTeam=team
        self.defenderRobotID=robotId
        self.defenderRobotPos=Vector2D(0,0)
        self.myState=the_defender.State.idle_state
        self.myPrevState=the_defender.State.idle_state
        self.pubTopic=rospy.Publisher("/gotopoint_output",gr_Commands,queue_size=3)

        self.gtp_instance=None
        self.otp_instance=None
        self.kickReadyFlag=False
        self.firstCall=True

        self.ballPosition=Vector2D(0,0)
        self.prevBallposition=Vector2D(0,0)
        self.moveToPoint=Vector2D(INV,INV)
        self.prevMoveToPoint=Vector2D(INV,INV)
        self.orientTowardsPoint = Vector2D(INV,INV)
        self.prevOrientTowardsPoint = Vector2D(INV,INV)
        self.kickReady = False
        self.lastKickTime = self.get_current_timestamp()

        self.considerBallAsObstacle=False

        self.anthemPosition = Vector2D(600,1000)


        if(self.defenderTeam):
            self.homePosition=Vector2D(-800,-400)#home pos for yellow team
        else:
            self.homePosition=Vector2D(800,400)#home pos for blue team

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    def vision_pos_callback(self,vision_data):
        
        if (self.gtp_instance is not None and self.otp_instance is not None):

            self.gtp_instance.update_robot_position(vision_data)
            self.otp_instance.update_robot_position(vision_data)
        #update defender_robot current pos once after belife state is updated
        if(self.defenderRobotID!=99):

            if(self.defenderTeam):
                self.defenderRobotPos=Vector2D(vision_data.robots_yellow[self.defenderRobotID].x,vision_data.robots_yellow[self.defenderRobotID].y)
            else:
                self.defenderRobotPos=Vector2D(vision_data.robots_blue[self.defenderRobotID].x,vision_data.robots_blue[self.defenderRobotID].y)
            #print(self.defenderRobotPos.x)
        else:
            print("waiting for belif state ")

    def Ball_pos_callback(self,ball_data): 
        if (self.gtp_instance is not None and self.otp_instance is not None):
            self.gtp_instance.update_ball_position(ball_data)
            self.otp_instance.update_ball_position(ball_data)

    def beliefdata_callback(self,beliefdata):
        #role definition based on belifedata
        self.defenderTeam = beliefdata.isteamyellow
        self.ballPosition.x,self.ballPosition.y = beliefdata.ballPos.x,beliefdata.ballPos.y
        
        if beliefdata.our_defender != self.defenderRobotID:
            self.defenderRobotID = beliefdata.our_defender
            #create obj with new id 
            self.gtp_instance = go_to_point(team=self.defenderTeam,robotID=self.defenderRobotID,pubTopic=self.pubTopic)
            self.otp_instance = orient_towards_point(team=self.defenderTeam,robotID=self.defenderRobotID,pubTopic=self.pubTopic)
        #removed first call ---will add it later if req.
        if (beliefdata.ref_command == the_defender.gameStates.idle or
            beliefdata.ref_command == the_defender.gameStates.stop_game or 
            beliefdata.ref_command == the_defender.gameStates.minion_Anthem):

            print("side pos time")

            self.gtp_instance.set_upper_threshold(0.7)
            self.gtp_instance.dribble_in_motion(False)
            self.otp_instance.dribble_in_motion(False)

            self.gtp_instance.kick_in_motion(False)
            self.otp_instance.kick_in_motion(False)

            
            self.moveToPoint = Vector2D(self.anthemPosition.x,self.anthemPosition.y)
            self.orientTowardsPoint = Vector2D(self.anthemPosition.x,0)

        elif(beliefdata.ref_command == the_defender.gameStates.take_positions or 
                beliefdata.ref_command == the_defender.gameStates.goal_scored):

            self.gtp_instance.set_upper_threshold(0.7)
            self.gtp_instance.dribble_in_motion(False)
            self.otp_instance.dribble_in_motion(False)

            self.gtp_instance.kick_in_motion(False)
            self.otp_instance.kick_in_motion(False)

            
            self.moveToPoint = Vector2D(self.homePosition.x,self.homePosition.y)
            self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
            
        elif(beliefdata.ref_command == the_defender.gameStates.game_on):

            self.gtp_instance.set_upper_threshold(1)
            if beliefdata.ball_in_our_possession and beliefdata.ball_in_whose_possession==self.defenderRobotID:
                if beliefdata.ball_in_our_half and beliefdata.ball_in_our_dbox==False:
                    self.myState = the_defender.State.ball_with_me
                    self.considerBallAsObstacle=False
                    if beliefdata.ballDetected==False:
                        self.myState = the_defender.State.idle_state

                else:
                    self.myState=the_defender.State.idle_state

            elif beliefdata.ball_in_our_possession and beliefdata.ball_in_whose_possession==beliefdata.our_attacker:
                self.myState = the_defender.State.ball_with_my_attacker
                self.considerBallAsObstacle=False



            elif beliefdata.ball_in_our_dbox:
                self.myState=the_defender.State.ball_with_goalie
                self.considerBallAsObstacle=False


            
            elif beliefdata.ball_in_our_half and beliefdata.ball_in_our_dbox==False :
                if self.defenderTeam:
                    if(self.ballPosition.x)<(self.defenderRobotPos.x):
                        self.myState=the_defender.State.ball_is_in_my_back
                        self.considerBallAsObstacle=True
                    else:
                        self.myState=the_defender.State.ball_is_in_my_front
                        self.considerBallAsObstacle=False

                else:
                    if(self.ballPosition.x)>(self.defenderRobotPos.x):
                        self.myState=the_defender.State.ball_is_in_my_back
                        self.considerBallAsObstacle=True
                    else:
                        self.myState=the_defender.State.ball_is_in_my_front
                        self.considerBallAsObstacle=False


            elif beliefdata.ball_in_our_half==False and abs(self.ballPosition.x)< 350:
                self.myState=the_defender.State.ball_comming_to_our_side
            
            elif beliefdata.ball_in_our_half==False:
                self.myState = the_defender.State.ball_opposit_side
            
            else:
                self.myState=the_defender.State.idle_state
            

            if self.myState == the_defender.State.ball_with_me :
                self.gtp_instance.dribble_in_motion(True)
                self.gtp_instance.dribble_in_motion(True)
                self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                self.orientTowardsPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                self.gtp_instance.kick_in_motion(True)
                #if beliefdata.ballDetected==False:



                #self.moveToPoint = Vector2D(self.ballPosition.x+50,self.ballPosition.y)

            elif self.myState == the_defender.State.ball_with_goalie:
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)#pppppppppppppppp
                self.moveToPoint = Vector2D(self.homePosition.x,self.ballPosition.y+200)
                self.orientTowardsPoint=Vector2D(0,0)


            elif self.myState == the_defender.State.ball_with_my_attacker:
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.moveToPoint = Vector2D(self.homePosition.x,self.homePosition.y)
                self.orientTowardsPoint=Vector2D(0,0)
            

            elif self.myState == the_defender.State.ball_is_in_my_front:
                self.gtp_instance.dribble_in_motion(True)
                #self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.orientTowardsPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
                self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                #add kicker here next version

            elif self.myState == the_defender.State.ball_is_in_my_back:
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.orientTowardsPoint=Vector2D(0,0)
                #for blue bot if yellow bot use - ve 
                if(self.defenderTeam):
                    if((abs(self.ballPosition.x)+300)>1400):
                        #if((abs(self.ballPosition.y)+200)<1200):
                            #self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y+200)
                        #else:
                        self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)#yellow
                    else:
                        self.moveToPoint = Vector2D(self.ballPosition.x-300,self.ballPosition.y)
                else:

                    if((abs(self.ballPosition.x)+300)>1400):
                        #if(abs(self.ballPosition.y+200)<1200):
                            #self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y+200)
                        #else:
                        self.moveToPoint = Vector2D(self.ballPosition.x,self.ballPosition.y)
                    else:
                        self.moveToPoint = Vector2D(self.ballPosition.x+300,self.ballPosition.y)
            
            elif self.myState == the_defender.State.ball_comming_to_our_side:
                print("im here")
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.orientTowardsPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
                self.moveToPoint=Vector2D(self.homePosition.x,self.ballPosition.y)

            elif self.myState == the_defender.State.ball_opposit_side:
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.moveToPoint=Vector2D(self.homePosition.x,self.homePosition.y)
                self.orientTowardsPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
                #ad to yellow bakki hey
            elif self.myState == the_defender.State.idle_state:
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.moveToPoint=Vector2D(self.homePosition.x,self.homePosition.y)
                self.orientTowardsPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)
            else:
                print("Invalid state")
                self.gtp_instance.dribble_in_motion(False)
                self.gtp_instance.kick_in_motion(False)
                self.moveToPoint=Vector2D(self.homePosition.x,self.homePosition.y)
                self.orientTowardsPoint=Vector2D(self.ballPosition.x,self.ballPosition.y)

            #print(self.myState)


        if(self.moveToPoint.x >1500):
            self.moveToPoint.x=1500
        if self.moveToPoint.x<-1500:
            self.moveToPoint.x=-1500
        
        if self.moveToPoint.y >1150:
            self.moveToPoint.y=1150
        if self.moveToPoint.y<-1150:
            self.moveToPoint.y=-1150
        

        #elif(-1150 < self.moveToPoint.y < 1150) and (abs(self.moveToPoint.x)<1600):#1500

            #print("valid")
            #if(math.fabs(self.moveToPoint.x-self.prevMoveToPoint.x)>10 or 
                   # math.fabs(self.moveToPoint.y-self.prevMoveToPoint.y)>10):
        #else:
        if(self.moveToPoint.x==INV or self.moveToPoint.y==INV):
            pass
        elif(math.fabs(self.moveToPoint.x-self.prevMoveToPoint.x)>10 or 
                math.fabs(self.moveToPoint.y-self.prevMoveToPoint.y)>10):
            if self.considerBallAsObstacle:
                self.gtp_instance.update_goal(goalPoint=self.moveToPoint,useRRT=True,considerBallAsObstacle=True)
                
            elif self.gtp_instance.needToReplanRRT(self.moveToPoint):
                self.gtp_instance.update_goal(goalPoint=self.moveToPoint,useRRT=True)
            else:
                self.gtp_instance.update_goal(goalPoint=self.moveToPoint)

            self.otp_instance.abort_orient_motion()
            self.prevMoveToPoint.x,self.prevMoveToPoint.y = self.moveToPoint.x,self.moveToPoint.y
        
        if(self.orientTowardsPoint.x==INV or self.orientTowardsPoint.y==INV):
            pass
        elif(math.fabs(self.orientTowardsPoint.x-self.prevOrientTowardsPoint.x)>10 or 
                math.fabs(self.orientTowardsPoint.y-self.prevOrientTowardsPoint.y)>10):
            self.otp_instance.update_orient_point(orientPoint=self.orientTowardsPoint)
            self.prevOrientTowardsPoint.x,self.prevOrientTowardsPoint.y = self.orientTowardsPoint.x,self.orientTowardsPoint.y
        

       #print("x y",self.moveToPoint.x,self.moveToPoint.y)
        #else:

            #print("invalid case aarreeey sending him home")
            #self.gtp_instance.update_goal(goalPoint=self.homePosition)
            #self.otp_instance.abort_orient_motion()
            #self.prevMoveToPoint.x,self.prevMoveToPoint.y = self.homePosition.x,self.homePosition.y
        #print("x and y",self.moveToPoint.x,self.moveToPoint.y)

            




if __name__=='__main__':
    
    
    rospy.init_node("defenderBlue_versionTwo",anonymous=False)
    
    defender = the_defender()
    
    rospy.Subscriber("/belief_state_output_blue",BeliefState,defender.beliefdata_callback,queue_size=2)
    rospy.Subscriber('/ballposition',point_2d,defender.Ball_pos_callback, queue_size=2)
    rospy.Subscriber('/vision',SSL_DetectionFrame,defender.vision_pos_callback, queue_size=2)

    spinRate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        if (defender.gtp_instance is not None and defender.otp_instance is not None):
            if(defender.gtp_instance.gtp_spin()):
                pass
                #print("=------------=")
                if(defender.otp_instance.otp_spin()):
                    pass
                    
                    #print("=------------=")
        spinRate.sleep()


    
