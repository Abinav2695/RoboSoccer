#!/usr/bin/env python3

import rospy
import sys
import math
import time


from robot_messages.msg import gr_Robot_Command, gr_Commands
from robot_messages.msg import point_2d,my_pos,obstacle,SSL_DetectionFrame,BeliefState
from std_msgs.msg import UInt8,String

from utils_updated.geometry_functions.geometry_functions import Vector2D
from utils_updated.path_planner.rrt_planner import RRTStar

yellow_bots_Vision_data=[None]*6
blue_bots_Vision_data=[None]*6

prev_home_detected=[]
prev_away_detected=[]

BOT_BALL_NEAR_THRESHOLD=200
BALL_IN_POSSESSION_THRESHOLD = 200
BOT_MARKING_THRESHOLD = 500

MAX_BOTS_PER_TEAM=3

HALF_FIELD_X=1650
HALF_FIELD_Y=1350

DBOX_X_BLUE = 1100
DBOX_X_YELLOW = -1100

DBOX_Y_MAX = 650
DBOX_Y_MIN = -650



class belief_state():

    
    def __init__(self,team=False,pubTopic=None):
    
        self.team = team
        self.firstCall=True
        self.pubTopic = pubTopic
        self.gameState = 0
    
    def referee_callback(self,state):
        self.gameState=state.data
       

    def bf_callback(self,robotPositions):


        global yellow_bots_Vision_data
        global blue_bots_Vision_data
        
        ##To store ids of home and away robots
        home_detected=[]
        away_detected=[]

        gameUpdateMessage = BeliefState()
        gameUpdateMessage.isteamyellow=self.team
        gameUpdateMessage.ref_command = self.gameState

        for i in range(0,6):
            #change list initialization
            if robotPositions.robots_yellow[i].confidence >0.8:
                #yellow_bots_Vision_data[i]=data.robots_yellow[i]
                yellow_bots_Vision_data[i]={"x":robotPositions.robots_yellow[i].x,
                        "y":robotPositions.robots_yellow[i].y,"orientation":robotPositions.robots_yellow[i].orientation}

                if self.team:
                    home_detected.append(i)
                else:
                    away_detected.append(i)

            if robotPositions.robots_blue[i].confidence >0.8:
                blue_bots_Vision_data[i]={"x":robotPositions.robots_blue[i].x,
                        "y":robotPositions.robots_blue[i].y,"orientation":robotPositions.robots_blue[i].orientation}
            #full_data=[yellow_bots_Vision_data,blue_bots_Vision_data] #not using for  now 
                if not self.team:
                    home_detected.append(i)
                else:
                    away_detected.append(i)
        
        ##Sort in ascending order
        home_detected.sort()
        away_detected.sort()


        if self.firstCall:
     
            if(len(home_detected)==MAX_BOTS_PER_TEAM):
                gameUpdateMessage.our_goalie = home_detected[0]
                gameUpdateMessage.our_defender = home_detected[1]
                gameUpdateMessage.our_attacker = home_detected[2]


                for i in range (0,MAX_BOTS_PER_TEAM):
                    prev_home_detected.append(home_detected[i])

                self.firstCall=False
                print('first call done')
            # if(len(away_detected)>2):
            #     gameUpdateMessage.opp_goalie = away_detected[0]
            #     gameUpdateMessage.opp_defender = away_detected[1]
            #     gameUpdateMessage.opp_attacker = away_detected[2]

        else:

            ### If all 3 are detected in current frame check for any change in Ids
            if(len(home_detected)==MAX_BOTS_PER_TEAM):
                if(home_detected!=prev_home_detected):

                    gameUpdateMessage.our_goalie = home_detected[0]
                    gameUpdateMessage.our_defender = home_detected[1]
                    gameUpdateMessage.our_attacker = home_detected[2]
                    for i in range (0,MAX_BOTS_PER_TEAM):
                        prev_home_detected[i]=home_detected[i]
                    
                else:
                    gameUpdateMessage.our_goalie = prev_home_detected[0]
                    gameUpdateMessage.our_defender = prev_home_detected[1]
                    gameUpdateMessage.our_attacker = prev_home_detected[2]
            
            ### If any bot has fluctuations in detection
            else:
                gameUpdateMessage.our_goalie = prev_home_detected[0]
                gameUpdateMessage.our_defender = prev_home_detected[1]
                gameUpdateMessage.our_attacker = prev_home_detected[2]
            
            
            

            ######## Ball position based flags######################

            if(robotPositions.balls[0].confidence>0.0):
                gameUpdateMessage.ballDetected=True

            else:
                gameUpdateMessage.ballDetected=False

            gameUpdateMessage.ball_in_our_possession=False

            ballPosition=Vector2D()
            ballPosition.x = robotPositions.balls[0].x
            ballPosition.y = robotPositions.balls[0].y

            ballPosition_point2D = point_2d()
            ballPosition_point2D.x,ballPosition_point2D.y = ballPosition.x,ballPosition.y
            
            gameUpdateMessage.ballPos = ballPosition_point2D

            ### check for ball at corners###
            if(math.fabs(ballPosition.x) >1500 or math.fabs(ballPosition.y)>1200):
                gameUpdateMessage.ball_at_corners = True

            
            ###*******************FOR YELLOW TEAM*************##############
            if(self.team):

                ourAttacker = Vector2D(yellow_bots_Vision_data[prev_home_detected[2]]['x'],
                                    yellow_bots_Vision_data[prev_home_detected[2]]['y'])

                if(ballPosition.x<0):
                    gameUpdateMessage.ball_in_our_half=True

                    if(ballPosition.x<DBOX_X_YELLOW):
                        if(DBOX_Y_MIN<ballPosition.y<DBOX_Y_MAX):
                            gameUpdateMessage.ball_in_our_dbox=True
                    
                    

                else:
                    gameUpdateMessage.ball_in_our_half=False

                    if(ballPosition.x>DBOX_X_BLUE):
                        if(DBOX_Y_MIN<ballPosition.y<DBOX_Y_MAX):
                            gameUpdateMessage.ball_in_opp_dbox=True
                

                our_bot_closest_to_ball = 0  #dummy value
                our_closest_distance = 4999  #dummy value
                opp_bot_closest_to_ball = 0  #dummy value
                opp_closest_distance = 4999  #dummy value
                ifAnyMarker=0
                

                for i in range (len(home_detected)):
                    currentRobot = Vector2D(yellow_bots_Vision_data[home_detected[i]]['x'],
                                            yellow_bots_Vision_data[home_detected[i]]['y'])
                    ball_bot_distance = currentRobot.dist(ballPosition)
                    if (ball_bot_distance<our_closest_distance):
                        our_bot_closest_to_ball = home_detected[i]
                        our_closest_distance = ball_bot_distance
                        if(our_closest_distance<BALL_IN_POSSESSION_THRESHOLD):
                            gameUpdateMessage.ball_in_our_possession=True
                            gameUpdateMessage.ball_in_opp_possession=False
                            gameUpdateMessage.ball_in_whose_possession = home_detected[i]



                for i in range (len(away_detected)):
                    currentRobot = Vector2D(blue_bots_Vision_data[away_detected[i]]['x'],
                                            blue_bots_Vision_data[away_detected[i]]['y'])



                    ball_bot_distance = currentRobot.dist(ballPosition)
                    attacker_marker_distacne = currentRobot.dist(ourAttacker)

                    if (ball_bot_distance<opp_closest_distance):
                        opp_bot_closest_to_ball = away_detected[i]
                        opp_closest_distance = ball_bot_distance
                        if(opp_closest_distance<BALL_IN_POSSESSION_THRESHOLD):
                            gameUpdateMessage.ball_in_our_possession=False
                            gameUpdateMessage.ball_in_opp_possession=True
                            gameUpdateMessage.ball_in_whose_possession = away_detected[i]


                    if(attacker_marker_distacne<BOT_MARKING_THRESHOLD):
                        ifAnyMarker+=1
                        gameUpdateMessage.opp_bot_marking_our_attacker=away_detected[i]
                
                if(ifAnyMarker<1):
                    gameUpdateMessage.opp_bot_marking_our_attacker=99

                if(our_closest_distance<opp_closest_distance):
                    gameUpdateMessage.closest_bot_to_ball = our_bot_closest_to_ball
                else:
                    gameUpdateMessage.closest_bot_to_ball = opp_bot_closest_to_ball
                
                gameUpdateMessage.our_bot_closest_to_ball = our_bot_closest_to_ball
                gameUpdateMessage.opp_bot_closest_to_ball = opp_bot_closest_to_ball
        


            ###*******************FOR BLUE TEAM*************##############
            else:

                ourAttacker = Vector2D(blue_bots_Vision_data[prev_home_detected[2]]['x'],
                                    blue_bots_Vision_data[prev_home_detected[2]]['y'])

                if(ballPosition.x>=0):
                    gameUpdateMessage.ball_in_our_half=True

                    if(ballPosition.x>DBOX_X_BLUE):
                        if(DBOX_Y_MIN<ballPosition.y<DBOX_Y_MAX):
                            gameUpdateMessage.ball_in_our_dbox=True
                    
                    

                else:
                    gameUpdateMessage.ball_in_our_half=False
                    if(ballPosition.x<DBOX_X_YELLOW):
                        if(DBOX_Y_MIN<ballPosition.y<DBOX_Y_MAX):
                            gameUpdateMessage.ball_in_opp_dbox=True
                

                our_bot_closest_to_ball = 0  #dummy value
                our_closest_distance = 4999  #dummy value
                opp_bot_closest_to_ball = 0  #dummy value
                opp_closest_distance = 4999  #dummy value
                ifAnyMarker=0
                

                for i in range (len(home_detected)):
                    currentRobot = Vector2D(blue_bots_Vision_data[home_detected[i]]['x'],
                                            blue_bots_Vision_data[home_detected[i]]['y'])
                    ball_bot_distance = currentRobot.dist(ballPosition)
                    if (ball_bot_distance<our_closest_distance):
                        our_bot_closest_to_ball = home_detected[i]
                        our_closest_distance = ball_bot_distance
                        if(our_closest_distance<BALL_IN_POSSESSION_THRESHOLD):
                            gameUpdateMessage.ball_in_our_possession=True
                            gameUpdateMessage.ball_in_whose_possession = home_detected[i]



                for i in range (len(away_detected)):
                    currentRobot = Vector2D(yellow_bots_Vision_data[away_detected[i]]['x'],
                                            yellow_bots_Vision_data[away_detected[i]]['y'])



                    ball_bot_distance = currentRobot.dist(ballPosition)
                    attacker_marker_distacne = currentRobot.dist(ourAttacker)

                    if (ball_bot_distance<opp_closest_distance):
                        opp_bot_closest_to_ball = away_detected[i]
                        opp_closest_distance = ball_bot_distance
                        if(opp_closest_distance<BALL_IN_POSSESSION_THRESHOLD):
                            gameUpdateMessage.ball_in_our_possession=False

                    if(attacker_marker_distacne<BOT_MARKING_THRESHOLD):
                        ifAnyMarker+=1
                        gameUpdateMessage.opp_bot_marking_our_attacker=away_detected[i]
                
                if(ifAnyMarker<1):
                    gameUpdateMessage.opp_bot_marking_our_attacker=99

                if(our_closest_distance<opp_closest_distance):
                    gameUpdateMessage.closest_bot_to_ball = our_bot_closest_to_ball
                else:
                    gameUpdateMessage.closest_bot_to_ball = opp_bot_closest_to_ball
                
                gameUpdateMessage.our_bot_closest_to_ball = our_bot_closest_to_ball
                gameUpdateMessage.opp_bot_closest_to_ball = opp_bot_closest_to_ball
                gameUpdateMessage.our_bot_closest_to_ball = our_bot_closest_to_ball
                gameUpdateMessage.opp_bot_closest_to_ball = opp_bot_closest_to_ball

                
            self.pubTopic.publish(gameUpdateMessage)    



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
        nodeName='belief_state_yellow'
        topicName='belief_state_output_yellow'
    
    else:
        nodeName='belief_state_blue'
        topicName='belief_state_output_blue'

    rospy.init_node(nodeName,anonymous=False)

    pub =rospy.Publisher (topicName,BeliefState, queue_size=2)

    playTactic = belief_state(team=currentTeam,pubTopic=pub)
    # rospy.Subscriber('/ballposition',point_2d,minion1.update_ball_position)
    
    rospy.Subscriber('/vision',SSL_DetectionFrame,playTactic.bf_callback)
    rospy.Subscriber('/referee_game_instructions',UInt8,playTactic.referee_callback)
    
    rospy.spin()    

    # while not rospy.is_shutdown():
    #     pass