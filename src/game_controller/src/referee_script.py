#!/usr/bin/env python3


import rospy
import roslaunch

import sys
import os
import rospkg

import time
import math


import vlc 
from enum import IntEnum
 

from utils_updated.geometry_functions.geometry_functions import Vector2D

from utils_updated.geometry_functions.geometry_functions import Line as Line2D
from utils_updated.geometry_functions.geometry_functions import Circle as Circle2D

from robot_messages.msg import BeliefState,gr_Robot_Command, gr_Commands,point_2d,my_pos,obstacle,SSL_DetectionFrame
from std_msgs.msg import UInt8,String

# creating vlc media player object 
media_player = vlc.MediaPlayer() 

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# list all packages, equivalent to rospack list
rospack.list() 

# get the file path for rospy_tutorials
mp3filePath = rospack.get_path('game_controller')
anthemSound = mp3filePath + '/src/media_files/minion_anthem.mp3'

kickOffSound = mp3filePath + '/src/media_files/kickoff_whistle.mp3'
endMatchSound = mp3filePath + '/src/media_files/end_match_whistle.mp3'
  

class referee():

    class State(IntEnum):
        idle=0
        minion_Anthem=1
        take_positions=2
        kick_off=3
        game_on=4
        goal_scored=5
        pause_game=6
        stop_game=7

    
    def __init__(self):
        self.myState = referee.State.idle
        self.prevState = referee.State.idle
        self.ballPosition = Vector2D(0,0)
        self.blueGoalCounter=0
        self.yellowGoalCounter=0
        self.TOTAL_GAME_TIME = 90
        self.gameStartTime=0
        self.gameStartFirstCall=False
        self.resetGoalValues=False

        self.pubTopic = rospy.Publisher('/referee_game_instructions', UInt8, queue_size=5)
        self.pubTopic2 = rospy.Publisher('/referee_topic', String, queue_size=5)

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    def Ball_pos_callback(self,ball_data):
        
        self.ballPosition.x = ball_data.x
        self.ballPosition.y = ball_data.y

        if(self.myState==referee.State.game_on and math.fabs(self.ballPosition.x)>1650):
            if(-300<self.ballPosition.y<300):
                self.myState = referee.State.goal_scored
                if(self.ballPosition.x<0):
                    self.blueGoalCounter+=1
                else:
                    self.yellowGoalCounter+=1


    def server_callback(self,server_data):
        print(server_data)
        if server_data.data=="start":
            print('Start command received')
            

            self.myState = referee.State.minion_Anthem
        elif server_data.data=="stop":
            self.myState = referee.State.stop_game
    

    def start_task(self):
        print("starting nodes..")

        package = "tactic_classes"

        attacker1_exec = "_Attacker_Test.py" 
        attacker2_exec = "_Attacker_Test.py" 

        goalie1_exec = "_Goalie_Test.py" 
        goalie2_exec = "_Goalie_Test.py" 

        defender1_exec = "_Defender_blue_v3.py" 
        defender2_exec = "_Defender_yellow_v3.py" 
        
        
        node1 = roslaunch.core.Node(package,goalie1_exec,args='0',output='screen')
        node2 = roslaunch.core.Node(package,goalie2_exec,args='1',output='screen')
        node3 = roslaunch.core.Node(package,attacker1_exec,args='0',output='screen')
        node4 = roslaunch.core.Node(package,attacker2_exec,args='1',output='screen')
        node5 = roslaunch.core.Node(package,defender1_exec)
        node6 = roslaunch.core.Node(package,defender2_exec)



        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        self.script1 = launch.launch(node1)
        self.script2 = launch.launch(node2)
        self.script3 = launch.launch(node3)
        self.script4 = launch.launch(node4)
        self.script5 = launch.launch(node5)
        self.script6 = launch.launch(node6)


        print(self.script1.is_alive())
    


        # script.stop()
        # print(script.is_alive())

    def stop_tasks(self):

        self.script1.stop()
        self.script2.stop()
        self.script3.stop()
        self.script4.stop()
        self.script5.stop()
        self.script6.stop()
               
if __name__ == "__main__":
    
    rospy.init_node('referee_node', anonymous=False)
    naya_referee = referee()

    rospy.Subscriber('/server_events',String,naya_referee.server_callback)
    rospy.Subscriber('/ballposition',point_2d,naya_referee.Ball_pos_callback, queue_size=2)

    
    spinRate = rospy.Rate(100)

    while not rospy.is_shutdown():
        
        if(naya_referee.myState==naya_referee.State.idle):
            naya_referee.pubTopic.publish(naya_referee.myState)
            time.sleep(1)
            #print(naya_referee.myState)
            naya_referee.blueGoalCounter=0
            naya_referee.yellowGoalCounter=0
            naya_referee.gameStartFirstCall=False
            if(not naya_referee.resetGoalValues):
                naya_referee.resetGoalValues=True
                naya_referee.pubTopic.publish(naya_referee.myState)
                goalUpdateData = 'goal:'+str(naya_referee.blueGoalCounter)+','+str(naya_referee.yellowGoalCounter)
            

        if(naya_referee.myState==naya_referee.State.minion_Anthem):
            # media object 
            naya_referee.start_task()
            time.sleep(2)
            print(naya_referee.myState)
            naya_referee.pubTopic.publish(naya_referee.myState)
            media = vlc.Media(anthemSound) 
            
            # setting media to the media player 
            media_player.set_media(media) 
            #media_player.play()
            #time.sleep(27)
            media_player.stop()
            naya_referee.myState=naya_referee.State.take_positions

        elif(naya_referee.myState==naya_referee.State.take_positions):  ###################3hereeeeeeeeeee#########
            print(naya_referee.myState)
            naya_referee.pubTopic.publish(naya_referee.myState)
            startTime = naya_referee.get_current_timestamp()
            take_position_timeout = 2.0
            while(naya_referee.get_current_timestamp()-startTime<
                    take_position_timeout):
                
                naya_referee.pubTopic.publish(naya_referee.myState)
                time.sleep(0.1)
            
            media = vlc.Media(kickOffSound) 
            
            # setting media to the media player 
            media_player.set_media(media) 
            media_player.play()
            time.sleep(2)
            media_player.stop()

            naya_referee.myState=naya_referee.State.kick_off

        elif(naya_referee.myState==naya_referee.State.kick_off):
            print(naya_referee.myState)
            naya_referee.pubTopic.publish(naya_referee.myState)
            if(math.fabs(naya_referee.ballPosition.x)>200 or 
                math.fabs(naya_referee.ballPosition.y)>200 ):
                
                naya_referee.myState=naya_referee.State.game_on
                naya_referee.pubTopic.publish(naya_referee.myState)
                if(not naya_referee.gameStartFirstCall):

                    naya_referee.gameStartTime = naya_referee.get_current_timestamp()
                    naya_referee.gameStartFirstCall=True
            
        elif(naya_referee.myState==naya_referee.State.game_on):
            print(naya_referee.myState)
            if(naya_referee.get_current_timestamp()- naya_referee.gameStartTime>naya_referee.TOTAL_GAME_TIME):

                naya_referee.myState=naya_referee.State.stop_game
                naya_referee.pubTopic2.publish('stop')
                naya_referee.pubTopic.publish(naya_referee.myState)
                media = vlc.Media(endMatchSound) 
            
                # setting media to the media player 
                media_player.set_media(media) 
                media_player.play()
                time.sleep(5)
                media_player.stop()
                time.sleep(5)
                naya_referee.stop_tasks()
                naya_referee.resetGoalValues=False
                naya_referee.myState=naya_referee.State.idle

        elif(naya_referee.myState==naya_referee.State.goal_scored):

            print(naya_referee.myState)
            naya_referee.pubTopic.publish(naya_referee.myState)
            goalUpdateData = 'goal:'+str(naya_referee.blueGoalCounter)+','+str(naya_referee.yellowGoalCounter)
            naya_referee.pubTopic2.publish(goalUpdateData)#publish goal values
            naya_referee.prevState = naya_referee.myState
            media = vlc.Media(kickOffSound) 
            
            # setting media to the media player 
            media_player.set_media(media) 
            media_player.play()
            time.sleep(3)
            media_player.stop()

            naya_referee.myState = naya_referee.State.take_positions
            #time.sleep(10)
            naya_referee.pubTopic.publish(naya_referee.myState)
            #time.sleep(10)


        spinRate.sleep()

    
