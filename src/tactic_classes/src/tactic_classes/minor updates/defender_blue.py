#!/usr/bin/env python3
import rospy
from robot_messages.msg import gr_Robot_Command, gr_Commands
from _Go_To_Point import go_to_point
from utils_updated.geometry_functions.geometry_functions import Vector2D
from robot_messages.msg import point_2d

GoalX=1552.76550293
BoxX=1174.40368652

YellowFieldCenterX=-672.96697998
BlueFieldCenterX=693.09387207

BoxMinY=-591.715454102
BoxMaxY=620.080444336

GoalMinY=-247.530349731
GoalMaxY=222.955490112


#print("haaai")

boatID=0
isteamyellow=False
#ball_pos[None]*2
ballX=None
ballY=None
STATE=0
pub =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
#craete instance of goto class for goalie

previousBallY=4000

def Ball_pos_callback(data):
	#print(data)
	global ballX,ballY,previousBallY
	#print(data.x)
	ballX=data.x
	ballY=data.y
	if ballY !=None:

		if STATE==2:#follow ball if inside my side
			#if ballY > BoxMaxY:
			#	ballY=BoxMaxY
			#if ballY < BoxMinY:
				#ballY=BoxMinY
			#if (ballY-previousBallY>60):

			
			if(abs(ballY-previousBallY)>100):
				print("State1 abs value:",abs(ballY-previousBallY))
				previousBallY=ballY
			if my_defender_blue.needToReplanRRT(Vector2D(ballX,ballY)): # simply check is ther is any obstracle present in the vector2d line from start to end pos/goal pos

				my_defender_blue.update_goal(goalPoint=Vector2D(ballX,ballY),useRRT=True)
			else:
				my_defender_blue.update_goal(goalPoint=Vector2D(ballX,ballY))

			
			
		if STATE==1:#goalie wil be inside goal post
			#if ballY > GoalMaxY:
			#	ballY=GoalMaxY
			#if ballY < GoalMinY:
			#	ballY=GoalMinYSS
			#if (abs(ballY-previousBallY)>60):
			
			if(abs(ballY-previousBallY)>100):
				print("State2 abs value:",abs(ballY-previousBallY))
				previousBallY=ballY
			if my_defender_blue.needToReplanRRT(Vector2D(BlueFieldCenterX,ballY)):

				my_defender_blue.update_goal(goalPoint=Vector2D(BlueFieldCenterX,ballY),useRRT=True)
			else:
				my_defender_blue.update_goal(goalPoint=Vector2D(BlueFieldCenterX,ballY))

			# previousBallY=ballY
			

if __name__=='__main__':
	#
	#STATE=1
	#in blue x -ve means ball in opposit side --blue basil side
	rospy.Subscriber('/ballposition',point_2d,Ball_pos_callback, queue_size=2)
	rospy.init_node('defender__test_blue',anonymous=True)
	my_defender_blue=go_to_point(boatID,isteamyellow,pubTopic=pub)
	
	while not rospy.is_shutdown():
		if ballX != None:
			if ballX > 0:
				STATE=2
			else :
				STATE=1
		if(my_defender_blue.gtp_spin()):
			print('Goal Reached')
