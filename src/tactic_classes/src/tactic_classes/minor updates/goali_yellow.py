#!/usr/bin/env python3
import rospy
from robot_messages.msg import gr_Robot_Command, gr_Commands
from _Go_To_Point import go_to_point
from utils_updated.geometry_functions.geometry_functions import Vector2D
from robot_messages.msg import point_2d

GoalX=-1578.2244873
BoxX=-1200.01599121

BoxMinY=-584.798278809
BoxMaxY=521.611022949

GoalMinY=-240.494400024
GoalMaxY=161.511962891

#print("haaai")

boatID=4
isteamyellow=True
#ball_pos[None]*2
ballX=None
ballY=None
STATE=1
pub =rospy.Publisher ("/gotopoint_output",gr_Commands, queue_size=2)
#craete instance of goto class for goalie



def Ball_pos_callback(data):
	#print(data)
	global ballX,ballY
	print(data.x)
	ballX=data.x
	ballY=data.y
	if ballY !=None:

		if STATE==1:#goalie will be inside goal box
			if ballY > BoxMaxY:
				ballY=BoxMaxY
			if ballY < BoxMinY:
				ballY=BoxMinY
			my_goalie.update_goal(goalPoint=Vector2D(BoxX,ballY))
		if STATE==2:#goalie wil be inside goal post
			if ballY > GoalMaxY:
				ballY=GoalMaxY
			if ballY < GoalMinY:
				ballY=GoalMinY
			my_goalie.update_goal(goalPoint=Vector2D(GoalX,ballY))
			#if ball inside the box --ball x is  bla bla bot shuld follw the ball


if __name__=='__main__':
	global ballY
	STATE=1
	rospy.Subscriber('/ballposition',point_2d,Ball_pos_callback, queue_size=2)
	rospy.init_node('golaie__test_yellow',anonymous=True)
	my_goalie=go_to_point(boatID,isteamyellow,pubTopic=pub,useRRT=True)
	
	while not rospy.is_shutdown():

		if ballX > 0:
			STATE=1
		else :
			STATE=2
        
		if(my_goalie.gtp_spin()):
			print('Goal Reached')


