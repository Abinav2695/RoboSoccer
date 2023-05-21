#!/usr/bin/env python3

from robot_messages.msg import SSL_DetectionFrame
from geometry import Vector2D
import rospy
import math
from robot_messages.msg import gr_Robot_Command
from std_msgs.msg import String
import time

class Roles:

    SHOOT_REACH_OFFSET = 150
    PI = 3.14159265358979323
    BOT_RADIUS = 200
    REACH_OFFSET = BOT_RADIUS + SHOOT_REACH_OFFSET
    SELF_GOAL_POINT = Vector2D(-2000, 0)
    
    def __init__(self):

        rospy.init_node('roles', anonymous=False)

        self.ball_pos = Vector2D(0,0)
        self.bot_pos = Vector2D(0,0)
        self.goal_pos = Vector2D(-2000,0)
        self.bot_orientation = 0.0
        self.bot_orientation_gyro = 0.0
        self.delta_theta = 0.0

        self.field_update = False
        self.error_prev = 0
        self.error_int = 0
        self.time_stamp = time.time()

        self.msg_publish = rospy.Publisher('/grsim_data', gr_Robot_Command, queue_size=1)

        rospy.Subscriber('vision', SSL_DetectionFrame, self.callbackForData, queue_size=1)
        rospy.Subscriber('/gyroDataBlue9', String, self.callbackForGyroData, queue_size=1)

    def defence(self):
        self.positionToGo()
        pass

    def positionToGo(self):  
        pass
    
    def offence(self):

        shootPoint = self.shootPoint()
        shoot_allowed = self.sendCommand(shootPoint)

        spinner = 0
        if shoot_allowed:
            rospy.loginfo("First shoot allowed!!!")
            spinner = 1
            new_pose = self.__goalToBallOffsetPos(Roles.SHOOT_REACH_OFFSET)
            shoot_allowed = self.sendCommand(new_pose, spinner=spinner)

        if spinner:
            rospy.loginfo("Kick has to be made by the bot!!!!")
            command = gr_Robot_Command()
            command.id = 3
            command.kickspeedx = 1
            command.spinner = 0
            self.msg_publish.publish(command)

    def sendCommand(self, shootPoint, Iid=9, kickspeedx=0, kickspeedz=0, spinner=0, wheelsspeed=0, orientation=0):
        
        UPPER_THRESHOLD = 0.4
        LOWER_THRESHOLD = 1.0
        point_to_reach = Vector2D(shootPoint[0], shootPoint[1])
        
        Kp = 1
        Kd = 0.5
        Ki = 0.01
        error = self.bot_pos.dist(point_to_reach)
        self.error_int += error
        dt = time.time() - self.time_stamp
        self.time_stamp = time.time()
        error_dot = (self.error_prev - error) / dt
        velocity_of_bot = Kp*error + Kd*error_dot + Ki*self.error_int
        if velocity_of_bot < LOWER_THRESHOLD:
            velocity_of_bot = LOWER_THRESHOLD
        if velocity_of_bot > UPPER_THRESHOLD:
            velocity_of_bot = UPPER_THRESHOLD
        self.error_prev = error

        rospy.loginfo("Bot orientation :- " + str(self.bot_orientation) + "\n Shoot orientation :- " + str(shootPoint[2])
                        + " \n Bot orientation - Shoot orientation :- " + str((self.bot_orientation - shootPoint[2])))
        if math.fabs(self.bot_orientation - shootPoint[2]) >= math.pi/12:
            velocity_angular = -1*0.8*(math.fabs(self.bot_orientation - shootPoint[2])/(self.bot_orientation - shootPoint[2]))
        else:
            velocity_angular = 0

        rospy.loginfo(" Velocity Selected is :- " + str(velocity_angular))
        
        command = gr_Robot_Command()

        angle_of_motion = self.bot_pos.angle(point_to_reach)
        angle_of_motion = angle_of_motion - self.bot_orientation 

        
        angle_of_motion += (math.pi*0.5)
        
        velocity_tangent = velocity_of_bot*math.cos(angle_of_motion)
        velocity_normal  = velocity_of_bot*math.sin(angle_of_motion)
        
        command.id = Iid
        command.kickspeedx = kickspeedx
        command.kickspeedz = kickspeedz
        command.veltangent = velocity_tangent
        command.velnormal = velocity_normal
        command.velangular = velocity_angular
        command.spinner = spinner
        command.wheelsspeed = wheelsspeed
        command.orientation = orientation

        rospy.loginfo("Current bot position :- "+str(self.bot_pos)+" Point to go to :- "+str(shootPoint[0])+","+str(shootPoint[1]))
        distance = math.sqrt((self.bot_pos.x - shootPoint[0])**2 + (self.bot_pos.y - shootPoint[1])**2)
        rospy.loginfo("Current distance :- "+str(distance))
        if distance >= 100:
            command.velangular = 0
            self.msg_publish.publish(command)

        if distance <= 100 and math.fabs(self.bot_orientation-shootPoint[2]) > math.pi/12:
            command.velnormal = 0
            command.veltangent = 0
            self.msg_publish.publish(command)

        if distance <= 100 and math.fabs(self.bot_orientation-shootPoint[2]) <= math.pi/12:
            command.velangular = 0
            command.velnormal = 0
            command.veltangent = 0
            self.msg_publish.publish(command)
            return True

        else:
            return False
        
    def shootPoint(self):
        pose = self.__goalToBallOffsetPos(Roles.REACH_OFFSET)
        return pose

    def __goalToBallOffsetPos(self, REACH):
        angle = self.goal_pos.angle(self.ball_pos)
        reach_pos_x = self.ball_pos.x + REACH*math.cos(angle)
        reach_pos_y = self.ball_pos.y + REACH*math.sin(angle)
        bot_orientation = math.pi + angle + math.pi/6

        return (reach_pos_x, reach_pos_y, bot_orientation)

    def callbackForData(self, data):
        
        defenderRobot = data.robots_blue[9]
        ballData = data.balls[0]

        self.ball_pos.x  = ballData.x
        self.ball_pos.y = ballData.y 

        self.bot_pos.x = defenderRobot.x
        self.bot_pos.y = defenderRobot.y
        self.bot_orientation = defenderRobot.orientation
        rospy.loginfo("Bot orientation :- " + str(self.bot_orientation))

        self.field_update = True

    def callbackForGyroData(self, msg):

        data = (msg.data).split(',')        
        self.bot_orientation_gyro = int(data[0])*(math.pi/180)
        rospy.loginfo("Gyro angle in radians is :- "+str(self.bot_orientation_gyro))
        

    def control(self):

        offence = False
        defence = True

        while not rospy.is_shutdown():

            offence = True
            defence = True

            if self.field_update:

                if offence:
                    self.offence()
                    offence = False
                
                self.field_update = False


if __name__=="__main__":
    
    roles = Roles()
    roles.control()