#!/usr/bin/env python3

from robot_messages.msg import SSL_DetectionFrame
from geometry import Vector2D
import rospy
import math
from robot_messages.msg import gr_Robot_Command, gr_Commands
from std_msgs.msg import String
import time
import sys
from argparse import ArgumentParser
import json

class Roles:

    SHOOT_REACH_OFFSET = 100       
    PI = 3.14159265358979323
    BOT_RADIUS = 150
    REACH_OFFSET = BOT_RADIUS + SHOOT_REACH_OFFSET
    SELF_GOAL_POINT = Vector2D(-2000, 0)
    
    def __init__(self, bot_id, role, team):

        rospy.init_node('roles', anonymous=False)
        self.bot_id = bot_id
        self.role = role
        self.team = team

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
        self.gyro_not = 0
        self.reached = False

        launch_files_path = rospy.get_param("/launch_files")
        config = None
        with open(launch_files_path+"/config/BOT_ID_TEAM_ROLE.json", "r") as f:
            config = json.loads(f.read())

        for key in config.keys():
            rospy.loginfo(key)
            if team == key:
                self.self_goal = Vector2D(int(config[key]["goal"]["x"]), int(config[key]["goal"]["y"]))
            else:
                self.opp_goal = Vector2D(int(config[key]["goal"]["x"]), int(config[key]["goal"]["y"]))

        topic = rospy.get_name() + '/grsim_data'
        self.msg_publish = rospy.Publisher(topic, gr_Robot_Command, queue_size=1)
        
        rospy.Subscriber("/vision", SSL_DetectionFrame, self.callbackForData, queue_size=1)
        rospy.Subscriber('/gyroDataBlue9', String, self.callbackForGyroData, queue_size=1)

    def defence(self):
        self.positionToGo()
        pass

    def positionToGo(self):        
        pass
    
    def offence(self):

        shootPoint = self.shootPoint()
        shoot_allowed = False
        while not shoot_allowed and not rospy.is_shutdown():
            shoot_allowed = self.sendCommand(shootPoint, Iid=self.bot_id)

        spinner = 1
        if shoot_allowed:
            spinner = 1
            new_pose = self.__goalToBallOffsetPos(Roles.SHOOT_REACH_OFFSET)
            shoot_allowed = self.sendCommand(new_pose, Iid=self.bot_id, spinner=spinner)

        if spinner:
            command = gr_Robot_Command()
            command.id = self.bot_id
            command.kickspeedx = 0
            command.spinner = 0
            self.msg_publish.publish(command)

    def sendCommand(self, shootPoint, Iid=0, kickspeedx=0, kickspeedz=0, spinner=0, wheelsspeed=0, orientation=0):
        
        UPPER_THRESHOLD = 1.0
        LOWER_THRESHOLD = 0.8
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

        if math.fabs(self.bot_orientation - shootPoint[2]) >= math.pi/12:
            velocity_angular = -1*0*(math.fabs(self.bot_orientation - shootPoint[2])/(self.bot_orientation - shootPoint[2]))
        else:
            velocity_angular = 0        
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

        distance = math.sqrt((self.bot_pos.x - shootPoint[0])**2 + (self.bot_pos.y - shootPoint[1])**2)
        if distance >= 100:
            command.velangular = 0
            self.msg_publish.publish(command)

        if distance <= 100 and math.fabs(self.bot_orientation-shootPoint[2]) > math.pi/12:
            command.velnormal = 0
            command.veltangent = 0
            self.msg_publish.publish(command)
            self.reached = True

        if distance <= 100 and math.fabs(self.bot_orientation-shootPoint[2]) <= math.pi/12:
            command.velangular = 0
            command.velnormal = 0
            command.veltangent = 0
            self.msg_publish.publish(command)
            self.reached = True
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
        bot_orientation = math.pi + angle

        return (reach_pos_x, reach_pos_y, bot_orientation)

    def callbackForData(self, data):
        
        ballData = data.balls[0]

        if self.team == "yellow":
            return                                                    
            defenderRobot = data.robots_yellow[self.bot_id]
            if defenderRobot.confidence < 0.01:
                return
            if int(defenderRobot.robot_id) != 1:
                return                           

        else:
            # return
            defenderRobot = data.robots_blue[self.bot_id]
            if defenderRobot.confidence < 0.01:
                return
            if int(defenderRobot.robot_id) != 4:
                return                                                  

        self.ball_pos.x  = ballData.x
        self.ball_pos.y = ballData.y 

        self.bot_pos.x = defenderRobot.x
        self.bot_pos.y = defenderRobot.y
        self.bot_orientation = defenderRobot.orientation

        self.field_update = True

    def callbackForGyroData(self, msg):

        data = (msg.data).split(',')        
        self.bot_orientation_gyro = int(data[0])*(math.pi/180)

    def pointAndOrientation(self):

        angle = self.self_goal.angle(self.ball_pos)
        slope, const = self.self_goal.slope_const(self.ball_pos)

        y = self.ball_pos.y
        x = self.self_goal.x - 0.2*self.self_goal.x
        
        rospy.loginfo("Value of x is {}, y is {}, and angle is {}".format(x, y, angle))

        return (x, y, angle)        

    def control(self):

        offence = True
        defence = True
        coordinate_list_s = [[-1500, 700], [-1500, -700]] if self.team == "blue" else [[-700, 700], [-700, -700]]
        coordinate_list_e = [[-1500, 200], [-1500, -200]] if self.team == "blue" else [[-700, 200], [-700, -200]]

        point = None

        while not rospy.is_shutdown():

            offence = True
            defence = True

            ballVec = Vector2D(self.ball_pos.x, self.ball_pos.y)
            botVec = Vector2D(self.bot_pos.x, self.bot_pos.y)
            angle_to_go = botVec.angle(ballVec)

            if self.field_update:
                
                x, y, angle = self.pointAndOrientation()
                point_1 = coordinate_list_s[int(self.bot_id%(len(coordinate_list_s)))]
                point_2 = coordinate_list_e[int(self.bot_id%(len(coordinate_list_e)))]

                if point == None:
                    point = point_1

                if self.reached:
                    if point == point_1:
                        point = point_2
                    else:
                        point = point_1

                    self.reached = False

                self.sendCommand((x,y,angle), Iid=self.bot_id)     
                           
                self.field_update = False



if __name__=="__main__":
    

    default_bot_id = rospy.get_param("~bot_id", "0")
    default_role = rospy.get_param("~role", "attacker")
    default_team = rospy.get_param("~team", "yellow")

    parser = ArgumentParser()
    parser.add_argument("--bot_id", dest="bot_id", default=default_bot_id,
                      help="Default bot id is '0'", metavar="BOT_ID")
    parser.add_argument("--role", dest="role", default=default_role,
                      help="Default bot role is 'attacker'", metavar="ROLE")
    parser.add_argument("--team", dest="team", default=default_team,
                      help="Default team is 'yellow'", metavar="TEAM")

    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])

    bot_id = int(args.bot_id)
    role = args.role
    team = args.team

    roles = Roles(bot_id, role, team)
    rospy.loginfo("Starting with role " + role + " team " + team + " bot id " + str(bot_id))
    roles.control()