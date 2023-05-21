#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from robot_messages.msg import game
from numpy import interp
import time
from robot_messages.msg import gr_Commands
from robot_messages.msg import gr_Robot_Command

'''
joy format
Reports the state of a joysticks axes and buttons.
Header header           # timestamp in the header is the time the data is received from the joystick
float32[] axes          # the axes measurements from a joystick
int32[] buttons         # the buttons measurements from a joystick
'''
BotID=0
stop_flag=0
Team="null"

def map_fn(x,in_min,in_max,out_min,out_max):
    return float((x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min)

def stop_robots():

    print("stop bot fn")
    
    for id in range(6,-1,-1):
        Bot_Command2.isteamyellow=True
        Bot_Command2.id=id
        Bot_Command2.kickspeedx=0
        Bot_Command2.spinner=0
        Bot_Command2.velangular=0
        Bot_Command2.veltangent=0
        Bot_Command2.velnormal=0
        Bot_Command2.wheelsspeed=0
        Bot_Command2.orientation=0
        Bot_Command2.timestamp=0

        command1 = gr_Commands()
        command1.timestamp=0
        command1.isteamyellow = True
        command1.robot_commands = Bot_Command2
        pubY.publish(command1)

        
        
        
        Bot_Command2.isteamyellow=False
        Bot_Command2.id=id
        Bot_Command2.kickspeedx=0
        Bot_Command2.spinner=0
        Bot_Command2.velangular=0
        Bot_Command2.veltangent=0
        Bot_Command2.velnormal=0
        Bot_Command2.wheelsspeed=0
        Bot_Command2.orientation=0
        Bot_Command2.timestamp=0
        

        command1 = gr_Commands()
        command1.timestamp=0
        command1.isteamyellow = False
        command1.robot_commands = Bot_Command2
        pubY.publish(command1)

            

def stop_robots_instant():
    print("stop bot instant")
    for count in range (0,2):
        for id in range(6,-1,-1):
            Bot_Command2.isteamyellow=True
            Bot_Command2.id=id
            Bot_Command2.kickspeedx=0
            Bot_Command2.spinner=0
            Bot_Command2.velangular=0
            Bot_Command2.veltangent=0
            Bot_Command2.velnormal=0
            Bot_Command2.wheelsspeed=0
            Bot_Command2.orientation=0
            Bot_Command2.timestamp=0

            command1 = gr_Commands()
            command1.timestamp=0
            command1.isteamyellow = True
            command1.robot_commands = Bot_Command2
            pubY.publish(command1)

            
            time.sleep(0.005)
            
            Bot_Command2.isteamyellow=False
            Bot_Command2.id=id
            Bot_Command2.kickspeedx=0
            Bot_Command2.spinner=0
            Bot_Command2.velangular=0
            Bot_Command2.veltangent=0
            Bot_Command2.velnormal=0
            Bot_Command2.wheelsspeed=0
            Bot_Command2.orientation=0
            Bot_Command2.timestamp=0
            

            command1 = gr_Commands()
            command1.timestamp=0
            command1.isteamyellow = False
            command1.robot_commands = Bot_Command2
            pubY.publish(command1)

            time.sleep(0.005)


def callback_joy(data):
    global BotID
    global stop_flag
    global Team

    if(data.buttons[8])==1: #back button in js for extra stop command
        
        stop_robots_instant()
            
    #state=data.buttons[9]# comment this 

    if data.buttons[9]==1:      #start button in  js
        
        stop_flag=not stop_flag #set flag for vision/js mode 
        stop_robots_instant()

    if bool(stop_flag) is True:
        # stop_robots()   #if pressed once - goto JS mode
                        #start switch auto manual
        print("Manual Mode")
        pubX.publish("Manual Mode")
        stop_robots()

        if (data.buttons[6]==1):#LT button in js
            #switch to selsct team use echo topic to verify the changes x and B button in JS
            BotTeamToggle1=data.buttons[0]
            if BotTeamToggle1 ==1 :
                Team="yellow"
                Bot_Command.isteamyellow=True
                #print("team is set to Yellow")
            BotTeamToggle2=data.buttons[2]
            if BotTeamToggle2==1:
                Team="blue"
                Bot_Command.isteamyellow=False
                #print("team is set to Blue")
                #updoen buttons for changing bot id 0-6
                #BotIdDown=data.buttons[1]
            if data.buttons[1]==1:

                if (BotID-1)>-1:
                    BotID-=1
                    Bot_Command.id=BotID
                    #print("Bot id set to :"+str(BotID))

            #BotIdUp=data.buttons[3]
            if data.buttons[3]==1:
                if(BotID+1)<7:
                    BotID+=1
                    #message.BotID=BotID
                    Bot_Command.id=BotID
            
            #kicker data two swiches are used for kicker /anyof the one
            #message.Kick=data.buttons[5]
            Bot_Command.kickspeedx=data.buttons[5]
            Bot_Command.kickspeedx=data.buttons[4]

            Bot_Command.kickspeedz=0

            #to set the control is it JS control the game or vision system 
            #Mode=data.buttons[6]

                #message.GameMode="Auto"

            # mode_fn()

            #dribbler command 
            #message.Dribbler=data.buttons[7]
            Bot_Command.spinner=data.buttons[7]

            #

                #message.Stop=stop_flag


            #message.Vz=data.axes[0]#angular
            #Bot_Command.velangular=data.axes[0]
            #Bot_Command.velangular=interp(data.axes[0],[0.0,1.0],[0.5,1.5])

            
            if data.axes[0] == -1.0 or 1.0:
                Bot_Command.velangular=2*data.axes[0]

        
            if -1.0 < data.axes[0] < -0.1:
                Bot_Command.velangular= map_fn(data.axes[0],-1.0,-0.1,1.2,1)
            elif -0.1< data.axes[0] < 0.1 :
                Bot_Command.velangular=0
            elif 0.1< data.axes[0] <1.0:
                Bot_Command.velangular= map_fn(data.axes[0],0.1,1.0,-1,-1.2)

            if -1.0 < data.axes[2] < -0.1:
                Bot_Command.veltangent= map_fn(data.axes[2],-1.0,-0.1,1,0.5)
            elif -0.1< data.axes[2] < 0.1 :
                Bot_Command.veltangent=0
            elif 0.1< data.axes[2] <1.0:
                Bot_Command.veltangent= map_fn(data.axes[2],0.1,1.0,-0.5,-1)    
            #message.Vy=data.axes[3]#normal
            if -1.0 < data.axes[3] < -0.1:
                Bot_Command.velnormal= map_fn(data.axes[3],-1.0,-0.1,-1,-0.5)
            elif -0.1< data.axes[3] < 0.1 :
                Bot_Command.velnormal=0
            elif 0.1< data.axes[3] <1.0:
                Bot_Command.velnormal= map_fn(data.axes[3],0.1,1.0,0.5,1)
            Bot_Command.wheelsspeed=0
            Bot_Command.orientation=0
            Bot_Command.timestamp=0
            #1->0.55 change 

            #callback_joy(data) #LT
            pubX.publish("commanding to: " +Team+str(Bot_Command.id))
            #pubX.publish(message)
            command1 = gr_Commands()
            command1.timestamp=0
            command1.isteamyellow = Bot_Command.isteamyellow
            command1.robot_commands = Bot_Command
            pubY.publish(command1)
            
        
        else:
            stop_robots()

    else:
        #rospy.Subscriber("/gotopoint_output",gr_Commands,goto_point_callback)
        pubX.publish("Auto Mode")
    
        print("Auto Mode")

        #vision_flag=True #not using
        #sub in if to avoid running call back al the time (when new data comes)
        #rospy.Subscriber("gotopoint_output",gr_Robot_Command,goto_point_callback)
    #pubY.publish(Bot_Command_vision)
        
        
        

   
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)


def auto_callback(gpdata):
    #print(data)
    #global BotID
    global stop_flag
    if stop_flag is False:
        pubY.publish(gpdata)
        pubX.publish("goto point")
    
    #print("ff")



    
def listener_joy():
    pass

if __name__ == '__main__':
    rospy.init_node('game_controller_node', anonymous=False)

    pubX = rospy.Publisher('/game_control_topic',String, queue_size=4) #not using 
    pubY = rospy.Publisher('/grsim_data',gr_Commands, queue_size=2)
    #rospy.Subscriber("/gotopoint_output",gr_Robot_Command,goto_point_callback)
    
    Bot_Command=gr_Robot_Command()
    Bot_Command2=gr_Robot_Command() #for avoiding over ride -- to keep the last value  frm js
    Bot_Command_vision=gr_Robot_Command()
    #for controlling bot from joystick
    
    rospy.Subscriber("joy",Joy, callback_joy,queue_size=20)
    rospy.Subscriber("/gotopoint_output",gr_Commands, auto_callback,queue_size=100)
    #/gotopoint_output
    #spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
