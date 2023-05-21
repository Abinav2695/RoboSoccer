#!/usr/bin/env python3

import socketio
import asyncio
import os
import json
import time


import rospkg
import rospy
import sys

from robot_messages.msg import BeliefState,gr_Robot_Command, gr_Commands,point_2d,my_pos,obstacle,SSL_DetectionFrame
from std_msgs.msg import String

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# list all packages, equivalent to rospack list
rospack.list() 

# get the file path for rospy_tutorials
filePath = rospack.get_path('game_controller')
filePath = filePath + '/src/socketio_client/credentials.json'

# print(filePath)

# asyncio
sio = socketio.AsyncClient()
creds = {}
creds['auth_type'] = 'login'
running = False


with open(filePath) as json_data_file:
    data=json.load(json_data_file)
    creds['id'] = data['device_id']
    creds['password']=data['password']

@sio.event
async def connect():
    await sio.emit('authenticate',creds)
    print("I'm connected!")


@sio.on(creds['id'])
async def onCommand(server_data):
    print("\n===================Command received ===============")
    server_data=dict(server_data)
    print("data read----",server_data)

    event=server_data["event"]
    event_data = server_data["event_data"]

    taskID=server_data["task_id"]
    taskCommand = event_data['command']
    
    if taskCommand=="start":
        
        server_pub.publish("start")
        print('Start command recieved')
    
    elif taskCommand=="stop":

        server_pub.publish("stop")
        print('Stop command recieved')





@sio.event
async def connect_error(err):
    print("The connection failed!")
    print("===========> Error: ",err)
    await asyncio.sleep(2)
    await connection_establish()

@sio.event
async def disconnect():
    print("I'm disconnected!")
    await asyncio.sleep(2)
    await connection_establish()

async def connection_establish():
    await sio.connect('http://10.10.1.17:3000')
    await sio.wait()

async def task_execute(task):
    # ===================================== Add condition to execute according to command send via server =============================
    
    # time.sleep(10)
    print("<<<======== Task Executed ========>>>")
    await sio.emit('device-events',task)

def referee_callback(calldata):
    pass


if __name__ == "__main__":
    
    rospy.init_node('socket_node', anonymous=False)
    rospy.Subscriber('/referee_topic',SSL_DetectionFrame,referee_callback)
    
    server_pub = rospy.Publisher('/server_events', String, queue_size=5)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(connection_establish())

    rospy.spin()

    
    

    