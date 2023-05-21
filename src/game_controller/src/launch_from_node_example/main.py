#! /usr/bin/env python3

import rospy
import roslaunch
import time

def start_task():
    rospy.loginfo("startin..")

    package = "game_controller"
    exec1 = "node1.py" 
    node = roslaunch.core.Node(package,exec1,args='1',output='screen')

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    script = launch.launch(node)
    print(script.is_alive())
    time.sleep(20)

    script.stop()
    print(script.is_alive())


def main():

    rospy.init_node('main_node')
    start_task()
    rospy.spin()

if __name__=="__main__":

    try:
        main()
    except rospy.ROSInterruptException:
        print('exception error')

    print('hello')