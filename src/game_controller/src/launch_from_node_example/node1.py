#! /usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String

def main():

    rospy.init_node('task_1')
    rate=rospy.Rate(2)

    pub = rospy.Publisher('chatter',String,queue_size=10)
    while not rospy.is_shutdown():

        pub.publish('Teri maa ka')
        rate.sleep()

if __name__=="__main__":
    
    arguments = rospy.myargv(sys.argv)
    #arguments = sys.argv
   
    try:
        main()
    except rospy.ROSInterruptException:
        print('nahi chala bhai')