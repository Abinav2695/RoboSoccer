#!/usr/bin/env python3

import sys
import rospy
from service_hub.srv import pid,pidResponse


def calculate_pid_output(error,prevError,dt):
    rospy.wait_for_service('pid_service')
    try:
        pid_output = rospy.ServiceProxy('pid_service', pid)
        resp1 = pid_output(error,prevError,dt)
        return resp1.pid_output
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        error = float(sys.argv[1])
        prevError = float(sys.argv[2])
        dt =  float(sys.argv[3])
        print(calculate_pid_output(error,prevError,dt))
    else:
        print(usage())
        sys.exit(1)
    