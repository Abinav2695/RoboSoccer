#!/usr/bin/env python3



import rospy
from path_planner.srv import pid,pidResponse


Kp = 2
Kd = 0.04
Ki = 0

UPPER_THRESHOLD = 1 # m/s
LOWER_THRESHOLD = 0.65 # m/s

def pid_output(req):
    
    error = req.error
    prevError = req.prevError
    dt = req.dt
    # Calculate proportional control output.
    P_out = error*Kp

    # Calculate derivative control output.
    # error values and dt for time difference.
    if prevError != None:
        D_out = (error-prevError)/dt *Kd
    else:
        D_out = 0
        # Set this to error.
    # Calculate final output.
    output = P_out + D_out

    print("PID_OUTPUT:=  ",output)

    if(output>UPPER_THRESHOLD):
        output=UPPER_THRESHOLD
    elif (output<LOWER_THRESHOLD):
        output=LOWER_THRESHOLD
    print("PID_OUTPUT:=  ",output)
    return pidResponse(output)

def pid_server():
    rospy.init_node('pid_node')
    s = rospy.Service('pid_service', pid, pid_output)
    print("Ready to calculate PID ouput.")
    rospy.spin()

if __name__ == "__main__":
    pid_server()