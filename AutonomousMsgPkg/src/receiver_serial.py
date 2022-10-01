#!/usr/bin/env python3

import rospy
import time
import sys
from custom_pkg.msg import CarState

global data
#global msg1
data=[]

def callback(msg):
    global data
    #global msg1
    print(type(msg))
    msg1=msg
    
    cnt=0
    
    data.append(msg.f_wheel_velocity_FL_ms)
    data.append(msg.f_wheel_velocity_FR_ms)
    data.append(msg.f_wheel_velocity_RL_ms)
    data.append(msg.f_wheel_velocity_RR_ms)
    data.append(msg.f_car_velocity_ms)
    data.append(msg.f_motor_torque_FL_Nm)
    data.append(msg.f_motor_torque_FR_Nm)
    data.append(msg.f_motor_torque_RL_Nm)
    data.append(msg.f_motor_torque_RR_Nm)
    data.append(msg.battery_voltage)
    data.append(msg.battery_current)

    print('===============================')
    for elem in data:
        cnt+=1
        print(str(cnt)+' : '+str(elem))
    print('===============================')
    
    data.clear()
    
def main():
    #global msg1
    rospy.init_node('receiver_serial',anonymous=False)
    rate=rospy.Rate(1)
    sub=rospy.Subscriber('chatter',CarState,callback)
    #pub=rospy.Publisher('mbed_topic',CarState)
    #pub.publish(msg1)
    rospy.spin()


if __name__ == "__main__" :
    main()