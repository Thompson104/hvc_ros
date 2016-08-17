#!/usr/bin/env python

# Node Opens and closes gripper in response to hand, moves arm

import rospy
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from time import sleep
from hvc_ros.msg import Hands
from hvc_ros.msg import Faces
from sensor_msgs.msg import Joy
from std_msgs.msg import String

#Global hand state
state = 0

#Global safety state
safe = 0

#Global joy state
joy = Joy()

#Global target
target = [0,0]

#Global home
home = [-3.84, -2.1, 1.22, 0.61, 1.57, 0]


def h_call(data):
    global state
    if state == 0: state = 255
    else: state = 0
    print data

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def f_call(data):
    global target
    if data.det_conf >= 100:
        target[0] = 100 - data.x
        target[1] = 420 - data.y
        target[0] = clamp(target[0],-20,20)
        target[1] = clamp(target[1],-20,20)
        print target
    #print data

def joy_call(data):
    global safe
    if data.buttons[13] == 1: safe= 1 #Autonomous mode
    elif data.buttons[15] == 1: safe=2 #Manual mode
    else: safe = 0 # E-STOP
    global joy
    joy = data

def demo():
    rospy.init_node('Case2016_Demo')
    pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)
    pub_arm = rospy.Publisher('/ur_driver/URScript', String)
    rospy.Subscriber("/hands", Hands, h_call)    
    rospy.Subscriber("/faces", Faces, f_call)    
    rospy.Subscriber("/bluetooth_teleop/joy", Joy, joy_call)    

    command = outputMsg.CModel_robot_output();
    pub.publish(command) #Reset
    command.rACT = 1
    command.rGTO = 1
    command.rSP = 255
    command.rFR = 75 
    pub.publish(command) #Activate 

    # Arm Control
    val = String()
    arm_array = [-3.84, -2.1, 1.22, 0.61, 1.57, 0]

    while not rospy.is_shutdown():
        if safe == 1: #Autonomous mode
            print "Auto"
            command.rPR = state
            pub.publish(command)
            global target
            arm_array[0] = arm_array[0] + target[0]*0.001
            arm_array[2] = arm_array[2] + target[1]*0.001
            target = [0,0]
            val.data = "movej(" + str(arm_array) +", a=0.1, v=0.1)"
            pub_arm.publish(val)
            print val.data


        if safe == 2: #Manual mode
            print "Manual"
            scale = 0.1
            if joy.buttons[12] == 1: command.rPR = 0
            if joy.buttons[14] == 1: command.rPR = 255
            if joy.buttons[11] == 1: arm_array = home[:]
            pub.publish(command)
            arm_array[0] = arm_array[0] + joy.axes[0]*scale #base angle value of arm, left analog stick
            arm_array[1] = arm_array[1] + joy.axes[1]*scale #shoulder value of arm, left analog stick
            arm_array[2] = arm_array[2] + joy.axes[2]*scale #elbow value of arm, right analog stick
            arm_array[3] = arm_array[3] + joy.axes[3]*scale #wrist1 value of arm, right analog stick
            #arm_array[4] = arm_array[4] + joy.axes[3]*scale #wrist2 value of arm, right analog stick
            #arm_array[5] = arm_array[5] + joy.axes[3]*scale #wrist3 value of arm, right analog stick
            val.data = "movej(" + str(arm_array) +", a=0.5, v=1.0)"
            pub_arm.publish(val)
            print val.data

        
        rospy.sleep(0.8)

if __name__ == '__main__':
    demo()
