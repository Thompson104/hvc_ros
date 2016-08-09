#!/usr/bin/env python

# Node talks with HVC module, allows configuration, returns data

import serial
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import Pose

#Global Shot Command:
command = [0xfe, 0x04, 0x03, 0x00, 0x04, 0x00, 0x00]

def ddyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global command
    command[4] = config.conf_byte_1
    command[5] = config.conf_byte_2
    command[6] = config.conf_byte_3
    return config

def a2s(arr):
    return ''.join(chr(b) for b in arr)

def hvc_comms():
    pub = rospy.Publisher('face', Pose, queue_size=1)
    rospy.init_node('hvc_node', anonymous=True)

    # Connect to device
    Device = serial.Serial(rospy.get_param("/hvc_node/port"), 921600)

    ddynrec = DDynamicReconfigure("ddyn_rec")
    ddynrec.add_variable("conf_byte_1", "integer variable", 4, 1, 255)
    ddynrec.add_variable("conf_byte_2", "integer variable", 0, 0, 2)
    ddynrec.add_variable("conf_byte_3", "integer variable", 0, 0, 2)

    ddynrec.start(ddyn_rec_callback)

    while not rospy.is_shutdown():
        Device.write(a2s(command))
        result = map(ord, Device.read(size=6))
        for x in range(0, result[2]):
            result.append(Device.read(size=1).encode('hex'))
        rospy.loginfo("HVC Reply: " + str(result))

if __name__ == '__main__':
    try:
        hvc_comms()
    except rospy.ROSInterruptException:
        pass
