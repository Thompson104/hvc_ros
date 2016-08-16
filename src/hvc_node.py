#!/usr/bin/env python

# Node talks with HVC module, allows configuration, returns data

import serial
import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from geometry_msgs.msg import Pose
import tf
from math import radians
from hvc_ros.msg import Face

#Global Shot Command:
command = [0xfe, 0x04, 0x03, 0x00, 0x0E, 0x00, 0x00]
thresh = 50

def ddyn_rec_callback(config, level):
    rospy.loginfo("Received reconf call: " + str(config))
    global command
    command[4] = config.conf_byte_1
    command[5] = config.conf_byte_2
    command[6] = config.conf_byte_3
    global thresh
    thresh = config.confidence_thresh
    return config

def a2s(arr):
    return ''.join(chr(b) for b in arr)

class Hand:
    name=""
    x = 0
    y = 0
    size = 0
    det_conf = 0

class Face:
    name =""
    x = 0
    y = 0
    size = 0
    det_conf = 0
    yaw = 0
    pitch = 0
    roll = 0
    dir_conf = 0

def hvc_comms():
    pub = rospy.Publisher('face', Pose, queue_size=1)
    rospy.init_node('hvc_node', anonymous=True)

    # Connect to device
    Device = serial.Serial(rospy.get_param("/hvc_node/port"), rospy.get_param("/hvc_node/baud"))

    ddynrec = DDynamicReconfigure("ddyn_rec")
    ddynrec.add_variable("conf_byte_1", "integer variable", 14, 1, 255)
    ddynrec.add_variable("conf_byte_2", "integer variable", 2, 0, 3)
    ddynrec.add_variable("conf_byte_3", "integer variable", 0, 0, 2)
    ddynrec.add_variable("confidence_thresh", "integer variable", 50, 0, 300)

    ddynrec.start(ddyn_rec_callback)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        Device.write(a2s(command))
        result = map(ord, Device.read(size=6))
        for x in range(0, result[2]):
            result.append(int(Device.read(size=1).encode('hex'),16))
        rospy.loginfo("HVC Reply: " + str(result))
        
        # Returned values
        num_bodies = int(result[6])
        num_hands = int(result[7])
        num_faces = int(result[8])
        
        offset = 10
        if num_bodies >= 1:
            body_array = result[offset:num_bodies*8+offset]
            print body_data
            offset = num_bodies*8+offset
 
        if num_hands >= 1:
            hand_array = result[offset:num_hands*8+offset]
            print hand_array
            offset = num_hands*8+offset
            hand_data = [Hand() for x in range(num_hands)]
            for p in range(num_hands):
                hand_data[p].name = "Hand_" + str(p)
                hand_data[p].x = hand_array[p*8 + 0] + hand_array[p*8 + 1] << 1
                hand_data[p].y = hand_array[p*8 + 2] + hand_array[p*8 + 3] << 1
                hand_data[p].size = hand_array[p*8 + 4] + hand_array[p*8 + 5] << 1
                hand_data[p].dir_conf = hand_array[p*8 + 6] + hand_array[p*8 + 7] << 1

        if num_faces >= 1: ## for now only supporting face detection and direction... 16 BYTES!!
            f_o = 16
            face_array = result[offset:num_faces*f_o+offset]
            print face_array
            offset = num_faces*f_o+offset
            face_data = [Face() for x in range(num_faces)]
            for p in range(num_faces):
                face_data[p].name = "Face_" + str(p)
                face_data[p].det_conf = face_array[p*f_o + 6] + face_array[p*f_o + 7] << 1
            #if face_data[p].det_conf >= thresh:
                face_data[p].x = face_array[p*f_o + 0] + face_array[p*f_o + 1] << 1
                face_data[p].y = face_array[p*f_o + 2] + face_array[p*f_o + 3] << 1
                face_data[p].size = face_array[p*f_o + 4] + face_array[p*f_o + 5] << 1
                face_data[p].dir_conf = face_array[p*f_o + 14] + face_array[p*f_o + 15] << 1
            #if face_data[p].dir_conf >= thresh:
                face_data[p].yaw = face_array[p*f_o + 8] + face_array[p*f_o + 9] << 1
                face_data[p].pitch = face_array[p*f_o + 10] + face_array[p*f_o + 11] << 1
                face_data[p].roll = face_array[p*f_o + 12] + face_array[p*f_o + 13] << 1

            # THIS needs a lot more effort, TF generation from head size estimation needs segmentation calibration, else I need medication

            #br.sendTransform((face_data[p].x/900.0, face_data[p].y/900.0, face_data[p].size/900.0),
            #         tf.transformations.quaternion_from_euler(radians(face_data[p].roll), radians(face_data[p].pitch), radians(face_data[p].yaw)),
            #         rospy.Time.now(),
            #        face_data[p].name,
            #         "hvc_cam")


if __name__ == '__main__':
    try:
        hvc_comms()
    except rospy.ROSInterruptException:
        pass
