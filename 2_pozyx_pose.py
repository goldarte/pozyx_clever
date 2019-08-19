#!/usr/bin/env python
"""
ROS node that publishes the pose (position + quaternion) of the Pozyx

Quaternion of Pozyx is not recommended to use with drones because of floating yaw of the Pozyx IMU
"""

from pypozyx import (PozyxConstants, Coordinates, POZYX_SUCCESS, PozyxRegisters,
                     DeviceCoordinates, PozyxSerial)
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Range
from math import atan2
import argparse


remote_id = None

# Enable or disable logging
enable_logging = False
# Global variable to collect data from lazer
distance = 0.0
# Necessary data for calibration
anchors = [DeviceCoordinates(0x6a11, 1, Coordinates(-108, 12145, 2900)),
            DeviceCoordinates(0x6a19, 1, Coordinates(5113, 11617, 2900)),
            DeviceCoordinates(0x6a6b, 1, Coordinates(0, 0, 2900)),
            DeviceCoordinates(0x676d, 1, Coordinates(4339, 0, 2800)),
            DeviceCoordinates(0x6a40, 1, Coordinates(672, 5127, 100))]

# Positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
# Positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
dimension = PozyxConstants.DIMENSION_3D

def callback(data):
    global distance
    distance = data.range

def pozyx_pose_pub(port1, port2):
    global distance
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=40)
    try:
        pozyx1 = PozyxSerial(port1)
    except:
        rospy.loginfo("Pozyx 1 not connected")
        return
    try:
        pozyx2 = PozyxSerial(port2)
    except:
        rospy.loginfo("Pozyx 2 not connected")
        return
    while not rospy.is_shutdown():
        pos1 = Coordinates()
        pos2 = Coordinates()
        pose = PoseStamped()
        pose.header.frame_id = "map"

        status1 = pozyx1.doPositioning(pos1, dimension=dimension, algorithm=algorithm)
        if status1 == POZYX_SUCCESS:
            status2 = pozyx2.doPositioning(pos2, dimension=dimension, algorithm=algorithm)
            if status2 == POZYX_SUCCESS:
                pose.pose.position = Point((pos1.x+pos2.x)/1000., (pos1.y+pos2.y)/1000., distance)
                pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, atan2(pos2.y-pos1.y, pos2.x-pos1.x)))
                #print pose.pose.position
                pose.header.stamp = rospy.Time.now()
                pub.publish(pose)
                if enable_logging:
                    rospy.loginfo("POS: %s, QUAT: %s" % (str(pos1), str(pos2)))

def set_anchor_configuration(port1, port2):
    rospy.init_node('uwb_configurator')
    rospy.loginfo("Configuring device list.")

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(port)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    for anchor in anchors:
        pozyx.addDevice(anchor, None)
    if len(anchors) > 4:
        pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO,
                                    len(anchors))
        pozyx.saveRegisters(settings_registers)
    pozyx.saveNetwork(remote_id=None)
    rospy.loginfo("Local device configured")
    rospy.loginfo("Configuration completed! Shutting down node now...")

if __name__ == '__main__':

    rospy.init_node('pozyx_pose_node')

    rospy.Subscriber("/mavros/distance_sensor/rangefinder_sub", Range, callback)
    
    parser = argparse.ArgumentParser(description='Send Pose_Stamped message, counted from 2 pozyx tags')
    parser.add_argument("-p1", "--port1", type=str, default='/dev/ttyACM1',
                        help="sets the uart port of the first pozyx")
    parser.add_argument("-p2", "--port2", type=str, default='/dev/ttyACM2',
                        help="sets the uart port of the second pozyx")    
    args = parser.parse_args()
    
    try:
        pozyx_pose_pub(args.port1, args.port2)
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
