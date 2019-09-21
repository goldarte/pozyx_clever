#!/usr/bin/env python
"""
ROS node that publishes PoseStamped message with position of the Pozyx

Quaternion of Pozyx is not recommended to use with drones because of floating yaw of the Pozyx IMU
"""

from pypozyx import (PozyxConstants, PozyxRegisters, DeviceCoordinates,
                    PozyxSerial, Coordinates, POZYX_SUCCESS, POZYX_ANCHOR_SEL_AUTO,
                    get_first_pozyx_serial_port)
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from sensor_msgs.msg import Range
import argparse

# Pozyx object and remote_id
remote_id = None
pozyx = None
# Set serial port or leave it empty to make auto connect
serial_port = ''
# Enable or disable logging
enable_logging = False
# Global variable to collect data from lazer
distance = 0.0
# Anchors data for calibration

''' COEX Soft Office 
anchors = [DeviceCoordinates(0x6a11, 1, Coordinates(-108, 12145, 2900)),
            DeviceCoordinates(0x6a19, 1, Coordinates(5113, 11617, 2900)),
            DeviceCoordinates(0x6a6b, 1, Coordinates(0, 0, 2900)),
            DeviceCoordinates(0x676d, 1, Coordinates(4339, 0, 2800)),
            DeviceCoordinates(0x6a40, 1, Coordinates(672, 5127, 100))]
'''

anchors = [DeviceCoordinates(0x6a11, 1, Coordinates(68, 5475, 150)),
            DeviceCoordinates(0x6a19, 1, Coordinates(5280, 5486, 150)),
            DeviceCoordinates(0x6a6b, 1, Coordinates(-1, 0, 150)),
            DeviceCoordinates(0x676d, 1, Coordinates(5551, 1, 2800))]

# Positioning algorithm. Variants: POSITIONING_ALGORITHM_UWB_ONLY, POSITIONING_ALGORITHM_TRACKING
algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
# Positioning dimension. Variants: DIMENSION_2D, DIMENSION_2_5D, DIMENSION_3D
dimension = PozyxConstants.DIMENSION_3D
# Positioning filter. Variants: FILTER_TYPE_NONE, FILTER_TYPE_MOVING_AVERAGE, FILTER_TYPE_MOVING_MEDIAN and FILTER_TYPE_FIR 
filter_type = PozyxConstants.FILTER_TYPE_MOVING_MEDIAN
# Filter strength. Integer from 0 to 15.
filter_strength = 5

def distance_callback(data):
    global distance
    distance = data.range

def pozyx_pose_pub(pozyx):
    global distance
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=40)
    pozyx.setPositionAlgorithm(algorithm=algorithm, dimension=dimension)
    pozyx.setPositionFilter(filter_type=filter_type, filter_strength=filter_strength)
    pos = Coordinates()
    pose = PoseStamped()
    pose.pose.orientation = Quaternion(0, 0, 0, 1)
    pose.header.frame_id = "map"

    while not rospy.is_shutdown():
        status = pozyx.doPositioning(pos, dimension=dimension, algorithm=algorithm, remote_id=remote_id)
        if status == POZYX_SUCCESS:
            pose.pose.position = Point(pos.x/1000., pos.y/1000., distance)
            pose.header.stamp = rospy.Time.now()
            pub.publish(pose)
            if enable_logging:
                rospy.loginfo("POS: %s" % str(pos))

def set_anchor_configuration(pozyx):
    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    for anchor in anchors:
        pozyx.addDevice(anchor, remote_id)
    if len(anchors) > 4:
        pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(anchors))
        pozyx.saveRegisters(settings_registers)
    pozyx.saveNetwork(remote_id)

if __name__ == '__main__':

    rospy.init_node('pozyx_pose_node')

    rospy.Subscriber("/mavros/distance_sensor/rangefinder_sub", Range, distance_callback)

    # Add parser
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=str,
                        help="sets the uart port of pozyx, use first pozyx device if empty")
    args = parser.parse_args()

    # Set up serial port
    if args.port:
        serial_port = args.port

    if serial_port == '':
        serial_port = get_first_pozyx_serial_port()
        print(serial_port)
        if serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()

    # Connect to pozyx and get pozyx object
    try:
        pozyx = PozyxSerial(serial_port)
    except:
        rospy.loginfo("Pozyx not connected")
        quit()

    if pozyx is not None:

        set_anchor_configuration(pozyx)
    
        try:
            pozyx_pose_pub(pozyx)
        except rospy.ROSInterruptException:
            quit()
