#!/usr/bin/env python
"""
Configures the device list of a given Pozyx device (local and remote) for positioning.

This is intended to be highly customisable, but will also be made with parameters and a launch
file in the future. Again, help/suggestions/feedback on these things are highly appreciated.

Run this after running the UWB configuration.
Automatic calibration at this point in time is highly discouraged.
"""

import pypozyx
import rospy
import argparse

from pypozyx import (POZYX_POS_ALG_UWB_ONLY, POZYX_3D, Coordinates, POZYX_SUCCESS, PozyxConstants, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, DeviceList, PozyxRegisters)

# Set serial port or leave it empty to make auto connect
serial_port = ''

# adding None will cause the local device to be configured for the anchors as well.
tag_ids = [None]

anchors = [DeviceCoordinates(0x6a11, 1, Coordinates(-155, 12210, 1500)),
            DeviceCoordinates(0x6a19, 1, Coordinates(5170, 11572, 2900)),
            DeviceCoordinates(0x6a6b, 1, Coordinates(0, 0, 2900)),
            DeviceCoordinates(0x676d, 1, Coordinates(4330, 0, 2900))]


def set_anchor_configuration(port):
    rospy.init_node('uwb_configurator')
    rospy.loginfo("Configuring device list.")

    settings_registers = [0x16, 0x17]  # POS ALG and NUM ANCHORS
    try:
        pozyx = pypozyx.PozyxSerial(port)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    for tag in tag_ids:
        for anchor in anchors:
            pozyx.addDevice(anchor, tag)
        if len(anchors) > 4:
            pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO,
                                        len(anchors), remote_id=tag)
            pozyx.saveRegisters(settings_registers, remote_id=tag)
        pozyx.saveNetwork(remote_id=tag)
        if tag is None:
            rospy.loginfo("Local device configured")
        else:
            rospy.loginfo("Device with ID 0x%0.4x configured." % tag)
    rospy.loginfo("Configuration completed! Shutting down node now...")


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--port", type=str,
                        help="sets the uart port of pozyx, use first pozyx device if empty")
    args = parser.parse_args()

    if args.port:
        serial_port = args.port

    # shortcut to not have to find out the port yourself
    if serial_port == '':
        serial_port = get_first_pozyx_serial_port()
        print("Pozyx device on {} port".format(serial_port))
        if serial_port is None:
            print("No Pozyx connected. Check your USB cable or your driver!")
            quit()

    set_anchor_configuration(serial_port)
