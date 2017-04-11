#!/usr/bin/env python

# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
vehicle_state.py:

Demonstrates how to get and set vehicle state and parameter information,
and how to observe vehicle attribute (state) changes.

Full documentation is provided at http://python.dronekit.io/examples/vehicle_state.html
"""
from dronekit import connect, VehicleMode
import time
#Set up option parsing to get connection string
import argparse


import rospy, tf
from geometry_msgs.msg import Vector3
from math import *
import numpy as np

# global variables
pixel_x = 0
pixel_y = 0

# horizontal and vertical field of view angle
hfv = 1.0618436422826059
vfv = 0.8648155511410757

def pixel_callback(data):
    global pixel_x
    global pixel_y
    # need to subtract the half size of image width and height so that the center of image plane is (0,0)
    pixel_x = data.tracks[0].meanx-x_half
    pixel_y = data.tracks[0].meany-y_half
    # print "pixel", pixel_x, pixel_y
    # When RRANSAC doesn't have any tracks pixel location, don't move rotor
    if len(data.tracks)==0:
        pixel_x = 0
        pixel_y = 0

def calculate_command(vehicle):
    global pixel_x
    global pixel_y
    global focal_length

    altitude = vehicle.location.global_reletive_frame
    phi = vehicle.attitude.roll
    theta = vehicle.attitude.pitch

    # angle between target and -z-axis of the rotor in roll axis
    x_angle = phi-pixel_x*hov/640.0
    py = altitude*tan(x_angle)
    y_angle = theta-pixel_y*vev/320.0
    px = -altitude*tan(y_angle)

    # print "px, py: ", px, py
    scale_factor = 0.2
    # py is basically vx. py is vy with scale factor
    vx = scale_factor*px
    vy = scale_factor*py
    print "vx, vy: ", vx, vy
    return (vx, vy)

def listener():
    rospy.init_node('relative_tracking', anonymous=True)
    rospy.Subscriber("pixel_location", vec3, pixel_callback)

    r = rospy.Rate(30) # 20hz

    #Set up option parsing to get connection string
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect',
                        help="vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None

    #Start SITL if no connection string specified
    if not connection_string:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    # Connect to the Vehicle.
    #   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
    print "\nConnecting to vehicle on: %s" % connection_string
    vehicle = connect(connection_string, wait_ready=True)
    vehicle.wait_ready('autopilot_version')

    while not rospy.is_shutdown():
        (vx, vy) = calculate_command(vehicle)
        point1 = LocationGlobalRelative
        r.sleep()

def saturate(v,limit):
    if v>limit:
        v = limit
    elif v<-limit:
        v = -limit
    return v

if __name__ == '__main__':
    listener()
