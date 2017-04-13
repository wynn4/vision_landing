#!/usr/bin/env python

from dronekit import connect, VehicleMode, LocationGlobalRelative, mavutil
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
pixel_z = 0

# horizontal and vertical field of view angle
hfv = 1.0618436422826059
vfv = 0.8648155511410757

def pixel_callback(data):
    global pixel_x
    global pixel_y
    global pixel_z
    # need to subtract the half size of image width and height so that the center of image plane is (0,0)
    pixel_x = data.x
    pixel_y = data.y
    pixel_z = data.z


def calculate_command(vehicle):
    global pixel_x
    global pixel_y
    global pixel_z
    global hfv
    global vfv

    altitude = vehicle.location.global_relative_frame.alt
    phi = vehicle.attitude.roll
    theta = vehicle.attitude.pitch
    descend = 0

    if pixel_z==0:
        print "nothing detected"
        return (0,0,descend)
    else:
        if pixel_x>-50 and pixel_x<50 and pixel_y>-50 and pixel_y<50:
            descend = 1
            print "descending"
            return (0,0,descend)
        else:
            descend = 0
            # angle between target and -z-axis of the rotor in roll axis
            x_angle = phi-pixel_x*hfv/640.0
            py = -altitude*tan(x_angle)
            y_angle = theta-pixel_y*vfv/320.0
            px = altitude*tan(y_angle)
            # print "px, py: ", px, py
            scale_factor = 1.0
            # py is basically vx. py is vy with scale factor
            vx = scale_factor*px
            vy = scale_factor*py
            print "vx, vy: ", vx, vy
            return (vx, vy, descend)

def listener():
    rospy.init_node('relative_tracking', anonymous=True)
    rospy.Subscriber("pixel_coordinates", Vector3, pixel_callback)

    r = rospy.Rate(30) # 30 hz

    connection_string = '127.0.0.1:14550'   #simulation
    #connection_string = '127.0.0.1:14551'   #MAVProxy / Pixhawk


    print "\nConnecting to vehicle on: %s" % connection_string
    vehicle = connect(connection_string, wait_ready=True)

    arm_and_takeoff(10, vehicle)

    #set target airspeed to 1 m/s
    vehicle.airspeed = 1

    while not rospy.is_shutdown():
        (vx, vy, descend) = calculate_command(vehicle)
        if descend==1:
            goto_position_target_local_ned(vx, vy, 1, vehicle)
        else:
            goto_position_target_local_ned(vx, vy, 0, vehicle)

        r.sleep()

def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)


    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print "Reached target altitude"
            break
        time.sleep(1)

#function that generates and sends mavlink message for copter to move to a BODYFIXED relatie NED position
def goto_position_target_local_ned(north, east, down, vehicle):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # possible flags: MAV_FRAME_BODY_OFFSET_NED, MAV_FRAME_LOCAL_NED, MAV_FRAME_LOCAL_OFFSET_NED
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def saturate(v,limit):
    if v>limit:
        v = limit
    elif v<-limit:
        v = -limit
    return v

if __name__ == '__main__':
    listener()
