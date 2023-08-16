#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64, Int16, String, Bool
from sensor_msgs.msg import NavSatFix, TimeReference
from geometry_msgs.msg import Point, Twist
import traceback
import os
import sys
import requests
import time
import bisect
# import numpy as np
# import pandas as pd
# import shapely
# from shapely import LineString,Point
# from shapely.ops import nearest_points
# from shapely import Polygon
import json

velocity = 0.0
in_i24 = False
libpanda_controls_allowed = False
vsl_set_speed = 0
car_setpoint = 0

velocity_topic = "vel"
# gantry_topic = "/vsl/latest_gantry"

#controls allowed
libpanda_controls_allowed_topic = "/car/libpanda/controls_allowed"
#setpoint readers
car_setpoint_topic = "v_ref"#"/pcm_cruise_2" #"/car/setpoint" #currently located at msg_467
#vsl set speed
vsl_set_speed_topic = "/vsl/set_speed"
#is i24
in_i24_topic = "/vsl/in_i24"
vsl_good_topic = "/vsl/vsl_good"

# gantry = None
def velocity_callback(data):
    global velocity
    velocity = data.data

def in_i24_callback(data):
    global in_i24
    in_i24 = data.data

def vsl_set_speed_callback(data):
    global vsl_set_speed
    vsl_set_speed = data.data

def vsl_good_callback(data):
    global vsl_good
    vsl_good = data.data

# def libpanda_controls_allowed_callback(data):
#     global libpanda_controls_allowed
#     libpanda_controls_allowed = data.data

def car_setpoint_callback(data):
    global car_setpoint
    # car_setpoint = data.y this was for /msg_467 Point input
    car_setpoint = data.linear.x


class vslmux:
    def __init__(self):
        rospy.init_node('vslmux', anonymous=True)

        rospy.Subscriber(in_i24_topic,Bool,in_i24_callback)
        # rospy.Subscriber(libpanda_controls_allowed_topic,Bool,libpanda_controls_allowed_callback)
        rospy.Subscriber(vsl_set_speed_topic,Int16,vsl_set_speed_callback)
        rospy.Subscriber(car_setpoint_topic,Point,car_setpoint_callback)
        rospy.Subscriber(velocity_topic,Float64,velocity_callback)
        rospy.Subscriber(vsl_good_topic,Bool,vsl_good_callback)

        #map this to the desired speed topic
        self.mux_set_speed_pub = rospy.Publisher('/mux/set_speed', Float64, queue_size=10)
        self.rate = rospy.Rate(1)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                global libpanda_controls_allowed
                global in_i24
                global car_setpoint
                global vsl_set_speed
                global velocity
                global vsl_good

                # pub_twist = Twist()
                pub_float = Float64()
                if vsl_good:
                    if in_i24:
                        print('publishing vsl:', vsl_set_speed)
                        # pub_twist.linear.x = vsl_set_speed*0.44704
                        pub_float.data = vsl_set_speed*0.44704
                        # self.mux_set_speed_pub.publish(pub_float)
                    else:
                        print('publishing car setpoint:',car_setpoint)
                        # pub_twist.linear.x = car_setpoint
                        pub_float.data = car_setpoint
                        # self.mux_set_speed_pub.publish(pub_float)
                else:
                    print('publising velocity', velocity)
                    # pub_twist.linear.x = velocity
                    pub_float.data = velocity
                self.mux_set_speed_pub.publish(pub_float)


            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = vslmux()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
