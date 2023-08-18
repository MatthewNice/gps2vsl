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
# import json

cmd_vel_float= 0.0

cmd_vel_float_topic = "/ramp_out"

def cmd_vel_callback(data):
    global cmd_vel_float
    cmd_vel_float = data.data


class float2twist:
    def __init__(self):
        rospy.init_node('float2twist', anonymous=True)

        rospy.Subscriber(cmd_vel_float_topic,Float64,cmd_vel_callback)

        #map this to the desired speed topic
        self.cmd_vel_twist_pub = rospy.Publisher('/vsl/twist_out', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                global cmd_vel_float

                pub_twist = Twist()
                pub_twist.linear.x = cmd_vel_float
                self.cmd_vel_twist_pub.publish(pub_twist)


            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = float2twist()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
