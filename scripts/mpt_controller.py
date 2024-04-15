#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Int16, String, Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import time
import requests
import time
import bisect
import numpy as np
import pandas as pd
# import shapely
# from shapely import LineString,Point
# from shapely.ops import nearest_points
# from shapely import Polygon
import json
from mpc_accel import mpc_accel

velocity_topic = "/car/state/vel_x"
lead_x_topic = "/lead_dist"
lead_rv_topic = "/rel_vel"
cmd_accel_pre_topic = "/cmd_accel_pre"

cmd_accel_pre= None
velocity = None
lead_x=252
lead_rv=0

def velocity_callback(data):
    global velocity
    velocity = data.data

def lead_x_callback(data):
    global lead_x
    lead_x = data.data

def lead_rv_callback(data):
    global lead_rv
    lead_rv = data.data

def cmd_accel_pre_callback(data):
    global cmd_accel_pre
    cmd_accel_pre = data.data

class mpt:
    def __init__(self):
        rospy.init_node('mpt', anonymous=True)

        # rospy.Subscriber(gantry_topic,Int16,gantry_callback)
        rospy.Subscriber(velocity_topic,Float64,velocity_callback)
        rospy.Subscriber(cmd_accel_pre_topic,Float64,cmd_accel_pre_callback)

        rospy.Subscriber(lead_x_topic, Float64, lead_x_callback)
        rospy.Subscriber(lead_rv_topic, Float64, lead_rv_callback)


        global mpt_pub
        mpt_pub = rospy.Publisher('/cmd_accel', Float64, queue_size=1000)

        self.rate = rospy.Rate(20)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                global mpt_pub
                global cmd_accel_pre
                global velocity
                global lead_rv
                global lead_x

                state=np.array([[velocity],[velocity+lead_rv],[lead_x]]).reshape((3,1))
                mpt_accel_max = mpc_accel(state)

                mpt_value = min(cmd_accel_pre,mpt_accel_max)

                mpt_pub.publish(mpt_value)

            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = mpt()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
