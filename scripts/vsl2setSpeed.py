#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64, Int16, String, Bool
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import requests
import time
import bisect
import numpy as np
import pandas as pd
# import shapely
from shapely import LineString,Point
from shapely.ops import nearest_points
from shapely import Polygon
import json

# velocity_topic = "vel"
gantry_topic = "/vsl/latest_gantry"

gantry = None
mySpeeds = None
#TODO manage cronjob or looped bash which will pull the data from ransom PhP endpoint

#every 5 seconds, read in the json file
def getVSLspeeds(sleep_time=5):
    global mySpeeds
    while True:
        try:
            s = requests.get('http://ransom.isis.vanderbilt.edu/vsl/current_vsl_speeds.json')
            mySpeeds = s.json()
            # return s.json()
        except:
            print('could not get posted speed json from ransom')

        time.sleep(sleep_time)

def gantry_callback(data):
    global gantry
    gantry = data.data

def get_gantry_set_speed(gantry):
    # f = open('/etc/libpanda.d/current_vsl_speeds.json')
    # mySpeeds=json.load(f)
    global mySpeeds
    global gantry
    # lookup speed from gantry input
    speed_limits = {i[2]:i[9] for i in mySpeeds}
    gantry_set_speed = speed_limits[gantry]

    return gantry_set_speed

class vsl2setSpeed:
    def __init__(self):
        rospy.init_node('vsl2setSpeed', anonymous=True)

        rospy.Subscriber(gantry_topic,Int16,gantry_callback)

        # global in_i24_pub
        # in_i24_pub = rospy.Publisher('/vsl/in_i24', Bool, queue_size=10)
        self.vsl_set_speed_pub = rospy.Publisher('/vsl/set_speed', Int16, queue_size=10) #sample and hold, doest not publish until getting close to one
        self.rate = rospy.Rate(1)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                global gantry
                if gantry != None:
                    # print('\nis there a gantry?')
                    myGantrySetSpeed = get_gantry_set_speed(gantry)
                if myGantrySetSpeed != None: #either last gantry closest to, or a new one
                    self.vsl_set_speed_pub.publish(myGantrySetSpeed)


            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = vsl2setSpeed()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
