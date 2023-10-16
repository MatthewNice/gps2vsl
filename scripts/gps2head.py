#!/usr/bin/env python


import rospy
from std_msgs.msg import Float64, Int16, String
from sensor_msgs.msg import NavSatFix, TimeReference
import traceback
import os
import sys
import requests
import time
import bisect
import numpy as np

len_history = 50
count = 0
lat = 0.0
last_lats = [0.0] * len_history #one value added for the last n observations
lon = 0.0
last_lons = [0.0] * len_history #one value added for the last n observations
bearing = 0.0


gps_fix_topic = "/car/gps/fix"#formerly "gps_fix"
gps_fix_time_reference_topic = "gps_fix_time"
gpstime = None
systime = None
latitude = None
longitude = None
status = None
gps_update_time = None

def gps_fix_callback(data):
    global systime
    global latitude
    global longitude
    global status
    global gps_update_time
    global count

    latitude = data.latitude
    longitude = data.longitude
    status = data.status.status
    systime = rospy.Time.now()
    gps_update_time = systime

    if longitude is not None:
        last_lons.pop()
        last_lons.insert(0, longitude)
    if latitude is not None:
        last_lats.pop()
        last_lats.insert(0, latitude)

    if count >= len_history:
        heading = get_heading(last_lons, last_lats)

        heading_pub.publish(int(heading))

    if count < len_history:
        count +=1

def gps_fix_time_reference_callback(data):
    global gpstime
    gpstime = data.time_ref

#read in the points of the VSL?


#define calculation of bearing
def get_heading(last_lons,last_lats):
    #calculate the average heading on the last 10 seconds
    lat1=last_lats[-1] #oldest lat
    lat2=last_lats[0] #newest lat
    lon1=last_lons[-1] #oldest lon
    lon2=last_lons[0] #newest lon

    myHeading = get_bearing(lat1,lon1,lat2,lon2)
    return myHeading

def get_bearing(lat1, lon1, lat2, lon2):
    dLon = lon2 - lon1
    x = np.cos(lat2) * np.sin(dLon)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dLon)
    brng = np.rad2deg(np.arctan2(x, y))
    if brng < 0:
        brng += 360
    return brng


class gps2head:
    def __init__(self):
        # global vin
        rospy.init_node('gps2head', anonymous=True)

        rospy.Subscriber(gps_fix_topic, NavSatFix, gps_fix_callback)
        rospy.Subscriber(gps_fix_time_reference_topic, TimeReference, gps_fix_time_reference_callback)

        global heading_pub
        heading_pub = rospy.Publisher('/vsl/heading', Int16, queue_size=10)
        self.rate = rospy.Rate(1)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                global latitude
                global longitude
                current_time = rospy.Time.now()
                assert gps_update_time is not None, "GPS data has never been received!"
                # assert can_update_time is not None, "CAN data has never been received!"
                assert abs((current_time - gps_update_time).to_sec()) < 30, "GPS data more than 30 seconds old!"
                # assert abs((current_time - can_update_time).to_sec()) < 30, "CAN data more than 30 seconds old!

            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong in gps2head.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = gps2head()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
