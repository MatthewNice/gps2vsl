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
import pandas as pd
# import shapely
from shapely import LineString,Point
from shapely.ops import nearest_points


#read in calc_mm_locations
mm_locations = pd.read_csv('./calc_mm_locations')#set proper file locations
#read in the points of the VSL_location
vsl_locations = pd.read_csv('./vsl_locations')#set proper file locations

#pre-sort mm_locations by mm
mm_locs_sorted = mm_locations.sort_values(by='mm').reset_index(drop=True)
mm_linestring = LineString([(row.latitude,row.longitude) for index,row in mm_locs_sorted.iterrows()])

gantry=None
last_gantry=None

lat = 0.0
last_lats = [0.0] * len_history #one value added for the last n observations
lon = 0.0
last_lons = [0.0] * len_history #one value added for the last n observations

gps_fix_topic = "gps_fix"
gps_fix_time_reference_topic = "gps_fix_time"
bearing_topic = "bearing"
velocity_topic = "vel"

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

    latitude = data.latitude
    longitude = data.longitude
    status = data.status.status
    systime = rospy.Time.now()
    gps_update_time = systime

def gps_fix_time_reference_callback(data):
    global gpstime
    gpstime = data.time_ref

def bearing_callback(data):
    global bearing
    bearing = data.data

# def velocity_callback(data):
#     global velocity
#     velocity = data.data

def get_direction(bearing):
    #set status of direction as bearing-related switch/case
    #eastbound I-24 [180,200], westbound I-24 [0,20], or neither
    center_eastbound_bearing=190
    center_westbound_bearing=10
    if abs(bearing-center_eastbound_bearing) < 45:
        return direction = "e"
    elif abs(bearing-center_westbound_bearing) < 45:
        return direction = "w"
    else:
        return None


def get_gantry():
    global last_gantry
    #use polygon of I-24 corridor as a bound? not now
    direction = get_direction(bearing)
    gantry = findVSL(latitude,longitude,direction)

    if gantry != None:
        #update the gantry memory
        last_gantry = gantry
        #return the ganty to the gantry topic
        return gantry
    else:
        #publish the last value of the gantry, since there is not another clearly assigned
        #initially this will be None
        return last_gantry

def findVSL(lat,long, direction, distance_threshold=0.2):
    """This function will return the closest vsl gantry location, if it is within 'distance_threshold' initialized
    at 0.2 miles. If the point is not within the threshold, or an improper travel direction is given, then
    this function returns None.
    """
    global mm_locs_sorted
    global mm_linestring
    #use gps_fix and distance of at least ~300m (0.2 mile is a little longer) to set a gantry
    point = Point(lat,long) #i.e. gps_fix
    #find closest point in calc_mm_locations LineString to gps_fix point
    closest_mm = nearest_points(mm_linestring,point)[0] #this Point is the closest value

    mm = mm_locs_sorted.loc[
        (abs(mm_locs_sorted.latitude-list(closest_mm.coords)[0][0]) < 1e-5)&
        (abs(mm_locs_sorted.longitude-list(closest_mm.coords)[0][1]) < 1e-5)
    ].mm.values[0] #this is the closest milemarker

    #use mm and heading to lookup closest gantry
    direction_vsl_locations = vsl_locations.loc[vsl_locations.latitude==direction]#filter by direction
    min_dist = abs(direction_vsl_locations.calculated_milemarker-mm).min() #distance to closest mm_location in miles

    if (direction_vsl_locations.shape[0]>0) & (min_dist < distance_threshold):
        return direction_vsl_locations.loc[
            direction_vsl_locations.calculated_milemarker-mm == min_dist
        ].vsl_id.values[0] #this is the closest gantry
    else:
        return None


class gps2head:
    def __init__(self):
        # global vin
        rospy.init_node('head2vsl', anonymous=True)

        rospy.Subscriber(gps_fix_topic, NavSatFix, gps_fix_callback)
        rospy.Subscriber(gps_fix_time_reference_topic, TimeReference, gps_fix_time_reference_callback)
        rospy.Subscriber(bearing_topic,Int16,bearing_callback)
        # rospy.Subscriber(velocity_topic,Float64,velocity_callback)

        self.latest_gantry_pub = rospy.Publisher('/latest_gantry', Int16, queue_size=10) #sample and hold, doest not publish until getting close to one
        self.rate = rospy.Rate(1)

    def loop(self):
        while not rospy.is_shutdown():
            try:
                # global latitude
                # global longitude
                # current_time = rospy.Time.now()
                # assert gps_update_time is not None, "GPS data has never been received!"
                # assert can_update_time is not None, "CAN data has never been received!"
                # assert abs((current_time - gps_update_time).to_sec()) < 30, "GPS data more than 30 seconds old!"
                # assert abs((current_time - can_update_time).to_sec()) < 30, "CAN data more than 30 seconds old!
                myGantry = get_gantry()
                if myGantry != None: #either last gantry closest to, or a new one
                    self.latest_gantry_pub.publish(myGantry)


            except Exception as e:
                print(e)
                traceback.print_exc()
                print("Something has gone wrong.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        head = gps2head()
        head.loop()
    except Exception as e:
        print(e)
        traceback.print_exc()
        print("An exception occurred")
