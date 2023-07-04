#!/usr/bin/env python
import rospy
import sys
import random
import os
import threading
import time

from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sqrt
from std_msgs.msg import Float64

ROBOT_PREFIX = "carter"
QTY_ROBOTS = 10
INTERVAL_PUB_MSGS = 1

ROBOTS_PROXIMITIES = [0] * QTY_ROBOTS
POS_PICKUP = [-7.5, 10.0]

lPublishers = [0] * QTY_ROBOTS


def create_publishers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        topic = f'/{ROBOT_PREFIX}{robot_id}/proximity_pick_up_location'
        lPublishers[robot_id - 1] = rospy.Publisher(topic, Float64, queue_size=10)
        
def create_subscribers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        #subscribe to amcl_pose to get current position
        rospy.Subscriber(f'/{ROBOT_PREFIX}{robot_id}/amcl_pose', PoseWithCovarianceStamped, update_proximity, callback_args=robot_id)


#publish rate each interval
def publish_proximity_each_interval():
    while True:
        for robot_id in range(1, QTY_ROBOTS + 1):
            proximity = ROBOTS_PROXIMITIES[robot_id - 1]
            pub = lPublishers[robot_id - 1]
            pub.publish(proximity)
        time.sleep(INTERVAL_PUB_MSGS)
    
def update_proximity(data, robot_id):
    #euclidian distance
    distance = sqrt((data.pose.pose.position.x - POS_PICKUP[0])**2 + (data.pose.pose.position.y - POS_PICKUP[1])**2)
    distance = round(distance, 2)
    ROBOTS_PROXIMITIES[robot_id - 1] = distance
    rospy.loginfo(f"{ROBOT_PREFIX}{robot_id} proximity to pickup: {distance}")
    

def initialize_proximity_all_robots():
    for robot_id in range(1, QTY_ROBOTS + 1):
        proximity = 100
        ROBOTS_PROXIMITIES[robot_id - 1] = proximity
        rospy.loginfo(f"{ROBOT_PREFIX}{robot_id} proximity to pickup: {proximity}")
        
def get_all_robots_current_ppul():
    return ROBOTS_PROXIMITIES
    
def get_robot_current_ppul(robot_id):
    return ROBOTS_PROXIMITIES[robot_id - 1]

def set_all_robots_ppul(new_ppul):
    for robot_id in range(1, QTY_ROBOTS + 1):
        set_robot_ppul(robot_id, new_ppul[robot_id - 1])
    
def set_robot_ppul(robot_id, ppul):
    ROBOTS_PROXIMITIES[robot_id - 1] = ppul

def start(prefixRobot = 'carter', qtyRobots = 1, intervalPubMsgs = 1):
    ROBOT_PREFIX = prefixRobot
    QTY_ROBOTS = qtyRobots
    INTERVAL_PUB_MSGS = intervalPubMsgs
    ROBOTS_PROXIMITIES = [0] * QTY_ROBOTS
    lPublishers = [0] * QTY_ROBOTS
    
    #initialize all robots proximities
    initialize_proximity_all_robots()
    
    #create publishers
    create_publishers()
    
    #create subscribers robots to amcl_pose
    create_subscribers()
    
    #start a thread to publish messages each interval
    tPublish = threading.Thread(target=publish_proximity_each_interval)
    tPublish.start()
    
    #rospy.spin()
    
    #Wait for all threads to finish
    tPublish.join()
