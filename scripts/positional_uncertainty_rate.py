#!/usr/bin/env python
import rospy
import sys
import random
import os
import threading
import time

from math import sqrt
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

ROBOT_PREFIX = "carter"
QTY_ROBOTS = 10
INTERVAL_PUB_MSGS = 1

ROBOTS_PUR = [0] * QTY_ROBOTS
lPublishers = [0] * QTY_ROBOTS


def create_publishers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        topic = f'/{ROBOT_PREFIX}{robot_id}/positional_uncertainty_rate'
        lPublishers[robot_id - 1] = rospy.Publisher(topic, Float64, queue_size=10)
        
def create_subscribers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        #subscribe to amcl_pose to get covariances
        rospy.Subscriber(f'/{ROBOT_PREFIX}{robot_id}/amcl_pose', PoseWithCovarianceStamped, update_positional_uncertainty_rate, callback_args=robot_id)

#publish rate each interval
def publish_positional_ucr_each_interval():
    while True:
        for robot_id in range(1, QTY_ROBOTS + 1):
            rate = ROBOTS_PUR[robot_id - 1]
            pub = lPublishers[robot_id - 1]
            pub.publish(rate)
        time.sleep(INTERVAL_PUB_MSGS)

#calculate and update rate
def update_positional_uncertainty_rate(data, robot_id):
    cov = data.pose.covariance
    #elements 0 and 7 represent X and Y respectively. Element 14 represents Z, but we are not meassuring this axis.
    rate = ((cov[0] + cov[7]) / 2) * 100
    rate = round(rate, 2)
    ROBOTS_PUR[robot_id - 1] = rate
    rospy.loginfo(f'{ROBOT_PREFIX}{robot_id}: Positional Uncertainty Rate: {rate}')

    
def initialize_pur_all_robots():
    for robot_id in range(1, QTY_ROBOTS + 1):
        rate = 100
        ROBOTS_PUR[robot_id - 1] = rate
        rospy.loginfo(f'{ROBOT_PREFIX}{robot_id}: Positional Uncertainty Rate: {rate}')

def get_all_robots_current_purs():
    return ROBOTS_PUR
    
def get_robot_current_pur(robot_id):
    return ROBOTS_PUR[robot_id - 1]

def set_all_robots_purs(new_purs):
    for robot_id in range(1, QTY_ROBOTS + 1):
        set_robot_pur(robot_id, new_purs[robot_id - 1])
    
def set_robot_pur(robot_id, pur):
    ROBOTS_PUR[robot_id - 1] = pur

def start(prefixRobot = 'carter', qtyRobots = 1, intervalPubMsgs = 1):
    ROBOT_PREFIX = prefixRobot
    QTY_ROBOTS = qtyRobots
    INTERVAL_PUB_MSGS = intervalPubMsgs
    ROBOTS_PUR = [0] * QTY_ROBOTS
    lPublishers = [0] * QTY_ROBOTS
    
    #initialize all robots pur
    initialize_pur_all_robots()
    
    #create publishers
    create_publishers()
    
    #create subscribers robots to amcl_pose
    create_subscribers()
    
    #start a thread to publish messages each interval
    tPublish = threading.Thread(target=publish_positional_ucr_each_interval)
    tPublish.start()
    
    #rospy.spin()
    
    #Wait for all threads to finish
    tPublish.join()
