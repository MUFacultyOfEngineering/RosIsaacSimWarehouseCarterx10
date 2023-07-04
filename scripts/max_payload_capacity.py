#!/usr/bin/env python
import rospy
import sys
import random
import os
import threading
import time

from math import sqrt
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

#this program sets max payload capacity to robots
ROBOT_PREFIX = "carter"
QTY_ROBOTS = 10
INTERVAL_PUB_MSGS = 1

ROBOTS_MPC = [0] * QTY_ROBOTS
MIN_CAPACITY = 0.5
MAX_CAPACITY = 2.0

lPublishers = [0] * QTY_ROBOTS


def create_publishers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        topic = f'/{ROBOT_PREFIX}{robot_id}/max_payload_capacity'
        lPublishers[robot_id - 1] = rospy.Publisher(topic, Float64, queue_size=10)
        
#publish rate each interval
def publish_max_capacity_each_interval():
    while True:
        for robot_id in range(1, QTY_ROBOTS + 1):
            payload = ROBOTS_MPC[robot_id -1]
            pub = lPublishers[robot_id - 1]            
            pub.publish(payload)
        time.sleep(INTERVAL_PUB_MSGS)

def reset_max_payload_capacity_all_robots():
    for robot_id in range(1, QTY_ROBOTS + 1):
        payload = round(random.uniform(MIN_CAPACITY, MAX_CAPACITY), 2)
        ROBOTS_MPC[robot_id -1] = payload
        rospy.loginfo(f"{ROBOT_PREFIX}{robot_id} max payload capacity: {payload}")

def get_all_robots_current_payload_capacity():
    return ROBOTS_MPC
    
def get_robot_current_payload_capacity(robot_id):
    return ROBOTS_MPC[robot_id - 1]

def set_all_robots_payload_capacities(new_payload_capacities):
    for robot_id in range(1, QTY_ROBOTS + 1):
        set_robot_payload_capacity(robot_id, new_payload_capacities[robot_id - 1])
    
def set_robot_payload_capacity(robot_id, payload_capacity):
    ROBOTS_MPC[robot_id - 1] = payload_capacity

def start(prefixRobot = 'carter', qtyRobots = 1, intervalPubMsgs = 1):    
    ROBOT_PREFIX = prefixRobot
    QTY_ROBOTS = qtyRobots
    INTERVAL_PUB_MSGS = intervalPubMsgs
    ROBOTS_MPC = [0] * QTY_ROBOTS
    lPublishers = [0] * QTY_ROBOTS
    
    #create publishers
    create_publishers()
    
    #initialize all robots mpc
    reset_max_payload_capacity_all_robots()
    
    #start a thread to publish messages each interval
    tPublish = threading.Thread(target=publish_max_capacity_each_interval)
    tPublish.start()
    
    #rospy.spin()
    
    tPublish.join()
