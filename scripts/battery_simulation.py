#!/usr/bin/env python
import rospy
import sys
import random
import os
import threading
import time

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist

ROBOT_PREFIX = "carter"
QTY_ROBOTS = 10
INTERVAL_PUB_MSGS = 1

ROBOTS_BATTERIES = [0] * QTY_ROBOTS
ROBOTS_POSITIONS = [0] * QTY_ROBOTS
ROBOTS_MOVING_STATE = [0] * QTY_ROBOTS
INTERVAL_UPDATE_BATTERY = 30
RND_BATTERY_MIN = 10
RND_BATTERY_MAX = 100
POS_CHARGING_PORT = [0, 17.0]
RANGE_POS_CHARGING_PORT = {'x': {'min': POS_CHARGING_PORT[0] - 1.5, 'max': POS_CHARGING_PORT[0] + 1.5}, 'y': {'min': POS_CHARGING_PORT[1] - 1.5, 'max': POS_CHARGING_PORT[1] + 1.5}}
VAL_BATTERY_LEVEL = {'MOVING': {'min': 3, 'max': 4}, 'IDLE': {'min': 1, 'max': 2}, 'CHARGING': {'min': 5, 'max': 10}}
lPublishers = [0] * QTY_ROBOTS


def create_publishers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        topic = f'/{ROBOT_PREFIX}{robot_id}/battery'
        lPublishers[robot_id - 1] = rospy.Publisher(topic, Int32, queue_size=10)
        
def create_subscribers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        #subscribe for current position of the robot and update its latest position
        rospy.Subscriber(f'/{ROBOT_PREFIX}{robot_id}/amcl_pose', PoseWithCovarianceStamped, update_latest_robot_pose, callback_args=robot_id)
        
        #subscribe for linear velocity of the robot and update its moving status
        rospy.Subscriber(f'/{ROBOT_PREFIX}{robot_id}/cmd_vel', Twist, update_robot_moving_status, callback_args=robot_id)

#publish rate each interval
def publish_battery_level_each_interval():
    while True:
        for robot_id in range(1, QTY_ROBOTS + 1):
            battery_level = ROBOTS_BATTERIES[robot_id - 1]
            pub = lPublishers[robot_id - 1]
            pub.publish(battery_level)
        time.sleep(INTERVAL_PUB_MSGS)

def update_battery_every_interval():
    while True:
        time.sleep(INTERVAL_UPDATE_BATTERY)
        for robot_id in range(1, QTY_ROBOTS + 1):
            eval_increase_decrease_battery(robot_id)        

    
def update_latest_robot_pose(data, robot_id):
    ROBOTS_POSITIONS[robot_id - 1] = data.pose.pose.position

#check and updates robot moving status, based on the linear and angular velocities
def update_robot_moving_status(data, robot_id):
    ROBOTS_MOVING_STATE[robot_id - 1] = (data.linear.x != 0 or data.linear.y != 0)
        
#evaluate whether to increase or decrease battery level according to current robot position and its moving status
def eval_increase_decrease_battery(robot_id):
    battery_level = ROBOTS_BATTERIES[robot_id - 1]    
    isMoving = ROBOTS_MOVING_STATE[robot_id - 1]
    
    #if near to charging port, increase, otherwise, decrease battery
    nearToChargingPort = (ROBOTS_POSITIONS[robot_id - 1] is not None and (ROBOTS_POSITIONS[robot_id - 1].x >= RANGE_POS_CHARGING_PORT['x']['min'] and ROBOTS_POSITIONS[robot_id - 1].x <= RANGE_POS_CHARGING_PORT['x']['max']) and (ROBOTS_POSITIONS[robot_id - 1].y <= RANGE_POS_CHARGING_PORT['y']['max'] and ROBOTS_POSITIONS[robot_id - 1].y >= RANGE_POS_CHARGING_PORT['y']['min']))
    if (nearToChargingPort):
        battery_level += random.randint(VAL_BATTERY_LEVEL['CHARGING']['min'], VAL_BATTERY_LEVEL['CHARGING']['max'])
    else:
        #if robot is moving, battery should decrease even more than when idle
        if (isMoving):
            battery_level -= random.randint(VAL_BATTERY_LEVEL['MOVING']['min'], VAL_BATTERY_LEVEL['MOVING']['max'])
        else:
            battery_level -= random.randint(VAL_BATTERY_LEVEL['IDLE']['min'], VAL_BATTERY_LEVEL['IDLE']['max'])
    
    #do not allow less than 0, neither more than 100
    if battery_level < 0:
        battery_level = 0
    if battery_level > 100:
        battery_level = 100
            
    #update final value
    ROBOTS_BATTERIES[robot_id - 1] = battery_level
    rospy.loginfo(f'{ROBOT_PREFIX}{robot_id}: Moving: {isMoving} Charging: {nearToChargingPort} Battery level: {battery_level}')
    
    
def reset_batteries_all_robots():
    for robot_id in range(1, QTY_ROBOTS + 1):
        battery_level = random.randint(RND_BATTERY_MIN, RND_BATTERY_MAX)
        ROBOTS_BATTERIES[robot_id - 1] = battery_level
        rospy.loginfo(f'{ROBOT_PREFIX}{robot_id}: Battery level: {battery_level}')
        
        #reset moving status
        ROBOTS_MOVING_STATE[robot_id - 1] = False


def get_all_robots_current_batteries():
    return ROBOTS_BATTERIES
    
def get_robot_current_battery(robot_id):
    return ROBOTS_BATTERIES[robot_id - 1]

def set_all_robots_batteries(new_robots_batteries):
    for robot_id in range(1, QTY_ROBOTS + 1):
        set_robot_battery(robot_id, new_robots_batteries[robot_id - 1])
    
def set_robot_battery(robot_id, battery):
    battery = int(battery)
    ROBOTS_BATTERIES[robot_id - 1] = battery
    rospy.loginfo(f'{ROBOT_PREFIX}{robot_id}: Battery level: {battery}')

def start(prefixRobot = 'carter', qtyRobots = 1, intervalPubMsgs = 1):
    ROBOT_PREFIX = prefixRobot
    QTY_ROBOTS = qtyRobots
    INTERVAL_PUB_MSGS = intervalPubMsgs
    ROBOTS_BATTERIES = [0] * QTY_ROBOTS
    ROBOTS_POSITIONS = [0] * QTY_ROBOTS
    ROBOTS_MOVING_STATE = [0] * QTY_ROBOTS
    lPublishers = [0] * QTY_ROBOTS
    
    #initialize all robots batteries
    reset_batteries_all_robots()
    
    #create publishers for all robots
    create_publishers()
    
    #subscribe robots to amcl_pose and cmd_vel
    create_subscribers()
        
    #start a thread to update batteries of all robots each interval
    tUpdateBatteries = threading.Thread(target=update_battery_every_interval)
    tUpdateBatteries.start()
    
    #start a thread to publish messages each interval
    tPublish = threading.Thread(target=publish_battery_level_each_interval)
    tPublish.start()
    
    #rospy.spin()
    
    #Wait for all threads to finish
    tPublish.join()
