#!/usr/bin/env python

import rospy
import sys
import random
import os
import threading
import time
import signal
import actionlib
import json

from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from flask import Flask, request, jsonify
from math import sqrt

import max_payload_capacity as mpc
import positional_uncertainty_rate as pur
import battery_simulation as bat
import proximity_pick_up_location as ppul

ROBOT_PREFIX = "carter"
QTY_ROBOTS = 10
INTERVAL_PUB_MSGS = 1

POS_START = [-2.0, 0.0]
POS_PICKUP = [-7.5, 10.0]
POS_DELIVERY = [1.0, 8.35]
POS_CHARGING_PORT = [0, 17.0]
POS_OBSTACLES = [[-0.19469, 0], [-0.19469, 6.1064], [0.80949, 1.4934]]

MAP_MIN_X = -8.0
MAP_MIN_Y = -11.5
MAP_MAX_X = 8.5
MAP_MAX_Y = 17.0

INIT_POS_ROBOTS = [[7.0, 6.0],[7.0, 3.0],[7.0, 0.0],[7.0, -3.0],[7.0, -6.0],[5.0, -10.0],[2.0, -10.0],[-1.0, -10.0],[-4.0, -10.0],[-7.0, -10.0]]
lPublishers = []
ROBOTS_POSITIONS = [0] * QTY_ROBOTS

rospy.init_node('ros_rest')
app = Flask(__name__)


class Publisher:
    def __init__(self, robot_id):
        self.client = actionlib.SimpleActionClient(f'/carter{robot_id}/move_base', MoveBaseAction)
        self.client.wait_for_server()

    def send_goal(self, x, y, should_wait = False):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 0.01
        
        if should_wait:
            return self.client.send_goal_and_wait(goal)
        else:
            self.client.send_goal(goal)
            return GoalStatus.PENDING
            
        #wait = self.client.wait_for_result()
        
    def get_state(self):
        stateCode = self.client.get_state()
        stateStr = GoalStatus.to_string(stateCode)
        return {"code": stateCode, "message": stateStr}
        #uint8 PENDING         = 0   # The goal has yet to be processed by the action server
        #uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
        #uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
        #                            #   and has since completed its execution (Terminal State)
        #uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
        #uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
        #                            #    to some failure (Terminal State)
        #uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
        #                            #    because the goal was unattainable or invalid (Terminal State)
        #uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
        #                            #    and has not yet completed execution
        #uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
        #                            #    but the action server has not yet confirmed that the goal is canceled
        #uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
        #                            #    and was successfully cancelled (Terminal State)
        #uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
        #                            #    sent over the wire by an action server

def create_subscribers():
    for robot_id in range(1, QTY_ROBOTS + 1):
        #subscribe for current position of the robot and update its latest position
        rospy.Subscriber(f'/{ROBOT_PREFIX}{robot_id}/amcl_pose', PoseWithCovarianceStamped, update_latest_robot_pose, callback_args=robot_id)
        
def update_latest_robot_pose(data, robot_id):
    ROBOTS_POSITIONS[robot_id - 1] = data.pose.pose.position
   
def fromStateCodeToStateJson(stateCode):
    if stateCode in range(0, 9):
        stateStr = GoalStatus.to_string(stateCode)
    else:
        stateStr = "Unknown"
    return {"code": stateCode, "message": stateStr}

@app.route('/')
def health_check():
    return 'Ok!'


@app.route('/MoveToStart/<robot_id>', methods=['POST'])
def move_to_start_function(robot_id):
    robot_id = int(robot_id)
    
    if robot_id < 1 or robot_id > len(lPublishers):
        return jsonify({'Error': 'Invalid robot_id'})
    
    publisher = lPublishers[robot_id - 1]

    # Get start position coordenates
    x = POS_START[0]
    y = POS_START[1]

    msg = f"Moving robot_{robot_id} to start position at [{x}, {y}]"
    print(msg)

    result = publisher.send_goal(x, y, True)    
    return jsonify(fromStateCodeToStateJson(result))


@app.route('/MoveToPickupLocation/<robot_id>', methods=['POST'])
def move_to_pickup_function(robot_id):
    robot_id = int(robot_id)
    
    if robot_id < 1 or robot_id > len(lPublishers):
        return jsonify({'Error': 'Invalid robot_id'})
    
    publisher = lPublishers[robot_id - 1]

    # Get start position coordenates
    x = POS_PICKUP[0]
    y = POS_PICKUP[1]

    msg = f"Moving robot_{robot_id} to pickup position at [{x}, {y}]"
    print(msg)

    result = publisher.send_goal(x, y, True)    
    return jsonify(fromStateCodeToStateJson(result))


@app.route('/PickupMaterialsContainer/<robot_id>', methods=['POST'])
def pick_up_material_container_function(robot_id):
    print(f'Picking up the materials container by robot_{robot_id}')
    time.sleep(2)
    print('Container of materials collected')

    # Return
    return jsonify({'pickup_container': 'Finished'})


@app.route('/MoveToDeliveryLocation/<robot_id>', methods=['POST'])
def move_to_delivery_function(robot_id):
    robot_id = int(robot_id)
    
    if robot_id < 1 or robot_id > len(lPublishers):
        return jsonify({'Error': 'Invalid robot_id'})
    
    publisher = lPublishers[robot_id - 1]

    # Get start position coordenates
    x = POS_DELIVERY[0]
    y = POS_DELIVERY[1]

    msg = f"Moving robot_{robot_id} to delivery position at [{x}, {y}]"
    print(msg)

    result = publisher.send_goal(x, y, True)    
    return jsonify(fromStateCodeToStateJson(result))

@app.route('/MoveToChargingPort/<robot_id>', methods=['POST'])
def move_to_charging_port(robot_id):
    robot_id = int(robot_id)
    
    if robot_id < 1 or robot_id > len(lPublishers):
        return jsonify({'Error': 'Invalid robot_id'})
    
    publisher = lPublishers[robot_id - 1]

    # Get start position coordenates
    x = POS_CHARGING_PORT[0]
    y = POS_CHARGING_PORT[1]

    msg = f"Moving robot_{robot_id} to charging port position at [{x}, {y}]"
    print(msg)
    
    result = publisher.send_goal(x, y, True)    
    return jsonify(fromStateCodeToStateJson(result))
    
@app.route('/collect_resource/<robot_id>', methods=['POST'])
def collect_resource(robot_id):
    move_to_pickup_function(robot_id)
    pick_up_material_container_function(robot_id)
    move_to_delivery_function(robot_id)
    
    return jsonify({'move_goal': 'Finished'})

@app.route('/move_async/<robot_id>', methods=['POST'])
def move_base_goal_async(robot_id):
    # Get the data from the request
    data = request.get_json()
    return move_base_goal_function(robot_id, False, data)


@app.route('/move/<robot_id>', methods=['POST'])
def move_base_goal_sync(robot_id):
    # Get the data from the request
    data = request.get_json()
    return move_base_goal_function(robot_id, True, data)

    
def move_base_goal_function(robot_id, should_wait, data):
    robot_id = int(robot_id)
    
    if robot_id < 1 or robot_id > len(lPublishers):
        return jsonify({'Error': 'Invalid robot_id'})
    
    publisher = lPublishers[robot_id - 1]

    # Get command
    x = float(data['x'])
    y = float(data['y'])

    msg = f"Moving robot_{robot_id} to [{x}, {y}]"
    print(msg)

    result = publisher.send_goal(x, y, should_wait)    
    return jsonify(fromStateCodeToStateJson(result))


@app.route('/MoveAllRandomPosition', methods=['POST'])
def move_all_robots_random_position():
    positions = []  # List to store the positions of the robots
    results = []
    
    for robot_id in range(1, QTY_ROBOTS + 1):
        collision = True
        while collision:
            # Get random position coordinates
            x = random.uniform(MAP_MIN_X, MAP_MAX_X)
            y = random.uniform(MAP_MIN_Y, MAP_MAX_Y)
            collision = any(distance(x, y, pos[0], pos[1]) < 2.0 for pos in positions) or robot_will_collide_obstacles(x, y)

        # Add the position to the list
        positions.append((x, y))

        msg = f"Moving robot_{robot_id} to initial random position at [{x}, {y}]"
        print(msg)

        publisher = lPublishers[robot_id - 1]        
        result = publisher.send_goal(x, y)
        results.append(fromStateCodeToStateJson(result))

    return jsonify(results)
    

@app.route('/MoveRandomPosition/<robot_id>', methods=['POST'])
def move_random_position(robot_id):
    robot_id = int(robot_id)
    x = random.uniform(MAP_MIN_X, MAP_MAX_X)
    y = random.uniform(MAP_MIN_Y, MAP_MAX_Y)

    msg = f"Moving robot_{robot_id} to initial random position at [{x}, {y}]"
    print(msg)

    publisher = lPublishers[robot_id - 1]
    result = publisher.send_goal(x, y)

    return jsonify(fromStateCodeToStateJson(result))
    
@app.route('/MoveToPositionRecoveryStrategy', methods=['POST'])
def move_desired_position_recovery_strategy():
    robot_id = request.args.get('robot_id', default = 1, type = int)
    destination = request.args.get('destination', default = 'INITIAL_POS', type = str)
    
    #desired position for the robot
    if(destination == 'INITIAL_POS'):
        xd = INIT_POS_ROBOTS[robot_id - 1][0]
        yd = INIT_POS_ROBOTS[robot_id - 1][1]
    elif(destination == 'PICKUP_POS'):
        xd = POS_PICKUP[0]
        yd = POS_PICKUP[1]
    elif(destination == 'DELIVERY_POS'):
        xd = POS_DELIVERY[0]
        yd = POS_DELIVERY[1]
    elif(destination == 'SPECIFIC_POS'):
        data = request.get_json()
        xd = data["x"]
        yd = data["y"]
    
    tMove = threading.Thread(target=thread_mdprs, args=(robot_id, xd, yd,))
    tMove.start()

    return get_state(robot_id)
    
def robot_will_collide_obstacles(x, y):        
    #check the robot wont collide with obstacles
    collision = any(distance(x, y, pos[0], pos[1]) < 1.5 for pos in POS_OBSTACLES)
    return collision
        
    
def thread_mdprs(robot_id, xd, yd):
    success = False
    metersToMove = 2.5
    metersToConsiderLost = 6
    distance = 100
    
    while(not success or distance >= metersToConsiderLost):
        #move 3 meters away from current position in direction to destination
        xcp = ROBOTS_POSITIONS[robot_id - 1].x
        ycp = ROBOTS_POSITIONS[robot_id - 1].y
        
        #calculate euclidian distance to destination
        distance = sqrt((xcp - xd)**2 + (ycp - yd)**2)
        
        #robot is usually lost when "metersToConsiderLost" meters away from destination
        if (distance < metersToConsiderLost):
            metersToMove = metersToMove - 0.5
        
        collision = True
        while collision:
            if (xcp > xd):
                xr = xcp - metersToMove
            elif (xcp < xd):
                xr = xcp + metersToMove
                
            if (ycp > yd):
                yr = ycp - metersToMove
            elif (ycp < yd):
                yr = ycp + metersToMove
            
            #check the robot wont collide with obstacles
            collision = robot_will_collide_obstacles(xr, yr)            
            
            #if the robot will collide, increment meters to move by 0.2 and try again
            if (collision):
                metersToMove = metersToMove + 0.2
                
            print(f"collision: {collision} metersToMove: {metersToMove}")
        
        msg = f"Recovery strategy. Moving robot_{robot_id} close to desired position at [{xr}, {yr}]"
        print(msg)

        publisher = lPublishers[robot_id - 1]
        result = publisher.send_goal(xr, yr, True)
        
        msg = f"Recovery strategy. Moving robot_{robot_id} close to desired position at [{xr}, {yr}] Result: {result}"
        print(msg)
        
        success = result == 3
    
    #move to the real inital pos
    msg = f"Recovery strategy. Moving robot_{robot_id} to desired position at [{xd}, {yd}]"
    print(msg)

    publisher = lPublishers[robot_id - 1]
    result = publisher.send_goal(xd, yd, True)
    
    msg = f"Recovery strategy. Moving robot_{robot_id} to desired position at [{xd}, {yd}] Result: {result}"
    print(msg)
    
    
@app.route('/MoveInitialPosition/<robot_id>', methods=['POST'])
def move_initial_position(robot_id):
    robot_id = int(robot_id)
    x = INIT_POS_ROBOTS[robot_id - 1][0]
    y = INIT_POS_ROBOTS[robot_id - 1][1]

    msg = f"Moving robot_{robot_id} to initial position at [{x}, {y}]"
    print(msg)

    publisher = lPublishers[robot_id - 1]
    result = publisher.send_goal(x, y)

    return jsonify(fromStateCodeToStateJson(result))


def distance(x1, y1, x2, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    
    
@app.route('/get_state/<robot_id>')
def get_state(robot_id):
    robot_id = int(robot_id)
    
    if robot_id < 1 or robot_id > len(lPublishers):
        return jsonify({'Error': 'Invalid robot_id'})
    
    publisher = lPublishers[robot_id - 1]
    return jsonify(publisher.get_state())
    
@app.route('/get_state_all_robots')
def get_state_all_robots():
    results = []

    for robot_id in range(1, QTY_ROBOTS + 1):
        publisher = lPublishers[robot_id - 1]
        results.append(publisher.get_state())
        
    return jsonify(results)

#batteries
@app.route('/get_all_robots_current_batteries')
def get_all_robots_current_batteries():
    return json.dumps(bat.get_all_robots_current_batteries())
    
@app.route('/get_robot_current_battery/<robot_id>')
def get_robot_current_battery(robot_id):
    robot_id = int(robot_id)
    return jsonify(bat.get_robot_current_battery(robot_id))
    
@app.route('/set_all_robots_batteries', methods=['POST'])
def set_all_robots_batteries():
    try:
        new_robots_batteries = request.get_json()
        new_robots_batteries = json.loads(new_robots_batteries)
        bat.set_all_robots_batteries(new_robots_batteries)
        return jsonify({'result': 'ok'})
    except:
        return jsonify({'result': 'error'})
        
#payload capacity
@app.route('/get_all_robots_current_payload_capacity')
def get_all_robots_current_payload_capacity():
    return json.dumps(mpc.get_all_robots_current_payload_capacity())
    
@app.route('/get_robot_current_payload_capacity/<robot_id>')
def get_robot_current_payload_capacity(robot_id):
    robot_id = int(robot_id)
    return jsonify(mpc.get_robot_current_payload_capacity(robot_id))
    
@app.route('/set_all_robots_payload_capacities', methods=['POST'])
def set_all_robots_payload_capacities():
    try:
        new_payload_capacities = request.get_json()
        new_payload_capacities = json.loads(new_payload_capacities)
        mpc.set_all_robots_payload_capacities(new_payload_capacities)
        return jsonify({'result': 'ok'})
    except:
        return jsonify({'result': 'error'})
        
#positional uncertainty rate
@app.route('/get_all_robots_current_purs')
def get_all_robots_current_purs():
    return json.dumps(pur.get_all_robots_current_purs())
    
@app.route('/get_robot_current_pur/<robot_id>')
def get_robot_current_pur(robot_id):
    robot_id = int(robot_id)
    return jsonify(pur.get_robot_current_pur(robot_id))

@app.route('/set_all_robots_purs', methods=['POST'])
def set_all_robots_purs():
    try:
        new_purs = request.get_json()
        new_purs = json.loads(new_purs)
        pur.set_all_robots_purs(new_purs)
        return jsonify({'result': 'ok'})
    except:
        return jsonify({'result': 'error'})

#proximity pickup
@app.route('/get_all_robots_current_ppul')
def get_all_robots_current_ppul():
    return json.dumps(ppul.get_all_robots_current_ppul())
    
@app.route('/get_robot_current_ppul/<robot_id>')
def get_robot_current_ppul(robot_id):
    robot_id = int(robot_id)
    return jsonify(ppul.get_robot_current_ppul(robot_id))

@app.route('/set_all_robots_ppul', methods=['POST'])
def set_all_robots_ppul():
    try:
        new_ppul = request.get_json()
        new_ppul = json.loads(new_ppul)
        ppul.set_all_robots_ppul(new_ppul)
        return jsonify({'result': 'ok'})
    except:
        return jsonify({'result': 'error'})

#position
@app.route('/get_all_robots_current_position')
def get_all_robots_current_position():
    results = []
    for robot_id in range(1, QTY_ROBOTS + 1):
        pointObj = ROBOTS_POSITIONS[robot_id - 1]
        pointDict = {"x": pointObj.x, "y": pointObj.y}
        results.append(pointDict)
    
    return json.dumps(results)

@app.route('/get_robot_current_position/<robot_id>')
def get_robot_current_position(robot_id):
    robot_id = int(robot_id)
    pointObj = ROBOTS_POSITIONS[robot_id - 1]
    pointDict = {"x": pointObj.x, "y": pointObj.y}
    return jsonify(pointDict)

    
@app.route('/MoveAllInitialPosition', methods=['POST'])
def move_all_robots_initial_position():
    results = []
    
    for robot_id in range(1, QTY_ROBOTS + 1):
        x = INIT_POS_ROBOTS[robot_id - 1][0]
        y = INIT_POS_ROBOTS[robot_id - 1][1]

        msg = f"Moving robot_{robot_id} to initial position at [{x}, {y}]"
        print(msg)

        publisher = lPublishers[robot_id - 1]
        result = publisher.send_goal(x, y)
        results.append(fromStateCodeToStateJson(result))

    # Return
    return jsonify(results)
    
@app.route('/RandomizeMaxPayloadCapacityAllRobots', methods=['POST'])
def reset_max_payload_capacity_all_robots():
    try:
        mpc.reset_max_payload_capacity_all_robots()
        return jsonify({'result': 'ok'})
    except:
        return jsonify({'result': 'error'})
    
@app.route('/ResetBatteriesAllRobots', methods=['POST'])
def reset_batteries_all_robots():
    try:
        bat.reset_batteries_all_robots()
        return jsonify({'result': 'ok'})
    except:
        return jsonify({'result': 'error'})
        
    
def signal_handler(signal, frame):
    print("Stopping the program...")
    sys.exit(0)
    
def start_qos_properties():
    #instantiate as many Publisher classes as QTY_ROBOTS 
    for i in range(1, QTY_ROBOTS + 1):
        lPublishers.append(Publisher(str(i)))
        
    #create subscriber for amcl_pose
    ROBOTS_POSITIONS = [0] * QTY_ROBOTS
    create_subscribers()    
        
    #start sub-scripts in threads
    threads = []
    tMaxPayloadCapacity = threading.Thread(target=mpc.start, args=(ROBOT_PREFIX, QTY_ROBOTS, INTERVAL_PUB_MSGS,))
    tMaxPayloadCapacity.start()
    threads.append(tMaxPayloadCapacity)
    
    tPositionalUncertainty = threading.Thread(target=pur.start, args=(ROBOT_PREFIX, QTY_ROBOTS, INTERVAL_PUB_MSGS,))
    tPositionalUncertainty.start()
    threads.append(tPositionalUncertainty)
    
    tBattery = threading.Thread(target=bat.start, args=(ROBOT_PREFIX, QTY_ROBOTS, INTERVAL_PUB_MSGS,))
    tBattery.start()
    threads.append(tBattery)
    
    tProximityPickup = threading.Thread(target=ppul.start, args=(ROBOT_PREFIX, QTY_ROBOTS, INTERVAL_PUB_MSGS,))
    tProximityPickup.start()
    threads.append(tProximityPickup)

  

if __name__ == '__main__':    
    try:
        QTY_ROBOTS = int(sys.argv[1])
    except IndexError:
        QTY_ROBOTS = 1
        
    # Register the signal handler for CTRL + C
    signal.signal(signal.SIGINT, signal_handler)

	#start publishing qos properties
    start_qos_properties()
    
    #start flask app
    port = int(os.environ.get('PORT', 5000))
    app.run(debug=True, host='0.0.0.0', port=port, use_reloader=False)
        
        
