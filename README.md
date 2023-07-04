# RosIsaacSimWarehouseCarterx10
Customizations to the "IsaacSim-ros_workspaces" to control 10 Carter robots in the "simple warehouse" scenario.

This code is for ROS noetic version. The files should be place under the path ``/home/[YourUserName]/IsaacSim-ros_workspaces/noetic_ws/src/``
After placing the files, you should run ``catkin_make`` and ``source devel/setup.bash``.

## Steps to enable quality properties
1. Run rosbridge through this command ``roslaunch rosbridge_server rosbridge_websocket.launch``
2. Run Nvidia Isaac Sim and open the warehouse world. You should add 10 Carter robots to it.
3. Run the command ``python3 src/scripts/carter_rest.py 10``. This script provides a REST API that allows to control the robots. It also simulates batteries and payload capacity. The script also provides data from sensors such as: proximity to pickup location, positional uncertainty rate to each robot. So that the data can be gathered through rosbridge using roslib by subscribing to topics.