# projectBlue

This code base is alpha. Its is hacked together for testing purposes. Please do not deploy directly. 

The stack has been developed for AUV Sedna on top of the Robot Operating System (Indigo)  written in c++ and python.The segregation of work is based on nodes with each node responsible for handling tasks specific to that node. The main distinction of nodes are based on :

- Controllers
- Mission planners
- Device Drivers
- Simulator
- Teleop
- Odometry

All nodes in the package are fired up when the robot starts using a Roslaunch file. The mission planner is responsible for pushing directives to other nodes.
