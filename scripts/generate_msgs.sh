#! /bin/bash

sudo rm -rf ~/sketchbook/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries
