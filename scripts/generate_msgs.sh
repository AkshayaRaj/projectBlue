#! /bin/bash

sudo rm -rf /usr/share/arduino/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py /usr/share/arduino/libraries
