#!/bin/sh

#sudo cp -r smcDriver_v2 /usr/share/arduino/libraries/
#sudo cp -r PID_v1 /usr/share/arduino/libraries/
#sudo cp -r Adafruit_ADS1X15 /usr/share/arduino/libraries/

rosrun rosserial_client make_library.py . bbauv_msgs

sudo cp -r ros_lib/bbauv_msgs/ /usr/share/arduino/libraries/
rm -r ros_lib
