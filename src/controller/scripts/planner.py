#!/usr/bin/env python
import rosbag
bag = rosbag.Bag('1.bag')
for topic, msg, t in bag.read_messages(topics=['/turtle1/cmd_vel','/turtle1/color_sensor','turtle1/pose']):
    print msg
bag.close()
