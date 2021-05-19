#!/usr/bin/env python3

import rospy
from pozyx_ros import PozyxDriver
rospy.init_node("pozyx")
a = PozyxDriver()