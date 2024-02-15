#!/usr/bin/env python3
 
import numpy as np
import sys
import os
import subprocess
 
import roslib
import rospy
from std_msgs.msg import Float32 as Float

Config = {
  "dev_ups": "/dev/ttyUSB0",
  "dev_cab": "/dev/ttyACM0",
}

def cb_cyclic(ev):
  print("watchdog::cyclic call back")
  t1=Float()
  t1.data=30
  pub_inner_t.publish(t1)
  rospy.set_param("/watchdog/inner_temp",t1.data)

########################################################
rospy.init_node("watchdog",anonymous=True)
###Load params
try:
  Config.update(rospy.get_param("/config/watchdog"))
except Exception as e:
  print("get_param exception:",e.args)

pub_inner_t=rospy.Publisher('/watchdog/inner_temp',Float,queue_size=1)

rospy.Timer(rospy.Duration(2),cb_cyclic)

while not rospy.is_shutdown():
  print("watchdog::main loop")
  rospy.sleep(1)

