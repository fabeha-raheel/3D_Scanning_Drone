import sys
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped
from LocalNavigationDrone import LocalNavigationDrone


# Example Usage
drone = LocalNavigationDrone()
drone.execute_takeoff_sequence(takeoff_alt=2)

drone.vehicle.goto_position_local(x=5, y=0, z=5)
time.sleep(10)

drone.vehicle.set_mode(mode="LAND")
rospy.spin()
