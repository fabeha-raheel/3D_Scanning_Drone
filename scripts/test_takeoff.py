#!/usr/bin/env python

import time
import rospy
from LocalNavigationDrone import LocalNavigationDrone


# Example Usage
drone = LocalNavigationDrone()
drone.execute_takeoff_sequence(takeoff_alt=3)


while not rospy.is_shutdown():
    print("x: {0}, y: {1}, z:{2}".format(round(drone.vehicle.data.local_position.x), 
                                         round(drone.vehicle.data.local_position.y), 
                                         round(drone.vehicle.data.local_position.z)))

drone.vehicle.set_mode(mode="LAND")
rospy.spin()
