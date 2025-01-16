#!/usr/bin/env python

import time
import rospy
from LocalNavigationDrone import LocalNavigationDrone


# Example Usage
drone = LocalNavigationDrone()
drone.execute_takeoff_sequence(takeoff_alt=3)
time.sleep(5)

# drone.vehicle.goto_position_local(x=0, y=round(drone.vehicle.data.local_position.y), z=3)
drone.vehicle.goto_position_local(x=2, y=0, z=3)
time.sleep(10)


drone.vehicle.set_mode(mode="LAND")
# rospy.spin()
