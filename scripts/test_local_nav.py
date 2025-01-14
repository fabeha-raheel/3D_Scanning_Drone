#!/usr/bin/env python

import time
from math import pi
from LocalNavigationDrone import LocalNavigationDrone

drone = LocalNavigationDrone()
drone.execute_takeoff_sequence(takeoff_alt=2)

drone.start_local_circle_mission(center_angle=pi/2)
time.sleep(10)
drone.vehicle.set_mode(mode="LAND")