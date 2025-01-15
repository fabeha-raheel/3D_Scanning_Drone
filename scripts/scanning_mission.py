#!/usr/bin/env python

import time
from math import pi
from LocalNavigationDrone import LocalNavigationDrone

drone = LocalNavigationDrone()
drone.execute_takeoff_sequence(takeoff_alt=2)

drone.start_local_circle_mission(altitude=5, radius=5, n_waypoints=10, center_angle=pi/2)
time.sleep(2)

drone.start_local_circle_mission(altitude=7, radius=5, n_waypoints=10, center_angle=pi/2)
time.sleep(2)

drone.start_local_circle_mission(altitude=10, radius=5, n_waypoints=10, center_angle=pi/2)
time.sleep(2)

drone.vehicle.set_mode(mode="LAND")