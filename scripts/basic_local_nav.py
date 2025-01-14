import sys
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt

from geometry_msgs.msg import PoseStamped
from mavrosDrone import MAVROS_Drone

class LocalNavigationDrone:

    def __init__(self):
        
        self.vehicle = MAVROS_Drone()

        self.vehicle.init_node()
        self.vehicle.init_subscribers()
        self.vehicle.init_publishers()

        self.min_takeoff_alt = 2

        self.waypoints = None

    def execute_takeoff_sequence(self, takeoff_alt=None):

        rospy.loginfo("Waiting for Guided")
        response = self.vehicle.set_mode(mode='GUIDED')
    
        if response.mode_sent:
            rospy.loginfo("Drone has been switched to GUIDED.")
        else:
            rospy.loginfo("Drone cannot be switched to GUIDED.")
        
        # self.vehicle.set_stream_rate()
        # rospy.loginfo("Mode is now GUIDED.")

        # self.vehicle.wait_for_GPS_Fix()       # check if vehicle has GPS Fix
            
        rospy.loginfo("Arming Drone")
        response = self.vehicle.arm()
        if response.success:
            rospy.loginfo("Drone is now ARMED.")
        else:
            rospy.logerr("Drone is not Armable. Aborting Mission...")
            sys.exit(1)

        # takeoff
        if takeoff_alt is None: response = self.vehicle.takeoff(altitude=self.min_takeoff_alt)
        else: response = self.vehicle.takeoff(altitude=takeoff_alt)
        if response.success:
            rospy.loginfo("Drone is Taking off.")
        else:
            rospy.logerr("Drone failed to takeoff. Aborting Mission...")
            sys.exit(1)
        # Waiting for drone to complete takeoff
        while not self.vehicle.check_takeoff_complete():
            # Waiting for drone to complete takeoff
            time.sleep(0.1)

        rospy.loginfo("Takeoff sequence completed!")


if __name__ == "__main__":

    # Example Usage
    drone = LocalNavigationDrone()
    drone.execute_takeoff_sequence(takeoff_alt=2)

    drone.vehicle.goto_position_local(x=20, y=20, z=5)
    time.sleep(10)
    drone.vehicle.set_mode(mode="LAND")
    rospy.spin()
