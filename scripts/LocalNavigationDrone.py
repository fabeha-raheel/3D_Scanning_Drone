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

    def start_local_circle_mission(self, altitude=5, radius=5, n_waypoints=10, center_angle=0):

        initial_position = (self.vehicle.data.local_position.x, self.vehicle.data.local_position.y)
        self.waypoints, circle_center = self.generate_circular_traj_waypoints(radius, n_waypoints, initial_position, center_angle)
        print("Waypoints:")
        print(self.waypoints)

        for waypoint in self.waypoints:
            self.vehicle.goto_position_local(x=waypoint[0], y=waypoint[1], z=altitude)
            time.sleep(5)

        print("Loop completed.")

        # self.vehicle.set_mode(mode="LAND")


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

    def generate_circular_traj_waypoints(self, radius, n_waypoints, initial_position, center_angle=0):
        """
        Generates waypoints for a circle relative to the drone's initial position,
        ensuring smooth transition from the initial position and forming a closed loop.
        :param r: Radius of the circle.
        :param n: Number of waypoints (excluding the repeated last point).
        :param initial_position: Drone's initial position (x_0, y_0).
        :param center_angle: Angle of the circle center relative to the drone's initial position (in radians).
        :return: Array of waypoints in the local frame.
        """
        # Unpack initial position
        x_0, y_0 = initial_position

        # Compute circle center
        x_c = x_0 + radius * np.cos(center_angle)
        y_c = y_0 + radius * np.sin(center_angle)
        
        # Compute the angle of the initial position relative to the circle center
        theta_0 = np.arctan2(y_0 - y_c, x_0 - x_c)
        
        # Generate angles for waypoints starting from theta_0
        angles = np.linspace(theta_0, theta_0 + 2 * np.pi, n_waypoints, endpoint=False)
        
        # Compute waypoints
        x_waypoints = x_c + radius * np.cos(angles)
        y_waypoints = y_c + radius * np.sin(angles)
        
        # Combine into waypoints
        waypoints = np.column_stack((x_waypoints, y_waypoints))
        
        # Ensure first waypoint matches initial position exactly
        waypoints[0] = [x_0, y_0]
        
        # Append the initial position as the last waypoint to form a closed loop
        waypoints = np.vstack([waypoints, [x_0, y_0]])
        
        return waypoints, (x_c, y_c)

    def plot_waypoints(waypoints, circle_center, initial_position):
        """
        Plots the waypoints and the circular path using Matplotlib.
        :param waypoints: Array of waypoints (x, y).
        :param circle_center: Tuple (x_c, y_c) for the circle center.
        :param initial_position: Initial position of the drone (x_0, y_0).
        """
        # Unpack data
        x_waypoints, y_waypoints = waypoints[:, 0], waypoints[:, 1]
        x_c, y_c = circle_center
        x_0, y_0 = initial_position
        
        # Plot circle and waypoints
        plt.figure(figsize=(8, 8))
        plt.plot(x_waypoints, y_waypoints, 'o-', label='Waypoints (Closed Loop)')  # Waypoints
        plt.scatter(x_0, y_0, c='red', label='Initial Position', zorder=5)  # Initial position
        plt.scatter(x_c, y_c, c='blue', label='Circle Center', zorder=5)  # Circle center
        plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio for proper circles
        
        # Add labels and legend
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')
        plt.title('Drone Circular Waypoints (Closed Loop)')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":

    # Example Usage
    drone = LocalNavigationDrone()
    drone.execute_takeoff_sequence(takeoff_alt=2)

    drone.start_local_circle_mission()
    time.sleep(10)
    drone.vehicle.set_mode(mode="LAND")
    # rospy.spin()