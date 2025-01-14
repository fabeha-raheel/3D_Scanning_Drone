import numpy as np
import matplotlib.pyplot as plt

def generate_circle_waypoints_closed_loop(r, n, initial_position, center_angle=0):
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
    x_c = x_0 + r * np.cos(center_angle)
    y_c = y_0 + r * np.sin(center_angle)
    
    # Compute the angle of the initial position relative to the circle center
    theta_0 = np.arctan2(y_0 - y_c, x_0 - x_c)
    
    # Generate angles for waypoints starting from theta_0
    angles = np.linspace(theta_0, theta_0 + 2 * np.pi, n, endpoint=False)
    
    # Compute waypoints
    x_waypoints = x_c + r * np.cos(angles)
    y_waypoints = y_c + r * np.sin(angles)
    
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

# Example Usage
radius = 5  # Radius of the circle
num_waypoints = 10  # Number of waypoints (excluding repeated initial position)
initial_position = (0, 0)  # Drone's initial home position
# center_angle = np.pi / 4  # Center of circle at 45 degrees from the initial position
center_angle = 0

# Generate waypoints and circle center
waypoints, circle_center = generate_circle_waypoints_closed_loop(radius, num_waypoints, initial_position, center_angle)

# Plot waypoints
plot_waypoints(waypoints, circle_center, initial_position)
