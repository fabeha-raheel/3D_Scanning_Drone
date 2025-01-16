import geopandas as gpd
from shapely.geometry import Point
import matplotlib.pyplot as plt
import numpy as np

def ned_to_gps(waypoints_ned, home_lat, home_lon):
    """
    Converts waypoints in the NED frame to GPS coordinates (latitude, longitude).
    
    :param waypoints_ned: Array of waypoints in the NED frame (x, y).
    :param home_lat: Latitude of the initial home position (degrees).
    :param home_lon: Longitude of the initial home position (degrees).
    :return: Array of GPS coordinates (latitude, longitude).
    """
    # Earth's radius (WGS84)
    R_earth = 6378137.0  # meters

    # Convert home_lat to radians for trigonometric calculations
    home_lat_rad = np.radians(home_lat)

    # Initialize list for GPS waypoints
    waypoints_gps = []

    # Convert each waypoint
    for waypoint in waypoints_ned:
        x, y = waypoint  # NED coordinates (North, East)
        
        # Latitude calculation
        delta_lat = (x / R_earth) * (180 / np.pi)
        lat = home_lat + delta_lat

        # Longitude calculation
        delta_lon = (y / (R_earth * np.cos(home_lat_rad))) * (180 / np.pi)
        lon = home_lon + delta_lon

        # Append to GPS waypoints
        waypoints_gps.append((lat, lon))

    return np.array(waypoints_gps)

def visualize_gps_waypoints(waypoints_gps, home_position):
    """
    Visualizes GPS waypoints using GeoPandas.
    
    :param waypoints_gps: Array of GPS waypoints (latitude, longitude).
    :param home_position: Tuple (latitude, longitude) of the home position.
    """
    # Create GeoDataFrame
    geometry = [Point(lon, lat) for lat, lon in waypoints_gps]
    gdf = gpd.GeoDataFrame(geometry=geometry)
    
    # Create home position point
    home_point = Point(home_position[1], home_position[0])
    
    # Plot waypoints
    ax = gdf.plot(marker='o', color='blue', label='Waypoints', figsize=(8, 8))
    gpd.GeoSeries([home_point]).plot(ax=ax, marker='x', color='red', label='Home Position', zorder=5)

    # Annotate waypoints
    for i, point in enumerate(geometry):
        plt.text(point.x, point.y, f"{i + 1}", fontsize=8, ha='right')

    # Add labels and legend
    plt.title("GPS Waypoints Visualization")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.legend()
    plt.grid(True)
    plt.show()

# Example Usage
# Waypoints in NED
waypoints_ned = np.array([
    [0, 0],  # Drone initial position
    [5, 0],  # North 5m
    [5, 5],  # Northeast 5m
    [0, 5],  # East 5m
    [0, 0]   # Back to initial position (closed loop)
])

# Home position in GPS (latitude, longitude)
home_lat = 37.7749  # San Francisco
home_lon = -122.4194

# Convert waypoints to GPS coordinates
waypoints_gps = ned_to_gps(waypoints_ned, home_lat, home_lon)

# Visualize GPS waypoints
visualize_gps_waypoints(waypoints_gps, (home_lat, home_lon))
