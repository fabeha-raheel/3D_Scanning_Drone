#!/usr/bin/python3

import math
import rospy
import time

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.msg import GlobalPositionTarget, PositionTarget, ParamValue, State, RCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, ParamSet, StreamRate
from geometry_msgs.msg import PoseStamped

class Drone_Data():
    
    def __init__(self) -> None:
        self.header = Header()
        self.local_position = LocalPosition()
        self.global_position = GlobalPosition()
        self.euler_orientation = EulerOrientation()
        self.linear_velocity = LinearVelocity()
        self.angular_velocity = AngularVelocity()
        self.linear_acceleration = LinearAcceleration()
        self.rc = RadioChannels()
        

class Header:
    def __init__(self) -> None:
        self.name = 'no-name'
        self.id = 1
        self.mode = 'NONE'

class LocalPosition:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z

class GlobalPosition:
    def __init__(self, fix=-1, latitude=0, longitude=0, altitude=0):
        self.gps_fix = fix
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

class EulerOrientation:
    def __init__(self, roll=0, pitch=0, yaw=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class LinearAcceleration:
    def __init__(self, ax=0, ay=0, az=0):
        self.ax = ax
        self.ay = ay
        self.az = az

class AngularVelocity:
    def __init__(self, wx=0, wy=0, wz=0):
        self.wx = wx
        self.wy = wy
        self.wz = wz

class LinearVelocity:
    def __init__(self, vx=0, vy=0, vz=0):
        self.vx = vx
        self.vy = vy
        self.vz = vz

class RadioChannels:
    def __init__(self):
        self.rc = []


class MAVROS_Drone():
    def __init__(self, ns=None) -> None:

        self.ns = ns
        self.data = Drone_Data()
        
    def init_node(self):
        rospy.init_node("MAVROS_Drone_Node")           
        
    def init_subscribers(self):
        if self.ns is not None:
            self.global_position_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            self.rel_alt_subscriber = rospy.Subscriber(self.ns + '/mavros/global_position/rel_alt',Float64, self.rel_alt_sub_cb)    
            self.state_subscriber = rospy.Subscriber(self.ns + '/mavros/state',State, self.state_sub_cb)
            self.rc_subscriber = rospy.Subscriber(self.ns + '/mavros/rc/in', RCIn, self.rc_sub_cb)        
        else:
            self.global_position_subscriber = rospy.Subscriber('/mavros/global_position/global',NavSatFix, self.global_sub_cb)
            self.local_position_subscriber = rospy.Subscriber('/mavros/global_position/local',Odometry, self.local_sub_cb)
            self.compass_hdg_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg',Float64, self.hdg_sub_cb)
            self.rel_alt_subscriber = rospy.Subscriber('/mavros/global_position/rel_alt',Float64, self.rel_alt_sub_cb) 
            self.state_subscriber = rospy.Subscriber('/mavros/state',State, self.state_sub_cb) 
            self.rc_subscriber = rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_sub_cb) 
            
    def init_publishers(self):
        if self.ns is not None:
            self.global_setpoint_publisher = rospy.Publisher(self.ns + '/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
            self.local_setpoint_publisher = rospy.Publisher(self.ns + '/mavros/setpoint_position/local',PoseStamped, queue_size=1)
        else:
            self.global_setpoint_publisher = rospy.Publisher('/mavros/setpoint_raw/global',GlobalPositionTarget, queue_size=1)
            self.local_setpoint_publisher = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped, queue_size=1)
            
    def check_GPS_fix(self):
        if self.data.global_position.gps_fix >= 0:
            return True
        else:
            return False
        
    def wait_for_GPS_Fix(self):
        while True:
            if self.data.global_position.gps_fix == 0:
                print("GPS Fix acquired!")
                break
    
    def arm(self):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armResponse = armService(True)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse

    def disarm(self):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy(self.ns + '/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=3)
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
                armResponse = armService(False)
                rospy.loginfo(armResponse)
            except rospy.ServiceException as e:
                print("Service call failed: %s" %e)
        
        return armResponse
            
            
    def set_mode(self, mode):
        if self.ns is not None:
            rospy.wait_for_service(self.ns + '/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy(self.ns + '/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        else:
            rospy.wait_for_service('/mavros/set_mode', timeout=3)
            try:
                modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                modeResponse = modeService(custom_mode=mode)
                rospy.loginfo(modeResponse)
            except rospy.ServiceException as e:
                print("Set mode failed: %s" %e)
        
        return modeResponse
            
    def takeoff(self, altitude=5, latitude=0, longitude=0, min_pitch=0, yaw=0):
        if self.ns is None:
            rospy.wait_for_service('/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/mavros/cmd/takeoff', timeout=3)
            try:
                takeoffService = rospy.ServiceProxy(self.ns + '/mavros/cmd/takeoff', CommandTOL)
                takeoffResponse = takeoffService(altitude=altitude, latitude=latitude, longitude=longitude, min_pitch=min_pitch, yaw=yaw)
                rospy.loginfo(takeoffResponse)
            except rospy.ServiceException as e:
                print("Takeoff failed: %s" %e)
            
        if takeoffResponse:
            self.takeoff_altitude = altitude
        else:
            self.takeoff_altitude = 0
            
        return takeoffResponse
    
    def set_stream_rate(self):
        if self.ns is None:
            rospy.wait_for_service('/mavros/set_stream_rate', timeout=3)
            try:
                streamService = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
                streamService(0, 10, 1)
            except rospy.ServiceException as e:
                print("Setting Stream Rate failed: %s" %e)
        else:
            rospy.wait_for_service(self.ns + '/mavros/set_stream_rate', timeout=3)
            try:
                streamService = rospy.ServiceProxy(self.ns + '/mavros/set_stream_rate', StreamRate)
                streamService(0, 10, 1)
            except rospy.ServiceException as e:
                print("Setting Stream Rate failed: %s" %e)

    def wait_for_guided(self):
        while self.data.header.mode != 'GUIDED':
            time.sleep(0.5)
    
    def goto_position_global(self, latitude, longitude, altitude, type_mask=4088, coordinate_frame=6):
        
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.type_mask=type_mask
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)

    def goto_position_local(self, x, y, z, type_mask=4088, coordinate_frame=8):
        
        command = PoseStamped()
        command.pose.position.x = x
        command.pose.position.y = y
        command.pose.position.z = z
        
        self.local_setpoint_publisher.publish(command)
        
    def goto_location_heading(self, latitude, longitude, altitude, yaw, type_mask=4088, coordinate_frame=6):
        
        command = GlobalPositionTarget()
        command.altitude = altitude
        command.latitude = latitude
        command.longitude = longitude
        command.yaw = yaw
        command.type_mask=GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ | GlobalPositionTarget.IGNORE_YAW_RATE
        command.coordinate_frame=coordinate_frame
        
        self.global_setpoint_publisher.publish(command)
    
    def set_param(self, param_name, param_value):
        if self.ns is None:
            rospy.wait_for_service('/mavros/param/set', timeout=3)
            try:
                param_set = rospy.ServiceProxy('/mavros/param/set', ParamSet)
                
                value = ParamValue()
                value.integer = 0
                value.real = float(param_value)
                response = param_set(param_name, value)
                rospy.loginfo("{0} parameter set to {1}".format(param_name, param_value))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
        
        else:
            rospy.wait_for_service(self.ns + '/mavros/param/set', timeout=3)
            try:
                param_set = rospy.ServiceProxy(self.ns + '/mavros/param/set', ParamSet)
                value = ParamValue()
                value.integer = 0
                value.real = float(param_value)
                response = param_set(param_name, value)
                rospy.loginfo("{0} parameter set to {1}".format(param_name, param_value))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                
        return response
    
    def check_takeoff_complete(self):
        if abs(self.takeoff_altitude - self.data.local_position.z) <= 0.5:
            return True
        else:
            return False
        
    def check_land_complete(self):
        if self.data.local_position.z <= 0.5:
            return True
        else:
            return False
        
    def check_target_location_reached(self, location):
        pass
                 
    def global_sub_cb(self, mssg):
        self.data.global_position.gps_fix = mssg.status.status
        self.data.global_position.latitude = mssg.latitude
        self.data.global_position.longitude = mssg.longitude
        
    def local_sub_cb(self, mssg):
        self.data.local_position.x = mssg.pose.pose.position.x
        self.data.local_position.y = mssg.pose.pose.position.y
        self.data.local_position.z = mssg.pose.pose.position.z
        
    def hdg_sub_cb(self,mssg):
        self.data.euler_orientation.yaw = mssg.data

    def rel_alt_sub_cb(self, mssg):
        self.data.global_position.altitude = mssg.data
        
    def state_sub_cb(self, mssg):
        self.data.header.mode = mssg.mode

    def rc_sub_cb(self, mssg):
        self.data.rc = mssg.channels
        
    def offset_location(self, latitude, longitude, dNorth, dEast):
        earth_radius = 6378137.0 #Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*latitude/180))
        #New position in decimal degrees
        newlat = latitude + (dLat * 180/math.pi)
        newlon = longitude + (dLon * 180/math.pi)
        targetlocation = (newlat, newlon)
            
        return targetlocation
    
    def drone_to_ned_conversion(self, dNorth, dEast, heading):
        if heading >= 270:
            hdg = heading - 270
            North = dNorth*math.sin(math.radians(hdg)) + dEast*math.cos(math.radians(hdg))
            East = -1*dNorth*math.cos(math.radians(hdg)) + dEast*math.sin(math.radians(hdg))
        elif heading <= 90:
            North = dNorth*math.cos(math.radians(heading)) - dEast*math.sin(math.radians(heading))
            East = dNorth*math.sin(math.radians(heading)) + dEast*math.cos(math.radians(heading))
        elif heading<=180 and heading>90:
            hdg = heading - 90
            North = -1*dNorth*math.sin(math.radians(hdg)) - dEast*math.cos(math.radians(hdg))
            East = dNorth*math.cos(math.radians(hdg)) - dEast*math.sin(math.radians(hdg))
        else:
            hdg = heading - 180
            North = -1*dNorth*math.cos(math.radians(hdg)) + dEast*math.sin(math.radians(hdg))
            East = -1*dNorth*math.sin(math.radians(hdg)) - dEast*math.cos(math.radians(hdg))
        return (North, East)
    
    def ned_to_drone_conversion(self, dNorth, dEast, heading):
        North = dNorth*math.cos(math.radians(heading)) - dEast*math.sin(math.radians(heading))
        East = dNorth*math.sin(math.radians(heading)) + dEast*math.cos(math.radians(heading))
        return (North, East)