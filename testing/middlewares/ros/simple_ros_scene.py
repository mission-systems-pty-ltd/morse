#!/usr/bin/python3
import sys
import math
from morse.testing.testing import MorseTestCase, testlogger
from time import sleep
from morse.builder import *

robot = ATRV()

sensors = [ Accelerometer, 
    # Airspeed,              # NO ROS DRIVER
    ArmaturePose,
    # Attitude,              # NO ROS DRIVER
    # Barometer,             # NO ROS DRIVER
    Battery, 
    # Camera,                # BROKEN
    Clock,       
    # Collision,             # NO ROS DRIVER
    # DepthCameraAggregator, # BROKEN
    # DepthCamera,           # BROKEN
    # DVL,                   # NO ROS DRIVER
    # FLS,                   # BROKEN / needs an FLS beam in the scene
    GPS, 
    Gyroscope,
    IMU,   
    # Lidar,                 # NO ROS DRIVER
    Magnetometer,
    # MultiStaticSonar,
    Odometry,
    # # OptixCamera,
    Pose,        
    Proximity, 
    # PTUPosture,            # BROKEN / Needs to be tested on a unit with a pan tilt unit.
    # RadarAltimeter,        # NO ROS DRIVER
    # SearchAndRescue,       # NO ROS DRIVER
    # SemanticCamera,        # BROKEN
    # Sonar,                 # NO ROS DRIVER
    # StereoUnit,            # NO ROS DRIVER
    # Thermometer,           # NO ROS DRIVER
    Velocity] 
    # VideoCamera]           # BROKEN

for sensor_type in sensors: 
    sensor = None
    if sensor_type == Camera:
        sensor = sensor_type("cam")
    else:
        sensor = sensor_type()

    topic = sensor.__class__.__name__
    sensor.add_stream('ros', topic=topic)
    robot.append(sensor)

env = Environment('sandbox', fastmode = True)
env.properties(latitude = -35.0637135, longitude = 150.735166, altitude = 0)
env.add_service('socket')

