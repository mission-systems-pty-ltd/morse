#! /usr/bin/python3
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math

from morse.testing.testing import MorseTestCase, testlogger
from time import sleep


# import std_msgs
# import nav_msgs.msg # do not conflict with morse builder
# from geometry_msgs.msg import Twist
# from time import sleep

# # Include this import to be able to use your test file as a regular 
# # builder script, ie, usable with: 'morse [run|exec] base_testing.py

try:
    from morse.builder import *
except ImportError:
    pass

class DataStreamTest(MorseTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = ATRV()


        # sensors = [ GPS, Odometry ]
        
        sensors = [ Accelerometer,
            # Airspeed,              # NO ROS DRIVER
            ArmaturePose,
            # Attitude,              # NO ROS DRIVER
            # Barometer,             # NO ROS DRIVER
            Battery, 
            # Camera,                # BROKEN
            Clock,                 # BROKEN
            # Collision,             # NO ROS DRIVER
            # DepthCameraAggregator, # BROKEN
            # DepthCamera,           # BROKEN
            # DVL,                   # NO ROS DRIVER
            # FLS,                   # BROKEN
            GPS, 
            # Gyroscope,             # BROKEN
            IMU,            # BROKEN
            # Lidar,                 # NO ROS DRIVER
            # Magnetometer,          # BROKEN
            # MultiStaticSonar,
            Odometry,
            # # OptixCamera,
            # Pose,                  # BROKEN
            # Proximity,             # BROKEN
            # PTUPosture,            # BROKEN
            # RadarAltimeter,        # NO ROS DRIVER
            # SearchAndRescue,       # NO ROS DRIVER
            # SemanticCamera,        # BROKEN
            # Sonar,                 # NO ROS DRIVER
            # StereoUnit,            # NO ROS DRIVER
            # Thermometer,           # NO ROS DRIVER
            Velocity] 
            # VideoCamera]           # BROKEN

        for sensor_type in sensors: 
            print(sensor_type)
            sensor = sensor_type()
            topic = sensor.__class__.__name__
            sensor.add_stream('ros', topic=topic)
            robot.append(sensor)

        env = Environment('empty', fastmode = True)
        env.properties(latitude = -35.0637135, longitude = 150.735166, altitude = 0)
        env.add_service('socket')


    def pose_callback(self, data):
        self.pos = data

    def test_init(self):
        print("Initialized")

        while True:
            sleep(1)


        return True
        

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DataStreamTest, time_modes = [TimeStrategies.BestEffort])
