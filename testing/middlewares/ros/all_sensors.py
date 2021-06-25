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


        sensors = [ Accelerometer(),
        Airspeed(),
        Armature_pose(),
        Attitude(),
        Barometer(),
        Battery(),
        Camera(),
        Clock(),
        Collision(),
        Depthaggregator(),
        Depth_camera_aggregator(),
        Depth_camera(),
        Dvl(),
        EGM9615(),
        Fls(),
        GeomagnetismHeader(),
        GeomagnetismLibrary(),
        GPS(),
        Gyroscope(),
        Human_posture(),
        Imu(),
        Laserscanner(),
        LidarMoos(),
        Lidar(),
        Magnetometer(),
        Magnetometer(),
        MultiStaticSonarCtrl(),
        MultiStaticSonar(),
        MultStaticReciever(),
        Object(),
        ObjectServerMoos(),
        ObjectServer(),
        Odometry(),
        OptixCameraMoos(),
        OptixCamera(),
        Pose(),
        Proximity(),
        Ptu_posture(),
        Radar_altimeter(),
        Search_and_rescue(),
        Semantic_camera(),
        Sonar(),
        Stereo_unit(),
        Thermometer(),
        Velocity(),
        Video_camera(),
        WMM(),
        Zbufferto3d(),
        Zbuffertodepth() ]

        gps = GPS()
        gps.add_stream('ros')
        robot.append(gps)

        odometry = Odometry()
        odometry.add_stream('ros')
        robot.append(odometry)




        env = Environment('empty', fastmode = True)
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
