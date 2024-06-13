#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import math
# from morse.testing.ros import RosTestCase
from morse.testing.testing import MorseTestCase, testlogger
import time 

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec
import nav_msgs.msg # do not conflict with morse builder
from geometry_msgs.msg import Twist
from time import sleep

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

# helper function to publish velocity commands to the robot
def send_speed(s, v, w, t):
    msg = Twist()
    msg.linear.x = v # v is the linear velocity
    msg.angular.z = w # w is the angular velocity
    s.publish(msg)
    sleep(t)
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    s.publish(msg)

# Subscriber() class to subscribe to the /robot/odometry topic to receive odometry messages
class Subscriber(Node): 
    
    def __init__(self):
        super().__init__('morse_ros_data_stream_test')
        self.subscription = self.create_subscription(nav_msgs.msg.Odometry, '/robot/odometry', self.pose_callback, 10)
        self.subscription # prevent unused variable warning
        self.pos = None

    def pose_callback(self, data):
        self.pos = data

# Publisher() class to publish velocity commands to the /robot/motion topic
class Publisher(Node):
    
    def __init__(self):
        super().__init__('morse_ros_data_stream_test_publisher')
        self.publisher = self.create_publisher(Twist, '/robot/motion', 10)

    def publish(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.linear.z = w
        self.publisher.publish(msg)

class DataStreamTest(MorseTestCase):

    # Define the test environment and the test case
    # --- Set up the robot and its sensors and actuators
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()

        odometry = Odometry()
        robot.append(odometry)
        odometry.add_stream('ros')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('ros')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_vw_controller(self):
        # Initialise the ROS2 interface with 'rclpy'
        rclpy.init()
        
        sub = Subscriber()
        pub = Publisher()

        # wait for the 1st message to be received
        while sub.pos is None:
            rclpy.spin_once(sub)
        
        precision = 0.15
        # perform same unit tests as 'ros1.py':
        self.assertAlmostEqual(sub.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.z, 0.0, delta=precision)

        # Wait for motion to complete
        sleep(1)

        # Drive forward
        send_speed(pub.publisher, 1.0, 0.0, 2.0)

        # Wait for motion to complete - sleep to make sure that the other peer can read it ...
        sleep(1)

        # Check new position
        rclpy.spin_once(sub)

        self.assertAlmostEqual(sub.pos.pose.pose.position.x, 2.0, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.z, 0.0, delta=precision)

        # Wait for motion to complete
        sleep(1)

        # Drive backward
        send_speed(pub.publisher, -1.0, 0.0, 2.0) # go back to where it was initially landed

        # Wait for motion to complete
        sleep(1)

        # Check new position
        rclpy.spin_once(sub)

        # all x,y,z values should be 0.0 as it was initially
        self.assertAlmostEqual(sub.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.z, 0.0, delta=precision)

        # Wait for motion to complete
        sleep(1)

        # Drive forward and rotate -math.pi/4.0
        send_speed(pub.publisher, 1.0, -math.pi/4.0, 2.0)

        # Wait for motion to complete
        sleep(1)

        # Check new position
        rclpy.spin_once(sub)

        self.assertAlmostEqual(sub.pos.pose.pose.position.x, 4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.y, -4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(sub.pos.pose.pose.position.z, 0.0, delta=precision)

        # Close ROS2 interface
        rclpy.shutdown()

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DataStreamTest, time_modes = [TimeStrategies.BestEffort])
