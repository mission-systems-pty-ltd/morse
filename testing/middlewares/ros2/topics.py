#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import math
# from morse.testing.ros import RosTestCase
from morse.testing.testing import MorseTestCase, testlogger

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec
import nav_msgs.msg # do not conflict with morse builder
from geometry_msgs.msg import Twist
from time import sleep
from rclpy.wait_for_message import wait_for_message

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, v, w, t):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    s.publish(msg)
    sleep(t)
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    s.publish(msg)

# class Subscriber(Node):
    
#     def __init__(self):
#         super().__init__('morse_ros_data_stream_test')
#         self.subscription = self.create_subscription(nav_msgs.msg.Odometry, '/robot/odometry', self.pose_callback, 10)
#         self.subscription # prevent unused variable warning

#     def pose_callback(self, data):
#         self.pos = data

def wait_for_message():
    """
    Wait for the next incoming message.
    :param msg_type: message type
    :param node: node to initialize the subscription on
    :param topic: topic name to wait for message
    :time_to_wait: seconds to wait before returning
    :return (True, msg) if a message was successfully received, (False, ()) if message
        could not be obtained or shutdown was triggered asynchronously on the context.
    """
    context = node.context
    wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
    wait_set.clear_entities()

    sub = node.create_subscription(msg_type, topic, lambda _: None, 1)
    wait_set.add_subscription(sub.handle)
    sigint_gc = SignalHandlerGuardCondition(context=context)
    wait_set.add_guard_condition(sigint_gc.handle)

    timeout_nsec = timeout_sec_to_nsec(time_to_wait)
    wait_set.wait(timeout_nsec)

    subs_ready = wait_set.get_ready_entities('subscription')
    guards_ready = wait_set.get_ready_entities('guard_condition')

    if guards_ready:
        if sigint_gc.handle.pointer in guards_ready:
            return (False, None)

    if subs_ready:
        if sub.handle.pointer in subs_ready:
            msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
            return (True, msg_info[0])

    return (False, None)


class DataStreamTest(MorseTestCase):

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

    def pose_callback(self, data):
        self.pos = data

    def test_vw_controller(self):
        # XXX
        # test the orientation part, but the easy way is to use numpy or
        # tf, and don't want to add too much dependency for test
        
        # rclpy.init() # initialize the nodecallback_group
        # node = rclpy.create_node('morse_ros_data_stream_test') # create a node
        # subscription = node.create_subscription(nav_msgs.msg.Odometry, '/robot/odometry', self.pose_callback, 10)

        # rclpy.create_subscription(nav_msgs.msg.Odometry, '/robot/odometry', self.pose_callback)
        # rclpy.spin_until_future_complete(node, subscription, timeout=10)


        rclpy.init() # can have args=sys.argv as the parameter of init()
        node = rclpy.create_node('morse_ros_data_stream_test')
        subscriber = node.create_subscription(nav_msgs.msg.Odometry, '/robot/odometry', self.pose_callback)

        msg = wait_for_message('/robot/odometry', nav_msgs.msg.Odometry, timeout = 10)
        self.assertIsNotNone(msg)

        cmd_stream = node.create_publisher('/robot/motion', Twist)
        '''
        rospy.init_node('morse_ros_data_stream_test')
        rospy.Subscriber('/robot/odometry', nav_msgs.msg.Odometry, self.pose_callback)

        msg = rospy.client.wait_for_message('/robot/odometry', nav_msgs.msg.Odometry, timeout = 10)
        self.assertIsNotNone(msg)
        '''

        # msg = rclpy.client.wait_for_message('/robot/odometry', nav_msgs.msg.Odometry, timeout = 10)
        # self.assertIsNotNone(msg)

        # cmd_stream = rclpy.Publisher('/robot/motion', Twist)
       
        self.assertTrue(hasattr(self, "pos"))
        precision=0.15

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)
 
        # sleep to make sure that the other peer can read it ...
        sleep(1)

        send_speed(cmd_stream, 1.0, 0.0, 2.0)
        sleep(1)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 2.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        send_speed(cmd_stream, -1.0, 0.0, 2.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        # sleep to make sure that the other peer can read it ...
        sleep(1)

        send_speed(cmd_stream, 1.0, -math.pi/4.0, 2.0)
        sleep(1)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, -4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        # node.destroy_node()
        # rclpy.shutdown()



# def main(args=None):
#     rclpy.init(args=args)
#     subscriber = DataStreamTest()

#     rclpy.spin(subscriber)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DataStreamTest, time_modes = [TimeStrategies.BestEffort])
