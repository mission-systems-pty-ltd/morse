""" ROS-based MorseTestCase (unit-tests)

The main feature is to launch ROS core at start.
"""

from morse.testing.testing import MorseTestCase, testlogger

import os
import sys
import subprocess


try:
    import rclpy
    print("Running ROS2!")
except ImportError as error:
    logger.error("Could not find ROS2. Defaulting to ROS1")
    try:
        import roslib
    except ImportError as error:
        logger.error("Could not find ROS. source setup.[ba]sh ?")
        raise error



class RosTestCase(MorseTestCase):
    def using_ros2(self):
        try:
            import rclpy
            return True
        except:
            return False

    def setUpMw(self):
        if self.using_ros2():
            return # Do nothing, ros2 does not need a roscore

        try:
            self.roscore_process = subprocess.Popen(['roscore'])
        except OSError as ose:
            testlogger.error("Error while launching roscore ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        if self.using_ros2():
            return # Do nothing, ros2 does not need a roscore
        self.roscore_process.terminate()
