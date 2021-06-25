
try:
    from .abstract_ros2 import *
except:
    print("ROS2 IMPORT FAILED_______________________")
    from .abstract_ros1 import *
