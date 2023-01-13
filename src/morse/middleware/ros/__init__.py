
try:
    from .abstract_ros2 import *
    print("Importing ROS 2")
except:
    from .abstract_ros1 import *
    print("Importing ROS 1")
