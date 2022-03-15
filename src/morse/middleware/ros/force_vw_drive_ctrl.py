# import logging; logger = logging.getLogger("morse." + __name__)

# from morse.middleware.ros import ROSSubscriber
# from morse.middleware.ros import ROSPublisher
# from geometry_msgs.msg import TwistStamped

# class CrawlerSubscriber(ROSSubscriber):
#     """ Subscribe to a motion command and set ``v`` and ``w`` local data. """
#     ros_class = TwistStamped
#     topic_name = "/crawler/cmd_vel"

#     def initialize(self):
#         ROSSubscriber.initialize(self)
#         print("Creating ROS CrawlerSubscriber - note: you should be running ROS2.")

#     def update(self, message):
#         print("Subscribing")
#         v_vel = message.twist.linear.x
#         w_vel = message.twist.angular.z
#         v_vel = self.constrain(v_vel, -1, 1)
#         w_vel = self.constrain(w_vel, -1, 1.1)
#         self.data["v"] = v_vel
#         self.data["w"] = w_vel

#     def constrain(self, val, _min, _max):
#         return min(max(val, _min), _max)

# class CrawlerTrackPublisher(ROSPublisher):
#     """ Subscribe to a motion command and set ``v`` and ``w`` local data. """
#     ros_class = TwistStamped
#     topic_name = "/crawler/cmd_vel"

#     def initialize(self, ci='unused'):
#         ROSPublisher.initialize(self)
#         self.child_frame_id = self.kwargs.get("child_frame_id", "/base_footprint")
#         print("Creating ROS CrawlerSubscriber - note: you should be running ROS2.")

#     def default(self, ci='unused'):
#         msg = TwistStamped()
#         msg.header = self.get_ros_header()
#         msg.twist.linear.x = self.data['v']
#         msg.twist.linear.y = 0.0
#         msg.twist.linear.z = 0.0
#         msg.twist.angular.x = 0.0
#         msg.twist.angular.y = 0.0
#         msg.twist.angular.z = self.data['w']
#         self.publish(msg)


# class CrawlerTrackSubscriber(ROSSubscriber):
#     """ Subscribe to a motion command and set ``v`` and ``w`` local data. """
#     ros_class = TwistStamped
#     topic_name = "/crawler/cmd_vel"

#     def initialize(self):
#         ROSSubscriber.initialize(self)
#         print("Creating ROS CrawlerSubscriber - note: you should be running ROS2.")

#     def update(self, message):
#         print("Subscribing")
