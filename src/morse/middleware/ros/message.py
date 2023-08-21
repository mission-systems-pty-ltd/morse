import logging; logger = logging.getLogger("morse." + __name__)
from std_msgs.msg import String
from morse.middleware.ros import ROSPublisher
import json 

class MessagePublisher(ROSPublisher):
    """ Publish a std string message. """
    ros_class = String
    default_frame_id = '/morse/info'

    def default(self, ci='unused'):
        message = String()
        if(self.data['message']):
            message.data = self.data['message']
            self.publish(message)
