import logging; logger = logging.getLogger("morse." + __name__)
from std_msgs.msg import String
from morse.middleware.ros import ROSPublisher
import json 

class GenericTargetPublisher(ROSPublisher):
    """ Publish targets in the blender scene as a JSON list of items and positions. """
    ros_class = String
    default_frame_id = '/targets'

    def default(self, ci='unused'):
        message = String()
        try:
            message.data = json.dumps( self.data['targets'] )
        except:
            logger.error("Filed to parse json string for GenericTargetPublisher")
        self.publish(message)

