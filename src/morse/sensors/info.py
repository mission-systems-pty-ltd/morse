import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data, add_property

class Info(morse.core.sensor.Sensor):
    """
        Holds a string message to republish. 
        Useful as a generic string message publisher. You could fill the data statically with 
        some form of JSON structure etc. 
        
        Example usage:
        Use the String Info sensor to Publish stub locations of targets from the environment. 
            static_msg = Info()
            static_msg.properties(init_message="{"targets": ["target1": [1,1,1], "target2": [1,2,3]]}")
            robot.append(static_msg) 
    """

    _name = "Info"

    add_property('init_message', [], "init_message", "string", "")
    add_data('message', [], "string", "")

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        # set targets local data with init targets
        self.local_data['message'] = self.init_message

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)

    def default_action(self):
        """ 
            There is no need to do anything with a static message. 
        """
        pass
            