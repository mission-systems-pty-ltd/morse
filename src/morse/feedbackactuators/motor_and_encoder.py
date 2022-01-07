import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.feedbackactuator
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *
import numpy as np 

class MotorAndEncoder(morse.core.feedbackactuator.FeedbackActuator):
    """
    This is not implemented properly. It is an example class to be used as a reference.
    """
    _name = "MotorAndEncoder"
    _short_desc = "MotorAndEncoder test actuator that can be subscribed to."

    add_data('set_speed', 0.0, "float", 'Current motor commanded speed')
    add_data('measured_speed', 0.0, "float", 'Current motor speed')
    add_property('noise_amplitude', 3.0, 'Noise', 'float', 'Motor measurement noise')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.feedbackactuator.FeedbackActuator.__init__(self, obj, parent)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        print("Running Motor and encoder feedback actuator.")
        self.local_data['set_speed'] = self.local_data['set_speed'] + 1.0
        self.local_data['measured_speed'] = self.local_data['set_speed'] + np.random.rand()*self.noise_amplitude-self.noise_amplitude/2

        # Game loop frequency
        delta_t = 1/self.frequency
        if delta_t == 0:
            return # Not ready yet!
