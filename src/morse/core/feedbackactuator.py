
import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
import morse.core.object
from morse.core.sensor import Sensor
from morse.core.actuator import Actuator
import time

class FeedbackActuator(Sensor, Actuator, morse.core.object.Object):
    """ Basic Class for all feedback actuator objects.

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta
 
    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        morse.core.object.Object.__init__(self, obj, parent)

        # Define lists of dynamically added functions
        # Cant call sensor and actuator init as they will both call morse.core.object.Object.__init__(...)
        self.input_functions = []
        self.input_modifiers = []
        self.output_functions = []
        self.output_modifiers = []

        self.profile = None
        if "profile" in self.bge_object:
            self.time = {}
            self.profile = ["profile", "profile_action", "profile_modifiers",
                            "profile_datastreams"]
            for key in self.profile:
                self.time[key] = 0.0
            self.time_start = time.time()
        logger.info("Initialized feedback actuator")

    def finalize(self):
        self._active = False
        morse.core.object.Object.finalize(self)
        del self.input_functions[:]
        del self.input_modifiers[:]
        del self.output_functions[:]
        del self.output_modifiers[:]
        logger.info("Finalized feedback actuator")

    # I was hoping to just run Actuator.action() & Sensor.action() but
    # That would call default action twice every tick :/
    def action(self):
        # Actuator.action(self)
        # Sensor.action(self)

        """ Call the action functions that have been added to the list. """
        # Do nothing if this component has been deactivated
        if not self._active:
            return

        if not self.periodic_call():
            return

        # Update the component's position in the world
        self.position_3d.update(self.bge_object)

        self.local_data['timestamp'] = self.robot_parent.gettime()
        if logger.isEnabledFor(logging.DEBUG):
            self.local_data['simulator_time'] = time.time()

        received = False
        status = False


        # First the input functions
        for function in self.input_functions:
            status = function(self)
            received = received or status

        if received:
            # Data modification functions
            for function in self.input_modifiers:
                function()

        # record the time before performing the default action for profiling
        if self.profile:
            time_before_action = time.time()

        # Call the regular action function of the component
        self.default_action()

        # record the time before calling modifiers for profiling
        if self.profile:
            time_before_modifiers = time.time()

        # Data modification functions
        for function in self.output_modifiers:
            function()

        # record the time before calling datastreams for profiling
        if self.profile:
            time_before_datastreams = time.time()

        # Lastly output functions
        for function in self.output_functions:
            function(self)

        # profiling
        if self.profile:
            time_now = time.time()
            self.time["profile"] += time_now - time_before_action
            self.time["profile_action"] += time_before_modifiers - time_before_action
            self.time["profile_modifiers"] += time_before_datastreams - time_before_modifiers
            self.time["profile_datastreams"] += time_now - time_before_datastreams
            morse_time = time_now - self.time_start
            for key in self.profile:
                ratio = self.time[key] / morse_time
                # format the display
                self.bge_object[key] = "%4.1f%% %s"% (100.0 * ratio, '█' * int(10 * ratio))
            if morse_time > 1: # re-init mean every sec
                for key in self.profile:
                    self.time[key] = 0.0
                self.time_start = time.time()
