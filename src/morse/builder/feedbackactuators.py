import logging; logger = logging.getLogger("morsebuilder." + __name__)
from morse.builder.creator import FeedbackActuatorCreator
from morse.builder.blenderobjects import *
from morse.core.exceptions import MorseBuilderError


class MotorAndEncoder(FeedbackActuatorCreator):
    _classpath = "morse.feedbackactuators.motor_and_encoder.MotorAndEncoder"
    _blendname = "motor_with_encoder"

    def __init__(self, name=None):
        FeedbackActuatorCreator.__init__(self, name)

