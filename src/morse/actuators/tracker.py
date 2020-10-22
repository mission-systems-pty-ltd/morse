import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *

class Tracker(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Tracker"
    _short_desc = "Continuously points the FP_camera at a selected object"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_property('target',  'robot', 'Target',   'string', 'Target object for camera tracker')
    add_property('standoff', 10,    'Standoff', 'float',  'Standoff distance for camera tracker')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get every bge object in the scene
        objs = blenderapi.scene().objects

        # Get the water surface object
        self.target_obj = objs[self.target]

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        # Game loop frequency
        delta_t = 1/self.frequency
        if delta_t == 0:
            return # Not ready yet!

        # Figure out which camera is active and
        # publish its position and view vector.
        camera = self.scene.active_camera
        pos = camera.worldPosition
        mat = camera.worldOrientation

        # Get target object position
        target = self.target_obj.worldPosition
        speed = self.target_obj.worldLinearVelocity.length

        direction = target - pos
        distance = direction.length
        rot_quat = direction.to_track_quat('-Z', 'Y')

        camera.worldOrientation = rot_quat

        if distance > self.standoff:
            camera.worldPosition += delta_t * speed * direction/direction.length