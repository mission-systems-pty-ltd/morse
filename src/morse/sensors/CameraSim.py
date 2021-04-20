import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.sensor
import morse.sensors.camera

from morse.core.services import service, async_service
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from morse.sensors.ObjectServer import create_trigger_msg
from math import radians, pi
import numpy as np
import json

class CameraSim(morse.sensors.camera.Camera):
    """Write here the general documentation of your sensor.
    It will appear in the generated online documentation.
    """
    _name = "CameraSim"
    _short_desc = "Ray-tracing accelerated morse camera sensor."

    # define here the data fields exported by your sensor
    # format is: field name, default initial value, type, description
    add_data('camera_name',     'CAMERASIM',    'string',   'Name of this camera device')
    add_data('camera_status',   'ON',           'string',   'Status of this camera device - ON/OFF')  # maybe we wont need all of these. some might have been used for the debug lines?

    # format is: field name, default initial value, name (must match the blender name), type, description
    add_property('image_width',         256,    'image_width',          'int',      'cam_width in pixels')
    add_property('image_height',        256,    'image_height',         'int',      'cam_height in pixels')
    add_property('horizontal_fov_deg',  256,    'horizontal_fov_deg',   'float',    'full horizontal FOV in degrees')
    add_property('vertical_fov_deg',    256,    'vertical_fov_deg',     'float',    'full vertical FOV in degrees')
    # add_property('image_focal',     25.0,   'cam_focal length in mm')
    # add_property('vertical_flip',   True,   'Vertical_Flip the final image')
    add_property('max_range',       200.0,  'max_range','float','Camera range in m')

    add_data('launch_trigger',   '',   'string', 'Information for a radar beam launch')
    add_property('send_json',       True,  'send_json',       'bool',  'Send small messages as json')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get the full name of the current camera
        # camera_name = self.bge_object.name.upper()
        # self.local_data['camera_name'] = camera_name.replace(".","_")

        logger.info('Component initialized')


    def default_action(self):
        """ Main loop of the sensor.

        Implements the component behaviour
        """

        pos = self.bge_object.worldPosition
        rotation = self.bge_object.worldOrientation.copy()

        if self.send_json:
            self.local_data['launch_trigger'] = {}
            self.local_data['launch_trigger']['launchTrigger'] = create_trigger_msg(pos, rotation, self.image_width,
                                                                                    self.image_height, 1, True)
            self.local_data['launch_trigger']['maxRange'] = self.max_range
            self.local_data['launch_trigger']['azimuthFov'] = pi * self.horizontal_fov_deg / 180.0
            self.local_data['launch_trigger']['elevationFov'] = pi * self.vertical_fov_deg / 180.0
        else:
            import capnp
            import sys
            sys.path.extend(["/usr/local/share", "/usr/local/share/camerasim"])
            import camerasim_capnp as camerasim
            self.local_data['launch_trigger'] = camerasim.CameraSimLaunchTrigger.new_message()
            self.local_data['launch_trigger'].launchTrigger = create_trigger_msg(pos, rotation, self.image_width,
                                                                                 self.image_height, 1, False)
            self.local_data['launch_trigger'].maxRange = self.max_range
            self.local_data['launch_trigger'].azimuthFov = pi * self.horizontal_fov_deg / 180.0
            self.local_data['launch_trigger'].elevationFov = pi * self.vertical_fov_deg / 180.0

        

