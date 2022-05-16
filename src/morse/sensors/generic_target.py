import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data
from morse.builder import bpymorse


class GenericTarget(morse.core.sensor.Sensor):
    """ 
    This sensor tracks all the objects in the scene that have a 'target' property attached to them
    in their .blend data. 
    """
    _name = "GenericTarget"

    add_data('targets', None, "json", 'List of all targets in the scene')    
    # add_data('x', 0.0, "float",
    #          'x coordinate of the sensor, in world coordinate, in meter')
    # add_data('y', 0.0, "float",
    #          'y coordinate of the sensor, in world coordinate, in meter')
    # add_data('z', 0.0, "float",
    #          'z coordinate of the sensor, in world coordinate, in meter')
    # add_data('yaw', 0.0, "float",
    #          'rotation around the Z axis of the sensor, in radian')
    # add_data('pitch', 0.0, "float",
    #          'rotation around the Y axis of the sensor, in radian')
    # add_data('roll', 0.0, "float",
    #          'rotation around the X axis of the sensor, in radian')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        scene = morse.core.blenderapi.scene()
        # All bge objects in scene
        bge_objs = scene.objects
        # All bpy objects in scene
        bpy_objs = bpymorse.get_objects()

        self.local_data['targets'] = {}

        # Get target objects
        self.target_objects = []
        for obj in bge_objs:
            if 'target' in obj:
                self.target_objects.append(obj)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)


    def default_action(self):
        """ Get the x, y, z, yaw, pitch and roll of the blender object. """
        
        for target in self.target_objects:
            position = {}
            position['x'] = target.localPosition.x 
            position['y'] = target.localPosition.y 
            position['z'] = target.localPosition.z
            # type = {}
            # type['type']
            self.local_data['targets'][target.name] = {}
            self.local_data['targets'][target.name]['position'] = position
            self.local_data['targets'][target.name]['type'] = target['target']

        # print(self.local_data['targets'])
