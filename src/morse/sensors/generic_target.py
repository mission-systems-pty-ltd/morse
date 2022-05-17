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
        for target in self.target_objects:
            position = {}
            position['x'] = target.localPosition.x 
            position['y'] = target.localPosition.y 
            position['z'] = target.localPosition.z
            self.local_data['targets'][target.name] = {}
            self.local_data['targets'][target.name]['position'] = position
            self.local_data['targets'][target.name]['label'] = target['target']
