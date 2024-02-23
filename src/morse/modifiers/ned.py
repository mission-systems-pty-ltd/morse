import logging; logger = logging.getLogger("morse." + __name__)
import math
from morse.modifiers.abstract_modifier import AbstractModifier

class NEDModifier(AbstractModifier):
    """ 
    This modifier converts the coordinates generated by the MORSE simulator to
    change to the North, East, Down (NED) coordinate system, instead of the East,
    North, Up (ENU) system normally used by Blender.

    This is achieved by switching the direction of the X and Y axis, as well as
    inverting the sense of the Z axis.

    This modifier attempts to alter data ``x``, ``y`` and ``z`` for position, 
    and ``yaw``, ``pitch`` and ``roll`` for orientation. 

    The NED modifier provides as modifiers:
    
    * :py:class:`morse.modifiers.ned.CoordinatesToNED`
    * :py:class:`morse.modifiers.ned.CoordinatesFromNED`
    * :py:class:`morse.modifiers.ned.AnglesToNED`
    * :py:class:`morse.modifiers.ned.AnglesFromNED`

    """
    
    _name = "NED"
    
    def modify(self):
        pass
    
class CoordinatesToNED(NEDModifier):
    """ Convert the coordinates from ENU to NED. """
    def modify(self):
        try:
            tmp = self.data['x']
            self.data['x'] = self.data['y']
            self.data['y'] = tmp
            self.data['z'] = - self.data['z']
        except KeyError as detail:
            self.key_error(detail)

class CoordinatesFromNED(NEDModifier):        
    """ Convert the coordinates from NED to ENU. """
    def modify(self):
        try:
            tmp = self.data['x']
            self.data['x'] = self.data['y']
            self.data['y'] = tmp
            self.data['z'] = - self.data['z']
        except KeyError as detail:
            self.key_error(detail)

class AnglesToNED(NEDModifier):
    """ Convert the angles from ENU to NED. """
    def modify(self):
        try:
            roll = math.pi/2 - self.data['yaw']
            self.data['yaw'] = self.data['roll']
            self.data['pitch'] = - self.data['pitch']
            self.data['roll'] = roll
        except KeyError as detail:
            self.key_error(detail)

class AnglesFromNED(NEDModifier):
    """ Convert the angles from NED to ENU. """
    def modify(self):
        try:
            yaw = math.pi/2 - self.data['roll']
            self.data['pitch'] = - self.data['pitch']
            self.data['roll'] = self.data['yaw']
            self.data['yaw'] = yaw
        except KeyError as detail:
            self.key_error(detail)

class PoseToNED(NEDModifier):
    """ Convert the coordinates from ENU to NED. """
    """ Convert the angles from ENU to NED. """
    def modify(self):
        try:
            # ENU to NED x=y, y=x, z=-z
            tmp = self.data['x']
            self.data['x'] = self.data['y']
            self.data['y'] = tmp
            self.data['z'] = -self.data['z']

            # Rotate about each axis to move from the ENU to NED Frame
            self.data['roll'] = self.data['roll']
            self.data['pitch'] = -self.data['pitch']
            self.data['yaw'] = -self.data['yaw'] + math.radians(90)
            
        except KeyError as detail:
            self.key_error(detail)
