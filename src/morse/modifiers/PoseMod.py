# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Mission Systems Pty Ltd ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Projects: WamV-Morse-Sim
# 
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Primary Author:
# david battle <david.battle@missionsystems.com.au>
# Other Author(s):
# none
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Date Created:
# 29/01/2019
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
from .abstract_modifier import AbstractModifier
from math import pi, radians

class Bluefin21PoseModifier(AbstractModifier):
    def initialize(self):
        """ initialization of parameters ... """
        
    def modify(self):
        # Place where the data modification occurs
        self.data['yaw']   =  self.data['yaw'] - pi / 2.0
        self.data['pitch'] = -self.data['pitch']

class WamVPoseModifier(AbstractModifier):
    def initialize(self):
        """ initialization of parameters ... """
        
    def modify(self):
        # Place where the data modification occurs
        self.data['yaw'] = self.data['yaw'] - pi/2.0
        

class PoseENUToNED(AbstractModifier):
    """ Works on properties used by the Pose sensor. """
    """ Convert the coordinates from ENU to NED. """
    """ Convert the angles from ENU to NED. """
    def constrain_angle(self, x):
        return (x + pi) % (2 * pi) - pi

    def modify(self):
        if 'x' in self.data and 'y' in self.data and 'z' in self.data:
            tmp = self.data['x']
            self.data['x'] =  self.data['y']
            self.data['y'] =  tmp
            self.data['z'] = -self.data['z']
        elif 'position' in self.data:
            tmp = self.data["position"][0]
            self.data["position"][0] =  self.data["position"][1]
            self.data["position"][1] =  tmp
            self.data["position"][2] = -self.data["position"][2]

        if 'roll' in self.data and 'pitch' in self.data and 'yaw' in self.data:
            self.data['roll']  =  self.data['roll']
            self.data['pitch'] = -self.data['pitch']
            self.data['yaw']   = -self.data['yaw'] + radians(90)
            self.data['yaw'] = self.constrain_angle(self.data['yaw'])
        elif "orientation" in self.data:
            mathutils_quat = self.data['orientation']
            euler = mathutils_quat.to_euler()
            euler.x =  euler.x
            euler.y = -euler.y
            euler.z = -euler.z + radians(90)
            euler.z = self.constrain_angle(euler.z)
            self.data['orientation'] = euler.to_quaternion()
