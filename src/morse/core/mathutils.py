""" This module wraps the calls to the Blender 'mathutils' API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""

import sys
import os

# running in Blender?
try:
    import bpy
    from mathutils import Matrix, Vector, Euler, Quaternion
except:
    print("WARNING: MORSE is running outside Blender!")
    def Matrix(*args):
        return None
    def Vector(*args):
        return None
    def Euler(*args):
        return None
    def Quaternion(*args):
        return None