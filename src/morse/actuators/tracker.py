import logging; logger = logging.getLogger("morse." + __name__)
import json
import morse.core.actuator
from morse.core import blenderapi
from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *
import time 

def keyboard_key_from_string(key_string):
    if (key_string == "A"):
        return blenderapi.AKEY
    elif (key_string == "B"):
        return blenderapi.BKEY
    elif (key_string == "C"):
        return blenderapi.CKEY
    elif (key_string == "D"):
        return blenderapi.DKEY
    elif (key_string == "E"):
        return blenderapi.EKEY
    elif (key_string == "F"):
        return blenderapi.FKEY
    elif (key_string == "G"):
        return blenderapi.GKEY
    elif (key_string == "H"):
        return blenderapi.HKEY
    elif (key_string == "I"):
        return blenderapi.IKEY
    elif (key_string == "J"):
        return blenderapi.JKEY
    elif (key_string == "K"):
        return blenderapi.KKEY
    elif (key_string == "L"):
        return blenderapi.LKEY
    elif (key_string == "M"):
        return blenderapi.MKEY
    elif (key_string == "N"):
        return blenderapi.NKEY
    elif (key_string == "O"):
        return blenderapi.OKEY
    elif (key_string == "P"):
        return blenderapi.PKEY
    elif (key_string == "Q"):
        return blenderapi.QKEY
    elif (key_string == "R"):
        return blenderapi.RKEY
    elif (key_string == "S"):
        return blenderapi.SKEY
    elif (key_string == "T"):
        return blenderapi.TKEY
    elif (key_string == "U"):
        return blenderapi.UKEY
    elif (key_string == "V"):
        return blenderapi.VKEY
    elif (key_string == "W"):
        return blenderapi.WKEY
    elif (key_string == "X"):
        return blenderapi.XKEY
    elif (key_string == "Y"):
        return blenderapi.YKEY
    elif (key_string == "Z"):
        return blenderapi.ZKEY

class Tracker(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Tracker"
    _short_desc = "Continuously points the FP_camera at a selected object"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_property('target',  ['robot'], 'Target',   'string', 'Target object for camera tracker')
    add_property('standoff', 10,    'Standoff', 'float',  'Standoff distance for camera tracker')
    add_property('lock_cam_key', "L",    'lock_cam_key', 'String',  'Keyboard key to lock/unlock the camera position')

    def __init__(self, obj, parent=None ):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get every bge object in the scene
        self.objs = blenderapi.scene().objects
    
        # Get the water surface object
        try:
            self.targets = json.loads( self.target )
            self.num_targets = len( self.targets["robots"] )
            self.multiple_targets = True
        except:
            self.multiple_targets = False
            self.target_obj = self.objs[ self.target ]

        if self.multiple_targets:
            self.target_idx = 0
            self.set_target() 

        self.lock_key = keyboard_key_from_string(self.lock_cam_key)

        # Useful if you want the camera to stay in the same position relative to a robot
        self.camera_locked = False
        self.prev_target_pos = self.target_obj.worldPosition
        self.locked_relative_position = self.get_relative_camera_position()

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):        
        self.robots_dict = blenderapi.persistantstorage()
        
        ##del self.robots_dict["fake"]
        #self.robots_names = list( self.robots_dict )
        #self.num_robots = len( self.robots_names )

        #target_name = self.robots_names[ self.robot_idx ]
        #self.target_obj = self.robots_dict[ target_name ]


        # Game loop frequency
        delta_t = 1/self.frequency
        if delta_t == 0:
            return # Not ready yet!

        # # Loop through the game objects to choose which one to track
        keyboard = blenderapi.keyboard()
        is_actived = blenderapi.input_just_activated()
        if self.multiple_targets:
            if keyboard.events[blenderapi.LEFTARROWKEY] == is_actived:
                self.set_target("prev")
            if keyboard.events[blenderapi.RIGHTARROWKEY] == is_actived:
                self.set_target("next")
        
        if keyboard.events[self.lock_key] == is_actived:
            self.toggle_locked_camera()

        if self.camera_locked:
            self.scene.active_camera.worldPosition = self.target_obj.worldPosition + self.locked_relative_position
        else:
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



    def set_target(self, key="current"):
        
        if key == "current":
            self.target_obj = self.objs[ self.targets["robots"][ self.target_idx ] ] #objs[self.target]
        
        elif key == "next":
            self.prev_target_obj = self.objs[ self.targets["robots"][ self.target_idx ] ] #objs[self.target]
            self.target_idx = self.target_idx+1
            if self.target_idx > (self.num_targets-1):
                self.target_idx = 0
            self.target_obj = self.objs[ self.targets["robots"][ self.target_idx ] ] #objs[self.target]
            self.update_camera_position()

        elif key == "prev":
            self.prev_target_obj = self.objs[ self.targets["robots"][ self.target_idx ] ] #objs[self.target]
            self.target_idx = self.target_idx-1
            if self.target_idx < 0:
                self.target_idx = self.num_targets-1
            self.target_obj = self.objs[ self.targets["robots"][ self.target_idx ] ] #objs[self.target]
            self.update_camera_position()
        else: 
            print("Error getting next target")

    def update_camera_position(self):
        # Move the camera so that it has the same offset to the new robot
        # that it had to the old robot
        self.prev_target_pos = self.prev_target_obj.worldPosition
        new_target_pos = self.target_obj.worldPosition

        if self.camera_locked:
            self.scene.active_camera.worldPosition = new_target_pos + self.locked_relative_position
        else:
            relative_position = self.get_relative_camera_position()        
            self.scene.active_camera.worldPosition = new_target_pos + relative_position

    def get_relative_camera_position(self):
        camera_pos = self.scene.active_camera.worldPosition
        relative_position = camera_pos - self.target_obj.worldPosition
        return relative_position

    def toggle_locked_camera(self):
        self.camera_locked = not self.camera_locked 
        self.locked_relative_position = self.get_relative_camera_position()
        if self.camera_locked:
            print("Camera position locked, press '"+self.lock_cam_key+"' to unlock.")
        else:
            print("Camera position unlocked, press '"+self.lock_cam_key+"' to lock.")
