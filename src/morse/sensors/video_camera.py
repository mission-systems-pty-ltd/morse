import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import async_service
from morse.core import status
from morse.core import mathutils
import morse.sensors.camera
from morse.helpers.components import add_data
import copy
from queue import Queue

BLENDER_HORIZONTAL_APERTURE = 32.0

class VideoCamera(morse.sensors.camera.Camera):
    """
    This sensor emulates a single video camera. It generates a series of
    RGBA images.  Images are encoded as binary char arrays, with 4 bytes
    per pixel.

    Camera calibration matrix
    -------------------------

    The camera configuration parameters implicitly define a geometric camera in
    blender units. Knowing that the **cam_focal** attribute is a value that
    represents the distance in Blender unit at which the largest image dimension is
    32.0 Blender units, the camera intrinsic calibration matrix is defined as

    +--------------+-------------+---------+
    | **alpha_u**  |      0      | **u_0** |
    +--------------+-------------+---------+
    |       0      | **alpha_v** | **v_0** |
    +--------------+-------------+---------+
    |       0      |      0      |    1    |
    +--------------+-------------+---------+

    where:

    - **alpha_u** == **alpha_v** = **cam_width** . **cam_focal** / 32 (we suppose
      here that **cam_width** > **cam_height**. If not, then use **cam_height** in
      the formula)
    - **u_0** = **cam_width** / 2
    - **v_0** = **cam_height** / 2

    See also :doc:`../sensors/camera` for generic informations about Morse cameras.
    """

    _name = "Video camera"
    _short_desc = "A camera capturing RGBA image"

    add_data('image', 'none', 'buffer',
           "The data captured by the camera, stored as a Python Buffer \
            class  object. The data is of size ``(cam_width * cam_height * 4)``\
            bytes. The image is stored as RGBA.")
    add_data('intrinsic_matrix', 'none', 'mat3<float>',
        "The intrinsic calibration matrix, stored as a 3x3 row major Matrix.")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.sensors.camera.Camera.__init__(self, obj, parent)

        # Prepare the exportable data of this sensor
        self.local_data['image'] = ''

        # Prepare the intrinsic matrix for this camera.
        # Note that the matrix is stored in row major
        self.calculate_intrinsic_matrix()

        self.capturing = False
        self._n = -1

        # Variable to indicate this is a camera
        self.camera_tag = True

        # Position of the robot where the last shot is taken
        self.robot_pose = copy.copy(self.robot_parent.position_3d)

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)

    def interrupt(self):
        self._n = 0
        morse.sensors.camera.Camera.interrupt(self)

    @async_service
    def capture(self, n):
        """
        Capture **n** images

        :param n: the number of images to take. A negative number means
                  take image indefinitely
        """
        self._n = n

    def calculate_intrinsic_matrix(self):
        intrinsic = mathutils.Matrix.Identity(3)
        alpha_u = self.image_width  * \
                  self.image_focal / BLENDER_HORIZONTAL_APERTURE
        intrinsic[0][0] = alpha_u
        intrinsic[1][1] = alpha_u
        intrinsic[0][2] = self.image_width / 2.0
        intrinsic[1][2] = self.image_height / 2.0
        self.local_data['intrinsic_matrix'] = intrinsic

    def default_action(self):
        """ Update the texture image. """

        # Grab an image from the texture
        if self.bge_object['capturing'] and (self._n != 0) :

            # Call the action of the parent class
            morse.sensors.camera.Camera.default_action(self)

            # Recalculate the intrinsic matrix, because it can be incorrect
            # (e.g. if image_fov was used instead of image_focal)
            self.calculate_intrinsic_matrix()

            self.robot_pose = copy.copy(self.robot_parent.position_3d)
            # Fill in the exportable data
            # NOTE: Blender returns the image as a binary string
            #  encoded as RGBA
            self.local_data['image'] = self.image_data
            self.capturing = True

            if self._n > 0:
                self._n -= 1
                if self._n == 0:
                    self.completed(status.SUCCESS)
        else:
            self.capturing = False

class TeleportingCamera(VideoCamera):
    """
    This sensor is a repositionable camera that produces images according to poses that come from an external stream.

    Currently supports ROS with:
     - morse.middleware.ros.video_camera.TeleportingCameraPublisher
     - morse.middleware.ros.read_pose.PoseToQueueReader
    """

    _name = "TeleportingCamera"
    _short_desc = "Teleporting (Repositionable) camera"

    add_data('pose_queue', Queue(), 'queue', "Queue of poses to capture from. A pose is a 4x4 matrix given by worldTransform")
    add_data('new_image', False, 'boolean', 'True if there is new data to publish')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        VideoCamera.__init__(self, obj, parent)

        # Boolean to indicate if a trigger should occur (see default action)
        self.trigger = False

    # Note that setting the bge)object worldTransform then calling the video camera default action does not work, but
    # will update the pose for the next image not the current image. So we process the queue with a slight (1 tick)
    # delay, i.e. the default action sets up the correct pose for the next default action.
    def default_action(self):
        if self.trigger:
            # Acquire the data
            VideoCamera.default_action(self)
            self.local_data['new_image'] = True

        if self.local_data['pose_queue'].empty():
            self.trigger = False
        else:
            self.trigger = True
            # Set the pose (popping the pose off the queue in the process)
            self.bge_object.worldTransform = self.local_data['pose_queue'].get()
