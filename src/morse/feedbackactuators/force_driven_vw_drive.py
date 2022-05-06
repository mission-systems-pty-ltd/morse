import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
import morse.core.feedbackactuator
from morse.helpers.components import add_data, add_property
from morse.core.mathutils import *
from morse.helpers.compound import *
from morse.helpers.controller import PIDController
import math


class ForceVWDrive(morse.core.feedbackactuator.FeedbackActuator):
    """ 
    """
    _name = "Steer/Force Actuator"
    _short_desc = "Motion controller using engine force and steer angle speeds"

    add_data('w', 0.0, "float", 'Angle of the wheels with respect to the vehicle (in radian)')
    add_data('v', 0.0, "float", 'The force applied to the traction wheels. A negative force will make the vehicle move forward. A positive force will make it go backwards.')
    add_data('brake', 0.0, "float", 'The force applied to the brake. It opposes to the force.')

    add_property('kp_v',      800.0,   'kp_v',       'float', 'PID linear velocity proportional gain')
    add_property('ki_v',      1.0,     'ki_v',       'float', 'PID linear velocity integral gain') 
    add_property('kd_v',      100.0,   'kd_v',       'float', 'PID linear velocity differential gain')
    add_property('i_limit_v', 1000.0,  'i_limit_v',  'float', 'PID linear velocity integrator limit')

    add_property('kp_w',      10000.0, 'kp_w',       'float', 'PID angular velocity proportional gain')
    add_property('ki_w',      100.0,   'ki_w',       'float', 'PID angular velocity integral gain')
    add_property('kd_w',      1.0,     'kd_w',       'float', 'PID angular velocity differential gain')
    add_property('i_limit_w', 1000.0,  'i_limit_w',  'float', 'PID angular velocity integrator limit')

    add_property('wheel_f_limit', 3000.0,  'wheel_f_limit',  'float', 'Maximum force to apply on the wheels (min is -ve max)')
    add_property('wheel_radius',  0.35,    'wheel_radius',   'float', 'Radius of the wheels in m')
    add_property('track_width',   1.0,     'track_width',    'float', 'distance in m between the left and right sets of wheels.')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.feedbackactuator.FeedbackActuator.__init__(self, obj, parent)

        self.robot = parent.bge_object
        children = self.robot.childrenRecursive

        self.wheels = {"FR": 0, "FL": 1, "RR": 2, "RL": 3}
        self._stopped = True

        # TODO: TUNE Guess so far
        self.pids = {}
        self.pid_v = PIDController(kp=self.kp_v, kd=self.ki_v, ki=self.kd_v, limits_integrator=self.i_limit_v)
        self.pid_w = PIDController(kp=self.kp_w, kd=self.ki_w, ki=self.kd_w, limits_integrator=self.i_limit_w)
        logger.info('Component initialized')


    def default_action(self):

        delta_t = 1/self.frequency
        if delta_t == 0:
            return

        vx = self.local_data["v"] / 2.0
        vw = self.local_data["w"] / 2.0        

        # print("v: ", vx)
        # print("w: ", vw)

        # Another formula for computing left and right wheel speeds:
        # http://www.uta.edu/utari/acs/jmireles/Robotics/KinematicsMobileRobots.pdf
        v_ws_l = vx - (self.track_width / 2.0) * vw 
        v_ws_r = vx + (self.track_width / 2.0) * vw 
        # convert to angular speeds
        w_ws_l = -1.0 * v_ws_l / self.wheel_radius
        w_ws_r = -1.0 * v_ws_r / self.wheel_radius

        """ Apply (steer, force) to the parent robot. """
        vehicle = self.robot_parent.vehicle

        # print("Running wheel PID loop")
        self.pid_v.setpoint = vx
        self.pid_w.setpoint = vw
        vel = self.robot.localLinearVelocity.copy()[1]
        angular_vel = self.robot.localAngularVelocity.copy()[2]

        # calculate desired wheel speeds and set them
        if abs(vel) < 0.001 and abs(angular_vel) < 0.001:
            self._stopped = True
        else:
            # this is need to "wake up" the physic objects if they have
            # gone to sleep apply a tiny impulse straight down on the object
            if self._stopped:
                self.robot.applyImpulse(self.robot.position, (0.0, 0.0, 0.000001))
            self._stopped = False

        # print("vel: ", vel)
        # print("angular_vel: ", angular_vel)

        computed_vx = self.pid_v.update(vel)
        computed_vw = self.pid_w.update(angular_vel)
        self._apply_vw_wheels(-computed_vx, -computed_vw)        

    def _apply_vw_wheels(self, cmd_v, cmd_w):

        vehicle = self.robot_parent.vehicle

        v_ws_l = cmd_v - (self.track_width / 2.0) * cmd_w
        v_ws_r = cmd_v + (self.track_width / 2.0) * cmd_w

        if (abs(v_ws_l) > self.wheel_f_limit): 
            v_ws_l = math.copysign(self.wheel_f_limit, v_ws_l)
        if (abs(v_ws_r) > self.wheel_f_limit): 
            v_ws_r = math.copysign(self.wheel_f_limit, v_ws_r)

        # print("v_ws_l: ", v_ws_l, " Error: ", self.pid_v._last_error, " integrator: ", self.pid_v._integrator)
        # print("v_ws_r: ", v_ws_r, " Error: ", self.pid_w._last_error, " integrator: ", self.pid_w._integrator)

        vehicle.applyEngineForce( v_ws_r / 2.0, self.wheels["FR"] )
        vehicle.applyEngineForce( v_ws_r / 2.0, self.wheels["RR"] )
        vehicle.applyEngineForce( v_ws_l / 2.0, self.wheels["FL"] )
        vehicle.applyEngineForce( v_ws_l / 2.0, self.wheels["RL"] )

