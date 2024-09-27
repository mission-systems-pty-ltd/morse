import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkActuator
from pymavlink.dialects.v10 import common as mavlink

class ThrustActuator(MavlinkActuator):
    _type_name = "HIL_ACTUATOR_CONTROLS"

    def process_msg(self):

        controls = self._msg.controls
        mode = self._msg.mode

        # Are we armed?
        if not mode & 0b10000000:

            # Motor shutdown
            controls = [0] * 16
            #print('Motor Shutdown.')

        self.data['rotor0'] = controls[2]
        self.data['rotor1'] = controls[2]

        self.data['rotor2'] = controls[0]
        self.data['rotor3'] = controls[0]

        self.data['rotor4'] = controls[1]
        self.data['rotor5'] = controls[1]

        self.data['rotor6'] = controls[3]
        self.data['rotor7'] = controls[3]

