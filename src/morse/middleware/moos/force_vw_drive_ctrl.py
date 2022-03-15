import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber
import json

class ForceVWDriveCtrl(MOOSSubscriber):
    """ Read MOOS commands and update local data. """
 
    def initialize(self):
        MOOSSubscriber.initialize(self) 
        # register for control variables from the database
        self.register_message_to_queue("VW_CTRL",'vw_queue', self.force_vw_ctrl_msg_callback)
        logger.debug('ForceVWDriveCtrl MOOS initialized.')

    def force_vw_ctrl_msg_callback(self, msg):
        if ( msg.key() == "VW_CTRL" and msg.is_string()):
            print(msg.string())
            try:
                msg_data = json.loads(msg.string())
                print(msg_data)
                self.data['v'] = msg_data["v"]
                self.data['w'] = msg_data["w"]
            except:
                print("decode failed")
        self._new_messages = True
        return True

    def update_morse_data(self):
        logger.debug('ForceVWDriveCtrl.update_morse_data() called.')
