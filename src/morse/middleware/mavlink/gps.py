import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
from pymavlink.dialects.v10 import common as mavlink
from math import sqrt, degrees, atan2
import random

class Gps(MavlinkSensor):
    _type_name = "HIL_GPS"

    def make_msg(self):
        alt = self.data['altitude']
        alt = random.gauss(alt, 0.1)

        # Geodetic coordinates
        lat = self.data['latitude']
        lon = self.data['longitude']

        # Linear velocities from GPS simulation 
        # ENU earth-fixed frame -> NED earth fixed frame.
        # See gazebo_mavlink_interface.cpp
        v_e =  self.data['velocity'][0]
        v_n =  self.data['velocity'][1]
        v_d = -self.data['velocity'][2]

        # Scale raw velocities
        v_n_cmps = int(v_n * 1.0e2)
        v_e_cmps = int(v_e * 1.0e2)
        v_d_cmps = int(v_d * 1.0e2)

        # Keep velocities in range
        v_n_cmps = max(min(v_n_cmps, 0x7fff), -0x7fff-1)
        v_e_cmps = max(min(v_e_cmps, 0x7fff), -0x7fff-1)
        v_d_cmps = max(min(v_d_cmps, 0x7fff), -0x7fff-1)

        vel = sqrt(v_e * v_e + v_n * v_n)
        vel_cmps = int(vel * 1.0e2)
        vel_cmps = min(vel_cmps, 0xffff)

        if abs(v_e) < 1e-6 and abs(v_n) < 1e-6:
            cog_cdeg = 0xffff
        else:    
            cog = degrees(atan2(v_e, v_n))
            if cog < 0: cog += 360.0
            cog_cdeg = int(cog * 1e2)

        ts = self.data['timestamp']

        # Expects coordinates in aeronautical frame
        self._msg = mavlink.MAVLink_hil_gps_message(
                int(ts * 1.0e6),
                3,
                int(lat * 1.0e7),
                int(lon * 1.0e7),
                int(alt * 1.0e3),
                100, # See gazebo_mavlink_interface.cpp
                100, # See gazebo_mavlink_interface.cpp
                vel_cmps,
                v_n_cmps,
                v_e_cmps,
                v_d_cmps,
                cog_cdeg,
                10
        )

