import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
from pymavlink.dialects.v10 import common as mavlink
from morse.builder.sensors import *
import random

# UPDATE_MASK = (1 << 12) - 1
UPDATE_MASK = int(65535)

class CompoundSensor(MavlinkSensor):
    _type_name = "HIL_SENSOR"

    initialised = False

    def make_msg(self):

        # Get the parent robot
        # robot = self.component_name.split('.')[0]

        # Components of compound sensor
        # mag =  self.data[robot + '.mag']
        # imu =  self.data[robot + '.imu']
        # asi =  self.data[robot + '.asi']
        # gps =  self.data[robot + '.gps']
        # baro = self.data[robot + '.baro']

        mag = None
        imu = None
        asi = None
        gps = None
        baro = None

        # Fetch known components of the compound sensor
        for component_name in self.data:
            if "mag" in component_name:
                mag = self.data[component_name]
            if "imu" in component_name:
                imu = self.data[component_name]
            if "asi" in component_name:
                asi = self.data[component_name]
            if "gps" in component_name:
                gps = self.data[component_name]
            if "baro" in component_name:
                baro = self.data[component_name]

        if mag == None or imu == None or asi == None or gps == None or baro == None:
            print("hilSensor can't find all required sensor components")

        if not self.initialised:
            self._msg = mavlink.MAVLink_set_mode_message(
                1,
                32,
                0
            )

            self.initialised = True

        else:

            acc = imu['linear_acceleration']

            # Translate to NED body frame
            acc.x =  random.gauss(acc.x, 0.001)
            acc.y = -random.gauss(acc.y, 0.001)
            acc.z = -random.gauss(acc.z, 0.001)

            gyro = imu['angular_velocity']

            # Translate to NED body frame
            gyro.x =  random.gauss(gyro.x, 0.001)
            gyro.y = -random.gauss(gyro.y, 0.001)
            gyro.z = -random.gauss(gyro.z, 0.001)

            # Scaled magnetic fields nT -> Gauss (already in NED body frame)
            mag_x = mag['x'] * 1e-5
            mag_y = mag['y'] * 1e-5
            mag_z = mag['z'] * 1e-5

            # Same noise level as jmavsim
            mag_x = random.gauss(mag_x, 0.001)
            mag_y = random.gauss(mag_y, 0.001)
            mag_z = random.gauss(mag_z, 0.001)

            press_alt = gps['altitude']
            press_alt = random.gauss(press_alt, 1)

            abs_press = baro['pressure']
            abs_press = random.gauss(abs_press, 1)

            diff_press = asi['diff_pressure']
            diff_press = random.gauss(diff_press, 1)

            self._msg = mavlink.MAVLink_hil_sensor_message(
                int(imu['timestamp'] * 1.0e6),
                acc.x,
                acc.y,
                acc.z,
                gyro.x,
                gyro.y,
                gyro.z,
                mag_x,
                mag_y,
                mag_z,
                abs_press * 0.01, # Pa -> mbars
                diff_press * 0.01, # Pa -> mbars
                press_alt,
                25.0,
                UPDATE_MASK             
            )

