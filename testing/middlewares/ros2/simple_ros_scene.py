#!/usr/bin/python3

# Import libraries
try:
    from morse.testing.testing import MorseTestCase
    from morse.builder import *
except ImportError:
    pass

# Ros Scene Test
class RosSceneTest(MorseTestCase):

    # Defines the test scenario, using the Builder API.
    def setUpEnv(self):
        
        # Initialise robot and sensors
        robot = ATRV()
        sensors = [Accelerometer, ArmaturePose, Battery, Clock, GPS, Gyroscope,
            IMU, Magnetometer, Odometry, Pose, Proximity, Velocity ]

        # Iterate through sensors
        for sensor_type in sensors: 
            sensor = None
            if sensor_type == Camera:
                sensor = sensor_type("cam")
            else:
                sensor = sensor_type()
            topic = sensor.__class__.__name__
            sensor.add_stream('ros', topic=topic)
            robot.append(sensor)

        # Create sandbox environment
        env = Environment('sandbox', fastmode = True)
        env.properties(latitude = -35.0637135, longitude = 150.735166, altitude = 0)
        env.add_service('socket')

    # Start
    def test_init(self):
        print("Initialized")
        return True

# Main function
if __name__ == "__main__":
    from morse.testing.testing import main
    main(RosSceneTest, time_modes = [TimeStrategies.BestEffort])
