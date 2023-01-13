from sensor_msgs.msg import Imu
from morse.middleware.ros import ROSPublisher

class ImuPublisher(ROSPublisher):
    """ Publish the data of the IMU sensor (without covariance). """
    ros_class = Imu
    default_frame_id = '/imu'

    def default(self, ci='unused'):
        # pass
        imu = Imu()
        imu.header = self.get_ros_header()

        quaternion = self.component_instance.bge_object.worldOrientation.to_quaternion()
        imu.orientation.x = quaternion.x 
        imu.orientation.y = quaternion.y
        imu.orientation.z = quaternion.z
        imu.orientation.w = quaternion.w 

        imu.angular_velocity.x = self.data['angular_velocity'][0]
        imu.angular_velocity.y = self.data['angular_velocity'][1]
        imu.angular_velocity.z = self.data['angular_velocity'][2]

        imu.linear_acceleration.x = self.data['linear_acceleration'][0]
        imu.linear_acceleration.y = self.data['linear_acceleration'][1]
        imu.linear_acceleration.z = self.data['linear_acceleration'][2]

        self.publish(imu)
