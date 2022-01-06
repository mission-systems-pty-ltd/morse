from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from morse.middleware.ros import ROSPublisher

class DvlPublisher(ROSPublisher):
    """ Publish the data of the IMU sensor (without covariance). """
    ros_class = Imu
    default_frame_id = '/dvl'

    def default(self, ci='unused'):
        imu = Imu()
        imu.header = self.get_ros_header()

        imu.orientation = self.component_instance.bge_object.worldOrientation.to_quaternion()

        imu.angular_velocity.x = self.data['angular_velocity'][0]
        imu.angular_velocity.y = self.data['angular_velocity'][1]
        imu.angular_velocity.z = self.data['angular_velocity'][2]

        imu.linear_acceleration.x = self.data['linear_acceleration'][0]
        imu.linear_acceleration.y = self.data['linear_acceleration'][1]
        imu.linear_acceleration.z = self.data['linear_acceleration'][2]

        self.publish(imu)

class DvlTwistPublisher(ROSPublisher):
    ros_class = TwistWithCovarianceStamped
    default_frame_id = '/dvl'

    def default(self, ci='unused'):
        msg = TwistWithCovarianceStamped()
        msg.header = self.get_ros_header()

        msg.twist.twist.linear.x = self.data['Bx']
        msg.twist.twist.linear.y = self.data['By']
        msg.twist.twist.linear.z = self.data['Bz']

        msg.twist.twist.angular.x = self.data['Bu']
        msg.twist.twist.angular.y = self.data['Bv']
        msg.twist.twist.angular.z = self.data['Bw']

        self.publish(msg)
