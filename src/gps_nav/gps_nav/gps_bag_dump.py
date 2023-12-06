import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import utm

class GPS_ODO_Subscribe(Node):
    def __init__(self):
        super().__init__('pose_converter')

        # Reset the files
        open("gps_data.txt", "w").close()

        self.gps_subscription = self.create_subscription(NavSatFix,'gps',self.gps_callback,10)

        self.gps_subscription

        self.x = 0.0
        self.y = 0.0
        self.first_val = True
        self.offset = [0.0, 0.0]

    def gps_callback(self, msg):
        self.x, self.y, zone_number, zone_letter = utm.from_latlon(msg.latitude, msg.longitude)
        
        with open("gps_data.txt", "a") as gps_file:
            gps_file.write(str(self.x) + "," + str(self.y) + "\n")

        


def main(args=None):
    rclpy.init(args=args)

    gps_odo_subscriber = GPS_ODO_Subscribe()

    rclpy.spin(gps_odo_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_odo_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()