import math
import numpy as np


import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from gps_nav_interfaces.msg import CurrentGoalPose

from gps_nav.uf_support.route_support import determine_arc_radius

class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")

        self.declare_parameter("L_wheelbase_m", 0.33)

        self.subscription1 = self.create_subscription(PoseStamped, "vehicle_pose", self.vehicle_pose_callback, 1)

        self.subscription2 = self.create_subscription(
            CurrentGoalPose, "current_goal_pose", self.current_goal_pose_callback, 1
        )

        self.subscription3 = self.create_subscription(Int8, "e_stop", self.e_stop_callback, 10)

        self.publisher = self.create_publisher(AckermannDriveStamped, "vehicle_command_ackermann", 10)

        # set up the timer (0.1 sec) to send over the current_carrot message to the vehicle controller
        self.main_timer = self.create_timer(timer_period_sec=0.1, callback=self.main_timer_callback)

        # define the variables that will store the data from the two message inputs
        self.current_goal_point = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad = 0.0
        self.closest_point = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad = 0.0
        self.speed = 0.0
        self.state = 0.0
        self.error = 0.0
        self.iteration = 0


        self.vehicle_point = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad = 0.0

        self.pause = False
        self.last_pause_value = False

        self.have_vehicle_pose = False
        self.have_goal_pose = False

        # reset the tracker file
        open("circ_arc_crosstrack_error.csv", "w").close()


    def vehicle_pose_callback(self, msg):
        

        self.vehicle_point[0] = msg.pose.position.x
        self.vehicle_point[1] = msg.pose.position.y
        self.vehicle_point[2] = 0.0
        self.vehicle_heading_rad = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        
        self.have_vehicle_pose = True

    def current_goal_pose_callback(self, msg):  

        self.current_goal_point[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point[2] = 0.0
        self.current_goal_heading_rad = 2.0 * math.atan2(
            msg.current_goal_pose.pose.orientation.z, msg.current_goal_pose.pose.orientation.w
        )

        self.closest_point[0] = msg.closest_pose.pose.position.x
        self.closest_point[1] = msg.closest_pose.pose.position.y
        self.closest_point[2] = 0.0
        self.closest_heading_rad = 2.0 * math.atan2(
            msg.closest_pose.pose.orientation.z, msg.closest_pose.pose.orientation.w
        )

        self.speed = msg.speed
        self.state = msg.state

        if self.speed > 2.5:
            self.speed = 2.5
        elif self.speed < -2.5:
            self.speed = -2.5

        self.have_goal_pose = True

    def e_stop_callback(self, msg):
        if msg.data == 0:
            self.pause = True
        elif msg.data == 1:
            self.pause = False

    def main_timer_callback(self):

        

        if self.pause == True:
            # send out a zero velocity Ackermann message
            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = 0.0
            out_msg.drive.steering_angle = 0.0

            self.publisher.publish(out_msg)
            return

        # This will only publish after each subscription has occured at least once
        if self.have_goal_pose and self.have_vehicle_pose:

            # keep an eye on the crosstrack error
            error = math.sqrt((self.vehicle_point[0] - self.closest_point[0])**2 + (self.vehicle_point[1] - self.closest_point[1])**2)
            with open("circ_arc_crosstrack_error.csv", "a") as file:
                file.write(str(self.iteration) + "," + str(error) + "\n")
            self.iteration += 1

            radius_of_curvature = determine_arc_radius(self.vehicle_point, self.vehicle_heading_rad, self.current_goal_point)

            self.L_wheelbase_m = self.get_parameter("L_wheelbase_m").value

            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = self.speed
            angle_rad = math.atan(self.L_wheelbase_m / radius_of_curvature)
            out_msg.drive.steering_angle = angle_rad

            self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    vehicle_controller = VehicleController()

    rclpy.spin(vehicle_controller)

    vehicle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
