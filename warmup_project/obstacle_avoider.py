"""
Wall follower module. 

Uses keyboard input (must press enter)

w - forward (and wall follow)
a - left and forward
d - right and forward
s - backward

any other key - stop

"""

import math
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoiderNode(Node):
    """
    Node which moves the neato based on keyboard input
    """

    def __init__(self):
        """
        initializes the class
        """
        super().__init__("ObstacleAvoiderNode")

        # initializing the values used through the whole node
        self.goal_delta_x = 1.0
        self.goal_delta_y = 0.0

        self.obstacle_delta_x = 0.0
        self.obstacle_delta_y = 0.0

        self.angular_velocity = 0.0
        self.linear_velocity_scale = 1.0

        self.beta = 0.5
        self.detection_field = 0.8  # when you enter the field

        self.key_input = ""

        # creates the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # create publisher to tell the motors to move
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription_lidar = self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )
        self.kthread = KeyboardThread(self.keyboard_input)

    def run_loop(self):
        """
        tells the robot to move when a certain key is pressed
        """
        # series of if statements telling the robot what to do for each key
        vel = Twist()

        vel.angular.z = self.angular_velocity * 0.5
        vel.linear.x = 0.4 * self.linear_velocity_scale
        self.publisher.publish(vel)

    def keyboard_input(self, inp):
        """
        Callback for Keyboard Input
        """
        self.key_input = inp

    def process_scan(self, msg: LaserScan):
        """
        Callback for LaserScan
        Changes the value of self.angular_vel according to the detected wall
        """
        distances = np.array(msg.ranges)
        angles = np.array(range(361))

        # removes scans that are too far away or returned a 0
        outside_bounds = np.where((distances > 0) & (distances < self.detection_field))
        distances = distances[outside_bounds]
        angles = angles[outside_bounds]

        # reduce scans to a 60 degree sweep, spread over 15 points
        angles = angles[:-1:4]
        left_bounds = np.where(angles < 30)
        right_bounds = np.where(angles > 330)

        self.angular_velocity = -(np.sum(distances[left_bounds])) + (
            np.sum(distances[right_bounds])
        )
        self.linear_velocity_scale = 0.001 / abs(self.angular_velocity + 0.001)

        if np.mean(distances[left_bounds]) - np.mean(distances[right_bounds]) < 0.1:
            # left side is further, or has more points, turn clockwise
            self.angular_velocity = 2.0
            self.linear_velocity_scale = 0.5

        print(self.angular_velocity, self.linear_velocity_scale)


class KeyboardThread(threading.Thread):
    """
    Keybaord input as a thread (uses python input())
    """

    def __init__(self, input_cbk=None, name="keyboard-input-thread"):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name, daemon=True)
        self.start()

    def run(self):
        while True:
            self.input_cbk(input())  # waits to get input + Return


def main(args=None):
    """
    main function
    """
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
