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

        self.beta = 0.5
        self.field_effect = 0.3  # when you enter the field

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

        vel.angular.z = self.angular_velocity
        vel.linear.x = 0.4
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
        outside_bounds = np.where((distances > 0) & (distances < self.field_effect))
        distances = distances[outside_bounds]
        angles = angles[outside_bounds]

        # reduce scans to a 60 degree sweep, spread over 15 points
        angles = angles[:-1:4]
        inside_bounds = np.where((angles > 330) | (angles < 30))
        distances = distances[inside_bounds]
        angles = angles[inside_bounds]

        self.obstacle_delta_x, self.obstacle_delta_y = 0, 0

        for d, theta in zip(distances, angles):
            self.obstacle_delta_x += (
                -self.beta * (self.field_effect - d) * math.cos(math.radians(theta))
            )
            self.obstacle_delta_y += (
                -self.beta * (self.field_effect - d) * math.sin(math.radians(theta))
            )

        if not self.obstacle_delta_x:
            self.obstacle_delta_x = 1e6
        if not self.obstacle_delta_y:
            self.obstacle_delta_y = 1e6
        self.obstacle_delta_x = self.obstacle_delta_x / (
            self.obstacle_delta_x + self.obstacle_delta_y
        )
        self.obstacle_delta_y = self.obstacle_delta_y / (
            self.obstacle_delta_x + self.obstacle_delta_y
        )

        self.angular_velocity = (
            math.atan2(self.obstacle_delta_y, self.obstacle_delta_x) * 0.4
        )


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
