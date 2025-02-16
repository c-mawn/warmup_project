"""
obstacle_avoider

Uses keyboard input (must press enter)

w - forward (and wall follow)
a - left and forward
d - right and forward
s - backward

any other key - stop


* Non-blocking keyboard input from https://stackoverflow.com/questions/2408560/non-blocking-console-input

"""

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
        self.angular_velocity_history = []
        self.linear_velocity_history = []

        self.angular_velocity = 0.0
        self.linear_velocity_scale = 1.0

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

        if "w" in self.key_input:
            vel.angular.z = self.angular_velocity * 0.5
            vel.linear.x = 0.4 * self.linear_velocity_scale
        elif "s" in self.key_input:
            vel.linear.x = -0.2
            self.reset_history()
        elif "a" in self.key_input:
            vel.angular.z = 0.8
            self.reset_history()
        elif "d" in self.key_input:
            vel.angular.z = -0.8
            self.reset_history()
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.reset_history()

        self.publisher.publish(vel)

    def reset_history(self):
        """
        Reset the angular and linear velocity history
        """
        self.angular_velocity_history = []
        self.linear_velocity_history = []

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

        # angular velocity is the difference between the left side laser scans and the right side laser scans
        self.angular_velocity = -(np.sum(distances[left_bounds])) + (
            np.sum(distances[right_bounds])
        )
        self.linear_velocity_scale = 0.001 / abs(self.angular_velocity + 0.001)

        # Normal to the obstacle
        if np.mean(distances[left_bounds]) - np.mean(distances[right_bounds]) < 0.1:
            self.angular_velocity = 2.0
            self.linear_velocity_scale = 0.4

        # Record angular and linear velocity to history
        if self.angular_velocity != 0:
            self.angular_velocity_history = [
                self.angular_velocity
            ] + self.angular_velocity_history
            self.linear_velocity_history = [
                self.linear_velocity_scale
            ] + self.linear_velocity_history

        # Checks if the robot is going straight, and there is an angular velocity history
        if (self.angular_velocity == 0) & (len(self.angular_velocity_history) > 0):
            # checks if the robot has passed the obstacle
            if all(
                [
                    len(np.where((angles >= 60) & (angles <= 80))[0]) < 3,
                    len(np.where((angles >= 100) & (angles <= 300))[0]) < 3,
                ]
            ):
                # sets the angular velocity to `the most recent velocity` * -1
                self.angular_velocity = -self.angular_velocity_history[0]
                self.linear_velocity_scale = self.linear_velocity_history[0]

                self.angular_velocity_history = self.angular_velocity_history[1:]
                self.linear_velocity_history = self.linear_velocity_history[1:]


class KeyboardThread(threading.Thread):
    """
    Keyboard input as a thread (uses python input())
    Waits for (enter)
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
