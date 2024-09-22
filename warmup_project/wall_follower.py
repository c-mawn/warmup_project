"""
Wall follower module. 

Uses keyboard input (must press enter)

w - forward (and wall follow)
a - left and forward
d - right and forward
s - backward

any other key - stop

"""

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowerNode(Node):
    """
    Node which moves the neato based on keyboard input
    """

    def __init__(self):
        """
        initializes the class
        """
        super().__init__("WallFollowerNode")
        # creates the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # create publisher to tell the motors to move
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription_lidar = self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )
        kthread = KeyboardThread(self.keyboard_input)

        self.angular_vel = 0.0
        self.key_input = ""

    def run_loop(self):
        """
        tells the robot to move when a certain key is pressed
        """
        # series of if statements telling the robot what to do for each key
        vel = Twist()

        if self.key_input == "w":
            vel.linear.x = 0.1
            vel.angular.z = self.angular_vel * 3.0
        elif self.key_input == "s":
            vel.linear.x = -0.2
        elif self.key_input == "a":
            vel.linear.x = 0.1
            vel.angular.z = 0.3
        elif self.key_input == "d":
            vel.linear.x = 0.1
            vel.angular.z = -0.3
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0

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
        scans = np.array(msg.ranges)
        ranges = np.array(range(361))

        # removes scans that are too far away or returned a 0
        outside_bounds = np.where((scans > 0) & (scans < 1.7))
        scans = scans[outside_bounds]
        ranges = ranges[outside_bounds]
        # find the value of walls
        left_wall = np.where(
            (ranges > 80) & (ranges < 100)
        )  # array that show left side
        right_wall = np.where(
            (ranges > 260) & (ranges < 280)
        )  # array that show right side

        # checks for NaN arrays
        if len(left_wall[0]) == 0:
            left_wall_mean = 100
        else:
            left_wall_mean = np.mean(scans[left_wall])

        if len(right_wall[0]) == 0:
            right_wall_mean = 100
        else:
            right_wall_mean = np.mean(scans[right_wall])

        # determine turn velocity
        if left_wall_mean < right_wall_mean:

            # use left wall
            front = np.mean(scans[np.where((ranges > 60) & (ranges < 90))])
            back = np.mean(scans[np.where((ranges > 90) & (ranges < 120))])

            if abs(front - back) > 0.01:
                # front is further
                print("left wall, turn counter-clockwise")
                self.angular_vel = front - back

            else:
                print("Do not turn")
                self.angular_vel = 0.0
        elif left_wall_mean > right_wall_mean:
            # use right wall
            front = np.mean(scans[np.where((ranges > 240) & (ranges < 270))])
            back = np.mean(scans[np.where((ranges > 270) & (ranges < 300))])

            if abs(front - back) > 0.01:
                # front is further
                print("right wall, turn counter-clockwise")
                self.angular_vel = front - back

            else:
                print("Do not turn")
                self.angular_vel = 0.0
        else:
            front, back = 0, 0
            print("Do not turn")
            self.angular_vel = 0.0

        print(front, back, self.angular_vel)


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
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
