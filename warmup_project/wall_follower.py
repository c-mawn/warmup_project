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
from visualization_msgs.msg import Marker


class WallFollowerNode(Node):
    """
    Node which moves the neato based on keyboard input
    """

    def __init__(self):
        """
        initializes the class
        """
        super().__init__("WallFollowerNode")
        self.left_wall_mean = 0
        self.right_wall_mean = 0
        # creates the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # create publisher to tell the motors to move
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.viz_pub = self.create_publisher(Marker, "marker", 10)
        self.subscription_lidar = self.create_subscription(
            LaserScan, "scan", self.process_scan, 10
        )
        self.kthread = KeyboardThread(self.keyboard_input)

        self.angular_vel = 0.0
        self.key_input = ""

    def run_loop(self):
        """
        tells the robot to move when a certain key is pressed
        """
        # series of if statements telling the robot what to do for each key
        vel = Twist()

        if "w" in self.key_input:
            vel.linear.x = 0.1
            vel.angular.z = self.angular_vel * 3.0
        elif "s" in self.key_input:
            vel.linear.x = -0.2
        elif "a" in self.key_input:
            vel.linear.x = 0.1
            vel.angular.z = 0.3
        elif "d" in self.key_input:
            vel.linear.x = 0.1
            vel.angular.z = -0.3
        else:
            vel.linear.x = 0.0
            vel.angular.z = 0.0

        marker = Marker()

        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        if self.right_wall_mean < self.left_wall_mean:
            marker.pose.position.x = 0.0
            marker.pose.position.y = -float(self.right_wall_mean)
        else:
            marker.pose.position.x = 0.0
            marker.pose.position.y = float(self.left_wall_mean)

        self.viz_pub.publish(marker)

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
        outside_bounds = np.where((distances > 0) & (distances < 1.7))
        distances = distances[outside_bounds]
        angles = angles[outside_bounds]
        # find the value of walls
        left_wall = np.where(
            (angles > 80) & (angles < 100)
        )  # array that show left side
        right_wall = np.where(
            (angles > 260) & (angles < 280)
        )  # array that show right side

        # checks for NaN arrays
        if len(left_wall[0]) == 0:
            self.left_wall_mean = 100
        else:
            self.left_wall_mean = np.mean(distances[left_wall])

        if len(right_wall[0]) == 0:
            self.right_wall_mean = 100
        else:
            self.right_wall_mean = np.mean(distances[right_wall])

        # determine turn velocity
        if self.left_wall_mean < self.right_wall_mean:

            # use left wall
            front = np.mean(distances[np.where((angles > 60) & (angles < 90))])
            back = np.mean(distances[np.where((angles > 90) & (angles < 120))])

            if math.isnan(front):
                front = 100
            if math.isnan(back):
                back = 100

            if abs(front - back) > 0.01:
                # sets the angular velocity to the difference between the front and back average distance
                print("left wall, turn counter-clockwise")
                self.angular_vel = front - back

            else:
                print("Do not turn")
                self.angular_vel = 0.0
        elif self.left_wall_mean > self.right_wall_mean:
            # use right wall
            front = np.mean(distances[np.where((angles > 240) & (angles < 270))])
            back = np.mean(distances[np.where((angles > 270) & (angles < 300))])

            if math.isnan(front):
                front = 100
            if math.isnan(back):
                back = 100

            if abs(front - back) > 0.01:
                # sets the angular velocity to the difference between the front and back average distance
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
