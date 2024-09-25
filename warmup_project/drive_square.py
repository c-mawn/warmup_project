from time import time
from math import pi
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DriveSquareNode(Node):
    """
    Node that tells the neato to drive in a 1m by 1m square
    """

    def __init__(self):
        """
        Initializes the class
        """
        super().__init__("DriveSquareNode")

        # creating the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)
        # creates a publisher that tells the neato its wheel velocities
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.start_time = time()

    def run_loop(self):
        """
        tells the robot to move
        """

        vel = Twist()
        # publish the move forward command for a set amount of time, then a turn command and repeat till theres a square
        current_time = time()
        delta_time = current_time - self.start_time
        if delta_time < 10:
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
        elif delta_time > 10 and delta_time < 20:
            vel.linear.x = 0.0
            vel.angular.z = pi / 19
            self.vel_publisher.publish(vel)
        elif delta_time > 20 and delta_time < 30:
            vel.angular.z = 0.0
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
        elif delta_time > 30 and delta_time < 40:
            vel.linear.x = 0.0
            vel.angular.z = pi / 19
            self.vel_publisher.publish(vel)
        elif delta_time > 40 and delta_time < 50:
            vel.angular.z = 0.0
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
        elif delta_time > 50 and delta_time < 60:
            vel.linear.x = 0.0
            vel.angular.z = pi / 19
            self.vel_publisher.publish(vel)
        elif delta_time > 60 and delta_time < 70:
            vel.angular.z = 0.0
            vel.linear.x = 0.1
            self.vel_publisher.publish(vel)
        else:
            vel.angular.z = 0.0
            vel.linear.x = 0.0
            self.vel_publisher.publish(vel)


def main(args=None):
    """
    main function
    """
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
