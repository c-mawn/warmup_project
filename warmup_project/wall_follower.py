import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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
        # creates a publisher to see what key is being pressed, used for debugging
        self.publisher_key = self.create_publisher(String, "key", 10)

    def run_loop(self):
        """
        tells the robot to move when a certain key is pressed
        """
        # series of if statements telling the robot what to do for each key
        vel = Twist()
        scan = LaserScan()

        # check which wall is further
        # incremental increase
        if sum(scan.ranges[80:100]) > sum(scan.ranges[260:280]):
            # left wall is closer
            # angular velocity = 0.1 * bigger/smaller
            # this means that the greater the ratio, the faster the turn
            #
            front = sum(scan.ranges[60:90])
            back = sum(scan.ranges[90:120])
            if front > back:
                vel.angular.z = 0.1 * front // back
            else:
                vel.angular.z = -0.1 * back // front
        else:
            # right wall is closer
            front = sum(scan.ranges[270:300])
            back = sum(scan.ranges[240:270])
            if front > back:
                vel.angular.z = -0.1 * front // back
            else:
                vel.angular.z = 0.1 * back // front

        if self.getKey() == "w":
            vel.linear.x = 0.1
        elif self.getKey() == "/x03":
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            self.publisher.publish(vel)
            rclpy.shutdown()
        else:
            # ALTERNATIVE METHOD we can update the position here
            vel.linear.x = 0.0
            vel.angular.z = 0.0

        self.publisher.publish(vel)
        key_data = String()
        key_data.data = self.getKey()
        self.publisher_key.publish(key_data)

    settings = termios.tcgetattr(sys.stdin)
    key = None

    def getKey(self):
        """
        Gets and returns the current key input
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


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