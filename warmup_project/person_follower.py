import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato2_interfaces.msg import Bump


class PersonFollowerNode(Node):
    """
    Person follower class, tells the neato to ajust motors based on scanned location of human
    """

    def __init__(self):
        """
        Initializes the class
        """

        # initializing the class
        super().__init__("PersonFollowerNode")

        # initializing the values used through the whole node
        self.item_error = 0.0
        self.avg_x = 0.0
        self.avg_y = 0.0
        self.item_distance = 0.0
        self.bumped = False

        # creating the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop())

        # creates subscribers for lidar and bump
        self.scan_subscriber = self.create_subscription(
            LaserScan, "scan", self.get_item_error, 10
        )
        self.bump_subscriber = self.create_subscription(
            Bump, "bump", self.handle_bump, 10
        )

        # creates publisher for the velocity to wheels
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def get_item_error(self, msg: LaserScan):
        """
        handles every laser scan from the subscriber
        """
        scans = np.array(msg.ranges)
        ranges = np.array(range(361))
        x = 0
        d_to_rad = np.pi / 180

        # removes scans that are too far away or returned a 0
        while x < len(scans):
            current_scan = scans[x]
            if current_scan > 1.0 or current_scan <= 0:
                scans = np.delete(scans, x)
                ranges = np.delete(ranges, x)
            else:
                x += 1

        if len(scans) > 0:
            # changes the scans to cartesian coordinates
            x_coordinates = np.multiply(scans, np.cos(np.multiply(ranges, d_to_rad)))
            y_coordinates = np.multiply(scans, np.sin(np.multiply(ranges, d_to_rad)))

            # averages the coordinates to find the center of the item
            self.avg_x = np.average(x_coordinates)
            self.avg_y = np.average(y_coordinates)

            # math to find the distance and angle from the neato to the person
            self.item_distance = np.sqrt(self.avg_x**2 + self.avg_y**2)
            theta = np.arctan2(self.avg_y, self.avg_x)

            self.item_error = theta
        else:
            self.item_error = 0.0

    def handle_bump(self, msg: Bump):
        """
        handles the bump state of the neato
        """
        self.bumped = (
            msg.left_front or msg.right_front or msg.left_side or msg.right_side
        )

    def run_loop(self):
        """
        loop that runs every timer incriment.
        """

        # initializing the vel
        vel = Twist()

        # checks to see if neato has been bumped, if it has been, then stop moving
        if self.bumped:
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        else:
            # checks to see how much of a turn the person is from the neato
            # if the turn is large enough, the linear movement stops, and it only turns
            if abs(self.item_error) > 0.5:
                vel.linear.x = 0.0
            else:
                vel.linear.x = 0.2
            # sets the angular speed to be .2
            vel.angular.z = 0.2
        self.vel_publisher.publish(vel)


def main(args=None):
    """
    main function
    """
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
