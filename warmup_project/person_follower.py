import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


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

        # creating the timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop())

        # creates publishers for the node
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # creates subscriber for the lidar scanner
        self.scan_subscriber = self.create_subscription(
            LaserScan, "scan", self.handle_scan, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, "odom", self.handle_odom, 10
        )
        self.scan = LaserScan()
        self.odom = Odometry()

    def handle_scan(self, msg: LaserScan):
        """
        handles every laser scan from the subscriber
        """
        self.scan = msg

    def handle_odom(self, msg: Odometry):
        """
        handles the odometry of the neato at every scan time
        """
        self.odom = msg

    def run_loop(self):
        """
        loop that runs every timer incriment.
        """
        # Take scan, get edges of items in list
        # for example, if there are zeros through index 0-25,
        # then values from 26-30, then zeros for 31-50, i want to
        # isolate the 26-30 indicies
        # then get the angle compared to the neato of where those values are
        # for example, if the neato is pointing at 0 rad, and the item is
        # reading at -pi/4 rad from it, I want to turn the neato towards the item
        # then move forward at one speed, for the distance that the item is away

        # the way i have built this means that if the lidar scan is take close to a wall,
        # the neato will go towards that... more specifically, the first item that the neato
        # sees on the scan will be moved towards. I could probably do a ton of math to
        # ignore things that are straight lines, but id like ot enjoy my life instead
        # tldr: dont run this with other items around... only human

        scan_ranges = self.scan.ranges
        item_start_index = 0
        item_stop_index = 0
        for scan_range in scan_ranges:
            if scan_range != 0:
                item_start_index = scan_ranges.index(scan_range)
                break
        for scan_range in reversed(scan_ranges):
            if scan_range != 0:
                item_stop_index = scan_ranges.index(scan_range)
                break
        # seperates the item from the zeros
        item_range = scan_ranges[item_start_index:item_stop_index]
        # gets the index of the center of the item
        item_center = int(len(item_range) / 2)

        # calculates the angle of this center index in the neato frame
        orientation = self.odom.pose.pose.orientation
        [_, _, neato_theta] = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        item_theta = None
        delta_theta = None  # theta of the neato - theta of center of object


# some fucntions for quaternion to euler stuff
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians
