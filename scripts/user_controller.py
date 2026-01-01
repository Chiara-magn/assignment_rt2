#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from assignment2_rt.srv import GetAverages, SetThreshold
from assignment2_rt.msg import ObstacleInfo


class UserController(Node):
    def __init__(self):
        super().__init__('user_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.obs_pub = self.create_publisher(ObstacleInfo, '/obstacle_info', 10)

        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.get_avg_client = self.create_client(GetAverages, 'get_averages')
        self.set_thr_client = self.create_client(SetThreshold, 'set_threshold')

        self.threshold = 0.5


    def laser_callback(self, scan):
        front_ranges = scan.ranges[len(scan.ranges)//3 : 2*len(scan.ranges)//3]
        min_front = min(front_ranges)

        info = ObstacleInfo()
        info.min_distance = float(min_front)
        info.threshold = float(self.threshold)
        info.is_obstacle = min_front < self.threshold
        self.obs_pub.publish(info)

        if min_front < self.threshold:
            stop = Twist()
            self.cmd_pub.publish(stop)
        else:
            go = Twist()
            go.linear.x = 0.2
            self.cmd_pub.publish(go)


def main(args=None):
    rclpy.init(args=args)
    node = UserController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()