#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_rt.srv import GetAverages, SetThreshold


class UserController(Node):
    def __init__(self):
        super().__init__('user_controller')

        # cmd/vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Threshold client
        self.set_thr_client = self.create_client(SetThreshold, 'set_threshold')

        # Average service
        self.avg_srv = self.create_service(GetAverages, 'get_averages', self.get_avg_callback)

        # Last 5 commands
        self.last_velocities = []

        # User Interface menu run
        self.user_interface()


# function user choices robot motion: velocities, direction
    def ask_choice(self):
        choice = {}

        choice["velocity"] = float(input("Choose velocity: "))

        print("Choose the direction where you want to move the robot:")
        print("  0 = linear forward")
        print("  1 = linear backwards")
        print("  2 = rotation left")
        print("  3 = rotation right")
        choice["direction"] = int(input("Direction: "))

        return choice


# User menu interface
    def user_interface(self):
        while rclpy.ok():
            print("1) Move the robot")
            print("2) Change robot threshold to obstacles")
            print("3) Get averages")
            print("4) Quit")

            choice = input("Select option: ")

            if choice == "1":
                c = self.ask_choice()
                msg = Twist()

                if c["direction"] == 0:
                    print(f"You chose to move forward, velocity {c['velocity']}")
                    msg.linear.x = c["velocity"]

                elif c["direction"] == 1:
                    print(f"You chose to move backwards, velocity {c['velocity']}")
                    msg.linear.x = -c["velocity"]

                elif c["direction"] == 2:
                    print(f"You chose to rotate left, angular velocity {c['velocity']}")
                    msg.angular.z = c["velocity"]

                elif c["direction"] == 3:
                    print(f"You chose to rotate right, angular velocity {c['velocity']}")
                    msg.angular.z = -c["velocity"]

                else:
                    print("Invalid direction")
                    continue


def main(args=None):
    rclpy.init(args=args)
    node = UserController()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()