#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist  ## cmd_vel topic accepts Twist type messages


class MoveRobot(Node):

    def __init__(self):

        super().__init__('move_robot')

        self.counter = 0  ## Counter variable to count the number of timer callbacks
        self.counter_period = 5  ## Change the direction every 5 count
        self.switch_direction = False  ## If this is True, the robot will change direction (forward or backward)
        self.vel_step = 0.1  ## Movement speed of the robot
        self.twist_cmd = Twist()

        self.cmd_vel_pub = self.create_publisher(

            msg_type=Twist,

            topic='/cmd_vel',

            qos_profile=1)
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback) ## The timer interrupt. timer_callback function is called every self.timer_period seconds.

    def timer_callback(self, event):
        if self.counter % self.counter_period == 0:
            self.switch_direction = True
        else:
            self.switch_direction = False
        self.move_robot()
        self.counter += 1

    def move_robot(self):
        self.vel_step = -self.vel_step if self.switch_direction else self.vel_step
        self.twist_cmd.linear.x = self.vel_step
        self.twist_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_cmd)


def main(args=None):

    try:
        rclpy.init(args=args)

        move_robot = MoveRobot()
        rclpy.spin(move_robot)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()