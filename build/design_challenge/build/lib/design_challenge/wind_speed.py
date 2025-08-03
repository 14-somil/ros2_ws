#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
import time
import math

class WindSpeed(Node):
    def __init__(self) -> None:
        super().__init__('wind_speed')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.wind_speed_publisher = self.create_publisher(Float32, '/wind_speed', qos_profile)

        self.counter = 0
        self.wind_speed = 0

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self) -> None:
        self.wind_speed = 2* math.sin(self.counter * math.pi / (2 * 100)) + 2
        self.counter += 1
        self.publish_wind_speed()

    def publish_wind_speed(self):
        msg = Float32()
        msg.data = float(self.wind_speed)
        self.wind_speed_publisher.publish(msg)



def main(args = None) -> None:
    print('Windspeed running')
    rclpy.init(args=args)

    wind_speed = WindSpeed()
    rclpy.spin(wind_speed)

    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)