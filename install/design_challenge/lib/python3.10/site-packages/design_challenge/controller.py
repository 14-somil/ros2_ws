#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
import time

class Controller(Node):
    def __init__(self) -> None:
        super().__init__('contoller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.duty_cycle_publisher = self.create_publisher(Float32, '/duty_cycle', qos_profile)

        self.motor_speed_subscriber = self.create_subscription(Float32, '/motor_speed', self.motor_speed_callback, qos_profile)

        self.motor_speed:float = None
        self.reference_speed = 800
        self.kp = 0.001
        self.ki = 0.0001
        self.kd = 0.00001
        self.cumm_err = 0
        self.prev_err = None
        self.duty_cycle:float = 0

        self.timer = self.create_timer(0.0001, self.timer_callback)

    def timer_callback(self) -> None:
        try:
            if self.motor_speed is not None:
                err = self.reference_speed - self.motor_speed
                self.cumm_err += err
                
                self.duty_cycle = self.kp*err + self.ki * self.cumm_err + self.kd * ((err - self.prev_err) if self.prev_err is not None else 0)

                if(self.duty_cycle > 1): self.duty_cycle = 1
                if(self.duty_cycle < 0): self.duty_cycle = 0

                self.prev_err = err

                self.publish_duty_cycle()
        except KeyboardInterrupt:
            pass

    def publish_duty_cycle(self):
        msg = Float32()
        msg.data = float(self.duty_cycle)
        self.duty_cycle_publisher.publish(msg)


    def motor_speed_callback(self, motor_speed) -> None:
        if self.motor_speed is None:
            self.get_logger().info('Motor feedback recieved')
        self.motor_speed = motor_speed.data

def main(args = None) -> None:
    print('Controller running')
    rclpy.init(args=args)

    controller = Controller()
    rclpy.spin(controller)

    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)