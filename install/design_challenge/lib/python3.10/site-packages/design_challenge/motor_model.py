#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from math import pi
import time
import random

class MotorModel(Node):
    def __init__(self) -> None:
        super().__init__('motor_model')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.speed_publisher = self.create_publisher(Float32, '/motor_speed', qos_profile)

        self.duty_cycle_subscriber = self.create_subscription(Float32, '/duty_cycle', self.duty_cycle_callback, qos_profile)

        self.wind_speed_subscriber = self.create_subscription(Float32, '/wind_speed', self.wind_speed_callback, qos_profile)

        self.motor_speed = 0
        self.duty_cycle = 1
        self.wind_speed : float= 3 #m/s
        self.V_battery = 200
        self.V_rated = 180
        self.Ia_rated = 10
        self.motor_speed_rated = 1000
        self.Ra = 0.5
        self.K = self.calculate_K()
        self.Ia = self.calculate_Ia()
        self.K_drag = 600
        self.rotor_radius = 0.15
        self.prev_time = time.time()
        self.noise_std_dev = 2

        self.timer = self.create_timer(0.01, self.timer_callback)

    def calculate_K(self) -> float:
        E = self.V_rated - self.Ia_rated * self.Ra
        motor_speed_rated_rad = self.motor_speed_rated * 2 * pi / 60
        return E/motor_speed_rated_rad
    
    def calculate_Ia(self) -> float:
        E = self.motor_speed * self.K * 2 * pi / 60
        return (self.duty_cycle * self.V_battery - E) / self.Ra

    def duty_cycle_callback(self, duty_cycle) -> None:
        # self.get_logger().info(f'Duty cycle: {duty_cycle.data}')
        self.duty_cycle = duty_cycle.data 
    
    def wind_speed_callback(self, msg) -> None:
        self.wind_speed = msg.data
    
    def timer_callback(self) -> None:
        self.Ia = self.calculate_Ia()
        torque_motor = self.K * self.Ia
        torque_load = (self.K_drag * pow(float(self.wind_speed), 3.0) / self.motor_speed) if self.motor_speed != 0 else (self.K_drag * pow(float(self.wind_speed), 2.0) * self.rotor_radius)

        dw_dt = torque_motor - torque_load
        self.motor_speed += dw_dt * 60 * (time.time() - self.prev_time) / (2*pi)
        # self.motor_speed += random.gauss(0, self.noise_std_dev)
        self.prev_time = time.time()

        self.publish_motor_speed()
    
    def publish_motor_speed(self) ->None:
        # self.get_logger().info(f'Publishing: {self.motor_speed}')
        msg = Float32()
        msg.data = self.motor_speed
        self.speed_publisher.publish(msg)

def main(args=None) -> None:
    print('Motor turned on...')
    rclpy.init(args=args)

    motor_model = MotorModel()
    rclpy.spin(motor_model)

    motor_model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)