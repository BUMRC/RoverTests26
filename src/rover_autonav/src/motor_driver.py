#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from enum import Enum
import numpy as np
import signal
import sys

import adafruit_pca9685
import adafruit_extended_bus


class MotorState(Enum):
    STOPPED = 0
    RUNNING = 1
    ERROR = 2


class Side(Enum):
    LEFT = 0
    RIGHT = 1


# Pin mapping: motor_id -> (reverse_pin, forward_pin)
MOTOR_PIN_CONFIG = {
    0: (0, 6),
    1: (7, 1),
    2: (2, 8),
    3: (9, 3),
    4: (4, 10),
    5: (11, 5),
}

MOTOR_CONFIG = {
    'max_output': int(65535 * 0.5),
    'minimum_output': int(65535 * 0.10),
    'max_acceleration': 65535 * 0.50,  # Faster ramp: 50% per second (was 25%)
    'control_frequency': 50,
    'timeout': 1.0,  # Increased from 0.5s to tolerate cmd_vel jitter
}


class MotorNode(Node):
    def __init__(self, motor_id, shield):
        super().__init__(f'motor_node_{motor_id}')

        self.motor_id = motor_id
        self.side = Side(motor_id % 2)
        self.shield = shield

        self.state = MotorState.STOPPED
        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.last_command_time = self.get_clock().now()

        self.velocity_subscriber = self.create_subscription(
            Float64,
            f'/motor_{self.motor_id}/target_velocity',
            self.velocity_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            Float64,
            f'/motor_{self.motor_id}/current_velocity',
            10
        )

        self.control_loop_timer = self.create_timer(
            1.0 / MOTOR_CONFIG['control_frequency'],
            self.control_loop
        )

        self.init_hardware()
        self.get_logger().info(f'Motor {motor_id} ({self.side.name}) initialized')

    def init_hardware(self):
        try:
            pins = MOTOR_PIN_CONFIG[self.motor_id]
            self.reverse_pin = self.shield.channels[pins[0]]
            self.forward_pin = self.shield.channels[pins[1]]
            self.get_logger().info(
                f'Motor {self.motor_id}: fwd_pin={pins[1]}, rev_pin={pins[0]}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to init motor hardware: {e}')
            self.state = MotorState.ERROR

    def velocity_callback(self, msg: Float64):
        self.target_velocity = msg.data
        self.last_command_time = self.get_clock().now()

    def ramp_velocity(self, current, target):
        dt = 1.0 / MOTOR_CONFIG['control_frequency']
        max_change = MOTOR_CONFIG['max_acceleration'] * dt

        error = target - current
        if abs(error) <= max_change:
            return target
        return current + np.sign(error) * max_change

    def set_motor_output(self, output):
        try:
            max_out = MOTOR_CONFIG['max_output']
            output = int(np.clip(output, -max_out, max_out))

            # Only apply dead zone when decelerating toward zero, not during ramp-up.
            # This prevents the stutter where motors repeatedly enter/exit the dead zone.
            if abs(output) < MOTOR_CONFIG['minimum_output']:
                if abs(self.target_velocity) < MOTOR_CONFIG['minimum_output']:
                    output = 0
                # else: let the small output through during ramp-up

            if output > 0:
                self.forward_pin.duty_cycle = output
                self.reverse_pin.duty_cycle = 0
            elif output < 0:
                self.forward_pin.duty_cycle = 0
                self.reverse_pin.duty_cycle = -output
            else:
                self.forward_pin.duty_cycle = 0
                self.reverse_pin.duty_cycle = 0

            self.state = MotorState.RUNNING if output != 0 else MotorState.STOPPED

        except Exception as e:
            self.get_logger().error(f'Motor {self.motor_id} output error: {e}')
            self.state = MotorState.ERROR

    def control_loop(self):
        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if elapsed > MOTOR_CONFIG['timeout']:
            if self.target_velocity != 0.0:
                self.get_logger().warn(
                    f'Motor {self.motor_id}: no command for {elapsed:.1f}s, stopping'
                )
            self.target_velocity = 0.0

        self.current_velocity = self.ramp_velocity(
            self.current_velocity, self.target_velocity
        )
        self.set_motor_output(self.current_velocity)

        status_msg = Float64()
        status_msg.data = self.current_velocity
        self.status_publisher.publish(status_msg)

    def stop(self):
        self.target_velocity = 0.0
        self.current_velocity = 0.0
        self.set_motor_output(0)


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.motor_publishers = [
            self.create_publisher(Float64, f'/motor_{i}/target_velocity', 10)
            for i in range(6)
        ]

        self.motor_velocities = [0.0] * 6

        # Keep-alive: republish last velocities at 20Hz so motors don't timeout
        # between cmd_vel messages from Nav2
        self.keepalive_timer = self.create_timer(0.05, self.republish_velocities)

        self.get_logger().info('MotorController started')

    def cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential mixing: left motors 0,2,4 — right motors 1,3,5
        left = np.clip(linear + angular, -1.0, 1.0) * 65535
        right = np.clip(linear - angular, -1.0, 1.0) * 65535

        for i in range(6):
            self.motor_velocities[i] = float(int(left if i % 2 == 0 else right))

        self.publish_all()

    def publish_all(self):
        for i in range(6):
            msg = Float64()
            msg.data = self.motor_velocities[i]
            self.motor_publishers[i].publish(msg)

    def republish_velocities(self):
        """Republish current velocities to keep motors alive between cmd_vel."""
        self.publish_all()


def main():
    rclpy.init()

    i2c = adafruit_extended_bus.ExtendedI2C(15)
    shield = adafruit_pca9685.PCA9685(i2c, address=0x40)
    shield.frequency = 250

    controller = MotorController()
    motors = [MotorNode(motor_id, shield) for motor_id in range(6)]

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    for motor in motors:
        executor.add_node(motor)

    def shutdown(*_):
        # Stop all motors via node method (uses correct pins)
        for motor in motors:
            motor.stop()

        # Also directly zero all known pins as safety fallback
        for motor_id, (rev_pin, fwd_pin) in MOTOR_PIN_CONFIG.items():
            try:
                shield.channels[fwd_pin].duty_cycle = 0
                shield.channels[rev_pin].duty_cycle = 0
            except Exception:
                pass

        shield.deinit()

        for motor in motors:
            motor.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == '__main__':
    main()
