#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuCovarianceNode(Node):
    """Pass IMU data through unchanged while replacing covariance matrices."""

    def __init__(self):
        super().__init__('imu_covariance_node')

        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_cov')

        # Actual measured/tuned covariance values belong in the YAML file.
        # These zero arrays only declare the parameters.
        self.declare_parameter('orientation_covariance', [0.0] * 9)
        self.declare_parameter('angular_velocity_covariance', [0.0] * 9)
        self.declare_parameter('linear_acceleration_covariance', [0.0] * 9)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.orientation_covariance = self._get_required_covariance(
            'orientation_covariance'
        )
        self.angular_velocity_covariance = self._get_required_covariance(
            'angular_velocity_covariance'
        )
        self.linear_acceleration_covariance = self._get_required_covariance(
            'linear_acceleration_covariance'
        )

        self.publisher = self.create_publisher(
            Imu,
            self.output_topic,
            10
        )

        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f'Injecting IMU covariances: '
            f'{self.input_topic} -> {self.output_topic}'
        )

        self.get_logger().info(
            f'orientation_covariance = {self.orientation_covariance}'
        )
        self.get_logger().info(
            f'angular_velocity_covariance = '
            f'{self.angular_velocity_covariance}'
        )
        self.get_logger().info(
            f'linear_acceleration_covariance = '
            f'{self.linear_acceleration_covariance}'
        )

    def _get_required_covariance(self, parameter_name):
        """Read and validate a 3x3 covariance parameter."""

        values = list(
            self.get_parameter(parameter_name).value
        )

        if len(values) != 9:
            raise ValueError(
                f'Parameter "{parameter_name}" must contain exactly '
                f'9 values; got {len(values)}.'
            )

        values = [
            float(value)
            for value in values
        ]

        if all(value == 0.0 for value in values):
            raise ValueError(
                f'Parameter "{parameter_name}" is all zeros. '
                f'Load the covariance values from imu_covariance.yaml.'
            )

        # Covariance diagonal entries are variances and cannot be negative.
        for index in (0, 4, 8):
            if values[index] < 0.0:
                raise ValueError(
                    f'Parameter "{parameter_name}" has a negative '
                    f'diagonal variance at index {index}: '
                    f'{values[index]}'
                )

        return values

    def imu_callback(self, msg):
        """Republish the IMU message unchanged except for covariance."""

        msg.orientation_covariance = self.orientation_covariance
        msg.angular_velocity_covariance = (
            self.angular_velocity_covariance
        )
        msg.linear_acceleration_covariance = (
            self.linear_acceleration_covariance
        )

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = ImuCovarianceNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
