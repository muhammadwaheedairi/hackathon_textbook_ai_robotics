---
title: "IMU Sensors and ROS 2 Integration"
sidebar_label: "Chapter 2: IMU Sensors"
sidebar_position: 2
---

# Chapter 2: IMU Sensors and ROS 2 Integration

## Overview

This chapter explores Inertial Measurement Units (IMUs) and their integration with ROS 2 for robot perception. You'll learn how IMUs measure orientation and motion, and how to combine LIDAR and IMU data for comprehensive environmental awareness.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Understand IMU components and their functions
- Process IMU data in ROS 2 applications
- Integrate multiple sensor types for sensor fusion
- Implement basic sensor fusion for obstacle detection
:::

## IMU Sensors

Inertial Measurement Units (IMUs) combine accelerometers, gyroscopes, and sometimes magnetometers to measure the robot's orientation, velocity, and gravitational forces.

### IMU Components

- **Accelerometer**: Measures linear acceleration
- **Gyroscope**: Measures angular velocity
- **Magnetometer**: Measures magnetic field orientation (compass)

### IMU Applications in Robotics

- Robot pose estimation
- Motion tracking and control
- Stabilization systems
- Dead reckoning navigation

## ROS 2 Integration

The Robot Operating System 2 (ROS 2) provides standardized interfaces for sensor integration, making it easier to work with LIDAR and IMU sensors in robotics applications.

### Common Sensor Message Types

- `sensor_msgs/LaserScan`: For LIDAR data
- `sensor_msgs/Imu`: For IMU data
- `sensor_msgs/PointCloud2`: For 3D point cloud data

## Code Examples

### IMU Data Processing

Here's a ROS 2 Python node for processing IMU data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.subscription
        self.get_logger().info('IMU Subscriber node initialized')

    def imu_callback(self, msg):
        """Process incoming IMU data"""
        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)

        angular_velocity = msg.angular_velocity
        linear_acceleration = msg.linear_acceleration

        self.get_logger().info(
            f'Orientation - Roll: {math.degrees(roll):.2f}°, '
            f'Pitch: {math.degrees(pitch):.2f}°, '
            f'Yaw: {math.degrees(yaw):.2f}°'
        )

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()

    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        imu_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic Sensor Fusion Example

Here's a simple example of combining LIDAR and IMU data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
import numpy as np
import math

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_lidar_data = None
        self.latest_imu_data = None

        self.get_logger().info('Sensor Fusion Node initialized')

    def lidar_callback(self, msg):
        """Store latest LIDAR data"""
        self.latest_lidar_data = msg

        if self.process_lidar_for_obstacles():
            self.get_logger().warn('Obstacle detected! Stopping robot.')
            self.stop_robot()

    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.latest_imu_data = msg

        orientation = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w)

        self.get_logger().info(f'Robot orientation: Yaw={math.degrees(yaw):.2f}°')

    def process_lidar_for_obstacles(self):
        """Check if there are obstacles in front of the robot"""
        if self.latest_lidar_data is None:
            return False

        ranges = np.array(self.latest_lidar_data.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            return False

        center_idx = len(ranges) // 2
        front_range = ranges[center_idx - len(ranges)//10:center_idx + len(ranges)//10]
        front_valid = front_range[np.isfinite(front_range)]

        if len(front_valid) > 0:
            min_front_distance = np.min(front_valid)
            return min_front_distance < 1.0

        return False

    def stop_robot(self):
        """Send stop command to robot"""
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_publisher.publish(stop_cmd)

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    sensor_fusion_node = SensorFusionNode()

    try:
        rclpy.spin(sensor_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Visualization of Sensor Data

For visualizing sensor data, you can use RViz2 which comes with ROS 2. Here's a simple example of publishing data for visualization:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class SensorVisualizationNode(Node):
    def __init__(self):
        super().__init__('sensor_visualization_node')

        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self.marker_publisher = self.create_publisher(Marker, '/lidar_points', 10)

        self.get_logger().info('Sensor Visualization Node initialized')

    def lidar_callback(self, msg):
        """Convert LIDAR scan to visualization markers"""
        marker = Marker()
        marker.header = msg.header
        marker.ns = "lidar_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if not math.isinf(range_val) and not math.isnan(range_val):
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)

                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0

                marker.points.append(point)

            angle += msg.angle_increment

        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    vis_node = SensorVisualizationNode()

    try:
        rclpy.spin(vis_node)
    except KeyboardInterrupt:
        pass
    finally:
        vis_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

IMU sensors provide essential orientation and motion data for robots, complementing LIDAR's distance measurements. By integrating multiple sensor types through ROS 2, robots can achieve comprehensive environmental awareness and make informed decisions about navigation and interaction.

## Key Takeaways

:::tip Key Takeaways
- IMUs combine accelerometers, gyroscopes, and magnetometers for motion sensing
- ROS 2 provides standardized message types for sensor data integration
- Sensor fusion combines data from multiple sensors for robust perception
- Proper quaternion-to-Euler conversion is essential for orientation processing
:::

## What's Next

In the next chapter, we'll dive into ROS 2 fundamentals, exploring nodes, topics, and the publish-subscribe communication pattern that enables distributed robotics systems.
