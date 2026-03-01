---
title: "Nav2 Integration"
sidebar_label: "Chapter 14: Nav2 Integration"
sidebar_position: 14
---

# Chapter 14: Nav2 Integration

## Overview

This chapter explores integrating Isaac ROS with the Navigation2 stack for advanced robot navigation. You'll learn how to combine hardware-accelerated perception with sophisticated path planning and obstacle avoidance.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Integrate Isaac ROS with Navigation2 stack
- Configure Nav2 parameters for optimal performance
- Implement hardware-accelerated navigation pipelines
- Deploy complete navigation systems on real robots
:::

## Hardware-Accelerated Navigation (Nav2 Integration)

Isaac ROS integrates seamlessly with the Navigation2 stack by providing high-quality pose estimates, accelerating perception tasks, improving real-time performance, and enabling more complex navigation behaviors.

### Navigation2 Configuration with Isaac ROS

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan

bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
```

## Code Examples

### Complete Navigation System with Isaac ROS

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from sensor_msgs.msg import Image
import numpy as np

class IsaacROSNavigationNode(Node):
    def __init__(self):
        super().__init__('isaac_ros_navigation_node')

        self.navigator = BasicNavigator()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Isaac ROS Navigation Node initialized')

    def image_callback(self, msg):
        pass

    def navigate_to_pose(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = np.sin(theta / 2.0)
        goal_pose.pose.orientation.w = np.cos(theta / 2.0)

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f'Navigation feedback: {feedback}')

        result = self.navigator.getResult()
        if result == 'succeeded':
            self.get_logger().info('Goal reached successfully!')
        else:
            self.get_logger().error(f'Navigation failed: {result}')

def main(args=None):
    rclpy.init(args=args)
    nav_node = IsaacROSNavigationNode()

    try:
        nav_node.navigate_to_pose(2.0, 3.0, 0.0)
        rclpy.spin(nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        nav_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Integrating Isaac ROS with Navigation2 combines hardware-accelerated perception with sophisticated navigation algorithms. This integration enables real-time obstacle avoidance, efficient path planning, and robust robot navigation in complex environments.

## Key Takeaways

:::tip Key Takeaways
- Isaac ROS provides high-quality pose estimates for Nav2
- Hardware acceleration enables real-time navigation in complex environments
- Proper configuration of Nav2 parameters optimizes navigation performance
- Complete navigation systems combine perception, planning, and control
:::

## What's Next

In the next chapter, we'll explore Isaac Sim for reinforcement learning, learning how to train robotic policies using GPU-accelerated simulation environments.
