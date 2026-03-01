---
title: "Gazebo ROS 2 Integration"
sidebar_label: "Chapter 8: Gazebo ROS 2 Integration"
sidebar_position: 8
---

# Chapter 8: Gazebo ROS 2 Integration

## Overview

This chapter explores the integration between Gazebo and ROS 2, enabling seamless communication between simulated robots and ROS 2 control systems. You'll learn how to bridge simulation and real-world robotics workflows.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Integrate Gazebo with ROS 2 for robot simulation
- Use gazebo_ros_pkgs for communication bridges
- Control simulated robots through ROS 2 interfaces
- Configure physics parameters dynamically from ROS 2
:::

## SDF (Simulation Description Format)

SDF is the native format for describing simulation worlds in Gazebo. It extends URDF capabilities to include simulation-specific features like physics properties, sensors, and plugins. An SDF file typically contains world definitions, model definitions, light sources, and plugins.

Basic SDF structure:

```xml
<sdf version="1.7">
  <world name="default">
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <model name="robot">
      <!-- Model definition -->
    </model>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
    </light>
  </world>
</sdf>
```

## Gazebo-ROS 2 Integration

The integration between Gazebo and ROS 2 is facilitated by the `gazebo_ros_pkgs` package, which provides plugins and tools for seamless communication. Key integration points include message bridges, TF tree integration, and service interfaces.

### Message Bridges

The integration provides automatic bridges between Gazebo topics and ROS 2 topics:
- `/clock` synchronization for simulation time
- Sensor data publishing to ROS 2 topics
- Actuator command subscription from ROS 2 topics
- Model state publishing and subscription

### TF Tree Integration

Gazebo automatically publishes transforms for all simulated models, creating a complete TF tree that ROS 2 nodes can use for spatial reasoning.

### Service Interfaces

The integration provides ROS 2 services for:
- Model spawning and deletion
- World state management
- Simulation control (pause, reset, step)
- Physics parameter adjustment

## Code Examples

### ROS 2 Node for Gazebo Interaction

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
import time

class GazeboControllerNode(Node):
    def __init__(self):
        super().__init__('gazebo_controller_node')

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.pause_client = self.create_client(Empty, '/pause_physics')
        self.unpause_client = self.create_client(Empty, '/unpause_physics')
        self.reset_client = self.create_client(Empty, '/reset_simulation')

        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete service...')

        self.get_logger().info('Gazebo Controller Node initialized')

    def spawn_model(self, model_name, model_xml, robot_namespace=''):
        """Spawn a model in Gazebo"""
        request = SpawnEntity.Request()
        request.name = model_name
        request.xml = model_xml
        request.robot_namespace = robot_namespace

        future = self.spawn_client.call_async(request)
        return future

    def delete_model(self, model_name):
        """Delete a model from Gazebo"""
        request = DeleteEntity.Request()
        request.name = model_name

        future = self.delete_client.call_async(request)
        return future

    def pause_simulation(self):
        """Pause the physics simulation"""
        future = self.pause_client.call_async(Empty.Request())
        return future

    def unpause_simulation(self):
        """Unpause the physics simulation"""
        future = self.unpause_client.call_async(Empty.Request())
        return future

    def reset_simulation(self):
        """Reset the entire simulation"""
        future = self.reset_client.call_async(Empty.Request())
        return future

    def model_states_callback(self, msg):
        """Handle model states updates"""
        for i, name in enumerate(msg.name):
            if name == 'mobile_robot':
                position = msg.pose[i].position
                velocity = msg.twist[i].linear
                self.get_logger().info(
                    f'Robot {name} position: ({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), '
                    f'velocity: ({velocity.x:.2f}, {velocity.y:.2f}, {velocity.z:.2f})'
                )

    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to simulated robot"""
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_x
        cmd_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    gazebo_controller = GazeboControllerNode()

    try:
        gazebo_controller.send_velocity_command(linear_x=0.5, angular_z=0.2)
        time.sleep(2.0)
        gazebo_controller.send_velocity_command()
        rclpy.spin_once(gazebo_controller, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        gazebo_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Physics Parameter Configuration Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from geometry_msgs.msg import Vector3

class PhysicsConfigNode(Node):
    def __init__(self):
        super().__init__('physics_config_node')

        self.set_physics_client = self.create_client(
            SetPhysicsProperties,
            '/set_physics_properties'
        )
        self.get_physics_client = self.create_client(
            GetPhysicsProperties,
            '/get_physics_properties'
        )

        self.timer = self.create_timer(5.0, self.adjust_physics_properties)

        self.get_logger().info('Physics Configuration Node initialized')

    def get_current_physics_properties(self):
        """Get current physics properties from Gazebo"""
        while not self.get_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get physics service...')

        request = GetPhysicsProperties.Request()
        future = self.get_physics_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_physics_properties(self, time_step=0.001, max_update_rate=1000.0,
                              gravity=(0, 0, -9.8)):
        """Set physics properties in Gazebo"""
        while not self.set_physics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set physics service...')

        request = SetPhysicsProperties.Request()
        request.time_step = time_step
        request.max_update_rate = max_update_rate

        gravity_msg = Vector3()
        gravity_msg.x = gravity[0]
        gravity_msg.y = gravity[1]
        gravity_msg.z = gravity[2]
        request.gravity = gravity_msg

        request.ode_config.auto_disable_bodies = False
        request.ode_config.sor_pgs_precon_iters = 2
        request.ode_config.sor_pgs_iters = 50
        request.ode_config.sor_pgs_w = 1.3
        request.ode_config.contact_surface_layer = 0.001
        request.ode_config.contact_max_correcting_vel = 100.0
        request.ode_config.cfm = 0.0
        request.ode_config.erp = 0.2
        request.ode_config.max_contacts = 20

        future = self.set_physics_client.call_async(request)
        return future

    def adjust_physics_properties(self):
        """Adjust physics properties based on simulation requirements"""
        current_props = self.get_current_physics_properties()

        if current_props:
            self.get_logger().info(f'Current time step: {current_props.time_step}')
            self.get_logger().info(f'Current max update rate: {current_props.max_update_rate}')
            self.get_logger().info(f'Current gravity: {current_props.gravity}')

        self.get_logger().info('Physics properties checked/adjusted')

def main(args=None):
    rclpy.init(args=args)
    physics_config_node = PhysicsConfigNode()

    try:
        rclpy.spin(physics_config_node)
    except KeyboardInterrupt:
        pass
    finally:
        physics_config_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Physics Simulation with Custom Contact Detection

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState
import math

class AdvancedPhysicsController(Node):
    def __init__(self):
        super().__init__('advanced_physics_controller')

        self.contact_sub = self.create_subscription(
            ContactsState,
            '/contact_sensor_state',
            self.contact_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.scan_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.physics_control_loop)

        self.collision_detected = False
        self.collision_force = 0.0
        self.scan_ranges = []
        self.safe_distance = 1.0

        self.get_logger().info('Advanced Physics Controller initialized')

    def contact_callback(self, msg):
        """Handle contact sensor messages"""
        if len(msg.states) > 0:
            self.collision_detected = True
            if len(msg.states[0].wrenches) > 0:
                force = msg.states[0].wrenches[0].force
                self.collision_force = math.sqrt(
                    force.x**2 + force.y**2 + force.z**2
                )
            self.get_logger().info(f'Collision detected with force: {self.collision_force:.2f}')
        else:
            self.collision_detected = False
            self.collision_force = 0.0

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        self.scan_ranges = msg.ranges

    def physics_control_loop(self):
        """Main physics-based control loop"""
        cmd_msg = Twist()

        if self.scan_ranges:
            front_scan = self.scan_ranges[len(self.scan_ranges)//2]
            if not math.isinf(front_scan) and front_scan < self.safe_distance:
                cmd_msg.linear.x = 0.2 * (front_scan / self.safe_distance)
                cmd_msg.angular.z = 0.0
            else:
                cmd_msg.linear.x = 0.5
                cmd_msg.angular.z = 0.0
        else:
            cmd_msg.linear.x = 0.3
            cmd_msg.angular.z = 0.0

        if self.collision_detected:
            cmd_msg.linear.x = -0.3
            cmd_msg.angular.z = 0.5
            self.get_logger().info('Collision response: reversing and turning')

        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    physics_controller = AdvancedPhysicsController()

    try:
        rclpy.spin(physics_controller)
    except KeyboardInterrupt:
        pass
    finally:
        physics_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Gazebo-ROS 2 integration enables comprehensive robot simulation with seamless communication between simulated environments and ROS 2 control systems. This integration supports model spawning, physics configuration, and sensor data streaming for effective robot development and testing.

## Key Takeaways

:::tip Key Takeaways
- SDF format provides simulation-specific features beyond URDF capabilities
- gazebo_ros_pkgs bridges Gazebo and ROS 2 communication
- Dynamic physics configuration enables adaptive simulation environments
- Contact sensors and collision detection support safety-critical testing
:::

## What's Next

In the next chapter, we'll explore high-fidelity rendering in Unity, learning how to create photorealistic environments for advanced robot perception training.
