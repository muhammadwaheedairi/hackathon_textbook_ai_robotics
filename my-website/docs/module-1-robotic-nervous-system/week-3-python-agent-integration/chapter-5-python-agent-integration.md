---
title: "Python Agent Integration with ROS Controllers"
sidebar_label: "Chapter 5: Python Agent Integration"
sidebar_position: 5
---

# Chapter 5: Python Agent Integration with ROS Controllers

## Overview

This chapter explores how to integrate Python-based intelligent agents with ROS 2 controllers for robot actuation. You'll learn how to bridge high-level AI decision-making with low-level hardware control, enabling sophisticated robot behaviors.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Integrate Python agents with ROS 2 controllers
- Understand the agent-controller architecture
- Implement control interfaces for robot actuation
- Configure ros2_control framework components
:::

## Python Agent Integration with ROS Controllers

Python agents in robotics are software components that implement decision-making algorithms, planning systems, or AI-based control strategies. These agents must interface with ROS controllers, which handle the low-level hardware control and ensure precise execution of commands. The integration enables sophisticated robot behaviors by combining high-level cognitive capabilities with precise low-level control.

### Agent-Controller Architecture

The integration between Python agents and ROS controllers typically follows this layered pattern:

1. **Agent Layer**: Implements high-level decision-making, planning, and reasoning using AI algorithms
2. **Interface Layer**: Translates agent decisions into controller commands and handles communication protocols
3. **Controller Layer**: Executes precise hardware control based on received commands
4. **Hardware Layer**: Physical robot components that execute the actions

### Types of ROS Controllers

ROS 2 supports several types of controllers for different control needs:

- **Joint Trajectory Controller**: Executes complete trajectories with position, velocity, and acceleration
- **Position Controllers**: Simple position control for individual joints
- **Velocity Controllers**: Velocity-based control for smooth motion
- **Effort Controllers**: Direct torque/force control for precise force application
- **Forward Command Controllers**: Forward desired states to hardware interfaces

### Control Interface Patterns

There are several common patterns for integrating Python agents with ROS controllers:

1. **Direct Command Pattern**: Agent sends immediate commands to controllers
2. **Trajectory Planning Pattern**: Agent plans complete trajectories and sends them to trajectory controllers
3. **Feedback Control Pattern**: Agent uses sensor feedback to adjust control commands
4. **State Machine Pattern**: Agent implements complex behaviors through state transitions

### Communication Mechanisms

Python agents communicate with controllers using several ROS 2 communication patterns:

- **Topics**: For continuous state publishing and command streaming
- **Services**: For synchronous configuration and immediate actions
- **Actions**: For long-running tasks with progress feedback
- **Parameters**: For dynamic configuration changes

### Python Agent Implementation Considerations

When implementing Python agents for ROS controller integration, consider:

- **Real-time constraints**: Ensure control loop timing requirements are met
- **Safety boundaries**: Implement safety checks and limits
- **Error handling**: Gracefully handle controller failures and exceptions
- **State management**: Maintain consistent state between agent and controllers
- **Logging and diagnostics**: Track agent decisions and controller responses

### ros2_control Framework Components

The `ros2_control` framework provides several key components for Python agent integration:

- **Hardware Interface**: Abstracts communication with specific hardware platforms
- **Controller Manager**: Manages the lifecycle of controllers (load, configure, start, stop)
- **Controller Types**: Pre-built controllers for common control tasks
- **Transmission Interface**: Maps actuator commands to joint commands
- **Resource Manager**: Tracks and manages hardware resources

### Controller Configuration

Controllers are configured using YAML files that specify parameters such as:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

## Code Examples

### Python Agent with Joint State Subscriber

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class RobotAgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent_node')

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_commands',
            10
        )

        self.current_positions = {}
        self.timer = self.create_timer(0.1, self.agent_decision_callback)

        self.get_logger().info('Robot Agent Node initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_positions[name] = msg.position[i]

        self.get_logger().info(f'Updated joint positions: {self.current_positions}')

    def agent_decision_callback(self):
        """Make decisions based on current state"""
        if len(self.current_positions) > 0:
            target_positions = []
            for joint_name, current_pos in self.current_positions.items():
                target_pos = current_pos + 0.01
                target_positions.append(target_pos)

            command_msg = Float64MultiArray()
            command_msg.data = target_positions
            self.joint_command_publisher.publish(command_msg)

            self.get_logger().info(f'Published joint commands: {target_positions}')

def main(args=None):
    rclpy.init(args=args)
    robot_agent_node = RobotAgentNode()

    try:
        rclpy.spin(robot_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Controller Interface Node

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ControllerInterfaceNode(Node):
    def __init__(self):
        super().__init__('controller_interface_node')

        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.send_trajectory_command)

        self.get_logger().info('Controller Interface Node initialized')

    def send_trajectory_command(self):
        """Send a trajectory command to the controller"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']

        point = JointTrajectoryPoint()
        point.positions = [0.5, 1.0, -0.5]
        point.velocities = [0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)

        trajectory_msg.points = [point]
        self.trajectory_publisher.publish(trajectory_msg)

        self.get_logger().info(f'Published trajectory command: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    controller_interface_node = ControllerInterfaceNode()

    try:
        rclpy.spin(controller_interface_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_interface_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced Python Agent with Action Client

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class AdvancedRobotAgentNode(Node):
    def __init__(self):
        super().__init__('advanced_robot_agent_node')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory'
        )

        self.timer = self.create_timer(5.0, self.execute_trajectory_callback)

        self.get_logger().info('Advanced Robot Agent Node initialized')

    def execute_trajectory_callback(self):
        """Execute a planned trajectory using action client"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0, 0.0]
        point1.velocities = [0.0, 0.0, 0.0]
        point1.time_from_start = Duration(sec=1, nanosec=0)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.5, 0.5]
        point2.velocities = [0.0, 0.0, 0.0]
        point2.time_from_start = Duration(sec=2, nanosec=0)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0, 0.0]
        point3.velocities = [0.0, 0.0, 0.0]
        point3.time_from_start = Duration(sec=3, nanosec=0)

        goal_msg.trajectory.points = [point1, point2, point3]

        self._send_goal_async(goal_msg)

    def _send_goal_async(self, goal_msg):
        """Send goal to trajectory controller"""
        self.get_logger().info('Waiting for action server...')

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle result of trajectory execution"""
        result = future.result().result
        self.get_logger().info(f'Trajectory execution result: {result.error_code}')

    def feedback_callback(self, feedback_msg):
        """Handle feedback during trajectory execution"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Trajectory progress: {feedback.joint_names}')

def main(args=None):
    rclpy.init(args=args)
    advanced_agent_node = AdvancedRobotAgentNode()

    try:
        rclpy.spin(advanced_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        advanced_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Python agents provide the cognitive layer for robot decision-making, while ROS controllers handle precise hardware execution. Integrating these components through topics, services, and actions enables sophisticated robot behaviors that combine high-level intelligence with low-level control precision.

## Key Takeaways

:::tip Key Takeaways
- Python agents implement high-level decision-making and planning algorithms
- ROS controllers execute precise hardware control based on agent commands
- Multiple communication patterns (topics, services, actions) enable flexible integration
- The ros2_control framework provides standardized interfaces for controller management
:::

## What's Next

In the next chapter, we'll explore URDF modeling for robot description, learning how to define robot structures and properties for simulation and control.
