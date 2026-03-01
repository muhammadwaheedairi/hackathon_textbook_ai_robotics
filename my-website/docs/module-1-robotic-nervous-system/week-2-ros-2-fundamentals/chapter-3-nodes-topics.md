---
title: "ROS 2 Nodes and Topics"
sidebar_label: "Chapter 3: Nodes & Topics"
sidebar_position: 3
---

# Chapter 3: ROS 2 Nodes and Topics

## Overview

This chapter introduces the fundamental building blocks of ROS 2: nodes and topics. You'll learn how nodes serve as independent computational units and how topics enable asynchronous communication through the publish-subscribe pattern.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Define and create ROS 2 nodes for distributed computing
- Understand the publish-subscribe communication pattern
- Implement publishers and subscribers in Python
- Configure Quality of Service (QoS) policies for topics
:::

## ROS 2 Nodes

A node is an executable that uses ROS 2 to communicate with other nodes. Nodes are the fundamental building blocks of a ROS 2 system. Each node runs independently and can communicate with other nodes through topics, services, or actions. Nodes encapsulate the computational units of a ROS 2 application and provide the interfaces (publishers, subscribers, services, etc.) that allow the node to interact with other nodes.

### Node Characteristics

- **Process-based**: Each node runs as a separate process, providing isolation and fault tolerance
- **Communication hub**: Nodes serve as endpoints for ROS 2 communication, managing publishers, subscribers, services, and parameters
- **Resource management**: Nodes manage their own resources including timers, callbacks, and internal state
- **Modularity**: Nodes can be developed, tested, and deployed independently
- **Naming**: Each node has a unique name within the ROS 2 graph that identifies it to other nodes
- **Namespace support**: Nodes can be organized under namespaces for better organization

### Node Implementation in Python

In Python, ROS 2 nodes are implemented by subclassing the `rclpy.node.Node` class:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('Node initialized successfully')
```

### Node Lifecycle

ROS 2 nodes follow a defined lifecycle that enables more sophisticated state management, especially for complex robotic applications:

1. **Unconfigured (Primary State)**: Node created but not configured; only basic operations allowed
2. **Inactive (Secondary State)**: Node configured but not active; can communicate but doesn't perform main functions
3. **Active (Secondary State)**: Node is fully operational and performing its intended functions
4. **Finalized (Secondary State)**: Node is shutting down and cleaning up resources

The lifecycle system is optional and primarily used for more complex systems that need sophisticated state management.

### Node Management Commands

Common command-line tools for working with nodes:

```bash
# List all active nodes
ros2 node list

# Show information about a specific node
ros2 node info <node_name>
```

## ROS 2 Topics and Publish-Subscribe Pattern

Topics enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to topics, and subscribers receive messages from topics. This decouples publishers and subscribers in time and space, allowing for flexible and scalable robot architectures.

### Topic Communication

- **Decoupled**: Publishers and subscribers don't need to know about each other
- **Asynchronous**: Messages are sent and received independently
- **Broadcast**: One publisher can send to multiple subscribers
- **Typed**: Each topic has a specific message type
- **Named**: Topics have unique names that identify the data stream
- **QoS support**: Quality of Service policies can be configured for different reliability and performance needs

### Topic Architecture

The publish-subscribe pattern works as follows:
1. A publisher node creates a publisher and sends messages to a named topic
2. A subscriber node creates a subscription to the same named topic
3. The ROS 2 middleware (DDS) handles message delivery between publishers and subscribers
4. Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic

### Quality of Service (QoS) Policies

ROS 2 topics support various QoS policies that define how messages are delivered:

- **Reliability**: `RELIABLE` (all messages delivered) or `BEST_EFFORT` (try to deliver messages)
- **Durability**: `TRANSIENT_LOCAL` (store messages for late-joining subscribers) or `VOLATILE` (don't store messages)
- **History**: `KEEP_LAST` (store N most recent messages) or `KEEP_ALL` (store all messages)
- **Depth**: Number of messages to store in history when using `KEEP_LAST`

### Common Topic Message Types

- `std_msgs`: Basic data types (integers, floats, strings)
- `sensor_msgs`: Sensor data (LIDAR, cameras, IMU)
- `geometry_msgs`: Geometric data (poses, velocities, points)
- `nav_msgs`: Navigation-related messages (paths, occupancy grids)
- `action_msgs`: Action-related messages
- `builtin_interfaces`: Time and duration messages

### Topic Implementation in Python

Here's how to implement publishers and subscribers in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TopicPublisherNode(Node):
    def __init__(self):
        super().__init__('topic_publisher')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

class TopicSubscriberNode(Node):
    def __init__(self):
        super().__init__('topic_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

### Example with Custom QoS Settings

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class QoSTopicNode(Node):
    def __init__(self):
        super().__init__('qos_topic_node')

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(String, 'reliable_topic', qos_profile)
```

### Topic Management Commands

Common command-line tools for working with topics:

```bash
# List all active topics
ros2 topic list

# Show information about a specific topic
ros2 topic info /topic_name

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# Publish a message to a topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Show the type of a topic
ros2 topic type /topic_name
```

## Code Examples

### Creating a Basic ROS 2 Node

```python
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node')
        self.get_logger().info('Basic Node initialized')

def main(args=None):
    rclpy.init(args=args)
    basic_node = BasicNode()

    try:
        rclpy.spin(basic_node)
    except KeyboardInterrupt:
        pass
    finally:
        basic_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('Publisher Node initialized')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()

    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription
        self.get_logger().info('Subscriber Node initialized')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()

    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 nodes are independent computational units that communicate through topics using the publish-subscribe pattern. This architecture enables flexible, scalable, and modular robot systems where components can be developed and tested independently.

## Key Takeaways

:::tip Key Takeaways
- Nodes are the fundamental building blocks of ROS 2 systems
- Topics enable asynchronous, decoupled communication between nodes
- QoS policies allow fine-tuning of message delivery characteristics
- The publish-subscribe pattern supports one-to-many communication
:::

## What's Next

In the next chapter, we'll explore ROS 2 services for synchronous request-response communication and learn how to organize code into packages for better project structure.
