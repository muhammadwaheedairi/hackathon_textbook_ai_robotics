---
title: "ROS 2 Services and Packages"
sidebar_label: "Chapter 4: Services & Packages"
sidebar_position: 4
---

# Chapter 4: ROS 2 Services and Packages

## Overview

This chapter explores ROS 2 services for synchronous communication and packages for code organization. You'll learn how to implement request-response patterns and structure your robotics projects effectively.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Implement service servers and clients for synchronous communication
- Understand when to use services versus topics
- Create and organize ROS 2 packages
- Manage package dependencies and build configurations
:::

## ROS 2 Services

Services provide synchronous request/response communication between nodes. A service client sends a request to a service server, which processes the request and returns a response. Services are ideal for operations that have a clear beginning and end, such as configuration changes, computation tasks, or triggering specific actions.

### Service Characteristics

- **Synchronous**: Client waits for response before continuing
- **Request/Response**: Defined request and response message types
- **Direct communication**: Client communicates directly with server
- **Blocking**: Service calls block until response is received
- **One-to-one**: Each request is handled by a single service server
- **Stateless**: Each service call is independent of others
- **Typed interfaces**: Service definitions specify request and response message types

### Service Architecture

The service pattern works as follows:
1. A service server node creates a service server and registers it with a service name
2. A service client node creates a service client and connects to the same service name
3. The client sends a request to the server
4. The server processes the request and sends back a response
5. The client receives the response and continues execution

### Service Implementation in Python

Here's how to implement service servers and clients in Python:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )
        self.get_logger().info('Service server initialized')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        return self.future
```

### Service Definition Files (.srv)

Service definitions are specified in `.srv` files that define the request and response message structure:

```
# Request part (before the '---')
int64 a
int64 b
---
# Response part (after the '---')
int64 sum
```

### Service Management Commands

Common command-line tools for working with services:

```bash
# List all active services
ros2 service list

# Show information about a specific service
ros2 service info /service_name

# Call a service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Show the type of a service
ros2 service type /service_name
```

### When to Use Services vs Topics

**Use Services when:**
- You need request/response interaction
- Operations have a clear beginning and end
- You need guaranteed delivery and response
- The operation is relatively fast (under a few seconds)
- You need to pass parameters and get computed results

**Use Topics when:**
- You need continuous data streaming
- Communication should be decoupled in time
- Multiple subscribers need the same data
- Data is updated continuously (sensor data, status updates)

## ROS 2 Packages

Packages are the basic building and distribution units in ROS 2. They contain source code, data, and configuration files organized in a standard structure. Packages provide a way to organize related functionality, manage dependencies, and distribute ROS 2 software.

### Package Structure

A standard ROS 2 package follows this structure:

```
package_name/
├── CMakeLists.txt         # Build configuration for C++ packages
├── package.xml            # Package metadata and dependencies
├── src/                   # Source code files
├── include/               # Header files (C++)
├── launch/                # Launch files for starting multiple nodes
├── config/                # Configuration files
├── test/                  # Unit and integration tests
├── scripts/               # Standalone executable scripts
├── msg/                   # Custom message definitions
├── srv/                   # Custom service definitions
├── action/                # Custom action definitions
└── setup.py               # Python package setup (for Python packages)
```

### Package Metadata (package.xml)

The `package.xml` file contains metadata about the package and its dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>A package for my robot functionality</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Creating Packages

Packages can be created using the `ros2 pkg create` command:

```bash
# Create a Python package
ros2 pkg create --build-type ament_python my_python_package

# Create a C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package

# Create a package with dependencies
ros2 pkg create --build-type ament_python my_package \
  --dependencies rclpy std_msgs sensor_msgs geometry_msgs
```

### Python Package Setup (setup.py)

For Python packages, the `setup.py` file configures the Python build process:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User Name',
    maintainer_email='user@example.com',
    description='A package for my robot functionality',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_robot_package.my_node:main',
            'another_node = my_robot_package.another_node:main',
        ],
    },
)
```

### Package Management Tools

ROS 2 provides several tools for managing packages:

- **ros2 pkg**: Package management commands
  - `ros2 pkg list`: List all available packages
  - `ros2 pkg xml <package_name>`: Show package.xml content
  - `ros2 pkg executables <package_name>`: List executables in a package

- **colcon**: Build system for ROS 2 packages
  - `colcon build`: Build all packages in the workspace
  - `colcon build --packages-select <package_name>`: Build specific package
  - `colcon test`: Run tests for packages

### Package Dependencies

Dependencies in ROS 2 packages are categorized as:

- **buildtool_depend**: Build system dependencies (e.g., `ament_cmake`)
- **build_depend**: Dependencies needed during build time
- **exec_depend**: Dependencies needed at runtime
- **test_depend**: Dependencies needed for testing
- **depend**: Shorthand for both build and exec dependencies

### Best Practices for Package Organization

- Use descriptive package names that clearly indicate functionality
- Follow the ROS 2 naming conventions (lowercase, underscores)
- Keep related functionality together in the same package
- Separate different types of functionality into different packages
- Use appropriate dependency management
- Include proper documentation and examples
- Write comprehensive tests for your packages

## Code Examples

### Service Server Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service Server Node initialized')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server_node = ServiceServerNode()

    try:
        rclpy.spin(service_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        service_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()
        self.get_logger().info('Service Client Node initialized')

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_client_node = ServiceClientNode()

    try:
        a = int(sys.argv[1]) if len(sys.argv) > 1 else 1
        b = int(sys.argv[2]) if len(sys.argv) > 2 else 2

        response = service_client_node.send_request(a, b)
        service_client_node.get_logger().info(f'Result of {a} + {b} = {response.sum}')
    except Exception as e:
        service_client_node.get_logger().error(f'Service call failed: {e}')
    finally:
        service_client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 services provide synchronous request-response communication for operations with clear start and end points. Packages organize related code, dependencies, and resources into distributable units, enabling modular and maintainable robotics software development.

## Key Takeaways

:::tip Key Takeaways
- Services enable synchronous request-response communication patterns
- Use services for operations with clear beginning and end, topics for continuous data
- Packages organize code into modular, distributable units
- Proper dependency management ensures reliable package builds and deployments
:::

## What's Next

In the next chapter, we'll explore Python agent integration with ROS 2 controllers, learning how to connect high-level AI decision-making with low-level robot control.
