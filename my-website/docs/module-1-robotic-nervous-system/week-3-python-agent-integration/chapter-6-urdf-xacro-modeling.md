---
title: "URDF and Xacro Modeling"
sidebar_label: "Chapter 6: URDF Modeling"
sidebar_position: 6
---

# Chapter 6: URDF and Xacro Modeling

## Overview

This chapter introduces URDF (Unified Robot Description Format) for defining robot models in ROS 2. You'll learn how to create robot descriptions, define kinematic structures, and use Xacro for modular robot modeling.

## Learning Objectives

:::info Learning Objectives
By the end of this chapter, you will be able to:
- Understand URDF structure and components
- Create robot models with links and joints
- Define physical properties for simulation
- Use Xacro for parameterized robot descriptions
:::

## URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the robot's physical and kinematic properties, including links, joints, inertial properties, visual and collision models. URDF serves as the foundation for robot simulation, visualization, and kinematic analysis in ROS-based systems.

### URDF Components and Structure

URDF documents have a hierarchical structure with several key components:

**Links**: Rigid parts of the robot (e.g., base, arms, wheels) that have physical properties:
- **Visual**: How the link appears in simulation and visualization
- **Collision**: How the link interacts with the environment for physics simulation
- **Inertial**: Physical properties like mass, center of mass, and inertia tensor

**Joints**: Connections between links that define the kinematic relationship:
- **Joint Types**: revolute (rotational), prismatic (linear), fixed, continuous, planar, floating
- **Joint Limits**: Position, velocity, and effort limits for safety
- **Joint Origins**: Position and orientation relative to parent link

**Materials**: Color and visual appearance definitions for rendering

### URDF XML Structure

A basic URDF follows this XML structure:

```xml
<robot name="robot_name">
  <material name="color_name">
    <color rgba="r g b a"/>
  </material>

  <link name="link_name">
    <visual>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <geometry>
        <!-- Shape definition: box, cylinder, sphere, mesh -->
      </geometry>
      <material name="material_name"/>
    </visual>
    <collision>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <geometry>
        <!-- Shape definition -->
      </geometry>
    </collision>
    <inertial>
      <mass value="mass_value"/>
      <origin xyz="x y z" rpy="roll pitch yaw"/>
      <inertia ixx="ixx" ixy="ixy" ixz="ixz" iyy="iyy" iyz="iyz" izz="izz"/>
    </inertial>
  </link>

  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
    <axis xyz="x y z"/>
    <limit lower="lower_limit" upper="upper_limit"
           effort="max_effort" velocity="max_velocity"/>
  </joint>
</robot>
```

### Joint Types in Detail

- **Revolute**: Rotational joint with limited range of motion (like an elbow)
- **Continuous**: Rotational joint without limits (like a wheel)
- **Prismatic**: Linear sliding joint (like a piston)
- **Fixed**: No movement, permanently connects two links
- **Planar**: Movement in a plane
- **Floating**: 6 DOF movement (no constraints)

### Geometry Types

URDF supports several geometric shapes for visual and collision models:
- **Box**: Defined by size="x_length y_width z_height"
- **Cylinder**: Defined by radius and length
- **Sphere**: Defined by radius
- **Mesh**: Defined by filename and scale (for complex shapes)

### Inertial Properties

The inertial properties are crucial for accurate physics simulation:
- **Mass**: The mass of the link in kilograms
- **Center of Mass**: The center of mass offset from the link origin
- **Inertia Tensor**: 6 values representing the 3x3 inertia matrix (ixx, ixy, ixz, iyy, iyz, izz)

### URDF Best Practices

- **Naming Conventions**: Use descriptive, consistent names (e.g., `base_link`, `arm_link1`)
- **Proper Inertial Properties**: Include realistic inertial values for accurate simulation
- **Collision vs Visual**: Use simpler shapes for collision models to improve performance
- **Tree Structure**: Maintain a proper tree structure (no loops in the kinematic chain)
- **Origin Conventions**: Use consistent origin placements (e.g., at joint centers)
- **Xacro Usage**: For complex robots, use Xacro (XML Macros) to parameterize and reuse components

### URDF Tools and Validation

ROS provides several tools for working with URDF:
- `check_urdf`: Validates URDF syntax and structure
- `urdf_to_graphiz`: Generates visual representation of the kinematic tree
- `robot_state_publisher`: Publishes transforms based on joint states
- `joint_state_publisher`: Publishes default joint states for visualization

## Xacro for Complex Models

Xacro (XML Macros) extends URDF with features like:
- Variables and constants
- Mathematical expressions
- Macros for reusable components
- Include statements for modular design

Example Xacro usage:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
  <xacro:property name="M_PI" value="3.14159"/>

  <xacro:macro name="simple_wheel" params="prefix parent xyz">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel_link"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel_link">
      <visual>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.1" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
      </inertial>
    </link>
  </xacro:macro>

  <xacro:simple_wheel prefix="front_left" parent="base_link" xyz="0.2 0.1 -0.05"/>
</robot>
```

## Code Examples

### URDF Example - Simple Robot Arm

```xml
<?xml version="1.0"?>
<robot name="simple_robot_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.0 0.0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <link name="link2">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>
</robot>
```

### Python URDF Parser Example

```python
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET

class URDFAnalyzerNode(Node):
    def __init__(self):
        super().__init__('urdf_analyzer_node')
        self.get_logger().info('URDF Analyzer Node initialized')

        self.example_urdf = '''<?xml version="1.0"?>
<robot name="simple_robot_arm">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.2"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>'''

        self.parse_urdf()

    def parse_urdf(self):
        """Parse the URDF and extract information"""
        try:
            root = ET.fromstring(self.example_urdf)

            robot_name = root.attrib.get('name', 'unknown')
            self.get_logger().info(f'Robot name: {robot_name}')

            links = root.findall('link')
            joints = root.findall('joint')

            self.get_logger().info(f'Number of links: {len(links)}')
            self.get_logger().info(f'Number of joints: {len(joints)}')

            for link in links:
                link_name = link.attrib.get('name')
                self.get_logger().info(f'Link: {link_name}')

            for joint in joints:
                joint_name = joint.attrib.get('name')
                joint_type = joint.attrib.get('type')
                self.get_logger().info(f'Joint: {joint_name}, Type: {joint_type}')

        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

def main(args=None):
    rclpy.init(args=args)
    urdf_analyzer_node = URDFAnalyzerNode()

    try:
        rclpy.spin_once(urdf_analyzer_node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        urdf_analyzer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

URDF provides a standardized format for describing robot structures in ROS 2, defining links, joints, and physical properties. Xacro extends URDF with macros and parameterization, enabling modular and maintainable robot descriptions for complex systems.

## Key Takeaways

:::tip Key Takeaways
- URDF defines robot structure with links, joints, and physical properties
- Proper inertial properties are essential for accurate physics simulation
- Xacro enables parameterized, reusable robot descriptions
- ROS 2 provides tools for validating and visualizing URDF models
:::

## What's Next

In the next chapter, we'll explore physics simulation in Gazebo, learning how to create realistic simulation environments for testing robot behaviors.
