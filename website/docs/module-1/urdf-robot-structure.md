---
sidebar_position: 3
title: 'Robot Structure with URDF'
description: 'Understanding Unified Robot Description Format for humanoid robot modeling and simulation readiness'
---

# Robot Structure with URDF

## Overview

The Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF is crucial for defining the physical structure, kinematic chains, and visual/physical properties. This chapter covers the fundamentals of URDF and how to create models suitable for humanoid robots and simulation.

## What is URDF?

URDF (Unified Robot Description Format) is an XML format that describes a robot's physical properties including:

- **Kinematic structure**: How different parts of the robot are connected
- **Visual properties**: How the robot appears in visualization tools
- **Collision properties**: How the robot interacts with its environment in simulation
- **Inertial properties**: Mass, center of mass, and inertia for physics simulation
- **Joint properties**: How different parts move relative to each other

## Basic URDF Structure

A typical URDF file contains several main elements:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid parts of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints define how links connect and move -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.02"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links: Defining Robot Parts

Links represent rigid parts of the robot. Each link can have multiple properties:

### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Options: box, cylinder, sphere, mesh -->
    <box size="0.1 0.2 0.05"/>
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="0.1 0.2 0.05"/>
  </geometry>
</collision>
```

### Inertial Properties
```xml
<inertial>
  <mass value="0.5"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.003"/>
</inertial>
```

## Joints: Connecting Robot Parts

Joints define how links connect and move relative to each other. Common joint types include:

- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Continuous rotation (like a wheel)
- **Prismatic**: Linear sliding movement
- **Floating**: 6 degrees of freedom
- **Planar**: Movement in a plane

### Joint Definition Example
```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="1.5" effort="50.0" velocity="2.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## URDF for Humanoid Robots

Humanoid robots have specific structural requirements that need to be captured in URDF:

### Typical Humanoid Structure
```
base_link (usually pelvis/torso)
├── left_leg
│   ├── left_upper_leg
│   ├── left_lower_leg
│   └── left_foot
├── right_leg
│   ├── right_upper_leg
│   ├── right_lower_leg
│   └── right_foot
├── torso
│   ├── head
│   ├── left_arm
│   │   ├── left_upper_arm
│   │   ├── left_lower_arm
│   │   └── left_hand
│   └── right_arm
│       ├── right_upper_arm
│       ├── right_lower_arm
│       └── right_hand
```

### Example: Simple Humanoid Torso
```xml
<link name="torso">
  <visual>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="grey">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.4"/>
  </inertial>
</link>

<joint name="torso_to_head" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.785" upper="0.785" effort="10.0" velocity="1.0"/>
</joint>

<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
  </inertial>
</link>
```

## Simulation Readiness

To make URDF models suitable for simulation, consider these factors:

### Collision Geometry
- Use simpler shapes for collision than for visual representation
- Ensure all parts have collision geometry to prevent unexpected behavior
- Avoid overlapping collision meshes

### Inertial Properties
- Accurately calculate mass and inertia values
- Use CAD tools or approximation formulas for complex shapes
- Ensure the center of mass is correctly positioned

### Joint Limits and Dynamics
- Set realistic joint limits based on physical constraints
- Configure appropriate damping and friction values
- Set reasonable effort and velocity limits

## Working with URDF in ROS 2

### Launching with Robot State Publisher
```xml
<launch>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher">
    <param name="robot_description" value="$(var robot_description)"/>
  </node>
</launch>
```

### Loading URDF in Code
```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF

class URDFLoader(Node):
    def __init__(self):
        super().__init__('urdf_loader')
        # Load URDF from parameter server or file
        self.declare_parameter('robot_description', '')
        robot_desc = self.get_parameter('robot_description').value
        self.robot = URDF.from_xml_string(robot_desc)
```

## Best Practices for Humanoid URDF

1. **Use consistent naming**: Follow a clear convention for link and joint names
2. **Start simple**: Begin with a basic skeleton and add complexity gradually
3. **Validate regularly**: Use tools like `check_urdf` to verify your model
4. **Separate files**: Break complex robots into multiple URDF files with xacro
5. **Consider mounting points**: Include attachment points for sensors and accessories

## Visualization Tools

Several tools help visualize and validate URDF models:

- **RViz2**: Real-time visualization of robot models
- **Robot Model Display**: Shows the robot with all links and joints
- **TF Tree**: Visualizes the transformation relationships between links
- **Gazebo**: Physics simulation environment for testing

## Summary

URDF is fundamental to representing robot structure in ROS 2. For humanoid robots, careful attention to kinematic structure, inertial properties, and joint constraints is essential for both simulation and real-world operation. Properly structured URDF files enable effective visualization, simulation, and control of humanoid robotic systems.