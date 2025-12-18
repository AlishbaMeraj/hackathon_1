---
sidebar_label: Robot Structure with URDF
sidebar_position: 3
---

# Robot Structure with URDF

## Understanding URDF

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robotics, URDF is essential for defining the physical structure, kinematic chains, and visual/collision properties of robots.

URDF enables:
- Visualization of robot models in RViz
- Simulation in Gazebo and other physics engines
- Kinematic analysis and inverse kinematics calculations
- Collision detection and planning

## Basic URDF Structure

A URDF file consists of:
- **Links**: Rigid parts of the robot (e.g., torso, limbs)
- **Joints**: Connections between links that allow relative motion
- **Visual**: How the robot appears visually
- **Collision**: How the robot interacts with the environment physically
- **Inertial**: Mass properties for physics simulation

## Example URDF for a Simple Humanoid

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.15 0.1 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.1 0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Hip Joint (Example) -->
  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_leg"/>
    <origin xyz="0 -0.075 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Key URDF Elements for Humanoid Robots

### Links
Links represent rigid bodies in the robot. For humanoid robots, common links include:
- Torso/Body
- Head
- Arms (upper arm, lower arm, hand)
- Legs (thigh, shank, foot)

### Joints
Joints define the connection between links and their allowed motion:
- **revolute**: Rotational joint with limited range (like human joints)
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint
- **fixed**: No motion allowed
- **floating**: 6-DOF motion (rarely used in humanoid robots)

### Materials
Materials define the visual appearance:
```xml
<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

## URDF for Simulation Readiness

To make URDF models suitable for simulation:

1. **Complete kinematic chains**: Ensure all parts are connected
2. **Proper inertial properties**: Accurate mass and inertia values
3. **Collision models**: Simplified but accurate collision geometry
4. **Joint limits**: Realistic range of motion based on physical constraints

## Xacro: URDF's Macro System

For complex humanoid robots, Xacro (XML Macros) helps manage complexity:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- Macro for creating a limb -->
  <xacro:macro name="limb" params="side parent *origin">
    <joint name="${side}_hip_joint" type="revolute">
      <parent link="${parent}"/>
      <child link="${side}_hip"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="1 0 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>

    <link name="${side}_hip">
      <visual>
        <geometry>
          <cylinder length="0.1" radius="0.05"/>
        </geometry>
      </visual>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:limb side="left" parent="torso">
    <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  </xacro:limb>

  <xacro:limb side="right" parent="torso">
    <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
  </xacro:limb>
</robot>
```

## Best Practices for Humanoid URDF

1. **Start simple**: Begin with basic shapes and add detail gradually
2. **Use proper naming**: Follow consistent naming conventions
3. **Validate regularly**: Use tools like `check_urdf` to verify structure
4. **Consider physics**: Accurate inertial properties for realistic simulation
5. **Plan for ROS integration**: Consider how the URDF will interface with ROS nodes

## Tools for Working with URDF

- **RViz**: Visualize robot models in ROS
- **Gazebo**: Physics simulation environment
- **URDF viewer**: Standalone tools to check models
- **Robot state publisher**: Publish joint states for visualization

## Learning Objectives

After completing this chapter, you should understand:
- The structure and components of URDF files
- How to define links and joints for humanoid robots
- The importance of proper inertial and collision properties
- How to use Xacro to simplify complex URDF models
- Best practices for creating simulation-ready robot models
- How URDF integrates with the ROS ecosystem for humanoid robotics