---
sidebar_position: 2
title: 'ROS 2 Communication Model'
description: 'Understanding nodes, topics, services, and basic rclpy-based agent and controller flows'
---

# ROS 2 Communication Model

## Overview

The ROS 2 communication model is the backbone of all robotic systems. Understanding how different components communicate is crucial for building effective humanoid robots. This chapter covers the fundamental communication patterns: nodes, topics, services, and actions, with practical examples using rclpy (Python client library).

## Nodes: The Building Blocks

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically performs a specific function within the larger robotic system:

- Sensor nodes: Process data from sensors
- Controller nodes: Send commands to actuators
- Planning nodes: Generate motion plans
- Perception nodes: Process sensory data to understand the environment

### Creating a Node with rclpy

Here's a basic example of creating a node in Python using rclpy:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    try:
        rclpy.spin(minimal_node)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, one-to-many communication through a publish-subscribe pattern. This is ideal for sensor data that multiple components might need to process simultaneously.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services: Request-Response Communication

Services provide synchronous, request-response communication, which is useful for operations that require a response before continuing.

### Service Server Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {response.sum}')
        return response
```

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    def send_request(self, a, b):
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.future = self.cli.call_async(request)
```

## Actions: Goal-Based Communication

Actions are used for long-running tasks that may take time to complete, provide feedback during execution, and can be canceled.

### Action Server Example

```python
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## Quality of Service (QoS) Settings

QoS settings allow you to configure communication behavior based on your application's requirements:

- **Reliability**: Best effort vs reliable delivery
- **Durability**: Volatile vs transient local
- **History**: Keep all messages vs keep last N messages
- **Deadline**: Maximum time between publications
- **Liveliness**: How to detect if a publisher is alive

## Agent and Controller Flows in Humanoid Robots

In humanoid robotics, communication patterns often follow these patterns:

1. **Sensor Fusion**: Multiple sensor nodes publish data to a central fusion node
2. **Control Loop**: Controller nodes subscribe to sensor data and publish actuator commands
3. **Planning**: High-level planners communicate with low-level controllers via services or actions
4. **State Management**: Robot state is shared via topics to all interested components

### Example: Balance Controller Communication

```python
# Balance controller subscribes to IMU data and publishes joint commands
class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        # Publish joint commands
        self.joint_pub = self.create_publisher(
            JointCommand, '/joint_commands', 10)

    def imu_callback(self, msg):
        # Process IMU data to determine balance adjustments
        joint_cmd = self.calculate_balance_correction(msg)
        self.joint_pub.publish(joint_cmd)
```

## Summary

The ROS 2 communication model provides flexible patterns for different types of interactions in robotic systems. Topics for asynchronous data sharing, services for request-response operations, and actions for long-running tasks with feedback. Understanding these patterns is essential for designing effective communication architectures in humanoid robotics applications.