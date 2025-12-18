---
sidebar_label: ROS 2 Communication Model
sidebar_position: 2
---

# ROS 2 Communication Model

## Communication Patterns in ROS 2

ROS 2 provides several communication patterns that enable different types of interactions between nodes. Understanding these patterns is crucial for developing effective humanoid robotics applications.

## Nodes: The Building Blocks

Nodes are the fundamental execution units in ROS 2. Each node is typically responsible for a specific task and communicates with other nodes through various communication mechanisms.

### Creating a Node in Python (rclpy)

```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Humanoid Controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish-Subscribe Communication

Topics enable asynchronous, one-to-many communication using a publish-subscribe pattern. This is ideal for sensor data, where multiple nodes might need to process the same information.

### Publisher Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['hip_joint', 'knee_joint', 'ankle_joint']
        msg.position = [0.0, 0.0, 0.0]  # Example positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')
```

### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint positions: {msg.position}')
```

## Services: Request-Response Communication

Services provide synchronous, one-to-one communication with request-response semantics. This is useful for actions that require confirmation or return specific results.

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
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
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

Actions are designed for long-running tasks that may take significant time to complete. They provide feedback during execution and can be canceled.

### Action Server Example

```python
import rclpy
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

## Communication in Humanoid Robot Context

In humanoid robotics, these communication patterns serve specific purposes:

- **Topics**: Sensor data distribution (IMU, cameras, joint encoders), actuator commands
- **Services**: Calibration routines, configuration changes, diagnostics
- **Actions**: Walking patterns, complex movement sequences, planning tasks

## Quality of Service (QoS) Settings

For humanoid robotics applications, QoS settings are crucial for ensuring reliable communication:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# For critical sensor data
sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE
)

# For less critical data
status_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE
)
```

## Learning Objectives

After completing this chapter, you should understand:
- The different communication patterns in ROS 2 (topics, services, actions)
- When to use each communication pattern in humanoid robotics
- How to implement publishers, subscribers, services, and actions in Python
- The importance of QoS settings for robot applications
- How communication patterns enable coordinated robot behavior