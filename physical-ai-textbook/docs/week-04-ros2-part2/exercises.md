---
title: Week 4 Exercises
sidebar_label: Exercises
sidebar_position: 99
description: Hands-on exercises for advanced ROS 2 concepts including services, actions, custom messages, and launch files
keywords: [ROS 2, exercises, services, actions, custom messages, launch files, practice]
---

# Week 4 Exercises: Advanced ROS 2 Topics

These exercises will help you practice and solidify your understanding of services, actions, custom message types, and launch files in ROS 2.

## Prerequisites

Before starting these exercises, ensure you have:
- Completed Week 3 exercises and understand basic ROS 2 concepts
- A working ROS 2 Humble installation
- Your ROS 2 workspace set up (`~/ros2_ws`)
- Basic familiarity with Python and the terminal

## Exercise 1: Calculator Service

**Difficulty**: Beginner
**Estimated Time**: 30 minutes
**Learning Objective**: Create a simple service that performs mathematical operations

### Instructions

Create a calculator service that can perform addition, subtraction, multiplication, and division. The service should:

1. Accept two numbers and an operation type
2. Return the result of the operation
3. Handle division by zero gracefully

**Steps**:
1. Create a new package called `calculator_service`
2. Define a custom service type with two float numbers, an operation string, and a result
3. Implement the service server
4. Create a client to test the service

### Expected Outcome

You should be able to call the service from the command line or a client node and receive correct calculations:

```bash
# Example service call
ros2 service call /calculator calculator_interfaces/srv/Calculate "{a: 10.0, b: 5.0, operation: 'add'}"
```

Expected response: `result: 15.0`

### Hints

- Use `std_srvs` as a reference for service structure
- Remember to source your workspace after building
- Test each operation separately
- Consider what should happen with invalid operations

### Solution

**Step 1: Create the package structure**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python calculator_service --dependencies rclpy
cd calculator_service
mkdir -p srv
```

**Step 2: Define the custom service (calculator_service/srv/Calculate.srv)**

```
# Request
float64 a
float64 b
string operation  # 'add', 'subtract', 'multiply', 'divide'
---
# Response
float64 result
bool success
string message
```

**Step 3: Update package.xml**

Add these lines inside `<package>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Step 4: Update setup.py**

```python
from setuptools import setup

package_name = 'calculator_service'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Calculator service for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calculator_server = calculator_service.calculator_server:main',
            'calculator_client = calculator_service.calculator_client:main',
        ],
    },
)
```

**Step 5: Update CMakeLists.txt (create if using ament_cmake)**

If using ament_python, create `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.5)
project(calculator_service)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Calculate.srv"
)

ament_package()
```

**Step 6: Create the service server (calculator_service/calculator_server.py)**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from calculator_service.srv import Calculate


class CalculatorServer(Node):
    """Service server that performs basic mathematical operations."""

    def __init__(self):
        super().__init__('calculator_server')

        # Create service
        self.srv = self.create_service(
            Calculate,
            'calculator',
            self.calculate_callback
        )

        self.get_logger().info('Calculator service is ready')

    def calculate_callback(self, request, response):
        """
        Handle calculation requests.

        Args:
            request: Calculate.Request with a, b, and operation
            response: Calculate.Response with result, success, and message

        Returns:
            response: Populated response object
        """
        a = request.a
        b = request.b
        operation = request.operation.lower()

        self.get_logger().info(
            f'Received request: {a} {operation} {b}'
        )

        try:
            if operation == 'add':
                response.result = a + b
                response.success = True
                response.message = f'{a} + {b} = {response.result}'

            elif operation == 'subtract':
                response.result = a - b
                response.success = True
                response.message = f'{a} - {b} = {response.result}'

            elif operation == 'multiply':
                response.result = a * b
                response.success = True
                response.message = f'{a} × {b} = {response.result}'

            elif operation == 'divide':
                if b == 0.0:
                    response.result = 0.0
                    response.success = False
                    response.message = 'Error: Division by zero'
                else:
                    response.result = a / b
                    response.success = True
                    response.message = f'{a} ÷ {b} = {response.result}'
            else:
                response.result = 0.0
                response.success = False
                response.message = f'Error: Unknown operation "{operation}"'

        except Exception as e:
            response.result = 0.0
            response.success = False
            response.message = f'Error: {str(e)}'

        self.get_logger().info(f'Response: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    calculator_server = CalculatorServer()

    try:
        rclpy.spin(calculator_server)
    except KeyboardInterrupt:
        pass
    finally:
        calculator_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 7: Create the service client (calculator_service/calculator_client.py)**

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from calculator_service.srv import Calculate


class CalculatorClient(Node):
    """Client for the calculator service."""

    def __init__(self):
        super().__init__('calculator_client')

        # Create client
        self.client = self.create_client(Calculate, 'calculator')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for calculator service...')

    def send_request(self, a, b, operation):
        """
        Send a calculation request to the service.

        Args:
            a: First number
            b: Second number
            operation: Operation to perform

        Returns:
            Response from the service
        """
        request = Calculate.Request()
        request.a = float(a)
        request.b = float(b)
        request.operation = operation

        self.get_logger().info(
            f'Sending request: {a} {operation} {b}'
        )

        # Call service asynchronously
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)

    # Check command line arguments
    if len(sys.argv) < 4:
        print('Usage: calculator_client <num1> <num2> <operation>')
        print('Operations: add, subtract, multiply, divide')
        print('Example: calculator_client 10 5 add')
        return

    a = sys.argv[1]
    b = sys.argv[2]
    operation = sys.argv[3]

    client = CalculatorClient()
    response = client.send_request(a, b, operation)

    if response.success:
        client.get_logger().info(f'✓ {response.message}')
    else:
        client.get_logger().error(f'✗ {response.message}')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 8: Build and test**

```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select calculator_service
source install/setup.bash

# Terminal 1: Run the server
ros2 run calculator_service calculator_server

# Terminal 2: Test the client
ros2 run calculator_service calculator_client 10 5 add
ros2 run calculator_service calculator_client 20 4 divide
ros2 run calculator_service calculator_client 7 3 multiply
ros2 run calculator_service calculator_client 100 0 divide  # Test division by zero
```

---

## Exercise 2: Progress-Tracking Action

**Difficulty**: Intermediate
**Estimated Time**: 45 minutes
**Learning Objective**: Implement an action server with feedback for long-running tasks

### Instructions

Create an action that simulates a file download with progress tracking. The action should:

1. Accept a file size and download speed
2. Provide regular feedback on download progress
3. Return success when complete or allow cancellation

**Steps**:
1. Create a package called `download_action`
2. Define a custom action with file size input, progress feedback, and success result
3. Implement an action server that simulates downloading
4. Create a client that displays progress

### Expected Outcome

When running the action, you should see regular progress updates:

```
Downloading... 10% complete
Downloading... 20% complete
...
Download complete! 100MB downloaded in 10.5 seconds
```

### Hints

- Use `time.sleep()` to simulate download time
- Send feedback at regular intervals (every 10%)
- Handle goal cancellation gracefully
- Calculate estimated time remaining in feedback

### Solution

**Step 1: Create the package**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python download_action --dependencies rclpy action_msgs
cd download_action
mkdir -p action
```

**Step 2: Define the action (download_action/action/Download.action)**

```
# Goal: What we want to achieve
float32 file_size_mb      # Size of file to download in MB
float32 download_speed_mbps  # Download speed in MB/s
---
# Result: What we return when done
bool success
float32 total_time_seconds
string message
---
# Feedback: Progress updates during execution
float32 percent_complete
float32 mb_downloaded
float32 estimated_time_remaining
```

**Step 3: Update package.xml**

Add these dependencies:

```xml
<depend>action_msgs</depend>
<depend>rclpy</depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Step 4: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.5)
project(download_action)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Download.action"
  DEPENDENCIES action_msgs
)

ament_package()
```

**Step 5: Update setup.py**

```python
from setuptools import setup

package_name = 'download_action'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Download action with progress tracking',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'download_server = download_action.download_server:main',
            'download_client = download_action.download_client:main',
        ],
    },
)
```

**Step 6: Create action server (download_action/download_server.py)**

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from download_action.action import Download


class DownloadActionServer(Node):
    """Action server that simulates file download with progress tracking."""

    def __init__(self):
        super().__init__('download_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Download,
            'download_file',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Download action server is ready')

    def goal_callback(self, goal_request):
        """
        Handle new goal requests.

        Args:
            goal_request: The goal request from the client

        Returns:
            GoalResponse indicating acceptance or rejection
        """
        # Validate goal parameters
        if goal_request.file_size_mb <= 0:
            self.get_logger().warn('Rejected: file size must be positive')
            return GoalResponse.REJECT

        if goal_request.download_speed_mbps <= 0:
            self.get_logger().warn('Rejected: download speed must be positive')
            return GoalResponse.REJECT

        self.get_logger().info('Accepted new download goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Handle cancellation requests.

        Args:
            goal_handle: Handle to the goal being cancelled

        Returns:
            CancelResponse indicating acceptance of cancellation
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute the download action.

        Args:
            goal_handle: Handle to the executing goal

        Returns:
            Result object with download statistics
        """
        self.get_logger().info('Executing download...')

        # Get goal parameters
        file_size = goal_handle.request.file_size_mb
        speed = goal_handle.request.download_speed_mbps

        # Calculate total time needed
        total_time = file_size / speed

        # Feedback message
        feedback_msg = Download.Feedback()

        # Simulate download with progress updates
        start_time = time.time()
        update_interval = 0.1  # Update every 100ms

        while True:
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Download.Result()
                result.success = False
                result.total_time_seconds = time.time() - start_time
                result.message = 'Download cancelled by user'
                self.get_logger().info('Download cancelled')
                return result

            # Calculate progress
            elapsed_time = time.time() - start_time
            mb_downloaded = min(speed * elapsed_time, file_size)
            percent_complete = (mb_downloaded / file_size) * 100.0
            time_remaining = max(0, total_time - elapsed_time)

            # Publish feedback
            feedback_msg.mb_downloaded = mb_downloaded
            feedback_msg.percent_complete = percent_complete
            feedback_msg.estimated_time_remaining = time_remaining
            goal_handle.publish_feedback(feedback_msg)

            # Check if download is complete
            if mb_downloaded >= file_size:
                break

            # Sleep before next update
            time.sleep(update_interval)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Prepare result
        result = Download.Result()
        result.success = True
        result.total_time_seconds = time.time() - start_time
        result.message = f'Successfully downloaded {file_size}MB in {result.total_time_seconds:.2f}s'

        self.get_logger().info(result.message)
        return result


def main(args=None):
    rclpy.init(args=args)
    download_server = DownloadActionServer()

    try:
        rclpy.spin(download_server)
    except KeyboardInterrupt:
        pass
    finally:
        download_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 7: Create action client (download_action/download_client.py)**

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from download_action.action import Download


class DownloadActionClient(Node):
    """Client for the download action server."""

    def __init__(self):
        super().__init__('download_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Download,
            'download_file'
        )

    def send_goal(self, file_size_mb, download_speed_mbps):
        """
        Send a download goal to the action server.

        Args:
            file_size_mb: Size of file to download
            download_speed_mbps: Download speed
        """
        # Wait for server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = Download.Goal()
        goal_msg.file_size_mb = file_size_mb
        goal_msg.download_speed_mbps = download_speed_mbps

        self.get_logger().info(
            f'Sending goal: Download {file_size_mb}MB at {download_speed_mbps}MB/s'
        )

        # Send goal with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted, downloading...')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback from the action server.

        Args:
            feedback_msg: Feedback message with progress information
        """
        feedback = feedback_msg.feedback

        # Create progress bar
        bar_length = 30
        filled_length = int(bar_length * feedback.percent_complete / 100)
        bar = '█' * filled_length + '-' * (bar_length - filled_length)

        # Display progress
        self.get_logger().info(
            f'Progress: |{bar}| {feedback.percent_complete:.1f}% '
            f'({feedback.mb_downloaded:.2f}MB) '
            f'ETA: {feedback.estimated_time_remaining:.1f}s'
        )

    def get_result_callback(self, future):
        """Handle the final result."""
        result = future.result().result

        if result.success:
            self.get_logger().info(f'✓ {result.message}')
        else:
            self.get_logger().warn(f'✗ {result.message}')

        # Shutdown after receiving result
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Parse command line arguments
    if len(sys.argv) < 3:
        print('Usage: download_client <file_size_mb> <download_speed_mbps>')
        print('Example: download_client 100 10')
        return

    file_size = float(sys.argv[1])
    speed = float(sys.argv[2])

    client = DownloadActionClient()
    client.send_goal(file_size, speed)

    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

**Step 8: Build and test**

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select download_action
source install/setup.bash

# Terminal 1: Run server
ros2 run download_action download_server

# Terminal 2: Test client
ros2 run download_action download_client 50 5  # Download 50MB at 5MB/s
```

---

## Exercise 3: Custom Message Definition and Usage

**Difficulty**: Intermediate
**Estimated Time**: 40 minutes
**Learning Objective**: Create and use custom message types for complex data structures

### Instructions

Create a robot status monitoring system using custom messages. Define a message type that includes:

1. Robot ID and name
2. Battery level (percentage)
3. Position (x, y, z coordinates)
4. Status (idle, moving, charging, error)
5. Timestamp

Create a publisher that simulates robot status updates and a subscriber that monitors for low battery warnings.

### Expected Outcome

The subscriber should display robot status and warn when battery is low:

```
[Robot-Alpha] Status: moving, Battery: 75%, Position: (1.2, 3.4, 0.0)
[Robot-Alpha] Status: moving, Battery: 45%, Position: (2.1, 4.5, 0.0)
⚠️  [Robot-Alpha] LOW BATTERY WARNING! 15% remaining
```

### Hints

- Use nested message definitions if needed
- Consider using `std_msgs/Header` for timestamp
- Enum-like behavior can be achieved with constants
- Test message structure with `ros2 interface show`

### Solution

**Step 1: Create the package**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python robot_status --dependencies rclpy std_msgs geometry_msgs
cd robot_status
mkdir -p msg
```

**Step 2: Define custom message (robot_status/msg/RobotStatus.msg)**

```
# Robot identification
string robot_id
string robot_name

# Battery information
float32 battery_percentage  # 0.0 to 100.0

# Position in 3D space
geometry_msgs/Point position

# Status codes
uint8 STATUS_IDLE=0
uint8 STATUS_MOVING=1
uint8 STATUS_CHARGING=2
uint8 STATUS_ERROR=3
uint8 status

# Timestamp
builtin_interfaces/Time timestamp

# Optional: Additional sensor data
float32 temperature  # In Celsius
bool emergency_stop
string[] active_tasks
```

**Step 3: Update package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_status</name>
  <version>0.0.1</version>
  <description>Robot status monitoring with custom messages</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 4: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.5)
project(robot_status)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()
```

**Step 5: Update setup.py**

```python
from setuptools import setup

package_name = 'robot_status'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot status monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_publisher = robot_status.status_publisher:main',
            'status_monitor = robot_status.status_monitor:main',
        ],
    },
)
```

**Step 6: Create publisher (robot_status/status_publisher.py)**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_status.msg import RobotStatus
from geometry_msgs.msg import Point
import random
import math


class RobotStatusPublisher(Node):
    """Simulates a robot publishing its status periodically."""

    def __init__(self):
        super().__init__('robot_status_publisher')

        # Declare parameters
        self.declare_parameter('robot_id', 'robot_001')
        self.declare_parameter('robot_name', 'Alpha')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # Get parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.robot_name = self.get_parameter('robot_name').value
        rate = self.get_parameter('publish_rate').value

        # Create publisher
        self.publisher = self.create_publisher(
            RobotStatus,
            'robot_status',
            10
        )

        # Create timer
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.publish_status)

        # Simulation state
        self.battery = 100.0
        self.position = [0.0, 0.0, 0.0]
        self.angle = 0.0
        self.status = RobotStatus.STATUS_IDLE
        self.temperature = 25.0
        self.tasks = []

        self.get_logger().info(
            f'Robot {self.robot_name} ({self.robot_id}) status publisher started'
        )

    def publish_status(self):
        """Publish current robot status."""
        # Simulate robot behavior
        self.simulate_robot()

        # Create status message
        msg = RobotStatus()
        msg.robot_id = self.robot_id
        msg.robot_name = self.robot_name
        msg.battery_percentage = self.battery

        # Set position
        msg.position = Point()
        msg.position.x = self.position[0]
        msg.position.y = self.position[1]
        msg.position.z = self.position[2]

        # Set status
        msg.status = self.status

        # Set timestamp
        msg.timestamp = self.get_clock().now().to_msg()

        # Set additional data
        msg.temperature = self.temperature
        msg.emergency_stop = False
        msg.active_tasks = self.tasks

        # Publish
        self.publisher.publish(msg)

        # Log status
        status_names = {
            RobotStatus.STATUS_IDLE: 'idle',
            RobotStatus.STATUS_MOVING: 'moving',
            RobotStatus.STATUS_CHARGING: 'charging',
            RobotStatus.STATUS_ERROR: 'error'
        }

        self.get_logger().info(
            f'[{self.robot_name}] Status: {status_names[self.status]}, '
            f'Battery: {self.battery:.1f}%, '
            f'Position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f}), '
            f'Temp: {self.temperature:.1f}°C'
        )

    def simulate_robot(self):
        """Simulate robot state changes."""
        # Battery management
        if self.battery < 20.0:
            # Start charging when low
            self.status = RobotStatus.STATUS_CHARGING
            self.battery += 5.0  # Charge at 5% per update
            self.tasks = ['charging']
        elif self.battery >= 95.0 and self.status == RobotStatus.STATUS_CHARGING:
            # Stop charging when full
            self.status = RobotStatus.STATUS_IDLE
            self.battery = 100.0
            self.tasks = []
        elif self.status == RobotStatus.STATUS_CHARGING:
            # Continue charging
            self.battery += 5.0
        elif random.random() > 0.3:
            # Move around
            self.status = RobotStatus.STATUS_MOVING
            self.battery -= 1.5  # Drain battery while moving
            self.angle += random.uniform(-0.5, 0.5)
            self.position[0] += math.cos(self.angle) * 0.5
            self.position[1] += math.sin(self.angle) * 0.5
            self.temperature = 25.0 + random.uniform(0, 15)
            self.tasks = ['navigation', 'obstacle_avoidance']
        else:
            # Idle
            self.status = RobotStatus.STATUS_IDLE
            self.battery -= 0.2  # Small drain when idle
            self.temperature = max(25.0, self.temperature - 1.0)
            self.tasks = []

        # Clamp battery
        self.battery = max(0.0, min(100.0, self.battery))

        # Random error state
        if random.random() < 0.02:
            self.status = RobotStatus.STATUS_ERROR
            self.tasks = ['error_recovery']


def main(args=None):
    rclpy.init(args=args)
    publisher = RobotStatusPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 7: Create monitor (robot_status/status_monitor.py)**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_status.msg import RobotStatus


class RobotStatusMonitor(Node):
    """Monitors robot status and provides warnings."""

    def __init__(self):
        super().__init__('robot_status_monitor')

        # Declare parameters
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('high_temp_threshold', 35.0)

        # Get parameters
        self.low_battery = self.get_parameter('low_battery_threshold').value
        self.high_temp = self.get_parameter('high_temp_threshold').value

        # Create subscription
        self.subscription = self.create_subscription(
            RobotStatus,
            'robot_status',
            self.status_callback,
            10
        )

        # Track robots
        self.robots = {}

        self.get_logger().info('Robot status monitor started')
        self.get_logger().info(f'Low battery threshold: {self.low_battery}%')
        self.get_logger().info(f'High temperature threshold: {self.high_temp}°C')

    def status_callback(self, msg: RobotStatus):
        """
        Process incoming status messages.

        Args:
            msg: RobotStatus message
        """
        robot_id = msg.robot_id

        # Track robot
        if robot_id not in self.robots:
            self.robots[robot_id] = {'name': msg.robot_name}
            self.get_logger().info(f'Now monitoring robot: {msg.robot_name} ({robot_id})')

        # Status name mapping
        status_names = {
            RobotStatus.STATUS_IDLE: 'IDLE',
            RobotStatus.STATUS_MOVING: 'MOVING',
            RobotStatus.STATUS_CHARGING: 'CHARGING',
            RobotStatus.STATUS_ERROR: 'ERROR'
        }

        status_name = status_names.get(msg.status, 'UNKNOWN')

        # Build status string
        status_str = (
            f'[{msg.robot_name}] {status_name} | '
            f'Battery: {msg.battery_percentage:.1f}% | '
            f'Position: ({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}) | '
            f'Temp: {msg.temperature:.1f}°C'
        )

        if msg.active_tasks:
            status_str += f' | Tasks: {", ".join(msg.active_tasks)}'

        # Check for warnings
        warnings = []

        # Low battery warning
        if msg.battery_percentage < self.low_battery and msg.status != RobotStatus.STATUS_CHARGING:
            warnings.append(f'LOW BATTERY ({msg.battery_percentage:.1f}%)')

        # High temperature warning
        if msg.temperature > self.high_temp:
            warnings.append(f'HIGH TEMPERATURE ({msg.temperature:.1f}°C)')

        # Error status
        if msg.status == RobotStatus.STATUS_ERROR:
            warnings.append('ERROR STATE')

        # Emergency stop
        if msg.emergency_stop:
            warnings.append('EMERGENCY STOP ACTIVE')

        # Log appropriately
        if warnings:
            self.get_logger().warn(f'⚠️  {status_str} | WARNINGS: {", ".join(warnings)}')
        else:
            self.get_logger().info(status_str)


def main(args=None):
    rclpy.init(args=args)
    monitor = RobotStatusMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 8: Build and test**

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select robot_status
source install/setup.bash

# Verify message definition
ros2 interface show robot_status/msg/RobotStatus

# Terminal 1: Run publisher
ros2 run robot_status status_publisher

# Terminal 2: Run monitor
ros2 run robot_status status_monitor

# Terminal 3: Check messages
ros2 topic echo /robot_status
```

---

## Exercise 4: Launch File Creation

**Difficulty**: Intermediate
**Estimated Time**: 35 minutes
**Learning Objective**: Create launch files to start multi-node systems with parameters

### Instructions

Create a launch file that starts a complete robot monitoring system:

1. Launch 2 robot status publishers (from Exercise 3) with different robot IDs
2. Launch 1 status monitor
3. Set different parameters for each robot (names, publish rates)
4. Include a parameter file for the monitor

### Expected Outcome

Running the launch file should start all nodes with proper configuration:

```bash
ros2 launch robot_status multi_robot.launch.py
# All three nodes should start and communicate
```

### Hints

- Use Python launch files for flexibility
- Pass parameters using dictionaries
- Consider namespace isolation for multiple robots
- Test parameter loading with `ros2 param list`

### Solution

**Step 1: Create launch directory**

```bash
cd ~/ros2_ws/src/robot_status
mkdir -p launch config
```

**Step 2: Create parameter file (robot_status/config/monitor_params.yaml)**

```yaml
# Monitor node parameters
robot_status_monitor:
  ros__parameters:
    low_battery_threshold: 25.0
    high_temp_threshold: 40.0
```

**Step 3: Create launch file (robot_status/launch/multi_robot.launch.py)**

```python
#!/usr/bin/env python3
"""
Launch file for multi-robot status monitoring system.

This launch file starts:
- Two robot status publishers with different configurations
- One status monitor with parameters from file
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for multi-robot system."""

    # Declare launch arguments
    monitor_config_arg = DeclareLaunchArgument(
        'monitor_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_status'),
            'config',
            'monitor_params.yaml'
        ]),
        description='Path to monitor configuration file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Get launch configurations
    monitor_config = LaunchConfiguration('monitor_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot 1: Alpha
    robot_alpha = Node(
        package='robot_status',
        executable='status_publisher',
        name='robot_alpha_publisher',
        namespace='robot_alpha',
        parameters=[{
            'robot_id': 'robot_001',
            'robot_name': 'Alpha',
            'publish_rate': 2.0,  # 2 Hz
            'use_sim_time': use_sim_time
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Robot 2: Beta
    robot_beta = Node(
        package='robot_status',
        executable='status_publisher',
        name='robot_beta_publisher',
        namespace='robot_beta',
        parameters=[{
            'robot_id': 'robot_002',
            'robot_name': 'Beta',
            'publish_rate': 1.0,  # 1 Hz
            'use_sim_time': use_sim_time
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Status monitor
    status_monitor = Node(
        package='robot_status',
        executable='status_monitor',
        name='robot_status_monitor',
        parameters=[
            monitor_config,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Monitor both robots by subscribing to both topics
            # Note: This simple example monitors one topic
            # For multiple robots, you'd need to modify the monitor node
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Log info
    log_info = LogInfo(
        msg='Starting multi-robot monitoring system with 2 robots and 1 monitor'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(monitor_config_arg)
    ld.add_action(use_sim_time_arg)

    # Add log
    ld.add_action(log_info)

    # Add nodes
    ld.add_action(robot_alpha)
    ld.add_action(robot_beta)
    ld.add_action(status_monitor)

    return ld
```

**Step 4: Create advanced launch file with conditions (robot_status/launch/configurable_robot.launch.py)**

```python
#!/usr/bin/env python3
"""
Configurable launch file with conditional execution and groups.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate configurable launch description."""

    # Declare arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='TestRobot',
        description='Name of the robot'
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_999',
        description='Unique robot identifier'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='1.0',
        description='Status publishing rate in Hz'
    )

    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Whether to start the status monitor'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for all nodes'
    )

    # Get configurations
    robot_name = LaunchConfiguration('robot_name')
    robot_id = LaunchConfiguration('robot_id')
    publish_rate = LaunchConfiguration('publish_rate')
    enable_monitor = LaunchConfiguration('enable_monitor')
    namespace = LaunchConfiguration('namespace')

    # Robot publisher node
    robot_publisher = Node(
        package='robot_status',
        executable='status_publisher',
        name='status_publisher',
        parameters=[{
            'robot_id': robot_id,
            'robot_name': robot_name,
            'publish_rate': publish_rate
        }],
        output='screen'
    )

    # Monitor node (conditional)
    monitor_node = Node(
        package='robot_status',
        executable='status_monitor',
        name='status_monitor',
        parameters=[{
            'low_battery_threshold': 30.0,
            'high_temp_threshold': 35.0
        }],
        output='screen',
        condition=IfCondition(enable_monitor)
    )

    # Group with namespace
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            robot_publisher,
            monitor_node
        ]
    )

    # Build launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(robot_name_arg)
    ld.add_action(robot_id_arg)
    ld.add_action(publish_rate_arg)
    ld.add_action(enable_monitor_arg)
    ld.add_action(namespace_arg)

    # Add startup log
    ld.add_action(LogInfo(msg=['Launching robot with name: ', robot_name]))

    # Add nodes
    ld.add_action(namespaced_group)

    return ld
```

**Step 5: Update setup.py to include launch files**

```python
import os
from glob import glob
from setuptools import setup

package_name = 'robot_status'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Robot status monitoring',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_publisher = robot_status.status_publisher:main',
            'status_monitor = robot_status.status_monitor:main',
        ],
    },
)
```

**Step 6: Build and test**

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select robot_status
source install/setup.bash

# Test multi-robot launch
ros2 launch robot_status multi_robot.launch.py

# Test configurable launch with arguments
ros2 launch robot_status configurable_robot.launch.py robot_name:=Charlie robot_id:=robot_003 publish_rate:=0.5

# Test with namespace
ros2 launch robot_status configurable_robot.launch.py namespace:=production

# Test without monitor
ros2 launch robot_status configurable_robot.launch.py enable_monitor:=false

# Check running nodes
ros2 node list

# Check parameters
ros2 param list
```

---

## Exercise 5: Sensor Pipeline Integration

**Difficulty**: Advanced
**Estimated Time**: 60 minutes
**Learning Objective**: Integrate multiple ROS 2 concepts into a complete sensor processing pipeline

### Instructions

Create a complete sensor data processing pipeline that demonstrates:

1. Custom message for sensor data (temperature, humidity, pressure)
2. Publisher simulating sensor readings
3. Service for sensor calibration
4. Action for data collection over time
5. Launch file to start the complete system

The system should:
- Publish sensor readings at 10Hz
- Provide a calibration service to adjust offsets
- Support an action to collect N samples and compute statistics (min, max, average)

### Expected Outcome

A complete working sensor system where you can:
- Monitor live sensor data
- Calibrate sensors via service calls
- Request data collection actions that return statistics

### Hints

- Reuse patterns from previous exercises
- Think about data flow: sensor → calibration → consumers
- Consider using QoS settings for reliable sensor data
- Add proper error handling for all operations

### Solution

**Step 1: Create package structure**

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python sensor_pipeline --dependencies rclpy std_msgs
cd sensor_pipeline
mkdir -p msg srv action launch config
```

**Step 2: Define custom message (sensor_pipeline/msg/SensorData.msg)**

```
# Raw sensor readings
float32 temperature    # Celsius
float32 humidity       # Percentage (0-100)
float32 pressure       # kPa

# Metadata
builtin_interfaces/Time timestamp
string sensor_id
uint32 sequence_number

# Calibration status
bool is_calibrated
float32 temperature_offset
float32 humidity_offset
float32 pressure_offset
```

**Step 3: Define calibration service (sensor_pipeline/srv/Calibrate.srv)**

```
# Request: calibration offsets to apply
float32 temperature_offset
float32 humidity_offset
float32 pressure_offset
---
# Response: confirmation
bool success
string message
float32 previous_temp_offset
float32 previous_humidity_offset
float32 previous_pressure_offset
```

**Step 4: Define data collection action (sensor_pipeline/action/CollectData.action)**

```
# Goal: how many samples to collect
uint32 num_samples
float32 sample_rate  # Hz
---
# Result: statistics
uint32 samples_collected
float32 temp_min
float32 temp_max
float32 temp_avg
float32 humidity_min
float32 humidity_max
float32 humidity_avg
float32 pressure_min
float32 pressure_max
float32 pressure_avg
bool success
string message
---
# Feedback: progress
uint32 samples_collected
float32 percent_complete
```

**Step 5: Update package.xml**

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>sensor_pipeline</name>
  <version>0.0.1</version>
  <description>Sensor data processing pipeline</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>action_msgs</depend>

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**Step 6: Create CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.5)
project(sensor_pipeline)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "srv/Calibrate.srv"
  "action/CollectData.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

ament_package()
```

**Step 7: Create sensor node (sensor_pipeline/sensor_node.py)**

```python
#!/usr/bin/env python3
"""
Sensor node that publishes data and provides calibration service.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_pipeline.msg import SensorData
from sensor_pipeline.srv import Calibrate
import random
import math


class SensorNode(Node):
    """Simulated environmental sensor with calibration support."""

    def __init__(self):
        super().__init__('sensor_node')

        # Parameters
        self.declare_parameter('sensor_id', 'ENV_SENSOR_001')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('base_temperature', 22.0)
        self.declare_parameter('base_humidity', 45.0)
        self.declare_parameter('base_pressure', 101.3)

        self.sensor_id = self.get_parameter('sensor_id').value
        rate = self.get_parameter('publish_rate').value
        self.base_temp = self.get_parameter('base_temperature').value
        self.base_humidity = self.get_parameter('base_humidity').value
        self.base_pressure = self.get_parameter('base_pressure').value

        # Calibration offsets
        self.temp_offset = 0.0
        self.humidity_offset = 0.0
        self.pressure_offset = 0.0
        self.is_calibrated = False

        # Sequence counter
        self.sequence = 0

        # QoS profile for sensor data (reliable, keep last 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher
        self.publisher = self.create_publisher(
            SensorData,
            'sensor_data',
            qos
        )

        # Service
        self.calibration_service = self.create_service(
            Calibrate,
            'calibrate_sensor',
            self.calibrate_callback
        )

        # Timer for publishing
        timer_period = 1.0 / rate
        self.timer = self.create_timer(timer_period, self.publish_data)

        # Simulation state
        self.time = 0.0

        self.get_logger().info(f'Sensor node started: {self.sensor_id}')
        self.get_logger().info(f'Publishing at {rate} Hz')

    def simulate_readings(self):
        """
        Generate simulated sensor readings with realistic variation.

        Returns:
            tuple: (temperature, humidity, pressure)
        """
        # Add time-based drift and random noise
        temp = self.base_temp + math.sin(self.time * 0.1) * 2.0 + random.gauss(0, 0.5)
        humidity = self.base_humidity + math.cos(self.time * 0.15) * 10.0 + random.gauss(0, 2.0)
        pressure = self.base_pressure + math.sin(self.time * 0.05) * 0.5 + random.gauss(0, 0.1)

        # Clamp humidity
        humidity = max(0.0, min(100.0, humidity))

        self.time += 0.1

        return temp, humidity, pressure

    def publish_data(self):
        """Publish sensor data."""
        # Get raw readings
        temp, humidity, pressure = self.simulate_readings()

        # Apply calibration
        temp += self.temp_offset
        humidity += self.humidity_offset
        pressure += self.pressure_offset

        # Create message
        msg = SensorData()
        msg.temperature = temp
        msg.humidity = humidity
        msg.pressure = pressure
        msg.timestamp = self.get_clock().now().to_msg()
        msg.sensor_id = self.sensor_id
        msg.sequence_number = self.sequence
        msg.is_calibrated = self.is_calibrated
        msg.temperature_offset = self.temp_offset
        msg.humidity_offset = self.humidity_offset
        msg.pressure_offset = self.pressure_offset

        # Publish
        self.publisher.publish(msg)
        self.sequence += 1

    def calibrate_callback(self, request, response):
        """
        Handle calibration requests.

        Args:
            request: Calibrate.Request
            response: Calibrate.Response

        Returns:
            response: Populated response
        """
        # Store previous offsets
        response.previous_temp_offset = self.temp_offset
        response.previous_humidity_offset = self.humidity_offset
        response.previous_pressure_offset = self.pressure_offset

        # Apply new offsets
        self.temp_offset = request.temperature_offset
        self.humidity_offset = request.humidity_offset
        self.pressure_offset = request.pressure_offset
        self.is_calibrated = True

        response.success = True
        response.message = (
            f'Calibration applied: Temp offset: {self.temp_offset:.2f}°C, '
            f'Humidity offset: {self.humidity_offset:.2f}%, '
            f'Pressure offset: {self.pressure_offset:.2f} kPa'
        )

        self.get_logger().info(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    sensor_node = SensorNode()

    try:
        rclpy.spin(sensor_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 8: Create data collector action server (sensor_pipeline/data_collector.py)**

```python
#!/usr/bin/env python3
"""
Data collector action server for sensor statistics.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from sensor_pipeline.action import CollectData
from sensor_pipeline.msg import SensorData


class DataCollectorServer(Node):
    """Action server that collects sensor data and computes statistics."""

    def __init__(self):
        super().__init__('data_collector_server')

        # Action server
        self._action_server = ActionServer(
            self,
            CollectData,
            'collect_sensor_data',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Subscription for sensor data
        self.subscription = self.create_subscription(
            SensorData,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Latest sensor reading
        self.latest_reading = None
        self.data_available = False

        self.get_logger().info('Data collector action server ready')

    def sensor_callback(self, msg: SensorData):
        """Store latest sensor reading."""
        self.latest_reading = msg
        self.data_available = True

    def goal_callback(self, goal_request):
        """Validate goal requests."""
        if goal_request.num_samples <= 0:
            self.get_logger().warn('Rejected: num_samples must be positive')
            return GoalResponse.REJECT

        if goal_request.sample_rate <= 0:
            self.get_logger().warn('Rejected: sample_rate must be positive')
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted goal: Collect {goal_request.num_samples} samples '
            f'at {goal_request.sample_rate} Hz'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancellation."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Execute data collection.

        Args:
            goal_handle: Handle to the executing goal

        Returns:
            Result with statistics
        """
        self.get_logger().info('Starting data collection...')

        num_samples = goal_handle.request.num_samples
        sample_rate = goal_handle.request.sample_rate

        # Wait for sensor data
        if not self.data_available:
            self.get_logger().info('Waiting for sensor data...')
            while not self.data_available:
                rclpy.spin_once(self, timeout_sec=0.1)

        # Collect samples
        samples = []
        feedback_msg = CollectData.Feedback()

        # Calculate sleep time between samples
        import time
        sleep_time = 1.0 / sample_rate

        for i in range(num_samples):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = CollectData.Result()
                result.success = False
                result.message = 'Collection cancelled'
                result.samples_collected = len(samples)
                return result

            # Collect sample
            if self.latest_reading is not None:
                samples.append({
                    'temp': self.latest_reading.temperature,
                    'humidity': self.latest_reading.humidity,
                    'pressure': self.latest_reading.pressure
                })

            # Send feedback
            feedback_msg.samples_collected = len(samples)
            feedback_msg.percent_complete = (len(samples) / num_samples) * 100.0
            goal_handle.publish_feedback(feedback_msg)

            # Wait for next sample
            time.sleep(sleep_time)

        # Compute statistics
        if not samples:
            goal_handle.succeed()
            result = CollectData.Result()
            result.success = False
            result.message = 'No samples collected'
            result.samples_collected = 0
            return result

        # Calculate min, max, avg
        temps = [s['temp'] for s in samples]
        humidities = [s['humidity'] for s in samples]
        pressures = [s['pressure'] for s in samples]

        result = CollectData.Result()
        result.samples_collected = len(samples)
        result.temp_min = min(temps)
        result.temp_max = max(temps)
        result.temp_avg = sum(temps) / len(temps)
        result.humidity_min = min(humidities)
        result.humidity_max = max(humidities)
        result.humidity_avg = sum(humidities) / len(humidities)
        result.pressure_min = min(pressures)
        result.pressure_max = max(pressures)
        result.pressure_avg = sum(pressures) / len(pressures)
        result.success = True
        result.message = f'Successfully collected {len(samples)} samples'

        goal_handle.succeed()

        self.get_logger().info(
            f'Collection complete: Temp: {result.temp_min:.2f}-{result.temp_max:.2f}°C '
            f'(avg: {result.temp_avg:.2f}°C)'
        )

        return result


def main(args=None):
    rclpy.init(args=args)
    server = DataCollectorServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Step 9: Create action client (sensor_pipeline/data_collector_client.py)**

```python
#!/usr/bin/env python3
"""
Client for data collection action.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_pipeline.action import CollectData


class DataCollectorClient(Node):
    """Client for data collection action."""

    def __init__(self):
        super().__init__('data_collector_client')

        self._action_client = ActionClient(
            self,
            CollectData,
            'collect_sensor_data'
        )

    def send_goal(self, num_samples, sample_rate):
        """Send collection goal."""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = CollectData.Goal()
        goal_msg.num_samples = num_samples
        goal_msg.sample_rate = sample_rate

        self.get_logger().info(
            f'Requesting {num_samples} samples at {sample_rate} Hz'
        )

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted, collecting data...')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback."""
        feedback = feedback_msg.feedback

        self.get_logger().info(
            f'Progress: {feedback.samples_collected} samples '
            f'({feedback.percent_complete:.1f}% complete)'
        )

    def get_result_callback(self, future):
        """Handle result."""
        result = future.result().result

        if result.success:
            self.get_logger().info('=== Collection Complete ===')
            self.get_logger().info(f'Samples collected: {result.samples_collected}')
            self.get_logger().info(
                f'Temperature: {result.temp_min:.2f} - {result.temp_max:.2f}°C '
                f'(avg: {result.temp_avg:.2f}°C)'
            )
            self.get_logger().info(
                f'Humidity: {result.humidity_min:.2f} - {result.humidity_max:.2f}% '
                f'(avg: {result.humidity_avg:.2f}%)'
            )
            self.get_logger().info(
                f'Pressure: {result.pressure_min:.2f} - {result.pressure_max:.2f} kPa '
                f'(avg: {result.pressure_avg:.2f} kPa)'
            )
        else:
            self.get_logger().error(f'Collection failed: {result.message}')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print('Usage: data_collector_client <num_samples> <sample_rate>')
        print('Example: data_collector_client 50 5')
        return

    num_samples = int(sys.argv[1])
    sample_rate = float(sys.argv[2])

    client = DataCollectorClient()
    client.send_goal(num_samples, sample_rate)

    rclpy.spin(client)


if __name__ == '__main__':
    main()
```

**Step 10: Create launch file (sensor_pipeline/launch/sensor_system.launch.py)**

```python
#!/usr/bin/env python3
"""
Complete sensor pipeline launch file.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for complete sensor system."""

    # Arguments
    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id',
        default_value='ENV_SENSOR_001',
        description='Sensor identifier'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Sensor data publishing rate in Hz'
    )

    # Get configurations
    sensor_id = LaunchConfiguration('sensor_id')
    publish_rate = LaunchConfiguration('publish_rate')

    # Sensor node
    sensor_node = Node(
        package='sensor_pipeline',
        executable='sensor_node',
        name='sensor_node',
        parameters=[{
            'sensor_id': sensor_id,
            'publish_rate': publish_rate,
            'base_temperature': 22.0,
            'base_humidity': 45.0,
            'base_pressure': 101.3
        }],
        output='screen'
    )

    # Data collector server
    collector_server = Node(
        package='sensor_pipeline',
        executable='data_collector',
        name='data_collector_server',
        output='screen'
    )

    # Build launch description
    ld = LaunchDescription()

    ld.add_action(sensor_id_arg)
    ld.add_action(publish_rate_arg)
    ld.add_action(sensor_node)
    ld.add_action(collector_server)

    return ld
```

**Step 11: Update setup.py**

```python
import os
from glob import glob
from setuptools import setup

package_name = 'sensor_pipeline'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Sensor data processing pipeline',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_pipeline.sensor_node:main',
            'data_collector = sensor_pipeline.data_collector:main',
            'data_collector_client = sensor_pipeline.data_collector_client:main',
        ],
    },
)
```

**Step 12: Build and test**

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select sensor_pipeline
source install/setup.bash

# Launch complete system
ros2 launch sensor_pipeline sensor_system.launch.py

# In another terminal, monitor sensor data
ros2 topic echo /sensor_data

# Test calibration service
ros2 service call /calibrate_sensor sensor_pipeline/srv/Calibrate "{temperature_offset: -2.0, humidity_offset: 5.0, pressure_offset: 0.5}"

# Test data collection action
ros2 run sensor_pipeline data_collector_client 100 5  # Collect 100 samples at 5 Hz
```

---

## Summary

Congratulations on completing Week 4 exercises! You have practiced:

1. **Service Implementation**: Creating request-response patterns
2. **Action Servers**: Building long-running tasks with feedback
3. **Custom Messages**: Defining complex data structures
4. **Launch Files**: Orchestrating multi-node systems
5. **Integration**: Combining all concepts into a complete pipeline

These skills form the foundation for building real ROS 2 robotics applications. Continue practicing by extending these examples or creating your own projects.

## Next Steps

- Explore ROS 2 lifecycle nodes for managed startup/shutdown
- Learn about ROS 2 component composition for efficiency
- Study TF2 for coordinate frame transformations
- Practice with real sensor hardware if available
- Join the ROS community and contribute to open source projects
