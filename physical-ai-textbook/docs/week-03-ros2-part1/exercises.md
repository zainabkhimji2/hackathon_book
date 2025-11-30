---
id: ros2-part1-exercises
slug: /ros2-part1/exercises
title: ROS 2 Part 1 - Exercises
sidebar_position: 5
---

# ROS 2 Part 1: Exercises

These exercises are designed to progressively build your understanding of fundamental ROS 2 concepts. Each exercise includes a prompt, hints, and expected outcomes to guide your learning.

---

## Exercise 1: Workspace Setup and First Node

**Prompt**: Your first task is to set up a ROS 2 workspace, create a new Python package, and implement a simple publisher node. This node should publish a "Hello ROS 2 from [your_name]!" message to a topic named `/hello_topic`.

:::info Hint
*   Start by sourcing your ROS 2 environment (`/opt/ros/humble/setup.bash` or similar).
*   Create a new directory for your workspace (e.g., `~/ros2_ws/src`).
*   Use `ros2 pkg create --build-type ament_python my_first_ros_package` to create your package.
*   In your Python node, remember to `import rclpy` and `import std_msgs.msg`.
*   Initialize `rclpy`, create a node, create a publisher for `std_msgs.msg.String` on `/hello_topic`, and publish a message in a loop.
*   Make sure to add the necessary entry point in `setup.py` and build your workspace (`colcon build`).
:::

:::success Expected Outcome
You should be able to:
1.  Create a ROS 2 package named `my_first_ros_package`.
2.  Write a Python node (`hello_publisher.py`) that publishes `std_msgs.msg.String` messages.
3.  Run your node and see messages being published using `ros2 run my_first_ros_package hello_publisher` and `ros2 topic echo /hello_topic`.
:::

```bash
# 1. Source your ROS 2 environment (if not already sourced)
source /opt/ros/humble/setup.bash

# 2. Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 3. Create a new Python package
ros2 pkg create --build-type ament_python my_first_ros_package

# 4. Navigate into the package source directory
cd my_first_ros_package

# 5. Create the Python node file (src/my_first_ros_package/my_first_ros_package/hello_publisher.py)
# (Contents provided below)

# 6. Open and modify setup.py to include your executable:
#    (Add the following inside the 'entry_points' dictionary)
#    'console_scripts': [
#        'hello_publisher = my_first_ros_package.hello_publisher:main',
#    ],

# 7. Go back to the workspace root and build
cd ~/ros2_ws
colcon build

# 8. Source the workspace setup files
source install/setup.bash

# 9. Run your publisher node
ros2 run my_first_ros_package hello_publisher

# 10. In a new terminal, echo the topic to see messages
source install/setup.bash # (source again in new terminal)
ros2 topic echo /hello_topic
```

```python
# ~/ros2_ws/src/my_first_ros_package/my_first_ros_package/hello_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):

    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Claude Code! Count: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    hello_publisher = HelloPublisher()
    rclpy.spin(hello_publisher)
    hello_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Exercise 2: Custom Message Definition

**Prompt**: Define a custom ROS 2 message named `SensorData.msg` within your `my_first_ros_package`. This message should contain:
*   `std_msgs/Header header` (for timestamp and frame_id)
*   `string sensor_name`
*   `float32 temperature`
*   `uint16 humidity`

After defining the message, modify your `package.xml` and `CMakeLists.txt` to correctly build and make this message available. Verify its definition using `ros2 interface show`.

:::info Hint
*   Create a `msg` directory inside your `my_first_ros_package`.
*   In `package.xml`, add `build_depend` for `rosidl_default_generators` and `exec_depend` for `rosidl_default_runtime`. Also, add `buildtool_depend` for `ament_cmake_rosidl`.
*   In `CMakeLists.txt`:
    *   Find package `rosidl_default_generators`.
    *   Add `ament_target_dependencies(my_first_ros_package rclpy std_msgs)`.
    *   Use `rosidl_generate_interfaces` to specify your message file.
:::

:::success Expected Outcome
You should be able to:
1.  Define `SensorData.msg` correctly.
2.  Build your workspace successfully after modifications.
3.  Execute `ros2 interface show my_first_ros_package/msg/SensorData` and see the message definition.
:::

```bash
# 1. Navigate to your package root
cd ~/ros2_ws/src/my_first_ros_package

# 2. Create a 'msg' directory
mkdir msg

# 3. Create the SensorData.msg file
# ~/ros2_ws/src/my_first_ros_package/msg/SensorData.msg
# (Contents provided below)

# 4. Modify package.xml
# ~/ros2_ws/src/my_first_ros_package/package.xml
# (Add the following lines)
#   <buildtool_depend>ament_cmake_rosidl</buildtool_depend>
#   <build_depend>rosidl_default_generators</build_depend>
#   <exec_depend>rosidl_default_runtime</exec_depend>

# 5. Modify CMakeLists.txt
# ~/ros2_ws/src/my_first_ros_package/CMakeLists.txt
# (Add the following lines. Ensure they are placed correctly, e.g., after find_package(ament_cmake))
# find_package(rosidl_default_generators REQUIRED)
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/SensorData.msg"
#   DEPENDENCIES std_msgs
# )
#
# ament_export_interfaces(rosidl_interface_packages)

# 6. Go back to the workspace root and build
cd ~/ros2_ws
colcon build

# 7. Source the workspace setup files
source install/setup.bash

# 8. Verify the custom message definition
ros2 interface show my_first_ros_package/msg/SensorData
```

```text
# ~/ros2_ws/src/my_first_ros_package/msg/SensorData.msg
std_msgs/Header header
string sensor_name
float32 temperature
uint16 humidity
```

---

## Exercise 3: Advanced Publisher

**Prompt**: Create a new Python publisher node named `sensor_publisher.py` in `my_first_ros_package`. This node should:
1.  Publish messages of your custom `SensorData` type.
2.  Publish to a topic named `/sensor_data`.
3.  Publish at a rate of 2 Hz (every 0.5 seconds).
4.  Populate the `header` field with `timestamp` (using `self.get_clock().now().to_msg()`) and a `frame_id`.
5.  Set `sensor_name` to "RoomSensor".
6.  Generate a `temperature` value that increments by 0.1 with each message, resetting after 30.0.
7.  Generate a `humidity` value that alternates between 50 and 55.

:::info Hint
*   Import your custom message: `from my_first_ros_package.msg import SensorData`.
*   Use `self.get_clock().now().to_msg()` for the timestamp.
*   Implement `create_timer` for the publishing rate.
*   Use `rclpy.qos.qos_profile_sensor_data` for the QoS profile, as sensor data is often time-sensitive.
:::

:::success Expected Outcome
You should be able to:
1.  Run `sensor_publisher.py` and see messages published to `/sensor_data`.
2.  Use `ros2 topic echo /sensor_data` to observe the `SensorData` messages with incrementing temperature and alternating humidity, along with timestamps.
:::

```python
# ~/ros2_ws/src/my_first_ros_package/my_first_ros_package/sensor_publisher.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from my_first_ros_package.msg import SensorData # Import your custom message

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(SensorData, 'sensor_data', qos_profile)
        timer_period = 0.5  # seconds (2 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.temperature = 20.0
        self.humidity_state = 0

    def timer_callback(self):
        msg = SensorData()

        # Populate header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'

        msg.sensor_name = 'RoomSensor'

        # Increment temperature
        self.temperature += 0.1
        if self.temperature > 30.0:
            self.temperature = 20.0
        msg.temperature = float(f"{self.temperature:.1f}") # Format to one decimal place

        # Alternate humidity
        if self.humidity_state == 0:
            msg.humidity = 50
            self.humidity_state = 1
        else:
            msg.humidity = 55
            self.humidity_state = 0

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: Header(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}, frame_id={msg.header.frame_id}), '
            f'Name={msg.sensor_name}, Temp={msg.temperature}°C, Humidity={msg.humidity}%'
        )

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
# 1. Add your new executable to setup.py (similar to Exercise 1)
#    'console_scripts': [
#        'hello_publisher = my_first_ros_package.hello_publisher:main',
#        'sensor_publisher = my_first_ros_package.sensor_publisher:main', # Add this line
#    ],

# 2. Rebuild your workspace
cd ~/ros2_ws
colcon build

# 3. Source the workspace setup files
source install/setup.bash

# 4. Run your new sensor publisher node
ros2 run my_first_ros_package sensor_publisher

# 5. In a new terminal, echo the topic to see messages
source install/setup.bash
ros2 topic echo /sensor_data
```

---

## Exercise 4: Advanced Subscriber

**Prompt**: Create a new Python subscriber node named `sensor_subscriber.py` in `my_first_ros_package`. This node should:
1.  Subscribe to the `/sensor_data` topic, expecting messages of type `SensorData`.
2.  In its callback function, process the received message.
3.  Calculate the "heat index" using a simplified formula: `heat_index = temperature + (humidity / 5.0)`.
4.  Log the received `sensor_name`, `temperature`, `humidity`, and the calculated `heat_index`.

:::info Hint
*   Use `self.create_subscription(SensorData, 'sensor_data', self.listener_callback, qos_profile)`.
*   Ensure the QoS profile matches or is compatible with the publisher's QoS profile.
*   Access message fields directly, e.g., `msg.temperature`, `msg.humidity`.
:::

:::success Expected Outcome
You should be able to:
1.  Run `sensor_publisher.py` and `sensor_subscriber.py` concurrently.
2.  Observe the subscriber logging the received sensor data and the calculated heat index for each message.
:::

```python
# ~/ros2_ws/src/my_first_ros_package/my_first_ros_package/sensor_subscriber.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from my_first_ros_package.msg import SensorData # Import your custom message

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, # Must match or be compatible with publisher
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.subscription = self.create_subscription(
            SensorData,
            'sensor_data',
            self.listener_callback,
            qos_profile
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Calculate a simplified heat index
        heat_index = msg.temperature + (msg.humidity / 5.0)

        self.get_logger().info(
            f'Received: Name={msg.sensor_name}, Temp={msg.temperature}°C, Humidity={msg.humidity}% '
            f'-> Heat Index: {heat_index:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    rclpy.spin(sensor_subscriber)
    sensor_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
# 1. Add your new executable to setup.py (similar to Exercise 1 & 3)
#    'console_scripts': [
#        'hello_publisher = my_first_ros_package.hello_publisher:main',
#        'sensor_publisher = my_first_ros_package.sensor_publisher:main',
#        'sensor_subscriber = my_first_ros_package.sensor_subscriber:main', # Add this line
#    ],

# 2. Rebuild your workspace
cd ~/ros2_ws
colcon build

# 3. Source the workspace setup files
source install/setup.bash

# 4. Run the publisher in one terminal
ros2 run my_first_ros_package sensor_publisher

# 5. Run the subscriber in a new terminal
source install/setup.bash # (source again in new terminal)
ros2 run my_first_ros_package sensor_subscriber
```

---

## Exercise 5: Debugging a Pub-Sub System

**Prompt**: You are given two nodes, `buggy_publisher.py` and `buggy_subscriber.py`. When you run them, the subscriber does not receive any messages. Your task is to:
1.  Identify the bug using ROS 2 introspection tools.
2.  Explain *why* the bug prevents communication.
3.  Fix the bug in *both* files and demonstrate that communication is now working.

The buggy files are intentionally designed with a common pub-sub issue.

:::info Hint
*   Use `ros2 topic list` to see active topics.
*   Use `ros2 topic info <topic_name>` to inspect message type and publisher/subscriber details for a specific topic.
*   Use `ros2 node info <node_name>` to see what topics a node is publishing/subscribing to.
*   Pay close attention to topic names and message types.
:::

:::success Expected Outcome
You should be able to:
1.  Pinpoint the exact cause of the communication failure (e.g., mismatch in topic names).
2.  Modify the `buggy_publisher.py` and `buggy_subscriber.py` files to resolve the issue.
3.  Show both fixed nodes communicating successfully, with the subscriber logging messages.
:::

:::danger Common Bug
A common mistake is having a mismatch in the topic name between the publisher and the subscriber. They must publish to and subscribe from the *exact same topic name* for communication to occur. Also, ensure the message types are identical.
:::

```python
# ~/ros2_ws/src/my_first_ros_package/my_first_ros_package/buggy_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BuggyPublisher(Node):

    def __init__(self):
        super().__init__('buggy_publisher')
        # BUG: The topic name here is intentionally misspelled or different
        self.publisher_ = self.create_publisher(String, 'buggy_topic_name', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Buggy Message: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    buggy_publisher = BuggyPublisher()
    rclpy.spin(buggy_publisher)
    buggy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```python
# ~/ros2_ws/src/my_first_ros_package/my_first_ros_package/buggy_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BuggySubscriber(Node):

    def __init__(self):
        super().__init__('buggy_subscriber')
        # BUG: This subscriber is looking for a different topic name
        self.subscription = self.create_subscription(
            String,
            'incorrect_topic', # This topic name does not match the publisher's
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    buggy_subscriber = BuggySubscriber()
    rclpy.spin(buggy_subscriber)
    buggy_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
# 1. Add your new executable to setup.py
#    'console_scripts': [
#        'hello_publisher = my_first_ros_package.hello_publisher:main',
#        'sensor_publisher = my_first_ros_package.sensor_publisher:main',
#        'sensor_subscriber = my_first_ros_package.sensor_subscriber:main',
#        'buggy_publisher = my_first_ros_package.buggy_publisher:main', # Add this line
#        'buggy_subscriber = my_first_ros_package.buggy_subscriber:main', # Add this line
#    ],

# 2. Rebuild your workspace
cd ~/ros2_ws
colcon build

# 3. Source the workspace setup files
source install/setup.bash

# 4. Run the buggy publisher in one terminal
ros2 run my_first_ros_package buggy_publisher

# 5. Run the buggy subscriber in a new terminal
source install/setup.bash
ros2 run my_first_ros_package buggy_subscriber

# --- Debugging Steps ---
# Observe that the subscriber does not print anything.

# Use introspection tools:
# In a new terminal:
source install/setup.bash
ros2 topic list           # What topics are actually active?
ros2 topic info /buggy_topic_name # What are the details of the publisher's topic?
ros2 topic info /incorrect_topic  # What are the details of the subscriber's intended topic?

# --- Solution ---
# The bug is that the publisher publishes to 'buggy_topic_name' but the subscriber
# is listening on 'incorrect_topic'. To fix this, both must agree on a common topic name.

# --- Fixed buggy_publisher.py ---
# Change: self.publisher_ = self.create_publisher(String, 'buggy_topic_name', 10)
# To:     self.publisher_ = self.create_publisher(String, 'fixed_debug_topic', 10)

# --- Fixed buggy_subscriber.py ---
# Change: self.subscription = self.create_subscription(String, 'incorrect_topic', ...)
# To:     self.subscription = self.create_subscription(String, 'fixed_debug_topic', ...)

# After fixing and rebuilding:
# cd ~/ros2_ws && colcon build && source install/setup.bash
# ros2 run my_first_ros_package buggy_publisher # (now fixed)
# ros2 run my_first_ros_package buggy_subscriber # (now fixed)
# You should see the subscriber receiving messages.
```