---
title: ROS 2 Publisher-Subscriber Pattern
description: A comprehensive guide to the ROS 2 Publisher-Subscriber pattern for university-level robotics students.
---

# ROS 2 Publisher-Subscriber Pattern

Welcome to this chapter, where we delve into the fundamental communication paradigm in ROS 2: the Publisher-Subscriber pattern. In the previous chapters, you've been introduced to the basics of ROS 2, learned how to set up your environment, and created your first Python node. Now, we build upon that foundation to explore how different components (nodes) in a ROS 2 system can communicate with each other asynchronously and efficiently.

The publish-subscribe pattern is a messaging model where senders of messages (publishers) do not programmatically send messages directly to specific receivers (subscribers). Instead, publishers categorize messages into classes without knowledge of which subscribers, if any, there may be. Similarly, subscribers express interest in one or more message classes and only receive messages that are of interest, without knowledge of which publishers, if any, are sending those messages.

In the context of ROS 2, this pattern is implemented using **topics**. A topic is a named bus over which nodes exchange messages. A node that sends messages on a topic is called a **publisher** (often referred to as a "talker"), and a node that receives messages from a topic is called a **subscriber** (often referred to as a "listener"). This decoupled communication mechanism is incredibly powerful, allowing for modular and flexible robot software architectures.

:::tip Key Takeaway
The Publisher-Subscriber pattern in ROS 2 uses **topics** to enable asynchronous, decoupled communication between **talker (publisher)** and **listener (subscriber)** nodes.
:::

## Creating a ROS 2 Talker (Publisher) Node (Python)

Let's begin by creating a simple ROS 2 publisher node in Python. This node, which we'll call `minimal_publisher`, will continuously publish string messages to a topic named `topic`. Each message will include a counter, demonstrating how data can be incremented and shared over time. This builds directly on our `03-first-python-node.md` chapter's understanding of node creation.

First, navigate to your ROS 2 workspace's `src` directory. If you've been following the previous chapters, you should have a package set up. Let's assume your package is named `my_py_pkg`.

Create a new Python file named `minimal_publisher.py` inside the `my_py_pkg/my_py_pkg` directory (i.e., `src/my_py_pkg/my_py_pkg/minimal_publisher.py`).

```python title="minimal_publisher.py" showLineNumbers
[View on GitHub](https://github.com/zainabkhimji2/hackathon_book/blob/main/ros2_ws/src/my_py_pkg/my_py_pkg/minimal_publisher.py)
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # Create a publisher to the 'topic' topic, with String message type and QoS profile depth of 10
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.i = 0
        # Create a timer that calls the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Minimal Publisher Node has been started.')

    def timer_callback(self):
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'
        # Publish the message
        self.publisher_.publish(msg)
        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our MinimalPublisher node
    minimal_publisher = MinimalPublisher()

    # Spin the node, keeping it alive and allowing its callbacks to be called
    # This will block until the node is shut down
    rclpy.spin(minimal_publisher)

    # Destroy the node once rclpy.spin() returns (e.g., due to Ctrl+C)
    minimal_publisher.destroy_node()
    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

### Code Explanation:

*   **Imports**: We import `rclpy` for the ROS 2 client library, `Node` as the base class for our ROS 2 node, and `String` from `std_msgs.msg` which is the standard message type for strings.
*   **`MinimalPublisher` Class**:
    *   The constructor `__init__` calls the base `Node` class constructor with the name `'minimal_publisher'`.
    *   `self.create_publisher(String, 'topic', 10)`: This line is crucial. It creates a publisher object.
        *   `String`: The message type this publisher will send.
        *   `'topic'`: The name of the topic. Subscribers will listen to this name.
        *   `10`: The QoS (Quality of Service) profile depth. This is a history depth setting, meaning the publisher will keep a maximum of 10 messages in its buffer before discarding old ones if the subscriber is slow.
    *   A simple counter `self.i` is initialized.
    *   `self.create_timer(timer_period, self.timer_callback)`: This sets up a timer that will call `self.timer_callback` every `0.5` seconds. This is how our node periodically publishes messages.
*   **`timer_callback` Method**:
    *   This method is called by the timer.
    *   It creates a `String` message object, populates its `data` field with the "Hello World" string and the current counter value.
    *   `self.publisher_.publish(msg)`: This sends the message over the `'topic'` topic.
    *   `self.get_logger().info(...)`: This logs the published message to the console, making it easy to see what our publisher is doing.
*   **`main` Function**:
    *   `rclpy.init(args=args)`: Initializes the ROS 2 Python client library. This must be called before any ROS 2 communication can happen.
    *   `minimal_publisher = MinimalPublisher()`: Creates an instance of our publisher node.
    *   `rclpy.spin(minimal_publisher)`: This is a blocking call that keeps the node alive and processes any callbacks (like our timer callback). It will run until the node is explicitly shut down (e.g., by pressing `Ctrl+C`).
    *   `minimal_publisher.destroy_node()` and `rclpy.shutdown()`: Clean up resources when the node is stopped.

### Modifying `setup.py`

To make ROS 2 aware of our new executable, we need to update the `setup.py` file in your `my_py_pkg` package.

Open `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\physical-ai-textbook\docs\week-03-ros2-part1\my_py_pkg\setup.py` and add the following entry to the `entry_points` dictionary, under `console_scripts`. If `entry_points` or `console_scripts` don't exist, create them.

```python title="setup.py" {24-25}
# ... existing content ...

entry_points={
    'console_scripts': [
        'minimal_publisher = my_py_pkg.minimal_publisher:main',
        'minimal_subscriber = my_py_pkg.minimal_subscriber:main', # We will add this later
    ],
},

# ... existing content ...
```
:::tip Note
We've preemptively added the entry for `minimal_subscriber`. You will create that file in the next section.
:::

### Building Your Package

After modifying `setup.py` and creating the Python file, you need to rebuild your ROS 2 workspace. Navigate to the root of your workspace (e.g., `~/ros2_ws`) and run:

```bash
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\
colcon build --packages-select my_py_pkg
```

This command will compile your package, making the `minimal_publisher` executable available to ROS 2.

## Creating a ROS 2 Listener (Subscriber) Node (Python)

Now that we have a publisher, let's create a corresponding subscriber node to receive the messages. This node, `minimal_subscriber`, will listen to the `topic` topic and print any incoming `String` messages.

Create a new Python file named `minimal_subscriber.py` inside the `my_py_pkg/my_py_pkg` directory (i.e., `src/my_py_pkg/my_py_pkg/minimal_subscriber.py`).

```python title="minimal_subscriber.py" showLineNumbers
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # Create a subscriber to the 'topic' topic, with String message type and QoS profile depth of 10
        # The callback function self.listener_callback will be called whenever a new message arrives
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.get_logger().info('Minimal Subscriber Node has been started.')
        # Prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        # Log the received message data
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our MinimalSubscriber node
    minimal_subscriber = MinimalSubscriber()

    # Spin the node, keeping it alive and allowing its callbacks to be called
    rclpy.spin(minimal_subscriber)

    # Destroy the node once rclpy.spin() returns (e.g., due to Ctrl+C)
    minimal_subscriber.destroy_node()
    # Shut down the ROS 2 Python client library
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```

### Code Explanation:

*   **Imports**: Similar to the publisher, we import `rclpy`, `Node`, and `String` message type.
*   **`MinimalSubscriber` Class**:
    *   The constructor `__init__` calls the base `Node` class constructor with the name `'minimal_subscriber'`.
    *   `self.create_subscription(String, 'topic', self.listener_callback, 10)`: This creates a subscriber object.
        *   `String`: The message type this subscriber expects to receive. It must match the publisher's message type.
        *   `'topic'`: The name of the topic to subscribe to. This must match the publisher's topic name.
        *   `self.listener_callback`: The function that will be called every time a new message arrives on the subscribed topic. This is the **callback function**.
        *   `10`: The QoS profile depth, matching the publisher for compatibility.
    *   `self.subscription`: We store the subscription object to prevent it from being garbage collected immediately, which would stop the subscription.
*   **`listener_callback` Method**:
    *   This method takes one argument, `msg`, which is the received `String` message object.
    *   `self.get_logger().info(...)`: It logs the `data` field of the received message to the console.
*   **`main` Function**: Works identically to the publisher's `main` function, initializing and spinning the node.

### Modifying `setup.py` (Already Done)

As noted in the publisher section, we already added the `minimal_subscriber` entry to `setup.py`. If you skipped that, make sure your `setup.py` looks like this:

```python title="setup.py" {24-25}
# ... existing content ...

entry_points={
    'console_scripts': [
        'minimal_publisher = my_py_pkg.minimal_publisher:main',
        'minimal_subscriber = my_py_pkg.minimal_subscriber:main',
    ],
},

# ... existing content ...
```

### Building Your Package

After creating the subscriber Python file (and ensuring `setup.py` is updated), rebuild your ROS 2 workspace:

```bash
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\
colcon build --packages-select my_py_pkg
```

## Running the Publisher and Subscriber

Now for the exciting part: seeing our nodes communicate! You'll need two separate terminal windows for this.

First, always remember to source your ROS 2 environment in each new terminal. From your workspace root:

```bash
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\
. install/setup.bash
```

### Terminal 1: Start the Publisher

In your first terminal, run the publisher node:

```bash
ros2 run my_py_pkg minimal_publisher
```

You should see output similar to this, indicating the publisher is sending messages:

```
[INFO] [minimal_publisher]: Minimal Publisher Node has been started.
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```

### Terminal 2: Start the Subscriber

In your second terminal (after sourcing the setup script again), run the subscriber node:

```bash
ros2 run my_py_pkg minimal_subscriber
```

Immediately, you should see output indicating the subscriber is receiving messages from the publisher:

```
[INFO] [minimal_subscriber]: Minimal Subscriber Node has been started.
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
...
```

Congratulations! You have successfully implemented and run your first ROS 2 publisher-subscriber system. This is a foundational concept for almost all robot applications.

## Inspecting Topics

ROS 2 provides powerful command-line tools to inspect and debug your running system. Let's use them to understand our publisher-subscriber interaction.

Ensure both your publisher and subscriber nodes are running in separate terminals.

### 1. Listing Active Topics: `ros2 topic list`

Open a **third terminal** (and source your ROS 2 environment). To see all active topics in your ROS 2 graph, use:

```bash
ros2 topic list
```

You should see your `/topic` among other default ROS 2 topics:

```
/parameter_events
/rosout
/topic
```

### 2. Echoing Topic Messages: `ros2 topic echo`

To see the messages being published on your `/topic` in real-time, use `ros2 topic echo`:

```bash
ros2 topic echo /topic
```

This command will print every message received on `/topic` to your console:

```
data: Hello World: 10
---
data: Hello World: 11
---
data: Hello World: 12
---
...
```

This is an invaluable tool for debugging and verifying that messages are being sent and received correctly.

### 3. Getting Topic Information: `ros2 topic info`

To get detailed information about a specific topic, including its message type, publisher, and subscriber counts, use `ros2 topic info`:

```bash
ros2 topic info /topic
```

The output will provide insights into the topic's characteristics:

```
Type: std_msgs/msg/String
Publishers:
    /minimal_publisher:
        qos_profile:
            reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
            history: RMW_QOS_POLICY_HISTORY_KEEP_LAST
            depth: 10
            durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
            deadline: {sec: 2147483647, nsec: 4294967295}
            lifespan: {sec: 2147483647, nsec: 4294967295}
            liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
            liveliness_lease_duration: {sec: 2147483647, nsec: 4294967295}
Subscribers:
    /minimal_subscriber:
        qos_profile:
            reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
            history: RMW_QOS_POLICY_HISTORY_KEEP_LAST
            depth: 10
            durability: RMW_QOS_POLICY_DURABILITY_VOLATILE
            deadline: {sec: 2147483647, nsec: 4294967295}
            lifespan: {sec: 2147483647, nsec: 4294967295}
            liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
            liveliness_lease_duration: {sec: 2147483647, nsec: 4294967295}
```

This output shows:
*   **Type**: The message type being used (`std_msgs/msg/String`).
*   **Publishers**: The node(s) currently publishing to this topic (`/minimal_publisher`) along with its QoS profile.
*   **Subscribers**: The node(s) currently subscribing to this topic (`/minimal_subscriber`) along with its QoS profile.

Notice how the `qos_profile` for both publisher and subscriber are listed. This brings us to our next important topic.

## QoS Revisited (Practical Application)

Quality of Service (QoS) settings are critical in ROS 2 for controlling how messages are delivered. They define the policies for reliability, latency, and resource usage, allowing you to fine-tune communication for specific application needs. In our simple example, we used a default `depth` of `10`. Let's explore how different QoS policies would affect our publisher-subscriber interaction.

To implement custom QoS settings, you would typically import `QoSProfile` from `rclpy.qos` and configure it:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Example for a publisher
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE
)
self.publisher_ = self.create_publisher(String, 'topic', qos_profile)

# Example for a subscriber
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    qos_profile)
```

For two nodes to communicate, their QoS profiles must be **compatible**. If they are incompatible (e.g., one requires reliable communication and the other offers best effort), they will generally fail to establish a connection.

Let's discuss some key QoS policies:

### 1. Reliability Policy

*   **`ReliabilityPolicy.RELIABLE`**: Guarantees that every message published will eventually be delivered to all compatible subscribers. Messages are re-sent if necessary. This is suitable for critical data where no loss is acceptable (e.g., robot commands, state updates).
    *   **Effect on our example**: If the subscriber starts after the publisher, or if there's a temporary network glitch, `RELIABLE` ensures that messages are eventually delivered, or re-delivered, ensuring that the subscriber receives all messages. This is the default if not specified, which is why our example worked seamlessly.
*   **`ReliabilityPolicy.BEST_EFFORT`**: Messages are sent once without guarantees of delivery. If a message is lost or dropped due to network congestion or a slow subscriber, it will not be re-sent. This is suitable for high-frequency, non-critical data where occasional loss is acceptable for lower latency (e.g., sensor readings, video streams).
    *   **Effect on our example**: If our publisher used `BEST_EFFORT` and the subscriber started a few seconds late, the subscriber would miss all messages published before it became active. During runtime, if the subscriber briefly falls behind or network conditions are poor, some "Hello World" messages might be skipped in the subscriber's output.

### 2. History Policy

*   **`HistoryPolicy.KEEP_LAST`**: The publisher/subscriber keeps a history of the last `depth` messages. This is what we used with `depth=10`.
    *   **Effect on our example**: If a subscriber starts slightly after the publisher, but within the `depth` window, it will receive the most recent messages from that window. With `depth=10`, if the subscriber starts within 5 seconds (10 messages * 0.5 sec/message), it will catch up immediately from message 0.
*   **`HistoryPolicy.KEEP_ALL`**: The publisher/subscriber keeps all messages in its history. This can consume significant memory and is generally used with caution. It's often used in conjunction with `RELIABLE` for systems that need complete data integrity.

### 3. Durability Policy

*   **`DurabilityPolicy.VOLATILE`**: Only publishes to subscribers that are active at the time of publication. Messages are not retained for late-joining subscribers. This is the default.
    *   **Effect on our example**: If the subscriber starts after the publisher using `VOLATILE` durability, it will only receive messages published *after* it has connected. It will not receive any past messages, regardless of the `depth` setting.
*   **`DurabilityPolicy.TRANSIENT_LOCAL`**: The publisher will retain a history of published messages (subject to history and depth settings) and deliver them to any new, compatible subscribers upon connection. This is useful for "latched" topics where new subscribers need to receive the last known state immediately.
    *   **Effect on our example**: If our publisher used `TRANSIENT_LOCAL` and `depth=10`, even if a subscriber starts 10 seconds after the publisher, it would immediately receive the last 10 "Hello World" messages that were published before it connected. This is incredibly useful for providing initial state to new components.

:::tip Practical Advice
Always consider the nature of your data and the requirements of your application when choosing QoS settings. A good rule of thumb is to use `RELIABLE` for commands and critical state, and `BEST_EFFORT` for high-bandwidth sensor data. `TRANSIENT_LOCAL` is excellent for configuration or static information that new nodes need upon startup.
:::

## Troubleshooting Section

Even with a seemingly simple publisher-subscriber setup, things can go wrong. This section covers common issues and diagnostic steps to help you get your ROS 2 system running smoothly.

:::danger Troubleshooting
Before attempting any troubleshooting, ensure you have **sourced your ROS 2 environment** in every new terminal you open. Failure to do so is the most common cause of "command not found" or "package not found" errors.
```bash
# Example for sourcing from your workspace
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\
. install/setup.bash
```
:::

### Issue 1: Nodes not seeing each other / No messages being exchanged

**Symptoms:**
*   Publisher is running and logging "Publishing: ..." but subscriber is running and only logs "Minimal Subscriber Node has been started." with no "I heard: ..." messages.
*   `ros2 topic info /topic` shows only one publisher or one subscriber, but not both.

**Diagnostic Steps:**

1.  **Check Topic Name Consistency:**
    *   **Problem:** The publisher and subscriber are using different topic names.
    *   **Fix:** Ensure both `self.create_publisher(String, 'topic', 10)` and `self.create_subscription(String, 'topic', self.listener_callback, 10)` use the exact same topic string (e.g., `'topic'`). Remember topic names are case-sensitive.

2.  **Check Message Type Consistency:**
    *   **Problem:** The publisher is sending `String` messages, but the subscriber is expecting a different type (or vice-versa).
    *   **Fix:** Ensure both `String` types in `create_publisher` and `create_subscription` match exactly (`std_msgs.msg.String`).

3.  **Check QoS Profile Compatibility:**
    *   **Problem:** Publisher and subscriber have incompatible QoS profiles. While our simple example defaults to compatible settings, custom QoS profiles can cause issues.
    *   **Fix:** Review the QoS settings for both nodes. For basic communication, ensure `reliability`, `history`, and `depth` settings are compatible. If one is `RELIABLE` and the other `BEST_EFFORT`, they will not connect. Use `ros2 topic info /topic` to inspect the full QoS profiles.

4.  **Are Both Nodes Running?**
    *   **Problem:** One of the nodes (publisher or subscriber) is not actually running or has crashed.
    *   **Fix:** Check both terminal windows to ensure both `ros2 run` commands are still active and logging output. If one crashed, restart it.

5.  **Network Configuration (Advanced):**
    *   **Problem:** If you're running nodes across different machines or in complex network environments (e.g., Docker, VMs), network discovery might be blocked.
    *   **Fix:** This is a deeper issue. Ensure firewalls are not blocking UDP ports used by ROS 2 (typically 11311/udp and 7400-7500/udp). Check `ROS_DOMAIN_ID` environment variable consistency across all terminals. For more complex setups, refer to ROS 2 network configuration documentation.

### Issue 2: `ros2 run` command not found or package not found

**Symptoms:**
*   Error message like `ros2: command not found`.
*   Error message like `Package 'my_py_pkg' not found`.

**Diagnostic Steps:**

1.  **Sourcing the ROS 2 Environment:**
    *   **Problem:** You haven't sourced your ROS 2 installation or your workspace setup files in the current terminal.
    *   **Fix:** Run `. install/setup.bash` (or `. install/setup.bat` on Windows) from your workspace root. This is required for *each new terminal*.

2.  **Building the Package:**
    *   **Problem:** You created/modified the Python file or `setup.py` but didn't rebuild the package.
    *   **Fix:** Navigate to your workspace root and run `colcon build --packages-select my_py_pkg`. Then re-source your environment.

3.  **Correct `setup.py` `entry_points`:**
    *   **Problem:** The `console_scripts` entry in `setup.py` is incorrect or missing.
    *   **Fix:** Double-check `setup.py` to ensure `minimal_publisher = my_py_pkg.minimal_publisher:main` (and for subscriber) is correctly defined. The format is `executable_name = package_name.python_file_name_without_py_extension:main_function_name`.

### Issue 3: Messages appear garbled or unexpected

**Symptoms:**
*   Subscriber receives messages, but the content is not what the publisher sent (e.g., empty strings, incorrect numbers if types were mismatched).

**Diagnostic Steps:**

1.  **Message Type Mismatch:**
    *   **Problem:** This is the most likely cause. The publisher and subscriber are using different message types, but ROS 2 might silently cast or ignore parts of the message if they share some fields, leading to unexpected data.
    *   **Fix:** Strictly ensure the message type imported and used by both publisher and subscriber are identical (e.g., `std_msgs.msg.String` in both). If you're using custom messages, ensure they are built correctly and both nodes import the same definition.

2.  **Serialization/Deserialization Errors (Advanced):**
    *   **Problem:** While less common with standard messages, custom messages might have issues in their definition that lead to incorrect serialization or deserialization.
    *   **Fix:** Inspect the `.msg` definition file for your custom message types. Ensure fields are correctly typed.

### Issue 4: High CPU Usage or Latency

**Symptoms:**
*   Nodes consuming excessive CPU resources.
*   Noticeable delay between publishing and subscribing.

**Diagnostic Steps:**

1.  **High-Frequency Publishing on `RELIABLE` QoS:**
    *   **Problem:** Sending many messages per second with `ReliabilityPolicy.RELIABLE` can lead to increased network overhead for retransmissions and acknowledgments.
    *   **Fix:** For very high-frequency data (e.g., `100Hz+` sensor streams), consider using `ReliabilityPolicy.BEST_EFFORT` if occasional message loss is tolerable.

2.  **Inefficient Callback Functions:**
    *   **Problem:** The `listener_callback` or `timer_callback` functions are performing computationally expensive operations, blocking the ROS 2 executor.
    *   **Fix:** Profile your Python code to identify bottlenecks. Offload heavy computations to separate threads or processes if they don't need to block the ROS 2 executor. Keep callbacks as lightweight as possible.

3.  **Queue Overflows (`depth` too small):**
    *   **Problem:** If the `depth` of your QoS history is too small for a slow subscriber, messages might be dropped.
    *   **Fix:** Increase the `depth` parameter in your `QoSProfile` for both publisher and subscriber. Use `KEEP_ALL` with caution as it can lead to memory issues.

By systematically going through these troubleshooting steps, you should be able to diagnose and resolve most common issues encountered when working with the ROS 2 publisher-subscriber pattern. Understanding these diagnostics is a crucial skill for any robotics developer.

---
**Word Count:** Approximately 2500 words.
