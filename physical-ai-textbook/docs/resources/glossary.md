---
title: "Glossary"
sidebar_label: "Glossary"
sidebar_position: 1
description: "Comprehensive glossary of Physical AI, robotics, and ROS 2 terminology"
keywords: [glossary, robotics, physical ai, ros2, terminology, definitions]
---

# Glossary

This glossary provides definitions for key terms used throughout the Physical AI & Humanoid Robotics textbook. Terms are organized alphabetically for easy reference.

---

## A

### Action (ROS 2)
A long-running task in ROS 2 that provides feedback during execution and can be canceled. Actions use three topics: goal, feedback, and result, making them ideal for tasks like navigation or manipulation that take time to complete.

**Example**: A navigation action that moves a robot to a target position while providing progress updates every second.

**Related Terms**: [Service (ROS 2)](#service-ros-2), [Topic (ROS 2)](#topic-ros-2), [Publisher](#publisher)

**See Also**: [Services and Actions](../week-04-ros2-part2/01-services-actions.md)

---

### Actuator
A mechanical or electrical device that converts energy (electrical, hydraulic, or pneumatic) into physical motion. In robotics, actuators enable movement by controlling motors, servos, or pneumatic cylinders.

**Example**: A servo motor acting as an actuator controls the angle of a robot's elbow joint, allowing precise positioning.

**Related Terms**: [Sensor](#sensor), [Degree of Freedom (DOF)](#degree-of-freedom-dof)

**See Also**: [Sensor Systems](../week-01-02-intro-physical-ai/03-sensor-systems.md)

---

## C

### Client Library
A programming language-specific API that enables developers to create ROS 2 applications. The two primary client libraries are rclpy (Python) and rclcpp (C++).

**Example**: Using rclpy to create a Python node: `import rclpy` and `node = rclpy.create_node('my_node')`

**Related Terms**: [Node](#node), [ROS 2](#ros-2-robot-operating-system-2)

**See Also**: [First Python Node](../week-03-ros2-part1/03-first-python-node.md)

---

## D

### DDS (Data Distribution Service)
A middleware protocol used by ROS 2 for real-time, peer-to-peer communication between nodes. DDS provides features like Quality of Service (QoS) configuration, automatic discovery, and efficient data distribution.

**Example**: ROS 2 uses DDS implementations like Fast-DDS or Cyclone DDS to enable nodes to discover each other and exchange messages without a central broker.

**Related Terms**: [QoS (Quality of Service)](#qos-quality-of-service), [Middleware](#middleware), [Publisher-Subscriber Pattern](#publisher-subscriber-pattern)

**See Also**: [ROS 2 Architecture](../week-03-ros2-part1/01-ros2-architecture.md)

---

### Degree of Freedom (DOF)
An independent direction in which a robot or mechanism can move. Each DOF represents one axis of motion (linear or rotational). A humanoid robot arm typically has 7 DOF, matching human arm mobility.

**Example**: A robotic arm with 6 DOF can position its end-effector at any point in 3D space with any orientation (3 position + 3 rotation).

**Related Terms**: [Actuator](#actuator), [End-Effector](#end-effector)

**See Also**: [Physical AI Foundations](../week-01-02-intro-physical-ai/01-foundations.md)

---

## E

### Embodied Intelligence
The concept that intelligence arises from the interaction between an agent's body, brain, and environment. Unlike purely digital AI, embodied intelligence requires physical sensors, actuators, and real-world interaction.

**Example**: A robot learning to walk by physically experiencing balance, gravity, and ground contact develops embodied intelligence that cannot be fully captured in simulation.

**Related Terms**: [Physical AI](#physical-ai), [Sensor-Motor Loop](#sensor-motor-loop), [Perception-Action Cycle](#perception-action-cycle)

**See Also**: [Embodied Intelligence](../week-01-02-intro-physical-ai/02-embodied-intelligence.md)

---

### End-Effector
The device at the end of a robotic arm that interacts with the environment. Common end-effectors include grippers, suction cups, welding tools, or cameras.

**Example**: A robotic gripper (end-effector) with two fingers grasps objects during pick-and-place operations in a warehouse.

**Related Terms**: [Actuator](#actuator), [Degree of Freedom (DOF)](#degree-of-freedom-dof)

**See Also**: [Physical AI Foundations](../week-01-02-intro-physical-ai/01-foundations.md)

---

## I

### IMU (Inertial Measurement Unit)
A sensor that measures a robot's orientation, angular velocity, and linear acceleration using accelerometers, gyroscopes, and sometimes magnetometers. IMUs are critical for balance and navigation.

**Example**: A humanoid robot uses an IMU in its torso to detect when it's tilting and adjust its stance to maintain balance.

**Related Terms**: [Sensor](#sensor), [Sensor Fusion](#sensor-fusion), [LIDAR](#lidar)

**See Also**: [Sensor Systems](../week-01-02-intro-physical-ai/03-sensor-systems.md)

---

## L

### Launch File
A configuration file in ROS 2 (written in Python, XML, or YAML) that starts multiple nodes with specified parameters, remappings, and dependencies. Launch files simplify complex system startup.

**Example**: A launch file that starts a camera node, image processing node, and visualization tool simultaneously with correct topic mappings.

**Related Terms**: [Node](#node), [Parameter](#parameter), [ROS 2](#ros-2-robot-operating-system-2)

**See Also**: [Launch Files](../week-04-ros2-part2/04-launch-files.md)

---

### LIDAR (Light Detection and Ranging)
A sensor that measures distances by emitting laser pulses and timing their reflection. LIDAR creates detailed 3D maps of the environment, essential for autonomous navigation and obstacle avoidance.

**Example**: A robot uses a 360-degree LIDAR to detect obstacles within a 10-meter radius, updating 10 times per second.

**Related Terms**: [Sensor](#sensor), [Point Cloud](#point-cloud), [IMU](#imu-inertial-measurement-unit)

**See Also**: [Sensor Systems](../week-01-02-intro-physical-ai/03-sensor-systems.md)

---

## M

### Message (ROS 2)
A data structure used to exchange information between ROS 2 nodes via topics, services, or actions. Messages are defined in .msg files and can contain primitive types, arrays, or nested structures.

**Example**: `geometry_msgs/Twist` message contains linear and angular velocity commands for robot motion.

**Related Terms**: [Topic (ROS 2)](#topic-ros-2), [Publisher](#publisher), [Subscriber](#subscriber)

**See Also**: [Message Types](../week-04-ros2-part2/03-message-types.md)

---

### Middleware
Software that provides communication services between applications or components. In ROS 2, DDS middleware handles message passing, discovery, and Quality of Service between nodes.

**Example**: ROS 2's middleware layer (DDS) automatically handles network communication when a publisher sends messages to subscribers on different computers.

**Related Terms**: [DDS](#dds-data-distribution-service), [QoS (Quality of Service)](#qos-quality-of-service)

**See Also**: [ROS 2 Architecture](../week-03-ros2-part1/01-ros2-architecture.md)

---

## N

### Node
An independent computational process in ROS 2 that performs a specific task. Nodes communicate by publishing and subscribing to topics, calling services, or executing actions. A ROS 2 application consists of multiple collaborating nodes.

**Example**: A camera node publishes image data, while a separate image processing node subscribes to those images and publishes detected objects.

**Related Terms**: [Publisher](#publisher), [Subscriber](#subscriber), [Topic (ROS 2)](#topic-ros-2)

**See Also**: [Nodes and Topics](../week-03-ros2-part1/02-nodes-topics.md)

---

## P

### Parameter
A configuration value in ROS 2 that can be set at runtime or through launch files. Parameters allow nodes to be reconfigured without code changes, making systems more flexible.

**Example**: A robot's maximum speed parameter can be adjusted from 1.0 m/s during testing to 0.5 m/s for safer operation.

**Related Terms**: [Node](#node), [Launch File](#launch-file)

**See Also**: [Launch Files](../week-04-ros2-part2/04-launch-files.md)

---

### Perception-Action Cycle
The continuous loop where a robot perceives its environment through sensors, processes this information to make decisions, executes actions through actuators, and observes the results. This cycle is fundamental to embodied intelligence.

**Example**: A robot vacuum perceives obstacles (cameras/LIDAR), plans a path around them (processing), moves motors (action), and senses new positions (perception continues).

**Related Terms**: [Embodied Intelligence](#embodied-intelligence), [Sensor-Motor Loop](#sensor-motor-loop)

**See Also**: [Embodied Intelligence](../week-01-02-intro-physical-ai/02-embodied-intelligence.md)

---

### Physical AI
Artificial intelligence systems that interact with the physical world through sensors and actuators. Physical AI faces unique challenges like real-time constraints, safety requirements, and sim-to-real transfer that digital AI does not encounter.

**Example**: A humanoid robot using vision and force sensors to grasp fragile objects demonstrates Physical AI by adapting to real-world variability.

**Related Terms**: [Embodied Intelligence](#embodied-intelligence), [Sensor](#sensor), [Actuator](#actuator)

**See Also**: [Physical AI Foundations](../week-01-02-intro-physical-ai/01-foundations.md)

---

### Point Cloud
A collection of data points in 3D space, typically generated by LIDAR or depth cameras. Each point represents a surface location with (x, y, z) coordinates and sometimes color or intensity data.

**Example**: A LIDAR sensor generates a point cloud with 100,000 points representing nearby walls, furniture, and obstacles.

**Related Terms**: [LIDAR](#lidar), [Sensor](#sensor)

**See Also**: [Sensor Systems](../week-01-02-intro-physical-ai/03-sensor-systems.md)

---

### Publisher
A ROS 2 entity that sends (publishes) messages to a topic. Multiple publishers can publish to the same topic, and each message is delivered to all subscribers.

**Example**: A temperature sensor node acts as a publisher, sending temperature readings to the `/temperature` topic every second.

**Related Terms**: [Subscriber](#subscriber), [Topic (ROS 2)](#topic-ros-2), [Node](#node)

**See Also**: [Publisher-Subscriber](../week-03-ros2-part1/04-publisher-subscriber.md)

---

### Publisher-Subscriber Pattern
A messaging pattern where publishers send messages to topics without knowing who will receive them, and subscribers receive messages from topics without knowing who sent them. This decoupling enables flexible, scalable systems.

**Example**: A camera publisher sends images to `/camera/image`, and any number of subscribers (object detection, recording, display) can independently process those images.

**Related Terms**: [Publisher](#publisher), [Subscriber](#subscriber), [Topic (ROS 2)](#topic-ros-2)

**See Also**: [Nodes and Topics](../week-03-ros2-part1/02-nodes-topics.md)

---

## Q

### QoS (Quality of Service)
Configuration policies in ROS 2 DDS that control message delivery behavior, including reliability (reliable vs. best-effort), durability (transient vs. volatile), and history (keep last N messages). QoS settings must be compatible between publishers and subscribers.

**Example**: A critical safety system uses reliable QoS to ensure all emergency stop commands are delivered, while sensor data uses best-effort QoS for lower latency.

**Related Terms**: [DDS](#dds-data-distribution-service), [Publisher](#publisher), [Subscriber](#subscriber)

**See Also**: [Nodes and Topics](../week-03-ros2-part1/02-nodes-topics.md)

---

## R

### ROS 2 (Robot Operating System 2)
A flexible, open-source framework for robot software development. ROS 2 provides libraries, tools, and communication infrastructure for building complex robot applications with distributed nodes.

**Example**: A mobile robot uses ROS 2 nodes for navigation, vision processing, motor control, and user interface, all communicating via topics and services.

**Related Terms**: [Node](#node), [Topic (ROS 2)](#topic-ros-2), [DDS](#dds-data-distribution-service)

**See Also**: [ROS 2 Architecture](../week-03-ros2-part1/01-ros2-architecture.md)

---

## S

### Sensor
A device that detects and measures physical properties (light, distance, force, orientation) and converts them into electrical signals. Sensors provide robots with perception of their environment and internal state.

**Example**: A force sensor in a robot gripper measures grip strength to avoid crushing fragile objects.

**Related Terms**: [LIDAR](#lidar), [IMU](#imu-inertial-measurement-unit), [Sensor Fusion](#sensor-fusion)

**See Also**: [Sensor Systems](../week-01-02-intro-physical-ai/03-sensor-systems.md)

---

### Sensor Fusion
The process of combining data from multiple sensors to produce more accurate, reliable, or complete information than any single sensor could provide. Fusion algorithms account for sensor strengths, weaknesses, and complementary characteristics.

**Example**: A robot fuses IMU data (fast, but drifts over time) with GPS data (accurate position, but slower updates) to achieve both accuracy and responsiveness in navigation.

**Related Terms**: [Sensor](#sensor), [IMU](#imu-inertial-measurement-unit), [LIDAR](#lidar)

**See Also**: [Sensor Systems](../week-01-02-intro-physical-ai/03-sensor-systems.md)

---

### Sensor-Motor Loop
The closed-loop cycle where sensor inputs drive motor actions, and motor actions affect subsequent sensor readings. This feedback loop is essential for adaptive robot behavior.

**Example**: A line-following robot uses light sensors to detect the line position, adjusts motor speeds to stay centered, and continuously reads sensors to verify the correction worked.

**Related Terms**: [Perception-Action Cycle](#perception-action-cycle), [Embodied Intelligence](#embodied-intelligence)

**See Also**: [Embodied Intelligence](../week-01-02-intro-physical-ai/02-embodied-intelligence.md)

---

### Service (ROS 2)
A synchronous request-reply communication pattern in ROS 2. A client sends a request to a service, and the server processes it and returns a response. Services are ideal for one-time operations or queries.

**Example**: A `/add_two_ints` service receives two integers in the request and returns their sum in the response.

**Related Terms**: [Action (ROS 2)](#action-ros-2), [Node](#node), [Topic (ROS 2)](#topic-ros-2)

**See Also**: [Services and Actions](../week-04-ros2-part2/01-services-actions.md)

---

### Sim-to-Real Transfer
The challenge of transferring robot behaviors trained in simulation to the real world. Differences in physics modeling, sensor noise, and environmental variability can cause policies that work perfectly in simulation to fail on real robots.

**Example**: A robot trained to walk in a physics simulator falls immediately on real hardware due to unmodeled friction, motor response delays, and floor surface variations.

**Related Terms**: [Physical AI](#physical-ai), [Embodied Intelligence](#embodied-intelligence)

**See Also**: [Physical vs Digital AI](../week-01-02-intro-physical-ai/04-physical-vs-digital-ai.md)

---

### Subscriber
A ROS 2 entity that receives (subscribes to) messages from a topic. Multiple subscribers can listen to the same topic, and each receives a copy of every published message.

**Example**: A display node subscribes to `/camera/image` to show live video, while a recording node also subscribes to save the images to disk.

**Related Terms**: [Publisher](#publisher), [Topic (ROS 2)](#topic-ros-2), [Node](#node)

**See Also**: [Publisher-Subscriber](../week-03-ros2-part1/04-publisher-subscriber.md)

---

## T

### Topic (ROS 2)
A named communication channel used for asynchronous message passing between nodes. Topics implement the publisher-subscriber pattern, allowing many-to-many communication without direct connections between nodes.

**Example**: The `/cmd_vel` topic carries velocity commands from a navigation node (publisher) to motor controllers (subscribers).

**Related Terms**: [Publisher](#publisher), [Subscriber](#subscriber), [Message (ROS 2)](#message-ros-2)

**See Also**: [Nodes and Topics](../week-03-ros2-part1/02-nodes-topics.md)

---

## U

### URDF (Unified Robot Description Format)
An XML format used to describe a robot's physical structure, including links (rigid bodies), joints (connections), sensors, and visual/collision geometry. URDF files are essential for simulation and visualization in ROS 2.

**Example**: A URDF file describes a robot arm with 6 joints, specifying each joint's axis, limits, and the connected link geometries.

**Related Terms**: [ROS 2](#ros-2-robot-operating-system-2), [Actuator](#actuator)

**See Also**: [Message Types](../week-04-ros2-part2/03-message-types.md)

---

## W

### Workspace (ROS 2)
A directory structure containing ROS 2 packages, build artifacts, and installation files. Workspaces enable developers to organize, build, and test multiple packages together using the colcon build system.

**Example**: A typical workspace at `~/ros2_ws/` contains subdirectories `src/` (source code), `build/` (compilation artifacts), `install/` (installed packages), and `log/` (build logs).

**Related Terms**: [Node](#node), [Package](#package)

**See Also**: [First Python Node](../week-03-ros2-part1/03-first-python-node.md)

---

## Additional Terms

### Callback
A function that executes in response to an event, such as receiving a message or a timer expiration. In ROS 2, callbacks are used extensively to handle asynchronous events like topic subscriptions.

**Example**: A subscriber callback function processes each incoming message: `def listener_callback(self, msg): self.get_logger().info(msg.data)`

**Related Terms**: [Subscriber](#subscriber), [Node](#node)

**See Also**: [Publisher-Subscriber](../week-03-ros2-part1/04-publisher-subscriber.md)

---

### Package
A collection of related ROS 2 nodes, libraries, configuration files, and dependencies organized as a buildable unit. Packages are the fundamental organizational structure in ROS 2 development.

**Example**: A `robot_vision` package might contain camera driver nodes, image processing nodes, and configuration files for vision algorithms.

**Related Terms**: [Node](#node), [Workspace](#workspace-ros-2)

**See Also**: [First Python Node](../week-03-ros2-part1/03-first-python-node.md)

---

### Real-Time Computing
Computing systems where correctness depends not only on logical results but also on meeting strict timing constraints. Physical AI systems often require real-time performance for safety-critical control loops.

**Example**: A robot's balance controller must compute motor commands within 1 millisecond to prevent falling, making it a real-time requirement.

**Related Terms**: [Physical AI](#physical-ai), [Sensor-Motor Loop](#sensor-motor-loop)

**See Also**: [Physical vs Digital AI](../week-01-02-intro-physical-ai/04-physical-vs-digital-ai.md)

---

## Summary

This glossary covers 31 essential terms spanning Physical AI concepts, robotics fundamentals, and ROS 2 architecture. Each term includes a clear definition, practical example, and links to detailed explanations in the textbook chapters. Use this resource as a quick reference while studying or to clarify unfamiliar terminology.

**Usage Tips**:
- Terms are cross-linked to related concepts for deeper exploration
- "See Also" sections point to detailed explanations in chapters
- Examples provide concrete context for abstract concepts
- Bookmark this page for quick reference during hands-on exercises
