---
title: "ROS 2 Launch Files"
sidebar_label: "Launch Files"
sidebar_position: 4
description: "Master ROS 2 launch files to start multiple nodes, set parameters, and orchestrate complex robot systems"
keywords: [ros2, launch files, python launch, xml launch, parameters, multi-node, orchestration, ros2 launch]
reading_time: 25
difficulty: intermediate
prerequisites:
  - Week 3: ROS 2 Part 1 (Nodes and Topics)
  - Previous sections in Week 4
  - Python basics
  - Understanding of command-line arguments
---

# ROS 2 Launch Files

## Overview

As your robot systems grow more complex, manually starting each node in separate terminals becomes impractical. Launch files solve this problem by allowing you to start multiple nodes, set parameters, remap topics, and configure your entire system with a single command.

ROS 2 supports two launch file formats:

- **Python launch files**: Most flexible, use Python code for dynamic configuration
- **XML launch files**: Declarative, easier for simple static configurations

This chapter teaches you to:

- Write Python launch files to start multiple nodes
- Pass parameters and arguments to nodes
- Use XML launch files for static configurations
- Compose complex launch files from smaller ones
- Apply best practices for maintainable launch systems

## Why Use Launch Files?

Consider a mobile robot with these components:

- Camera driver node
- Laser scanner node
- Odometry publisher
- Navigation stack (5+ nodes)
- Visualization tools

**Without launch files:**
```bash
# Terminal 1
ros2 run camera_driver camera_node --ros-args -p frame_rate:=30

# Terminal 2
ros2 run laser_driver laser_node --ros-args -p scan_rate:=10

# Terminal 3
ros2 run robot_base odom_publisher

# ... 7 more terminals ...
```

**With a launch file:**
```bash
ros2 launch my_robot complete_system.launch.py
```

Launch files provide:

- **Convenience**: Start entire system with one command
- **Configuration management**: Centralized parameter settings
- **Repeatability**: Same configuration every time
- **Modularity**: Compose systems from reusable components
- **Conditional logic**: Start different nodes based on conditions

## Python Launch Files

Python launch files offer maximum flexibility using Python code.

### Basic Python Launch File

Create a simple launch file that starts two nodes:

**File: `basic_launch.py`**

```python
"""
Basic Launch File Example

Starts a talker and listener node to demonstrate basic launch file structure.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description with two nodes.

    This function must be named 'generate_launch_description' and
    must return a LaunchDescription object.
    """

    return LaunchDescription([
        # First node: talker
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker',
            output='screen'
        ),

        # Second node: listener
        Node(
            package='demo_nodes_cpp',
            executable='listener',
            name='listener',
            output='screen'
        ),
    ])
```

**Launch file anatomy:**

1. **Import statements**: Import launch components
2. **`generate_launch_description()` function**: Must return `LaunchDescription`
3. **LaunchDescription**: Contains list of actions (nodes, events, etc.)
4. **Node actions**: Define nodes to start

**Running the launch file:**

```bash
# From package with launch file
ros2 launch my_package basic_launch.py

# From any directory (absolute path)
ros2 launch /path/to/basic_launch.py
```

### Launch File with Parameters

Pass parameters to nodes at launch time:

```python
"""
Launch File with Parameters

Demonstrates how to pass parameters to nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot',
            executable='velocity_publisher',
            name='velocity_publisher',
            output='screen',
            parameters=[{
                'linear_speed': 0.5,
                'angular_speed': 0.3,
                'publish_rate': 20.0
            }]
        ),

        Node(
            package='my_robot',
            executable='status_subscriber',
            name='status_monitor',
            output='screen',
            parameters=[{
                'timeout_threshold': 5.0,
                'log_level': 'INFO'
            }]
        ),
    ])
```

**Parameter formats:**

```python
# Inline dictionary
parameters=[{
    'param_name': value,
    'another_param': value2
}]

# YAML file
parameters=['/path/to/params.yaml']

# Mixed
parameters=[
    {'param1': value1},
    '/path/to/more_params.yaml'
]
```

### Launch Arguments and Substitutions

Make launch files configurable with command-line arguments:

```python
"""
Launch File with Arguments

Allows users to customize behavior via command-line arguments.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_1',
        description='Name of the robot'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error'],
        description='Logging level'
    )

    # Use arguments via LaunchConfiguration
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        # Declare arguments first
        use_sim_time_arg,
        robot_name_arg,
        log_level_arg,

        # Use arguments in node configuration
        Node(
            package='my_robot',
            executable='robot_controller',
            name=robot_name,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_id': robot_name
            }],
            arguments=['--ros-args', '--log-level', log_level]
        ),
    ])
```

**Using launch arguments:**

```bash
# Use defaults
ros2 launch my_robot robot_launch.py

# Override arguments
ros2 launch my_robot robot_launch.py use_sim_time:=true robot_name:=explorer

# Set log level
ros2 launch my_robot robot_launch.py log_level:=debug
```

### Complete System Launch Example

Real-world launch file for a mobile robot:

```python
"""
Complete Mobile Robot Launch File

Starts all components needed for autonomous navigation:
- Sensor drivers
- Odometry publisher
- Control system
- Visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('my_robot')
    config_dir = os.path.join(pkg_share, 'config')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'robot_params.yaml'),
        description='Full path to robot parameters file'
    )

    # Nodes
    # 1. Laser scanner driver
    laser_node = Node(
        package='laser_driver',
        executable='laser_scanner',
        name='laser_scanner',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/robot/scan')  # Remap topic
        ]
    )

    # 2. Camera driver
    camera_node = Node(
        package='camera_driver',
        executable='camera_publisher',
        name='camera',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_rate': 30,
            'image_width': 640,
            'image_height': 480
        }]
    )

    # 3. Odometry publisher
    odom_node = Node(
        package='my_robot',
        executable='odometry_publisher',
        name='odometry',
        output='screen',
        parameters=[params_file]
    )

    # 4. Robot controller
    controller_node = Node(
        package='my_robot',
        executable='feedback_controller',
        name='controller',
        output='screen',
        parameters=[params_file]
    )

    # 5. RViz (conditional)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz),  # Only launch if use_rviz is true
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_use_rviz,
        declare_params_file,

        # Nodes
        laser_node,
        camera_node,
        odom_node,
        controller_node,
        rviz_node,
    ])
```

**Usage:**

```bash
# Launch full system
ros2 launch my_robot complete_system.launch.py

# Launch without visualization
ros2 launch my_robot complete_system.launch.py use_rviz:=false

# Use simulation time and custom params
ros2 launch my_robot complete_system.launch.py \
    use_sim_time:=true \
    params_file:=/path/to/custom_params.yaml
```

### Advanced: Composing Launch Files

Include other launch files to build modular systems:

```python
"""
Composed Launch File

Includes multiple launch files to build complete system.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package paths
    sensors_pkg = get_package_share_directory('sensor_drivers')
    nav_pkg = get_package_share_directory('navigation')

    # Include sensor drivers launch file
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_pkg, 'launch', 'all_sensors.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'sensor_rate': '10'
        }.items()
    )

    # Include navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_file': '/path/to/map.yaml'
        }.items()
    )

    return LaunchDescription([
        sensors_launch,
        navigation_launch,
    ])
```

## XML Launch Files

XML launch files provide a declarative syntax for simpler configurations.

### Basic XML Launch File

**File: `basic_launch.xml`**

```xml
<launch>
  <!-- Simple XML launch file -->

  <!-- Talker node -->
  <node pkg="demo_nodes_cpp" exec="talker" name="talker" output="screen"/>

  <!-- Listener node -->
  <node pkg="demo_nodes_cpp" exec="listener" name="listener" output="screen"/>

</launch>
```

### XML with Parameters

```xml
<launch>
  <!-- Velocity publisher with parameters -->
  <node pkg="my_robot" exec="velocity_publisher" name="velocity_pub" output="screen">
    <param name="linear_speed" value="0.5"/>
    <param name="angular_speed" value="0.3"/>
    <param name="publish_rate" value="20.0"/>
  </node>

  <!-- Controller with parameter file -->
  <node pkg="my_robot" exec="feedback_controller" name="controller" output="screen">
    <param from="$(find-pkg-share my_robot)/config/controller_params.yaml"/>
  </node>

</launch>
```

### XML with Arguments

```xml
<launch>
  <!-- Declare arguments -->
  <arg name="use_sim_time" default="false" description="Use simulation time"/>
  <arg name="robot_name" default="robot_1" description="Robot identifier"/>
  <arg name="log_level" default="info" description="Logging level"/>

  <!-- Use arguments -->
  <node pkg="my_robot" exec="robot_controller" name="$(var robot_name)" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="robot_id" value="$(var robot_name)"/>
  </node>

</launch>
```

### XML with Remapping

```xml
<launch>
  <arg name="robot_namespace" default="robot1"/>

  <!-- Remap topics for multi-robot systems -->
  <node pkg="laser_driver" exec="laser_scanner" name="laser" output="screen">
    <remap from="/scan" to="/$(var robot_namespace)/scan"/>
  </node>

  <node pkg="my_robot" exec="controller" name="controller" output="screen">
    <remap from="/cmd_vel" to="/$(var robot_namespace)/cmd_vel"/>
    <remap from="/odom" to="/$(var robot_namespace)/odom"/>
  </node>

</launch>
```

**Running XML launch files:**

```bash
ros2 launch my_robot basic_launch.xml

# With arguments
ros2 launch my_robot robot_launch.xml robot_name:=explorer use_sim_time:=true
```

## Parameter Files

Separate configuration from launch files using YAML parameter files.

### YAML Parameter File

**File: `config/robot_params.yaml`**

```yaml
# Robot Controller Parameters

/**:
  ros__parameters:
    # Control parameters
    kp_linear: 0.5
    kp_angular: 1.0
    goal_tolerance: 0.1

    # Velocity limits
    max_linear_velocity: 0.5
    max_angular_velocity: 1.0

    # Update rates
    control_frequency: 20.0
    status_frequency: 1.0

    # Safety
    emergency_stop_distance: 0.3
    timeout_threshold: 5.0

velocity_publisher:
  ros__parameters:
    linear_speed: 0.3
    angular_speed: 0.0
    publish_rate: 10.0

laser_scanner:
  ros__parameters:
    frame_id: "laser_frame"
    scan_rate: 10
    angle_min: -1.57  # -90 degrees
    angle_max: 1.57   # +90 degrees
    range_min: 0.1
    range_max: 10.0
```

**Loading in launch file:**

```python
# Python launch file
parameters=[params_file]

# Or specific node section
parameters=[{
    'laser_scanner': params_file
}]
```

**Loading in XML:**

```xml
<node pkg="my_robot" exec="controller" name="controller">
  <param from="$(find-pkg-share my_robot)/config/robot_params.yaml"/>
</node>
```

## Package Setup for Launch Files

Proper package structure for launch files:

### Directory Structure

```
my_robot/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── basic_launch.py
│   ├── complete_system.launch.py
│   └── sensors.launch.xml
├── config/
│   ├── robot_params.yaml
│   └── navigation_params.yaml
├── rviz/
│   └── robot_view.rviz
└── src/
    └── (your Python nodes)
```

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/velocity_publisher.py
  src/feedback_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz configs
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

### package.xml

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot</name>
  <version>0.1.0</version>
  <description>My robot package with launch files</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>

  <!-- Launch file dependency -->
  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Best Practices

### Practice 1: Use Arguments for Flexibility

```python
# Good: Configurable via arguments
DeclareLaunchArgument('robot_name', default_value='robot_1')

# Bad: Hardcoded values
Node(name='robot_1', ...)
```

### Practice 2: Separate Parameters into YAML Files

```python
# Good: External configuration
parameters=[params_file]

# Bad: Inline parameters everywhere
parameters=[{
    'param1': value1,
    'param2': value2,
    # ... 50 more parameters
}]
```

### Practice 3: Use Meaningful Names

```python
# Good: Descriptive names
laser_scanner_node = Node(
    package='laser_driver',
    executable='laser_scanner',
    name='front_laser'
)

# Bad: Generic names
node1 = Node(package='pkg', executable='exec', name='node')
```

### Practice 4: Document Arguments

```python
# Good: Clear documentation
DeclareLaunchArgument(
    'max_velocity',
    default_value='0.5',
    description='Maximum linear velocity in m/s (range: 0.0-2.0)'
)

# Bad: No description
DeclareLaunchArgument('max_velocity', default_value='0.5')
```

### Practice 5: Group Related Nodes

```python
# Good: Logical grouping with comments
return LaunchDescription([
    # Arguments
    declare_arg1,
    declare_arg2,

    # Sensor drivers
    laser_node,
    camera_node,
    imu_node,

    # Control system
    controller_node,
    safety_monitor_node,

    # Visualization
    rviz_node,
])
```

## Common Pitfalls

:::danger Wrong Function Name
**Problem**: Launch file not found or doesn't execute.

**Solution**: Function must be named exactly `generate_launch_description`:

```python
# Correct
def generate_launch_description():
    return LaunchDescription([...])

# Wrong - won't work
def create_launch_description():
    return LaunchDescription([...])
```
:::

:::danger File Not Installed
**Problem**: `ros2 launch` can't find your launch file.

**Solution**: Ensure launch files are installed in CMakeLists.txt:

```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

Then rebuild:
```bash
colcon build --packages-select my_robot
```
:::

:::danger Parameter Type Mismatch
**Problem**: Node fails to start due to parameter type error.

**Solution**: Ensure YAML parameter types match node expectations:

```yaml
# Correct types
linear_speed: 0.5      # float
max_count: 10          # int
enabled: true          # bool
robot_name: "explorer" # string

# Wrong - all strings
linear_speed: "0.5"    # Should be float, not string
```
:::

:::danger Circular Include
**Problem**: Launch files include each other, causing infinite loop.

**Solution**: Design hierarchy carefully, avoid circular dependencies.
:::

## Summary

In this chapter, you learned:

- **Python launch files**: Flexible, programmatic node orchestration
- **XML launch files**: Declarative syntax for simple configurations
- **Launch arguments**: Make launch files configurable
- **Parameter files**: Separate configuration from code
- **Package setup**: Proper structure and installation
- **Best practices**: Maintainable, documented launch systems

**Key Capabilities:**
- Start multiple nodes with one command
- Configure parameters centrally
- Create reusable, composable launch files
- Support different deployment scenarios (sim vs. real robot)
- Reduce manual configuration errors

**When to Use Each Format:**
- **Python**: Complex logic, conditionals, dynamic configuration
- **XML**: Simple static configurations, easier for non-programmers

With launch files, you can now orchestrate complete robot systems efficiently and reproducibly. This completes Week 4: ROS 2 Part 2!

## Exercises

1. **Create a launch file** that starts your velocity publisher and status subscriber from earlier sections
2. **Add launch arguments** to configure velocities without editing the launch file
3. **Extract parameters** to a YAML file and load them in your launch file
4. **Build a composed system** with multiple included launch files
5. **Create conditional launch** that starts different nodes based on a `simulation` argument

## Further Reading

- [ROS 2 Launch System Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Creating Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Using Substitutions in Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)
- [Launch File Architecture](https://design.ros2.org/articles/roslaunch.html)
