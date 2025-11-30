# Your First Python ROS 2 Node

Welcome to the exciting world of robotics with ROS 2! This chapter will guide you through creating your very first ROS 2 node using Python, a journey that forms the bedrock of building complex robotic applications. By the end of this tutorial, you'll have a running ROS 2 node, understand its fundamental components, and be ready to explore more advanced topics.

## 1. Introduction to ROS 2 Nodes and Python

### What is a ROS 2 Node?

In ROS 2, a "node" is the fundamental unit of computation. Think of it as an executable program that performs a specific task within the ROS 2 ecosystem. A robot's software architecture is typically composed of many nodes, each responsible for a distinct function, such as:

*   Reading sensor data (e.g., camera, LiDAR, IMU)
*   Controlling motors
*   Performing localization or mapping
*   Executing navigation algorithms
*   Planning robot motions

These nodes communicate with each other using various ROS 2 communication mechanisms: topics, services, and actions. This distributed architecture allows for modularity, reusability, and fault tolerance in complex robotic systems.

### Why Python for Beginners?

Python is an excellent choice for beginners (and even experienced developers) in ROS 2 due to several advantages:

*   **Readability and Simplicity**: Python's clear syntax and high-level nature make it easy to write and understand code, reducing the learning curve.
*   **Rapid Prototyping**: Its interpreted nature allows for quick iteration and testing, speeding up development cycles.
*   **Extensive Libraries**: Python boasts a vast ecosystem of libraries for data processing, machine learning, computer vision, and more, which are invaluable in robotics.
*   **`rclpy`**: ROS 2 provides `rclpy`, a Python client library that offers a convenient and idiomatic way to interact with the core ROS 2 functionalities.

While C++ (`rclcpp`) is often used for performance-critical components, Python (`rclpy`) is perfectly suitable for many robotics tasks, especially for initial development, high-level control, and scripting.

## 2. Setting Up Your ROS 2 Workspace

Before we can write our first ROS 2 node, we need to set up a workspace. A ROS 2 workspace is a directory where you store your ROS 2 packages and build them. This allows you to manage multiple packages together and easily integrate them into your ROS 2 environment.

:::tip Key Takeaway
Always organize your ROS 2 projects within a dedicated workspace. This keeps your code separate from the system-wide ROS 2 installation and makes it easier to manage your own packages.
:::

Follow these steps to create and initialize your workspace:

### Step 2.1: Create the Workspace Directory

First, let's create a directory for our new workspace. It's common practice to name it `ros2_ws`, but you can choose any name you prefer. The `src` subdirectory is where your ROS 2 packages will reside.

```bash
mkdir -p E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src
```

### Step 2.2: Initialize the `colcon` Workspace

`colcon` is the build system used in ROS 2. To initialize your workspace, navigate to the root of your workspace (`ros2_ws`) and run `colcon build`. The `--symlink-install` option creates symbolic links to your install space, which is very useful during development as you don't need to rebuild every time you make small changes to your Python scripts.

```bash
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws
colcon build --symlink-install
```

After running this command, `colcon` will create `install`, `log`, and `build` directories in your `ros2_ws`. Initially, the `install` directory will contain the setup files for the base ROS 2 environment.

### Step 2.3: Source the Setup Files

To use the packages within your workspace, you need to "source" the setup file that `colcon` generated. This command adds your workspace's packages to your environment variables, making them discoverable by ROS 2.

```bash
# For Bash/Zsh (Linux/macOS)
source E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.bash

# For PowerShell (Windows)
# . E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.ps1

# For Command Prompt (Windows)
# call E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.bat
```

:::tip Important
You must source this setup file in *every new terminal* you open that you intend to use for ROS 2 development within this workspace. If you forget to source it, ROS 2 commands will not be able to find your packages. It's often helpful to add the `source` command to your shell's startup script (e.g., `.bashrc`, `.zshrc`) for convenience, but for learning purposes, manually sourcing is a good practice.
:::

## 3. Creating a New ROS 2 Package

Now that your workspace is set up, let's create a new ROS 2 package for our "Hello ROS 2" node. ROS 2 packages are the containers for your nodes, launch files, configurations, and other resources.

### Step 3.1: Create the Python Package

Navigate into the `src` directory of your workspace and use the `ros2 pkg create` command to generate a new Python package. We'll name it `my_py_pkg`.

```bash
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src
ros2 pkg create --build-type ament_python my_py_pkg
```

### Step 3.2: Understanding the Generated Directory Structure

After running the command, ROS 2 generates a basic package structure for `ament_python` packages:

```
my_py_pkg/
├── my_py_pkg/             # Python module directory
│   └── __init__.py        # Initializes the Python module
├── package.xml            # Package manifest: metadata about the package
└── setup.py               # Python build script for setuptools
```

*   **`my_py_pkg/` (inner directory)**: This is the actual Python module. Your Python source code will go here.
*   **`__init__.py`**: This file makes the `my_py_pkg` directory a Python package.
*   **`package.xml`**: This XML file contains metadata about your package, such as its name, version, description, maintainers, license, and its dependencies on other ROS 2 packages or system libraries.
*   **`setup.py`**: This Python script is used by `setuptools` (the Python packaging system) to define how your package should be built and installed.

## 4. Writing Your First Python Node

It's time to write the Python code for our "Hello ROS 2" node. We'll create a simple node that initializes, prints a message, and then shuts down gracefully.

### Step 4.1: Create the Node File

Inside your package's Python module directory (`my_py_pkg/my_py_pkg/`), create a new Python file named `hello_ros2_node.py`.

```bash
touch E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\my_py_pkg\hello_ros2_node.py
```

### Step 4.2: Add the Python Code

Now, open `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\my_py_pkg\hello_ros2_node.py` and add the following code:

```python title="hello_ros2_node.py" showLineNumbers
[View on GitHub](https://github.com/zainabkhimji2/hackathon_book/blob/main/ros2_ws/src/my_py_pkg/my_py_pkg/hello_ros2_node.py)
import rclpy
from rclpy.node import Node

class HelloROS2Node(Node):
    """
    A simple ROS 2 node that prints "Hello ROS 2" to the console.
    """
    def __init__(self):
        # Call the base Node class constructor to initialize the node
        # and give it the name 'hello_ros2_node'
        super().__init__('hello_ros2_node')
        # Log a message to the console using the node's logger
        self.get_logger().info('Hello ROS 2 from my_py_pkg!')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of our HelloROS2Node class
    hello_node = HelloROS2Node()

    # Keep the node alive. In this simple example, it will just execute
    # the __init__ method and then immediately shut down because spin_once
    # would only process one callback if there were any, and then finish.
    # For nodes with subscriptions, publishers, or timers, rclpy.spin()
    # is used to keep the node running and process callbacks.
    # For now, we'll just let it create and then destroy itself.
    hello_node.get_logger().info('Node created and message logged. Spinning down.')

    # Destroy the node explicitly
    hello_node.destroy_node()

    # Shutdown the rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

:::tip Code Explanation
*   **`import rclpy` and `from rclpy.node import Node`**: These lines import the necessary ROS 2 Python client library (`rclpy`) and the `Node` class, which is the base class for all ROS 2 nodes.
*   **`class HelloROS2Node(Node):`**: We define a class `HelloROS2Node` that inherits from `rclpy.node.Node`. This inheritance provides our class with all the functionalities of a ROS 2 node.
*   **`super().__init__('hello_ros2_node')`**: In the constructor (`__init__`), we call the constructor of the base `Node` class and pass the name of our node, which is `hello_ros2_node`.
*   **`self.get_logger().info(...)`**: This is the standard way to print messages from a ROS 2 node. `get_logger()` provides access to the node's logger, and `info()` logs an informational message. This is preferred over `print()` for better integration with ROS 2 logging tools.
*   **`main(args=None):`**: This is the entry point function for our node.
    *   **`rclpy.init(args=args)`**: Initializes the `rclpy` client library. This *must* be called before any other `rclpy` functions.
    *   **`hello_node = HelloROS2Node()`**: Creates an instance of our `HelloROS2Node` class.
    *   **`rclpy.spin_once(hello_node)`**: For a continuously running node that processes callbacks (e.g., from topics or timers), `rclpy.spin(hello_node)` would be used. For this simple node that just prints a message on startup, `spin_once` or even just letting the `__init__` finish and then shutting down works. We've added a final log message to illustrate the flow.
    *   **`hello_node.destroy_node()`**: Explicitly destroys the node. Good practice for resource management.
    *   **`rclpy.shutdown()`**: Shuts down the `rclpy` client library. This *must* be called after `rclpy.init()`.
*   **`if __name__ == '__main__':`**: This standard Python construct ensures that `main()` is called only when the script is executed directly (not when imported as a module).
:::

## 5. Configuring `setup.py`

For ROS 2 to recognize and execute your Python script as a node, you need to inform `setuptools` about it by modifying your `setup.py` file. This involves adding an `entry_points` section.

### Step 5.1: Open `setup.py`

Open the `setup.py` file located at `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\setup.py`.

### Step 5.2: Modify `setup.py`

Find the `setup(` function call and add the `entry_points` dictionary. This dictionary tells `colcon` to create an executable named `hello_ros2` that points to the `main` function in our `my_py_pkg.hello_ros2_node` module.

The `setup.py` file should look like this:

```python title="setup.py" showLineNumbers
[View on GitHub](https://github.com/zainabkhimji2/hackathon_book/blob/main/ros2_ws/src/my_py_pkg/setup.py)
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # TODO: Replace with your actual name
    maintainer_email='your.email@example.com', # TODO: Replace with your actual email
    description='My first Python ROS 2 package',
    license='Apache-2.0', # TODO: Choose an appropriate license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_ros2 = my_py_pkg.hello_ros2_node:main',
        ],
    },
)

```

:::tip `entry_points` Explanation
*   **`'console_scripts': [`**: This key tells `setuptools` that we are defining command-line executable scripts.
*   **`'hello_ros2 = my_py_pkg.hello_ros2_node:main'`**: This is the core of our entry point.
    *   `hello_ros2`: This is the name of the executable that you will use with `ros2 run`.
    *   `my_py_pkg.hello_ros2_node`: This specifies the Python module path (package name `my_py_pkg` and file `hello_ros2_node.py`).
    *   `main`: This is the function within `hello_ros2_node.py` that will be called when the executable `hello_ros2` is run.
:::

## 6. Building Your Package

After creating your node and configuring `setup.py`, you need to build your package. This process makes your node discoverable and executable by ROS 2.

### Step 6.1: Build with `colcon`

Navigate back to the root of your workspace (`ros2_ws`) and run `colcon build`. To speed up the build process for a single package, you can use the `--packages-select` option.

```bash
cd E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws
colcon build --packages-select my_py_pkg
```

This command will compile any C++ code (not applicable here) and install Python scripts into the `install` directory of your workspace.

### Step 6.2: Understanding the `install` Directory

After a successful build, if you inspect your `ros2_ws/install/my_py_pkg` directory, you'll find:
*   A `lib` directory containing your Python package (a symbolic link if you used `--symlink-install`).
*   An `share` directory with your `package.xml`.
*   A `local_setup.bash` (or `.ps1`, `.bat`) file, which, when sourced, adds your package's executables and resources to your environment.

## 7. Running Your ROS 2 Node

Now for the moment of truth! Let's run your first ROS 2 node.

### Step 7.1: Source the Workspace (Again)

If you've opened a new terminal since building, or if you modified your `setup.py` or added new executables, you *must* source your workspace's `setup` file again to ensure your new package is recognized.

```bash
# For Bash/Zsh (Linux/macOS)
source E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.bash

# For PowerShell (Windows)
# . E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.ps1

# For Command Prompt (Windows)
# call E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.bat
```

### Step 7.2: Run the Node

Use the `ros2 run` command to execute your node:

```bash
ros2 run my_py_pkg hello_ros2
```

### Expected Output

You should see output similar to this in your terminal:

```
[INFO] [1700000000.123456789] [hello_ros2_node]: Hello ROS 2 from my_py_pkg!
[INFO] [1700000000.123456789] [hello_ros2_node]: Node created and message logged. Spinning down.
```

This output confirms that your ROS 2 node `hello_ros2_node` was successfully initialized and printed its message!

## 8. Inspecting Your Node

ROS 2 provides command-line tools to inspect the active ROS 2 graph (the collection of running nodes and their connections). Let's use `ros2 node list` to see if our node is recognized.

### Step 8.1: Check Node List

Open a *new terminal*, source your workspace, and then run:

```bash
ros2 node list
```

Since our `hello_ros2` node executes its `__init__` and then immediately shuts down (as it has no ongoing tasks), you might not see it in the `ros2 node list` output unless you run `ros2 run my_py_pkg hello_ros2` in another terminal at the exact same moment.

To demonstrate `ros2 node list` more effectively, let's briefly modify `hello_ros2_node.py` to keep it spinning for a few seconds.

Temporarily modify `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\my_py_pkg\hello_ros2_node.py`:

```python title="hello_ros2_node.py" showLineNumbers
[View on GitHub](https://github.com/zainabkhimji2/hackathon_book/blob/main/ros2_ws/src/my_py_pkg/my_py_pkg/hello_ros2_node.py)
import rclpy
from rclpy.node import Node
import time # Import time module

class HelloROS2Node(Node):
    def __init__(self):
        super().__init__('hello_ros2_node')
        self.get_logger().info('Hello ROS 2 from my_py_pkg! Node is now active.')

def main(args=None):
    rclpy.init(args=args)
    hello_node = HelloROS2Node()

    # Keep the node spinning for 5 seconds to be visible in ros2 node list
    # For actual nodes with subscriptions/publishers, rclpy.spin(hello_node) is used.
    end_time = time.time() + 5
    while rclpy.ok() and time.time() < end_time:
        rclpy.spin_once(hello_node, timeout_sec=0.1) # Process callbacks once

    hello_node.get_logger().info('5 seconds elapsed. Shutting down node.')
    hello_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

After saving this change, you don't need to rebuild because `colcon build --symlink-install` symlinks the Python files. Just source your workspace again (if you haven't recently), then run `ros2 run my_py_pkg hello_ros2` in one terminal. Quickly open a *second terminal*, source it, and run `ros2 node list`. You should now see:

```
/hello_ros2_node
```

This confirms that your node is indeed running and visible in the ROS 2 graph!

:::warning Remember to revert!
Don't forget to revert the temporary `time` module changes in `hello_ros2_node.py` to keep the chapter's example simple and focused on the initial message.
:::

## 9. Troubleshooting Common Issues

Beginners often encounter a few common pitfalls. Here's how to diagnose and fix them:

:::danger Troubleshooting

### Issue 1: `ros2: command not found` or `ros2 pkg: command not found`
**Symptom**: You try to run `ros2` commands, and your shell can't find them.
**Cause**: Your ROS 2 environment has not been sourced.
**Solution**: Ensure you have sourced your main ROS 2 installation (e.g., `/opt/ros/humble/setup.bash` for Humble on Linux) and then your workspace setup file (e.g., `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.bash`). Remember to do this in *every new terminal*.

### Issue 2: `[ros2run]: Could not find package 'my_py_pkg'`
**Symptom**: When trying to run `ros2 run my_py_pkg hello_ros2`, it says the package cannot be found.
**Cause**: Your workspace's `install` directory hasn't been sourced, or the package wasn't built correctly.
**Solution**:
1.  **Source the workspace**: Make sure you've sourced `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\install\setup.bash`.
2.  **Build the package**: Run `colcon build --packages-select my_py_pkg` from your `ros2_ws` directory. Check for any errors during the build.
3.  **Check `package.xml` and `setup.py`**: Ensure `package.xml` is present and `setup.py` correctly defines the package name.

### Issue 3: `[ros2run]: Could not find executable 'hello_ros2'`
**Symptom**: ROS 2 finds the package but not the executable.
**Cause**: `setup.py` is not correctly configured to create the `hello_ros2` executable.
**Solution**:
1.  **Verify `setup.py`**: Double-check the `entry_points` section in `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\setup.py`. Ensure `hello_ros2 = my_py_pkg.hello_ros2_node:main` is correct, paying close attention to module name (`my_py_pkg.hello_ros2_node`) and function name (`main`).
2.  **Rebuild**: After modifying `setup.py`, you *must* rebuild your package: `colcon build --packages-select my_py_pkg`. Then, re-source your workspace.

### Issue 4: Python errors (e.g., `ModuleNotFoundError`)
**Symptom**: When running your node, you see Python tracebacks like `ModuleNotFoundError: No module named 'rclpy'`.
**Cause**: Your Python environment isn't set up correctly for ROS 2, or there's a typo in your code.
**Solution**:
1.  **ROS 2 Python dependencies**: Ensure ROS 2's Python dependencies are installed. On Ubuntu, this usually means `sudo apt install ros-humble-ros-tutorials ros-humble-rqt-common-plugins python3-colcon-common-extensions`. For Windows, ensure you followed the official ROS 2 installation guide carefully, including installing Python packages.
2.  **Check Python code**: Carefully review `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\my_py_pkg\hello_ros2_node.py` for typos, especially in `import` statements or class/function names.

### Issue 5: No output from node
**Symptom**: You run `ros2 run my_py_pkg hello_ros2`, and nothing appears.
**Cause**: The node might be crashing immediately, or the `get_logger().info()` call is not reached.
**Solution**:
1.  **Check for Python syntax errors**: Run your Python script directly using `python3 E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\ros2_ws\src\my_py_pkg\my_py_pkg\hello_ros2_node.py`.
2.  **Add more log statements**: Temporarily add more `self.get_logger().info()` calls at different points in your code to pinpoint where execution stops.
:::

Congratulations! You've successfully created, built, and run your first Python ROS 2 node. This fundamental understanding of workspaces, packages, nodes, and `setup.py` will serve you well as you delve deeper into building more sophisticated robotic applications. The next steps involve learning how nodes communicate using topics, services, and actions.
