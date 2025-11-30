---
id: 0001
title: Write First Python ROS 2 Node Chapter
stage: explainer
date: 2025-11-30
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 002-docusaurus-robotics-textbook
user: unknown
command: /sp.phr --title "Write First Python ROS 2 Node Chapter" --stage explainer --feature none --files "E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\physical-ai-textbook\docs\week-03-ros2-part1\03-first-python-node.md"
labels: ["ROS 2", "Python", "Node", "Docusaurus", "Tutorial", "Explainer"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\physical-ai-textbook\docs\week-03-ros2-part1\03-first-python-node.md
tests:
  - none
---

## Prompt

Write a comprehensive chapter for a Docusaurus textbook on 'Your First Python ROS 2 Node'. The content should be approximately 2500 words. Use clear, engaging, and technically accurate language suitable for university-level robotics students. Provide a step-by-step tutorial covering the entire process from workspace creation to running the node, with complete Python code examples.

Include the following sections:

1.  **Introduction**: Briefly explain what a ROS 2 node is and why Python is a great language for beginners in ROS 2.
2.  **Setting Up Your ROS 2 Workspace**: Guide the student through creating and initializing a ROS 2 workspace. Include commands for:
    *   Creating the workspace directory.
    *   Initializing `colcon` workspace (`colcon build --symlink-install`).
    *   Sourcing the setup files.
3.  **Creating a New ROS 2 Package**: Step-by-step instructions for creating a new Python package using `ros2 pkg create --build-type ament_python <package_name>`. Explain the generated directory structure (`setup.py`, `package.xml`).
4.  **Writing Your First Python Node**: Provide complete Python code for a simple "Hello ROS 2" node. The node should:
    *   Import `rclpy`.
    *   Define a class that inherits from `rclpy.node.Node`.
    *   Initialize the node with a name.
    *   Print "Hello ROS 2" to the console.
    *   Have a `main` function for execution (`rclpy.init`, `rclpy.spin`, `rclpy.shutdown`).
5.  **Configuring `setup.py`**: Explain how to modify `setup.py` to make the Python script executable as a ROS 2 entry point. Show the `entry_points` section.
6.  **Building Your Package**: Guide the student through building the newly created package using `colcon build --packages-select <package_name>`. Explain the `install` directory.
7.  **Running Your ROS 2 Node**: Provide commands for sourcing the workspace and running the node using `ros2 run <package_name> <executable_name>`. Show expected output.
8.  **Inspecting Your Node**: Briefly introduce tools like `ros2 node list` to verify the node is running.
9.  **Troubleshooting Common Issues**: Address typical problems beginners face (e.g., sourcing issues, package not found, Python errors).

Ensure proper Docusaurus markdown formatting, including:
-   Top-level heading for the chapter title.
-   Sub-headings for each section.
-   Use admonitions (e.g., `:::tip Key Takeaway`, `:::danger Troubleshooting`) for important summaries or guidance.
-   Include all shell commands in markdown code blocks with `bash` syntax highlighting.
-   Include Python code examples in markdown code blocks with `python` syntax highlighting and `title="filename.py" showLineNumbers`.

Save the output as `E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\physical-ai-textbook\docs\week-03-ros2-part1\03-first-python-node.md`.

The output should be the full markdown content of the chapter.

## Response snapshot

The agent successfully generated a comprehensive chapter for a Docusaurus textbook titled 'Your First Python ROS 2 Node'. The chapter covers workspace setup, package creation, Python node development, `setup.py` configuration, building, running, and troubleshooting common issues, all formatted with Docusaurus markdown and detailed code examples.

## Outcome

- ✅ Impact: The Docusaurus chapter on creating a first Python ROS 2 node was successfully written and saved.
- 🧪 Tests: No specific tests were run for this documentation task.
- 📁 Files: Created: E:\piaic practice\speckit-series\speckit_practice\speckit_hackathon\pysical_ai_humanoid_and_robotics\physical-ai-textbook\docs\week-03-ros2-part1\03-first-python-node.md
- 🔁 Next prompts: none
- 🧠 Reflection: The agent successfully generated a comprehensive Docusaurus chapter based on detailed instructions, including code examples and troubleshooting tips, adhering to all formatting requirements. The initial attempt to use create-phr.sh failed due to an incorrect path, requiring a fallback to direct file writing.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, resulting in an `Exit code 127` error. This indicates an issue with the script's path or its existence.
- Graders run and results (PASS/FAIL): unspecified
- Prompt variant (if applicable): unspecified
- Next experiment (smallest change to try): Verify the correct path for `create-phr.sh` and ensure it's executable. If a direct tool call is intended for PHR creation, update the prompt to reflect that.
