# IBVS with Aruco Tag
<h3> Qian Zilling, Atanda Abdullahi Adewale, Nauman </h#><br></br>


# Objective
Implement or simulate image-based visual servo control in Gazebo using a Doosan M0609 robotic arm manipulator with a camera mounted on its end effector. The visual features will be provided by a 4-point Aruco tag.


# Robot Structure Definition:
Build the robotic structure using URDF and Xacro.
Configure a 6-degree-of-freedom joint for the Doosan M0609 robotic arm.
Integrate a camera at the end effector joint.

# Gazebo Scene Setup:
Design a Gazebo world to simulate the environment.
Spawn the robot in the Gazebo scene.
Place an Aruco tag with 4 corner points within the scene.

# Aruco Tag Detection:
Implement a mechanism to detect the Aruco tag in the Gazebo scene.
Extract the 4 corner points of the detected Aruco tag.

# Visual Servoing with ViSP (C++):
Develop C++ code using ViSP for visual servoing based on the 4 corner points.
Calculate the necessary transformations to control the robot's end effector.

# MoveIt! Integration:
Integrate MoveIt! for motion planning and control.
Use the computed transformations to generate a trajectory for the robot's end effector.
Cartesian Space Manipulation Node:

# Develop a ROS node to manipulate the robot in Cartesian space.
Subscribe to the camera images and perform visual servoing.
Publish control commands to move the robot's end effector closer to the Aruco tag.

# Simulation and Testing:
Launch the Gazebo simulation environment.
Run the ROS nodes and observe the robot's movement in response to the Aruco tag.

# Optimization and Documentation:
Fine-tune the visual servoing algorithm and system parameters.
Document the project, including URDF, Xacro files, and ROS node descriptions.
