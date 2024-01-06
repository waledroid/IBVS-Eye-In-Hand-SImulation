# ARUCO IBVS SIMULATION
 Qian Zilling, Atanda Abdullahi Adewale, Nauman <br>


# Objective:
The aim of this VIBOT MSFT course project is to implement or simulate an eye-in-hand image-based visual servo control in Gazebo using a Doosan M0609 robotic arm manipulator with a camera mounted on its end effector. The visual features will be provided from 4-points in an Aruco tag.

# Dependencies:
This projects runs in ROS noetic, Opencv-python, Move-it! and Gazebo.

# PART 1
## Robot Structure Definition, Add components and Gazebo Scene Setup:

#### main.xacro
- Build and configure the robotic structure for the Doosan M0609 robotic arm using URDF and Xacro.
- Add a onrobot_rg2 gripper to the end effector.
- Add the Camera 

<pre>
  < robot >
  
  < !-- Define customizable properties -->
  ...
  < !-- Assign property values for clarity -->
  ...
  < !-- Include relevant Xacro files for modularity -->
  < xacro:include filename="$(find dsr_description)/xacro/macro.m0609.white.xacro" />
  < xacro:include filename="$(find manipulator_description)/urdf/onrobot_rg2_model_macro.xacro" />
  < xacro:include filename="$(find manipulator_description)/urdf/camera.xacro" />

  < !-- Include the onrobot_rg2 macro for gripper components -->
  < xacro:onrobot_rg2 prefix="my_robot_rg2"/>
    
  < !-- Define the world link -->
  < link name="world" />
  < !-- Create a fixed joint between the world and the robot's base -->
  
  < !-- Create a fixed joint representing the robot's wrist (effector) -->
  < joint name="wrist" type="fixed">
    < origin xyz="0 0 0" rpy="0 0 0"/>
    < parent link="link6"/>
    < child link="my_robot_rg2onrobot_rg2_base_link"/>
  < /joint>

  < /robot>
</pre>


#### Camera.xacro 
- Defines a camera link (camera_link) as a visual box with a specified size and a red color for Gazebo simulation.
- Integrate a camera at the end effector joint. Creates a fixed joint (camera_joint) to rigidly attach the camera link to its parent link ("link6")
- Configures the camera to simulate a camera sensor, then define a ROS Gazebo plugin interfaces for the simulated camera for information about camera topics and distortion parameters.

<pre>

  < robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="kinova">
    
  < !-- Define customizable properties -->
  < link name="camera_link"> ... </link>

  < joint name="camera_joint" type="fixed">
    < parent link="link6"/>
    < child link="camera_link"/>
    < origin rpy="${M_PI/2} ${-M_PI/2} 0" xyz="${offset_from_link_x} ${offset_from_link_y} ${offset_from_link_z}"/>
    < axis xyz="1 0 0" />
  < /joint>

  < gazebo reference="camera_link">
    < sensor type="camera" name="camera_camera_sensor">
      ...
      < plugin name="camera_camera_controller" filename="libgazebo_ros_camera.so">
      ...
      < /plugin>
    < /sensor>
  < /gazebo>

  < /robot>
</pre>

#### generatear.py
#### arucoworld.world
- This file generates differents 6x6 aruco tags [0-29] then plces them in the .gazebo/models folder 
- This will be accesible in gazebo with a drag and drop
- We save the gazebo scene containing Aruco to visual_servoing/world/arucoworld.world file

![Alt text](./aruco_in_scene.jpg?raw=true "Gazebo Setup")


#### main.launch
Spawn the robot in the Gazebo scene.
<pre></pre>

# PART 2
## Camera Image processing, Aruco Tag Detection, Extract 4 corner points, Coordinate Transformation: 

#### aruco_detect.cpp
- Convert Image Format to OpencV format via cv_bridge
- ArUco Detection using camera parameters (intrinsic matrix, distortion coefficients)
- Extract the 4 corner points of the detected Aruco tag.
- perform Coordinate Transformation to transform the ArUco tag's corner points to align them with the end effector's coordinate system.

<pre>
</pre>

# PART 3
## Visual Servoing with ViSP (C++), MoveIt! Integration, manipulate the robot in Cartesian space:

#### visp.cpp
- Visp code to Calculate the necessary transformations to control the robot's end effector.
- Integrate MoveIt! for motion planning and control.
- Use the computed transformations to generate a trajectory for the robot's end effector.
- Subscribe to the camera images and perform visual servoing.
- Publish control commands to move the robot's end effector closer to the Aruco tag.

<pre>
</pre>

<pre>moveit</pre>

# PART 4
## Simulation, Testing and Optimization:

#### main.launch
- Launch the Gazebo simulation environment.
- Run the ROS nodes and observe the robot's movement in response to the Aruco tag.
- Fine-tune the visual servoing algorithm and system parameters.

<pre></pre>


<video>
 
</video>
