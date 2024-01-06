# EYE-IN-HAND IBVS SIMULATION USING ARUCO TAGS
 Qian Zilling, Atanda Abdullahi Adewale, Nauman Shafique Hashmi <br>


# Objective:
The aim of this VIBOT MSFT course project is to implement or simulate an eye-in-hand image-based visual servo control in Gazebo using a Doosan M0609 robotic arm manipulator with a camera mounted on its end effector. The visual features will be provided from 4-points in an Aruco tag.

# Dependencies:
This projects runs in ROS noetic, Opencv-python, Move-it! and Gazebo.

# PART 1
## Robot Structure Definition, Add components and Gazebo Scene Setup:

#### [main.xacro](version2/manipulator_description/manipulator_description/xacro/main.xacro)
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

#### [camera.xacro](version2/manipulator_description/manipulator_description/urdf/camera.xacro)
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

#### [generatear.py](generatear.py)
#### [arucoworld.world](version1/visual_servoing/worlds/arucoworld.world)
- This file generates differents 6x6 aruco tags [0-29] then plces them in the .gazebo/models folder 
- This will be accesible in gazebo with a drag and drop operation
- We save the gazebo scene containing Aruco to visual_servoing/world/arucoworld.world file for easy reuse

![Gazebo Setup](images/ar.jpg)


# PART 2
## Move-it Integration, Motion Planning
#### [dsr_moveit_gazebo.launch](version1/m0609_moveit_config/m0609_moveit_config/launch/dsr_moveit_gazebo.launch) 
- This is a modified Move-it main launch file 
- It calls the default doosan move-it packages and config files to manage the movement of the m0609 robot arm.
![Motion Planning](images/moveit.png)

# PART 3
## Camera Image processing, Aruco Tag Detection, Extract 4 corner points, Coordinate Transformation: 

#### version 1 -->  [viso.py](version1/visual_servoing/scripts/viso.py)
#### version 2 -->  [viso_follow.py](version2/visual_servoing/scripts/viso_follow.py)

- Subcribe to gazebo_camera Topic at '/dsr01/kinova/camera/image_raw/compressed' to get raw image  
- Convert Image Format to OpencV format via cv_bridge
- ArUco Detection using camera parameters (intrinsic matrix, distortion coefficients)
- Extract the 4 corner points of the detected Aruco tag.
- perform Coordinate Transformation to transform the ArUco tag's corner points to align them with the end effector's coordinate system.

<pre>
</pre>

# PART 4
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

# PART 5
## Simulation, Testing and Optimization:

#### main.launch
- Launch the Gazebo simulation environment.
- Run the ROS nodes and observe the robot's movement in response to the Aruco tag.
- Fine-tune the visual servoing algorithm and system parameters.

<pre></pre>


<video>
 
</video>
