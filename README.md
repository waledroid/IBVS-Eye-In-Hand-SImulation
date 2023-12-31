# IBVS with Aruco Tag
<h3> Qian Zilling, Atanda Abdullahi Adewale, Nauman </h#><br></br>


# OBJECTIVE:
The aim of this VIBOT MSFT course project is to implement or simulate image-based visual servo control in Gazebo using a Doosan M0609 robotic arm manipulator with a camera mounted on its end effector. The visual features will be provided from 4-points in an Aruco tag.

# METHODS:

### PART 1
# Robot Structure Definition, Add components and Gazebo Scene Setup::

# main.xacro
- Build and configure the robotic structure for the Doosan M0609 robotic arm using URDF and Xacro.
- Add a onrobot_rg2 gripper to the end effector.
- Add the Camera 

<pre>
  <pre>
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="m0609" >
  
  <pre><!-- Define customizable properties --></pre>
  ...
  <-- Assign property values for clarity -->
  ...
  <-- Include relevant Xacro files for modularity -->
  <xacro:include filename="$(find dsr_description)/xacro/macro.m0609.white.xacro" />
  <xacro:include filename="$(find manipulator_description)/urdf/onrobot_rg2_model_macro.xacro" />
  <xacro:include filename="$(find manipulator_description)/urdf/camera.xacro" />

  <-- Include the onrobot_rg2 macro for gripper components -->
  <xacro:onrobot_rg2 prefix="my_robot_rg2"/>
    
  <-- Define the world link -->
  <link name="world" />
  <-- Create a fixed joint between the world and the robot's base -->
  
  <-- Create a fixed joint representing the robot's wrist (effector) -->
  <joint name="wrist" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="link6"/>
    <child link="my_robot_rg2onrobot_rg2_base_link"/>
  </joint>

  <--Create the Aruco wall link-->
  <-- Create a joint between the wall and the robot's base -->
      
  </robot>
  </pre>
</pre>

# Camera.xacro 
- Defines a camera link (camera_link) as a visual box with a specified size and a red color for Gazebo simulation.
- Integrate a camera at the end effector joint. Creates a fixed joint (camera_joint) to rigidly attach the camera link to its parent link ("link6")
- Configures the camera to simulate a camera sensor, then define a ROS Gazebo plugin interfaces for the simulated camera for information about camera topics and distortion parameters.

<pre>

  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="kinova">
    
  <!-- Define customizable properties -->
  <link name="camera_link"> ... </link>

  <joint name="camera_joint" type="fixed">
    <parent link="link6"/>
    <child link="camera_link"/>
    <origin rpy="${M_PI/2} ${-M_PI/2} 0" xyz="${offset_from_link_x} ${offset_from_link_y} ${offset_from_link_z}"/>
    <axis xyz="1 0 0" />
  </joint>

  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_camera_sensor">
      <!-- .... -->
      <plugin name="camera_camera_controller" filename="libgazebo_ros_camera.so">
      <!---...--->
      </plugin>
    </sensor>
  </gazebo>

  </robot>
</pre>

# Aruco.xarco
- Place an Aruco tag within the world scene.
<pre>
    <!-- gazebo_world.world -->
  <?xml version="1.0"?>
  
  <sdf version="1.4">
    <world name="default">
  
      <!-- ... other world properties ... -->
  
      <include>
        <uri>model://m0609_robot</uri>
        <pose>0 0 0 0 0 0</pose>
      </include>
  
      <model name="aruco_tag">
        <!-- Add Aruco tag properties here -->
        <pose>1 1 1 0 0 0</pose>
        <!-- ... other Aruco tag properties ... -->
      </model>
  
    </world>
  </sdf>
</pre>

# main.launch
Spawn the robot in the Gazebo scene.


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
