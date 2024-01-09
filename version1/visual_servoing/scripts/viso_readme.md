"viso.py" integrates computer vision, MoveIt (a motion planning framework for ROS), and robot control.

 **Initialization:**
   - It initializes a ROS node named "viso" and sets up publishers and subscribers for image processing (`image_sub`), receiving movement instructions (`target_sub`), and sending movement commands (`movement_pub`).

 **Vision Functionality:**
   - The `image_callback` function receives compressed image messages from the robot's camera topic (`/dsr01/kinova/camera/image_raw/compressed`) and processes them for ArUco marker detection.
   - It uses OpenCV functions to detect ArUco markers in the received images, estimate their poses in 3D space, and processes the detected markers on the image by drawing rectangles around them.
   - When ArUco markers are detected, it calculates their positions and publishes movement commands (`Pose` messages) to `/movement` topic.

 **Robot Motion Control with MoveIt:**
   - It initializes MoveIt components for planning and executing robot motions.
   - The `target_callback` function receives movement instructions (`Pose` messages) from the `/movement` topic. It disables the ArUco marker detector (`self.detector_enabled = False`) to focus on executing movement commands.
   - Upon receiving movement instructions, it calculates the target position based on the received movement increments and the current robot position.
   - It attempts to move the robot arm to the calculated target position using MoveIt's motion planning (`self.arm.go()`). It sets a target pose and executes the motion to reach the desired position in the workspace.

 **Camera Calibration Parameters and Marker Detection:**
   - The node includes camera calibration parameters (`self.camera_matrix`, `self.dist_coeffs`) used for the undistortion of images and marker detection.
   - The `my_estimatePoseSingleMarkers` function estimates the pose (rotation and translation) of detected ArUco markers based on the provided camera matrix and distortion coefficients.


Overall, this script integrates image processing (ArUco marker detection) with MoveIt for robot motion control within a ROS environment, allowing the robot to receive movement instructions based on detected markers and execute planned motions to reach the specified positions.
