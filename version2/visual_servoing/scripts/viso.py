#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import CompressedImage
import moveit_commander
from geometry_msgs.msg import PoseStamped,Pose, Point
import numpy as np
import cv2
class VISO:
    def __init__(self):
        # Init ros node
        rospy.init_node('viso',anonymous=True)
        # Define publishers/subscribers
        self.movement_pub = rospy.Publisher('/movement', Pose, queue_size=1)
        self.image_sub = rospy.Subscriber('/dsr01/kinova/camera/image_raw/compressed',CompressedImage,self.image_callback)
        self.target_sub = rospy.Subscriber('/movement', Pose, self.target_callback)
        # Define Target and Robot Pose objects
        self.target = Pose()
        self.movement = Pose()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.measurements = []
        self.markerLength=0.33
        self.dist_coeffs = np.array([[-0.076064, 0.025951, -0.001627, -0.000192, 0.000000]])
        # self.camera_matrix = np.array([[154.30758,   0.     , 150.51915],
        #                                 [0.     , 155.07511, 109.41527],
        #                                 [0.     ,   0.     ,   1.     ]])
        self.camera_matrix = np.array([[100,   0.     , 510.65921052],
                                        [0.     , 100, 510.27468646],
                                        [0.     ,   0.     ,   1.     ]])       
        self.corners_list, self.ids, self.rejectedImgPoints = None, None, None
        self.detector_enabled = True    
        #Todo 检查米和厘米
        ####################load robot#########################
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander(robot_description="/dsr01m0609/robot_description", ns = '/dsr01m0609')
        self.scene = moveit_commander.PlanningSceneInterface(ns='/dsr01m0609')
        self.arm = moveit_commander.MoveGroupCommander(name="arm",robot_description="/dsr01m0609/robot_description", ns = '/dsr01m0609')
        self.end_effector_link = self.arm.get_end_effector_link()
        self.reference_frame = 'base_0'
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        rospy.loginfo("succeed to load robot's parameters")
        ########################################################
        rospy.spin()   
    def image_callback(self,msg):
        if self.detector_enabled:# TODO self.detector_enabled 由 ‘/movement' 话题的subscriber设为False  将 motion.py合并过来
            rospy.loginfo("Seen an image")
            np_arr = np.frombuffer(msg.data, np.uint8)
            #rospy.loginfo(np_arr)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)
            self.corners_list, self.ids, self.rejectedImgPoints  = detector.detectMarkers(gray)
            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(self.corners_list, self.markerLength, self.camera_matrix, self.dist_coeffs)  
            # Draw qr boxes, undistort and get cooridnates
            rospy.loginfo(cv_image.shape)
            self.process_ar(self.corners_list, cv_image)

            if self.ids is not None and len(self.ids) > 0:#######################改为如果检测到ARUCO
                rospy.loginfo("Seen aruco!")
                self.measurements.append(tvec[0])
                cv2.imwrite(f"AR Code Detection{len(self.measurements)}.jpg", cv_image)
                # If we have collected enough measurements, calculate the average
                if len(self.measurements) == 5:
                    # Convert the list of measurements to a NumPy array for easier manipulation
                    measurements_array = np.array(self.measurements)
                    # Calculate the average for each dimension
                    avg_diff = np.mean(measurements_array, axis=0)
                    # print('gagaga',measurements_array)
                    # print('lalala',avg_diff)
                    # Set the average values to the movement position
                    self.movement.position.x = avg_diff[2][0]#z(camera frame)
                    self.movement.position.y = -avg_diff[0][0]#-x
                    self.movement.position.z = -avg_diff[1][0]#-y
                    print("Average Movement:\n", self.movement.position)
                    # Clear the measurements list for the next set of measurements
                    self.measurements = []
                    # Publish the movement
                    self.movement_pub.publish(self.movement)
                    
                    rospy.loginfo("vision task finished!")
                    ####################################end################################################                       
    def my_estimatePoseSingleMarkers(self, corners_list, marker_size, mtx, distortion):
        '''
        This will estimate the rvec and tvec for each of the marker corners detected by:
        corners_list, ids, rejectedImgPoints = detector.detectMarkers(image)
        corners_list - is an array of detected corners for each detected marker in the image
        marker_size - is the size of the detected markers
        mtx - is the camera matrix
        distortion - is the camera distortion matrix
        RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
        '''
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners_list:
            nada, R, t = cv2.solvePnP(marker_points, c[0], mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash
    def process_ar(self,corners_list, frame):
        #print(corners_list)
        for corners in corners_list:  
            print('hahaha',corners.shape)     
            color = (255, 255, 0)
            #print(np.min(corners[:,0]))
            cv2.rectangle(frame, (int(np.min(corners[0][:,0])), int(np.min(corners[0][:,1]))),(int(np.max(corners[0][:,0])), int(np.max(corners[0][:,1]))),color, 2)
    def target_callback(self,msg):
        self.detector_enabled = False
        #global target, current_position, ready, diferencia, moving
        # if not ready or moving:
        #     return
        movements = msg.position
        current_position = self.arm.get_current_pose().pose # Get current position
        print("Current position: ", current_position.position) #单位是米 目标地方
        self.target.position.x = current_position.position.x +movements.x
        self.target.position.y = current_position.position.y +movements.y 
        self.target.position.z = current_position.position.z +movements.z 
        print('START POSITION',current_position)
        print('TARGET POSITION',self.target)
        #global current_position, diferencia, moving,target
        print("Starting to listen for instructions")
        #while not rospy.is_shutdown():
        current_x = round(current_position.position.x, 2)
        current_y = round(current_position.position.y, 2)
        current_z = round(current_position.position.z, 2)
        target_x = round(self.target.position.x, 2)
        target_y = round(self.target.position.y, 2)
        target_z = round(self.target.position.z, 2)
        # print("Current: ", current_x, current_y, current_z)
        if target_x != current_x or target_y != current_y or target_z != current_z:
            # ###############method1 use planing(doesn't work)####################### 
            # target_pose = PoseStamped()
            # target_pose.header.frame_id = self.reference_frame
            # target_pose.header.stamp = rospy.Time.now()     
            # target_pose.pose.position.x = target_x
            # target_pose.pose.position.y = target_y
            # target_pose.pose.position.z = target_z
            # target_pose.pose.orientation.x = 1
            # target_pose.pose.orientation.y = 0
            # target_pose.pose.orientation.z = 0
            # target_pose.pose.orientation.w = 0
            # self.arm.set_pose_target(target_pose, self.end_effector_link)
            # traj = self.arm.plan()
            # self.arm.execute(traj) 
            ###############method2 no use planing#######################
            #self.arm.set_pose_target([ target_x, target_y, target_z, 1, 0, 0, 0],self.end_effector_link)
            self.arm.set_pose_target([ target_x, target_y, target_z, 0.98, 0, 0, 0])
            self.arm.go()
            #############################################
            #######question:what's the difference?
            print('Target position reached!')
        else:
            print("Current state is equal to goal state")

        # Update robot instance
        self.robot = moveit_commander.RobotCommander(robot_description="/dsr01m0609/robot_description", ns ='/dsr01m0609')
        # Clean up
        moveit_commander.roscpp_shutdown()  
      
        
if __name__ == '__main__':
    try:
        node = VISO()

    except rospy.ROSInterruptException:
        pass
     

