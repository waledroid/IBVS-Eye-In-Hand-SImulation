#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import CompressedImage
import moveit_commander
from geometry_msgs.msg import PoseStamped,Pose, Point
from visual_servoing.srv import Movement,MovementRequest,MovementResponse
import numpy as np
import cv2
#TODO check errors and consider rotation in Quaternion part
class VISO_FOLLOW:
    def __init__(self):
        # Init ros node
        rospy.init_node('viso',anonymous=True)
        # Define publishers/subscribers
        #self.movement_pub = rospy.Publisher('/movement', Pose, queue_size=1)
        self.movement_service_client = rospy.ServiceProxy('/movement', Movement)
        self.movement_service_server = rospy.Service('/movement', Movement , self.target_callback) # create the Service called my_service with the defined callback
        #rospy.wait_for_service('/movement')        
        self.image_sub = rospy.Subscriber('/dsr01/kinova/camera/image_raw/compressed',CompressedImage,self.image_callback, queue_size=1)
        #self.target_sub = rospy.Subscriber('/movement', Pose, self.target_callback)
        # Define Target and Robot Pose objects
        self.target_posi = Pose()
        self.target_orient = Pose()
        self.movement = Pose()
        self.rotation = Pose()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
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
        self.arm.set_goal_position_tolerance(0.005)
        self.arm.set_goal_orientation_tolerance(0.01)
        rospy.loginfo("succeed to load robot's parameters")
        ########################################################
        self.Jacobi = None
        self.e = None
        self.u = None
        self.v = None
        self.pixelsize = 2*10**(-4)#m
        self.u_star =1020/2*self.pixelsize#target const
        self.v_star =1020/2*self.pixelsize#target
        self.Z = None
        self.threshold=0.007
        self.counter =0
        self.lambd=10
        self.moving = False
        rospy.spin()   
    def image_callback(self,msg):
        if self.detector_enabled:# TODO self.detector_enabled 由 ‘/movement' 话题的subscriber设为False  将 motion.py合并过来
            #rospy.loginfo("Seen an image")
            np_arr = np.frombuffer(msg.data, np.uint8)
            #rospy.loginfo(np_arr)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(self.aruco_dict, parameters)
            self.corners_list, self.ids, self.rejectedImgPoints  = detector.detectMarkers(gray)
            rvec, tvec, _ = self.my_estimatePoseSingleMarkers(self.corners_list, self.markerLength, self.camera_matrix, self.dist_coeffs)  
            # Draw qr boxes, undistort and get cooridnates
            #rospy.loginfo(cv_image.shape)
            self.process_ar(self.corners_list, cv_image)
            if self.ids is not None and len(self.ids) > 0:#######################改为如果检测到ARUCO
                #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                #print(self.corners_list[0][0])
                self.u = int((np.min(self.corners_list[0][0][:,0])+np.max(self.corners_list[0][0][:,0]))/2)*self.pixelsize
                self.v = int((np.min(self.corners_list[0][0][:,1])+np.max(self.corners_list[0][0][:,1]))/2)*self.pixelsize
                self.Z = tvec[0][-1].item()
                self.Jacobi =np.array([[-1/self.Z, 0, self.u/self.Z, self.u*self.v, -(1+self.u**2), self.v], [0, -1/self.Z, self.v/self.Z, 1+self.v**2, -self.u*self.v, -self.u]])
                #print(self.Jacobi)
                self.Jacobi = np.linalg.pinv(self.Jacobi)
                
                self.e =np.array([[self.u_star-self.u],[self.v_star-self.v]])
                self.movement_temp = self.lambd*self.Jacobi@self.e
                #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
                #rospy.loginfo("Seen aruco!")
                #cv2.imwrite("AR Code Detection_following.jpg", cv_image)
                # If we have collected enough measurements, calculate the average
                # Convert the list of measurements to a NumPy array for easier manipulation
                # # Calculate the average for each dimension
                # print('gagaga',measurements_array)
                # print('lalala',avg_diff)
                # Set the average values to the movement position
                self.movement.position.x = self.movement_temp[2][0]#z(camera frame)
                self.movement.position.y = -self.movement_temp[0][0]#-x
                self.movement.position.z = -self.movement_temp[1][0]#-y    
                self.rotation.position.x = self.movement_temp[5][0]
                self.rotation.position.y = -self.movement_temp[3][0]
                self.rotation.position.z = -self.movement_temp[4][0]

                # Clear the measurements list for the next set of measurements
                #self.movement_pub.publish(self.movement)        
                movement_object = MovementRequest()   
                movement_object.movement = self.movement
                movement_object.rotation = self.rotation
                if not self.moving:
                    result = self.movement_service_client(movement_object)
                #print('hahahaha',result)     
               
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
            #print('hahaha',corners.shape)#(1, 4, 2)   
            color = (255, 255, 0)
            #print(np.min(corners[:,0]))
            cv2.rectangle(frame, (int(np.min(corners[0][:,0])), int(np.min(corners[0][:,1]))),(int(np.max(corners[0][:,0])), int(np.max(corners[0][:,1]))),color, 2)
    #TODO 添加误差小于一定程度，停止检测 self.detector_enabled = False 
    def target_callback(self,request):
        self.moving =True
        if np.linalg.norm(self.e)< self.threshold :
            self.detector_enabled = False
            moveit_commander.roscpp_shutdown()  # Clean up
            print('arrived!')
            response = MovementResponse()
            response.success = True
            return response
        print('current error:',self.e)
        print(f'current iter is {self.counter}')  
        print("Average Movement of current iter:\n", self.movement.position)      
        movements = request.movement.position
        current_position = self.arm.get_current_pose().pose # Get current position
        print("Current position and start position:\n ", current_position) #单位是米 目标地方
        self.target_posi.position.x = current_position.position.x +movements.x
        self.target_posi.position.y = current_position.position.y +movements.y 
        self.target_posi.position.z = current_position.position.z +movements.z 
        print('TARGET POSITION',self.target_posi)
        #global current_position, diferencia, moving,target
        print("Starting to listen for instructions")
        #while not rospy.is_shutdown():
        target_x = round(self.target_posi.position.x, 2)
        target_y = round(self.target_posi.position.y, 2)
        target_z = round(self.target_posi.position.z, 2)
        # print("Current: ", current_x, current_y, current_z)
        #if target_x != current_x or target_y != current_y or target_z != current_z:
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
        
        #rospy.sleep(10.0)
        #############################################
        #######question:what's the difference?
        print(f'iter {self.counter} finished')
        self.counter +=1
        # Update robot instance
        self.robot = moveit_commander.RobotCommander(robot_description="/dsr01m0609/robot_description", ns ='/dsr01m0609')
        self.moving = False
        
      
        
if __name__ == '__main__':
    try:
        node = VISO_FOLLOW()

    except rospy.ROSInterruptException:
        pass

